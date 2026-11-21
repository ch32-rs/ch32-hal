//! USB_FS and OTG_FS device mode peripheral driver
//! Note that this currently only implements device mode
//!
//! Serves the register models of the FS device block: `kind: otg` (`otg_v2`,
//! the V2/V3 OTG parts), `kind: usbfs` (`usbfs_h4`, CH32H417) and `kind: usb`
//! (`x0fs` on CH32X035, `v2fs` on CH32V20x/V303). The field-name differences
//! live in the private `regs` module.
//!
//! ## Bidirectional endpoints and buffer sizing
//!
//! Non-control endpoint numbers `1..=7` (except `EP4` on CH32X035, which shares
//! `EP0`'s DMA buffer in hardware) can be allocated in both `IN` and `OUT`
//! directions simultaneously (for example `0x81` and `0x01`).
//!
//! Each endpoint number has a single `UEPn_DMA` base register. With
//! `UEPn_BUF_MOD = 0`, hardware places `OUT` data at `[UEPn_DMA .. +64)` and,
//! when both `UEPn_RX_EN` and `UEPn_TX_EN` are enabled, offsets `IN` data to
//! `[UEPn_DMA + 64 .. +128)`. Because WCH USBFS has no per-endpoint max-RX
//! length register (and writes two CRC16 bytes past received payloads up to 64
//! bytes), `EP0` and any `OUT` endpoint always reserve 64 bytes in `ep_buffer`,
//! and bidirectional endpoint pairs reserve 128 contiguous bytes regardless of
//! the order in which `alloc_endpoint_in` and `alloc_endpoint_out` were called.
//! Budget 64 bytes per allocated endpoint plus 64 bytes for `EP0` (for example,
//! one `EndpointDataBuffer64` per allocated endpoint plus one for `EP0`).
//!
//! <div class="warning">
//! There's a lot of TODOs and panics where things are not implemented
//! </div>
//!
//! List of things that is tested
//! Untested but expected to work items are noted as well
//!
//! Control Pipe:
//! - [x] Receive `SETUP` packet (Host -> Dev)
//! - [x] Send `IN` packet (Dev -> Host).
//! - [x] Receive `OUT` packet (Host -> Dev).
//!
//! Other Endpoints:
//! - [x] Interrupt Out
//! - [x] Interrupt In
//! - [x] Bulk Out (CDC-ACM echo on the CH32V203 USBFS)
//! - [x] Bulk In (same)
//!
//! Other Features:
//! - [ ] Set endpoint stall
//! - [ ] Get endpoint stall status
//! - [ ] Remote wakeup
//!

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{fence, AtomicBool, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering};
use core::task::Poll;

use bitmaps::Bitmap;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event};
use endpoint::{ControlPipe, Endpoint};
use regs::{EpRxResponse, EpTxResponse, Regs, UsbToken};

#[cfg(usb_x0fs)]
use crate::gpio::Pull;
#[cfg(otg_v2)]
use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointDataBuffer, In, Out};
use crate::{interrupt, peripherals, Peri, PeripheralType, RccPeripheral};

pub mod endpoint;
mod regs;

// TODO: We technically support 16, but we only allow 8 for now (0, 1-7).
const MAX_NR_EP: usize = 8;
const EP_MAX_PACKET_SIZE: u16 = 64;

/// Per-endpoint, per-direction completion state shared between `on_interrupt` and tasks.
///
/// Assumes a single USBFS/OTG_FS controller instance per chip (holds across all
/// `otg_v2`, `usbfs_h4`, `usb_x0fs` and `usb_v2fs` parts).
pub(crate) struct EpDirState {
    pub(crate) waker: AtomicWaker,
    pub(crate) ready: AtomicBool,
    pub(crate) len: AtomicU16,
}

impl EpDirState {
    #[allow(clippy::declare_interior_mutable_const)]
    const NEW: Self = Self {
        waker: AtomicWaker::new(),
        ready: AtomicBool::new(false),
        len: AtomicU16::new(0),
    };

    fn reset(&self) {
        self.ready.store(false, Ordering::Relaxed);
        self.len.store(0, Ordering::Relaxed);
    }
}

static BUS_WAKER: AtomicWaker = AtomicWaker::new();
pub(crate) static EP0_WAKER: AtomicWaker = AtomicWaker::new();
pub(crate) static IN_STATE: [EpDirState; MAX_NR_EP] = [EpDirState::NEW; MAX_NR_EP];
pub(crate) static OUT_STATE: [EpDirState; MAX_NR_EP] = [EpDirState::NEW; MAX_NR_EP];

/// Latest 8-byte SETUP packet copied out of EP0's DMA buffer by `on_interrupt`.
static SETUP_WORDS: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
static SETUP_PENDING: AtomicBool = AtomicBool::new(false);
static BUS_RST_PENDING: AtomicBool = AtomicBool::new(false);

const ZERO_ADDR: AtomicUsize = AtomicUsize::new(0);
/// Full RAM base address backing `UEPn_DMA` for each endpoint number (`0..MAX_NR_EP`).
///
/// Stored separately from `UEPn_DMA` because `x0fs`/`v2fs`/`otg_v2` DMA
/// registers only hold the low 15–16 SRAM offset bits rather than a full pointer.
static EP_BUF_ADDR: [AtomicUsize; MAX_NR_EP] = [ZERO_ADDR; MAX_NR_EP];
/// Bitmask of endpoint numbers (`1..MAX_NR_EP`) allocated in both IN and OUT directions.
static EP_BIDIR_MASK: AtomicU8 = AtomicU8::new(0);

pub(crate) fn ep_buf_addr(ep: usize) -> usize {
    EP_BUF_ADDR[ep].load(Ordering::Acquire)
}

pub(crate) fn ep_is_bidir(ep: usize) -> bool {
    (EP_BIDIR_MASK.load(Ordering::Acquire) & (1 << ep)) != 0
}

pub(crate) fn setup_pending() -> bool {
    SETUP_PENDING.load(Ordering::Acquire)
}

pub(crate) fn bus_rst_pending() -> bool {
    BUS_RST_PENDING.load(Ordering::Acquire)
}

pub(crate) fn take_setup() -> Option<[u8; 8]> {
    critical_section::with(|_| {
        if !SETUP_PENDING.load(Ordering::Acquire) {
            return None;
        }
        SETUP_PENDING.store(false, Ordering::Release);
        let w0 = SETUP_WORDS[0].load(Ordering::Relaxed).to_le_bytes();
        let w1 = SETUP_WORDS[1].load(Ordering::Relaxed).to_le_bytes();
        let mut out = [0u8; 8];
        out[..4].copy_from_slice(&w0);
        out[4..].copy_from_slice(&w1);
        Some(out)
    })
}

/// Validates that `[addr .. addr + len)` is 4-byte aligned and addressable by
/// this chip's USBFS DMA engine.
fn validate_dma_region(addr: usize, len: usize) {
    assert!(addr != 0 && addr % 4 == 0, "USBFS DMA buffer must be 4-byte aligned");
    let end = addr.checked_add(len).expect("USBFS DMA buffer address overflow");
    // X035 `UEPn_DMA` latches only bits [14:0] as an offset from 0x2000_0000 (32 KiB window).
    #[cfg(usb_x0fs)]
    assert!(
        addr >= 0x2000_0000 && end <= 0x2000_8000,
        "CH32X035 USBFS DMA buffer {:#x}..{:#x} outside 0x2000_0000..0x2000_8000",
        addr,
        end
    );
    // V20x/V30x `UEPn_DMA` addresses a 64 KiB SRAM window starting at 0x2000_0000.
    #[cfg(any(usb_v2fs, otg_v2))]
    assert!(
        addr >= 0x2000_0000 && end <= 0x2001_0000,
        "USBFS DMA buffer {:#x}..{:#x} outside 64 KiB SRAM window 0x2000_0000..0x2001_0000",
        addr,
        end
    );
    let _ = end;
}

/// USB PHY configuration (X035 only).
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Use the direct VDD PHY output and pull-up. Clear this when VDD is
    /// above 4 V.
    #[cfg(usb_x0fs)]
    pub usb_phy_v33: bool,
    /// AFIO pull-up mode for UDP. `0b11` is the USB 1.5 kΩ pull-up for 3.3 V.
    #[cfg(usb_x0fs)]
    pub udp_pue: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(usb_x0fs)]
            usb_phy_v33: true,
            #[cfg(usb_x0fs)]
            udp_pue: 0b11,
        }
    }
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = T::regs();
        let int_fg = regs::int_fg(regs);

        if int_fg.fifo_ov {
            error!("[USBFS] FIFO overflow");
            regs::clear_fifo_ov(regs);
        }

        if int_fg.bus_rst {
            // Mask bus_rst until `Bus::poll` processes it; wake EP0 and all
            // active endpoints so any in-flight control or data transfer aborts.
            regs::set_bus_rst_ie(regs, false);
            BUS_RST_PENDING.store(true, Ordering::Release);
            BUS_WAKER.wake();
            EP0_WAKER.wake();
            for ep in 1..MAX_NR_EP {
                IN_STATE[ep].waker.wake();
                OUT_STATE[ep].waker.wake();
            }
        }

        if int_fg.suspend {
            regs::set_suspend_ie(regs, false);
            BUS_WAKER.wake();
        }

        if int_fg.transfer {
            let status = regs::int_st(regs);
            let token = status.token;

            match token {
                UsbToken::SETUP => {
                    // Disarm EP0 both ways, snapshot the 8-byte SETUP packet, and
                    // release `UIF_TRANSFER` immediately so `INT_BUSY` never wedges.
                    regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
                    regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::NAK);
                    IN_STATE[0].reset();
                    OUT_STATE[0].reset();

                    let ep0_addr = EP_BUF_ADDR[0].load(Ordering::Acquire);
                    if ep0_addr != 0 {
                        fence(Ordering::Acquire);
                        let p = ep0_addr as *const u32;
                        SETUP_WORDS[0].store(core::ptr::read_volatile(p), Ordering::Relaxed);
                        SETUP_WORDS[1].store(core::ptr::read_volatile(p.add(1)), Ordering::Relaxed);
                        SETUP_PENDING.store(true, Ordering::Release);
                    }
                    regs::clear_transfer(regs);
                    EP0_WAKER.wake();
                }
                UsbToken::IN => {
                    let ep = status.ep as usize;
                    if ep == 0 {
                        regs::ep_tx_ctrl_set_res(regs, 0, EpTxResponse::NAK);
                        IN_STATE[0].ready.store(true, Ordering::Release);
                        regs::clear_transfer(regs);
                        EP0_WAKER.wake();
                    } else if ep < MAX_NR_EP {
                        regs::ep_tx_ctrl_flip_tog(regs, ep, EpTxResponse::NAK);
                        IN_STATE[ep].ready.store(true, Ordering::Release);
                        regs::clear_transfer(regs);
                        IN_STATE[ep].waker.wake();
                    } else {
                        error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        regs::clear_transfer(regs);
                    }
                }
                UsbToken::OUT => {
                    let ep = status.ep as usize;
                    if ep == 0 {
                        if status.tog_ok {
                            let len = regs::rx_len(regs);
                            regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                            OUT_STATE[0].len.store(len, Ordering::Relaxed);
                            OUT_STATE[0].ready.store(true, Ordering::Release);
                            regs::clear_transfer(regs);
                            EP0_WAKER.wake();
                        } else {
                            // Retransmitted OUT packet: leave EP0 RX armed for the matching toggle.
                            regs::clear_transfer(regs);
                        }
                    } else if ep < MAX_NR_EP {
                        if status.tog_ok {
                            let len = regs::rx_len(regs);
                            regs::ep_rx_ctrl_flip_tog(regs, ep, EpRxResponse::NAK);
                            OUT_STATE[ep].len.store(len, Ordering::Relaxed);
                            OUT_STATE[ep].ready.store(true, Ordering::Release);
                            regs::clear_transfer(regs);
                            OUT_STATE[ep].waker.wake();
                        } else {
                            // Retransmitted OUT packet: leave endpoint armed.
                            regs::clear_transfer(regs);
                        }
                    } else {
                        error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        regs::clear_transfer(regs);
                    }
                }
                // X0 names bit 0 of the token field `SOF`, not `RSVD`.
                _ => {
                    regs::clear_transfer(regs);
                }
            }
        }

        // Clear the host sof, we are in device mode.... It's garbage
        if int_fg.hst_sof {
            regs::clear_hst_sof(regs);
        }
    }
}

pub struct Driver<'d, T: Instance, const NR_EP: usize, const SIZE: usize> {
    phantom: PhantomData<&'d mut (T, [EndpointDataBuffer<SIZE>; NR_EP])>,
    buf_base: *mut u8,
    buf_cap: usize,
    allocated_in: Bitmap<MAX_NR_EP>,
    allocated_out: Bitmap<MAX_NR_EP>,
}

impl<'d, T, const NR_EP: usize, const SIZE: usize> Driver<'d, T, NR_EP, SIZE>
where
    T: Instance,
{
    pub fn new(
        _usb: Peri<'d, T>,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl crate::gpio::Pin>,
        dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ) -> Self {
        Self::new_with_config(_usb, dp, dm, ep_buffer, Config::default())
    }

    /// Use this when board-level USB PHY or pull-up settings differ from the
    /// default 3.3 V setup.
    pub fn new_with_config(
        _usb: Peri<'d, T>,
        // _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        dp: Peri<'d, impl crate::gpio::Pin>,
        dm: Peri<'d, impl crate::gpio::Pin>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
        config: Config,
    ) -> Self {
        assert!(!ep_buffer.is_empty());

        // Disable IRQ before resetting hardware and clearing shared static state.
        T::Interrupt::disable();

        // RM table 9-12: on H4 the FS controller drives DP/DM itself once it
        // is clocked, so the pins need no alternate-function setup; the
        // CH32V20x manual says the same of its USBFS (table 10-9).
        #[cfg(otg_v2)]
        {
            dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
            dm.set_as_af_output(AFType::OutputPushPull, Speed::High);
            let _ = config;
        }
        #[cfg(any(usbfs_h4, usb_v2fs))]
        let _ = (dp, dm, config);
        // X035 keeps DP/DM as GPIO inputs and switches the on-die pull-up on
        // through `AFIO.CTLR` instead of selecting an alternate function.
        #[cfg(usb_x0fs)]
        {
            dp.set_as_input(Pull::Up);
            dm.set_as_input(Pull::None);

            crate::pac::AFIO.ctlr().modify(|w| {
                w.set_usb_phy_v33(config.usb_phy_v33);
                w.set_udp_pue(config.udp_pue);
                w.set_udm_pue(0);
                w.set_usb_ioen(true);
            });
            regs::save_pull_up(config.udp_pue);
        }

        T::enable_and_reset();

        // Ensure all non-control endpoints are disabled after reset so no
        // endpoint appears enabled before `start()` + `SET_CONFIGURATION`.
        let regs = T::regs();
        for ep in 1..MAX_NR_EP {
            endpoint::set_out_logically_enabled(ep, false);
            regs::ep_set_enabled(regs, ep, Direction::In, false);
            regs::ep_set_enabled(regs, ep, Direction::Out, false);
        }

        SETUP_PENDING.store(false, Ordering::Relaxed);
        BUS_RST_PENDING.store(false, Ordering::Relaxed);
        EP_BIDIR_MASK.store(0, Ordering::Relaxed);
        for slot in &EP_BUF_ADDR {
            slot.store(0, Ordering::Relaxed);
        }
        for i in 0..MAX_NR_EP {
            IN_STATE[i].reset();
            OUT_STATE[i].reset();
        }

        let (buf_base, buf_cap) = EndpointDataBuffer::slice_as_mut_ptr(ep_buffer);

        // EP4 shares EP0's DMA buffer on X0, so it cannot be handed out.
        #[cfg(usb_x0fs)]
        let reserved = {
            let mut bm = Bitmap::new();
            bm.set(4, true);
            bm
        };
        #[cfg(not(usb_x0fs))]
        let reserved = Bitmap::new();

        Self {
            phantom: PhantomData,
            buf_base,
            buf_cap,
            allocated_in: reserved,
            allocated_out: reserved,
        }
    }

    #[inline]
    const fn is_reserved_ep(ep: usize) -> bool {
        #[cfg(usb_x0fs)]
        {
            ep == 4
        }
        #[cfg(not(usb_x0fs))]
        {
            let _ = ep;
            false
        }
    }

    /// DMA buffer bytes required for an endpoint `ep` (`1..MAX_NR_EP`).
    ///
    /// - Bidirectional (`IN` + `OUT`): 128 contiguous bytes (`OUT` at `+0`, `IN` at `+64`).
    /// - Unidirectional `OUT`: 64 bytes (WCH USBFS has no hardware max-RX-length
    ///   register and appends 2 CRC16 bytes up to 64 bytes; RM §23.2.2.6 Note 1).
    /// - Unidirectional `IN`: `min(SIZE, 64)` bytes (4-byte aligned since `SIZE` is a power of two >= 8).
    #[inline]
    const fn ep_slot_bytes(has_in: bool, has_out: bool) -> usize {
        let ep_max = EP_MAX_PACKET_SIZE as usize;
        match (has_in, has_out) {
            (true, true) => 2 * ep_max,
            (false, true) => ep_max,
            (true, false) => {
                if SIZE < ep_max {
                    SIZE
                } else {
                    ep_max
                }
            }
            (false, false) => 0,
        }
    }

    /// Total DMA buffer bytes required for `EP0` (64 bytes) plus all claimed endpoints.
    fn required_buffer_bytes(allocated_in: Bitmap<MAX_NR_EP>, allocated_out: Bitmap<MAX_NR_EP>) -> usize {
        let mut total = usize::from(EP_MAX_PACKET_SIZE); // 64 bytes reserved for EP0

        for ep in 1..MAX_NR_EP {
            if Self::is_reserved_ep(ep) {
                continue;
            }
            total += Self::ep_slot_bytes(allocated_in.get(ep), allocated_out.get(ep));
        }
        total
    }

    fn find_free_ep_address(&self, dir: Direction) -> Result<u8, EndpointAllocError> {
        // Endpoint 0 is reserved for the control pipe (claimed in start()).
        // `next_false_index` searches strictly *after* the given index, so
        // `next_false_index(0)` only returns non-control endpoints (>= 1),
        // matching the behavior of Embassy USB drivers.
        //
        // Prefer an endpoint number unused in both directions first; fall back
        // to sharing an endpoint number whose `dir` half is still free.
        let dir_allocated = match dir {
            Direction::In => self.allocated_in,
            Direction::Out => self.allocated_out,
        };
        let ep = (self.allocated_in | self.allocated_out)
            .next_false_index(0)
            .or_else(|| dir_allocated.next_false_index(0))
            .ok_or(EndpointAllocError)?;
        Ok(ep as u8)
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
        dir: Direction,
    ) -> Result<Endpoint<'d, T, D, SIZE>, EndpointAllocError> {
        // Isochronous transfers require different handshake/toggle handling not implemented here.
        if ep_type == EndpointType::Isochronous {
            return Err(EndpointAllocError);
        }
        if max_packet_size > EP_MAX_PACKET_SIZE || usize::from(max_packet_size) > SIZE {
            return Err(EndpointAllocError);
        }

        let addr = match ep_addr {
            Some(addr) => {
                // Use the provided endpoint address
                if addr.direction() != dir {
                    return Err(EndpointAllocError);
                }

                let ep_num = addr.index();
                // Endpoint 0 belongs to the control pipe, which claims it in
                // start(); class endpoint allocation must never hand it out.
                if ep_num == 0 || ep_num >= MAX_NR_EP || Self::is_reserved_ep(ep_num) {
                    return Err(EndpointAllocError);
                }

                let already_allocated = match dir {
                    Direction::In => self.allocated_in.get(ep_num),
                    Direction::Out => self.allocated_out.get(ep_num),
                };
                if already_allocated {
                    return Err(EndpointAllocError);
                }

                addr
            }
            None => {
                let ep_num = self.find_free_ep_address(dir)?;
                EndpointAddress::from_parts(ep_num as usize, dir)
            }
        };

        let mut next_in = self.allocated_in;
        let mut next_out = self.allocated_out;
        match dir {
            Direction::In => {
                next_in.set(addr.index(), true);
            }
            Direction::Out => {
                next_out.set(addr.index(), true);
            }
        }

        if Self::required_buffer_bytes(next_in, next_out) > self.buf_cap {
            error!("out of endpoint buffers");
            return Err(EndpointAllocError);
        }

        self.allocated_in = next_in;
        self.allocated_out = next_out;

        Ok(Endpoint::new(EndpointInfo {
            addr,
            ep_type,
            max_packet_size,
            interval_ms,
            max_burst: 0,
        }))
    }
}

impl<'d, T: Instance, const NR_EP: usize, const SIZE: usize> embassy_usb_driver::Driver<'d>
    for Driver<'d, T, NR_EP, SIZE>
{
    type EndpointOut = Endpoint<'d, T, Out, SIZE>;

    type EndpointIn = Endpoint<'d, T, In, SIZE>;

    type ControlPipe = ControlPipe<'d, T, SIZE>;

    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(
            config.ep_type,
            config.addr,
            config.max_packet_size,
            config.interval_ms,
            Direction::Out,
        )
    }

    fn alloc_endpoint_in(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(
            config.ep_type,
            config.addr,
            config.max_packet_size,
            config.interval_ms,
            Direction::In,
        )
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(control_max_packet_size <= EP_MAX_PACKET_SIZE);
        assert!(
            usize::from(EP_MAX_PACKET_SIZE) <= self.buf_cap,
            "ep_buffer pool ({} bytes) must hold at least 64 bytes for EP0",
            self.buf_cap
        );
        let regs = T::regs();

        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::NAK);

        regs::reset_sie(regs);

        crate::internal::delay::block_for_us(10);

        // Clear all
        regs::clear_ctrl(regs);

        // Host SOF is ignored, not our usecase here
        regs::enable_interrupts(regs);

        regs::enter_device_mode(regs);

        // Lay out contiguous DMA buffers now that all IN/OUT endpoints are known.
        let ep0_bytes = usize::from(EP_MAX_PACKET_SIZE);
        let mut offset = 0usize;

        let ep0_addr = unsafe { self.buf_base.add(offset) } as usize;
        offset += ep0_bytes;
        validate_dma_region(ep0_addr, ep0_bytes);
        EP_BUF_ADDR[0].store(ep0_addr, Ordering::Release);
        regs::set_ep_dma(regs, 0, ep0_addr as u32);

        let mut bidir_mask = 0u8;
        for ep in 1..MAX_NR_EP {
            if Self::is_reserved_ep(ep) {
                continue;
            }
            let has_in = self.allocated_in.get(ep);
            let has_out = self.allocated_out.get(ep);
            let slot_bytes = Self::ep_slot_bytes(has_in, has_out);
            if slot_bytes == 0 {
                continue;
            }
            if has_in && has_out {
                bidir_mask |= 1 << ep;
            }
            let addr = unsafe { self.buf_base.add(offset) } as usize;
            offset += slot_bytes;
            validate_dma_region(addr, slot_bytes);
            EP_BUF_ADDR[ep].store(addr, Ordering::Release);
            regs::set_ep_dma(regs, ep, addr as u32);
        }
        assert!(offset <= self.buf_cap, "endpoint buffer pool overflow");
        EP_BIDIR_MASK.store(bidir_mask, Ordering::Release);

        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::ACK);
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);

        critical_section::with(|_cs| {
            T::Interrupt::unpend();
            unsafe {
                T::Interrupt::enable();
            }
        });

        (
            Bus {
                _phantom: PhantomData,
                inited: false,
            },
            ControlPipe::new(control_max_packet_size),
        )
    }
}

pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    inited: bool,
}

impl<'d, T: Instance> Drop for Bus<'d, T> {
    fn drop(&mut self) {
        T::Interrupt::disable();
        let regs = T::regs();
        regs::set_pull_up(regs, false);
        regs::clear_ctrl(regs);
        for slot in &EP_BUF_ADDR {
            slot.store(0, Ordering::Release);
        }
    }
}

impl<'d, T: Instance> Bus<'d, T> {
    fn bus_reset(&mut self) {
        let regs = T::regs();

        // Reset device address
        regs::set_device_address(regs, 0);

        SETUP_PENDING.store(false, Ordering::Release);
        IN_STATE[0].reset();
        OUT_STATE[0].reset();

        // Restore DMA base pointers for all configured endpoints
        for (ep, slot) in EP_BUF_ADDR.iter().enumerate() {
            let addr = slot.load(Ordering::Acquire);
            if addr != 0 {
                regs::set_ep_dma(regs, ep, addr as u32);
            }
        }

        // Reset endpoint state on bus reset
        // EP0 gets ACK/NAK so it can receive setup
        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::ACK);
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);

        // Mark all other EPs as NAK and disabled
        for i in 1..MAX_NR_EP {
            use embassy_usb_driver::Bus;
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::In), false);
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::Out), false);
        }

        BUS_RST_PENDING.store(false, Ordering::Release);
        EP0_WAKER.wake();
    }
}

impl<'d, T> embassy_usb_driver::Bus for Bus<'d, T>
where
    T: Instance,
{
    async fn enable(&mut self) {
        // `start` already set the pull-up for the first enable; set it again after a `disable`.
        regs::set_pull_up(T::regs(), true);

        // Enable the port
        regs::enable_port(T::regs());

        // Do a bus reset on "enable"
        self.bus_reset();
    }

    async fn disable(&mut self) {
        // Detach from the bus, as TinyUSB's `dcd_disconnect` does on this block.
        regs::set_pull_up(T::regs(), false);
        // embassy-usb enables the bus again only on `PowerDetected`, which `poll` fakes once.
        self.inited = false;
    }

    async fn poll(&mut self) -> embassy_usb_driver::Event {
        // TODO: VBUS detection
        if !self.inited {
            self.inited = true;
            return Event::PowerDetected;
        }

        poll_fn(|ctx| {
            BUS_WAKER.register(ctx.waker());
            let regs = T::regs();
            let interrupt_flags = regs::int_fg(regs);

            // Either Suspend or Resume has happened
            if interrupt_flags.suspend {
                regs::clear_suspend(regs);
                regs::set_suspend_ie(regs, true);

                if regs::is_suspended(regs) {
                    Poll::Ready(Event::Suspend)
                } else {
                    Poll::Ready(Event::Resume)
                }
            } else if interrupt_flags.bus_rst || bus_rst_pending() {
                trace!("bus: reset");
                self.bus_reset();
                regs::clear_bus_rst(regs);
                regs::set_bus_rst_ie(regs, true);

                Poll::Ready(Event::Reset)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        #[cfg(feature = "defmt")]
        trace!(
            "[USBFS] Endpoint: {}, {}: Set enable={}",
            ep_addr.index(),
            ep_addr.direction(),
            enabled
        );
        let ep = ep_addr.index();
        assert!(ep > 0 && ep < MAX_NR_EP, "invalid USBFS endpoint index {}", ep);
        let regs = T::regs();

        // Reset data toggle to DATA0 and disarm endpoint to NAK before updating enable state.
        // For bidirectional endpoint numbers (`ep_is_bidir(ep)`), keep `RX_EN` asserted while
        // `TX_EN` is enabled (with `R_RES = NAK` when OUT itself is disabled so OUT packets
        // are never accepted) so hardware's IN DMA offset stays fixed at `+64` across
        // per-endpoint `SET_CONFIGURATION` / `SET_INTERFACE` enable calls.
        critical_section::with(|_| match ep_addr.direction() {
            Direction::In => {
                IN_STATE[ep].reset();
                regs::ep_tx_ctrl_write(regs, ep, false, EpTxResponse::NAK);
                regs::ep_set_enabled(regs, ep, Direction::In, enabled);
                if ep_is_bidir(ep) {
                    if enabled {
                        if !endpoint::is_out_logically_enabled(ep) {
                            regs::ep_rx_ctrl_write(regs, ep, false, EpRxResponse::NAK);
                        }
                        regs::ep_set_enabled(regs, ep, Direction::Out, true);
                    } else if !endpoint::is_out_logically_enabled(ep) {
                        regs::ep_set_enabled(regs, ep, Direction::Out, false);
                    }
                }
            }
            Direction::Out => {
                OUT_STATE[ep].reset();
                endpoint::set_out_logically_enabled(ep, enabled);
                regs::ep_rx_ctrl_write(regs, ep, false, EpRxResponse::NAK);
                let rx_hw_en = if !enabled && ep_is_bidir(ep) && regs::ep_is_enabled(regs, ep, Direction::In) {
                    true
                } else {
                    enabled
                };
                regs::ep_set_enabled(regs, ep, Direction::Out, rx_hw_en);
            }
        });

        match ep_addr.direction() {
            Direction::In => IN_STATE[ep].waker.wake(),
            Direction::Out => OUT_STATE[ep].waker.wake(),
        }
    }

    fn endpoint_set_stalled(&mut self, _ep_addr: EndpointAddress, _stalled: bool) {
        todo!()
    }

    fn endpoint_is_stalled(&mut self, _ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    async fn remote_wakeup(&mut self) -> Result<(), embassy_usb_driver::Unsupported> {
        todo!()
    }
}

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

trait SealedInstance: RccPeripheral {
    fn regs() -> Regs;
}

/// OTG_FS peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    type Interrupt: interrupt::typelevel::Interrupt;
}

#[cfg(otg_v2)]
foreach_peripheral!(
    (otg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> Regs {
                // datasheet
                unsafe { Regs::from_ptr(crate::pac::OTG_FS.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

// H4's FS controller (`kind: usbfs`, version `h4`) is the same device block as
// the V4x7 OTG FS, addressed at its own base (RM §26).
#[cfg(usbfs_h4)]
foreach_peripheral!(
    (usbfs, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> Regs {
                unsafe { Regs::from_ptr(crate::pac::USBFS.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

// X035's FS controller (`kind: usb`, version `x0fs`) keeps the same device
// core in array-grouped registers; the CH32V20x/V303 one (`v2fs`) is the
// `otg_v2` block without its OTG registers.
#[cfg(any(usb_x0fs, usb_v2fs))]
foreach_peripheral!(
    (usb, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> Regs {
                unsafe { Regs::from_ptr(crate::pac::$inst.as_ptr()) }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);
