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
//! `[UEPn_DMA + 64 .. +128)`. `Driver::start` assigns contiguous 128-byte
//! regions from `ep_buffer` to bidirectional endpoint pairs regardless of the
//! order in which `alloc_endpoint_in` and `alloc_endpoint_out` were called.
//! Budget one `EndpointDataBuffer64` per allocated endpoint plus one for `EP0`.
//!
//! <div class="warning">
//! There's a lot of TODOs and panics where things are not implemented
//! </div>
//!
//! List of things that is tested
//! Untested but expected to work items are noted as well
//!
//! Control Pipe:
//! - [x] Recieve `SETUP` packet (Host -> Dev)
//! - [x] Send `IN` packet (Dev -> Host).
//! - [x] Recieve `OUT` packet (Host -> Dev).
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
use core::sync::atomic::{AtomicUsize, Ordering};
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

const NEW_AW: AtomicWaker = AtomicWaker::new();
static BUS_WAKER: AtomicWaker = NEW_AW;
static EP0_WAKER: AtomicWaker = NEW_AW;
static EP_IN_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];
static EP_OUT_WAKERS: [AtomicWaker; MAX_NR_EP] = [NEW_AW; MAX_NR_EP];

const ZERO_ADDR: AtomicUsize = AtomicUsize::new(0);
/// Full RAM base address backing `UEPn_DMA` for each endpoint number (`0..MAX_NR_EP`).
///
/// Stored separately from `UEPn_DMA` because `x0fs`/`v2fs` DMA registers only
/// hold the low 15–16 SRAM offset bits rather than a full pointer.
static EP_BUF_ADDR: [AtomicUsize; MAX_NR_EP] = [ZERO_ADDR; MAX_NR_EP];

pub(crate) fn ep_buf_addr(ep: usize) -> usize {
    EP_BUF_ADDR[ep].load(Ordering::Relaxed)
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
            panic!("overflow");
        }

        // All the interrupt we handle
        if int_fg.suspend || int_fg.transfer || int_fg.bus_rst {
            T::Interrupt::disable();

            // Bus stuff we wakup bus
            if int_fg.bus_rst || int_fg.suspend {
                BUS_WAKER.wake();
            }

            if int_fg.transfer {
                let status = regs::int_st(regs);

                let token = status.token;
                match token {
                    UsbToken::IN => {
                        let ep = status.ep as usize;

                        if ep == 0 {
                            EP0_WAKER.wake();
                        } else if ep < MAX_NR_EP {
                            EP_IN_WAKERS[ep].wake();
                        } else {
                            error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        }
                    }
                    UsbToken::OUT => {
                        let ep = status.ep as usize;

                        if ep == 0 {
                            EP0_WAKER.wake();
                        } else if ep < MAX_NR_EP {
                            EP_OUT_WAKERS[ep].wake();
                        } else {
                            error!("[USBFS] Unexpected EP: {} got token: {:#x}", ep, token.to_bits());
                        }
                    }
                    UsbToken::SETUP => {
                        EP0_WAKER.wake();
                    }
                    // X0 names bit 0 of the token field `SOF`, not `RSVD`.
                    _ => panic!("rsvd token"),
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

        for slot in &EP_BUF_ADDR {
            slot.store(0, Ordering::Relaxed);
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

    /// Total DMA buffer bytes required for `EP0` plus all currently claimed
    /// endpoints. Unidirectional endpoints take one `min(SIZE, 64)` slot;
    /// bidirectional endpoint pairs always require 128 contiguous bytes
    /// (`OUT` at `+0`, `IN` at `+64`).
    fn required_buffer_bytes(allocated_in: Bitmap<MAX_NR_EP>, allocated_out: Bitmap<MAX_NR_EP>) -> usize {
        let uni_slot = SIZE.min(usize::from(EP_MAX_PACKET_SIZE));
        let bidir_slot = 2 * usize::from(EP_MAX_PACKET_SIZE);
        let mut total = uni_slot; // Reserved for EP0 (claimed in start())

        for ep in 1..MAX_NR_EP {
            if Self::is_reserved_ep(ep) {
                continue;
            }
            match (allocated_in.get(ep), allocated_out.get(ep)) {
                (true, true) => total += bidir_slot,
                (true, false) | (false, true) => total += uni_slot,
                (false, false) => {}
            }
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
            usize::from(control_max_packet_size) <= SIZE,
            "control_max_packet_size {} exceeds buffer size {}",
            control_max_packet_size,
            SIZE
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
        let uni_slot = SIZE.min(usize::from(EP_MAX_PACKET_SIZE));
        let bidir_slot = 2 * usize::from(EP_MAX_PACKET_SIZE);
        let mut offset = 0usize;

        let ep0_addr = unsafe { self.buf_base.add(offset) } as usize;
        offset += uni_slot;
        EP_BUF_ADDR[0].store(ep0_addr, Ordering::Relaxed);
        regs::set_ep_dma(regs, 0, ep0_addr as u32);

        for ep in 1..MAX_NR_EP {
            if Self::is_reserved_ep(ep) {
                continue;
            }
            let slot_bytes = match (self.allocated_in.get(ep), self.allocated_out.get(ep)) {
                (true, true) => bidir_slot,
                (true, false) | (false, true) => uni_slot,
                (false, false) => continue,
            };
            let addr = unsafe { self.buf_base.add(offset) } as usize;
            offset += slot_bytes;
            EP_BUF_ADDR[ep].store(addr, Ordering::Relaxed);
            regs::set_ep_dma(regs, ep, addr as u32);
        }
        assert!(offset <= self.buf_cap, "endpoint buffer pool overflow");

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

pub struct Bus<'d, T> {
    _phantom: PhantomData<&'d T>,
    inited: bool,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn bus_reset(&mut self) {
        let regs = T::regs();

        // Reset device address
        regs::set_device_address(regs, 0);

        // Restore DMA base pointers for all configured endpoints
        for (ep, slot) in EP_BUF_ADDR.iter().enumerate() {
            let addr = slot.load(Ordering::Relaxed);
            if addr != 0 {
                regs::set_ep_dma(regs, ep, addr as u32);
            }
        }

        // Reset endpoint state on bus reset
        // EP0 get ACK/NAK so it can recieve setup
        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::ACK);
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);

        // Mark all other EPs as NAK
        for i in 1..=7 {
            use embassy_usb_driver::Bus;
            regs::ep_rx_ctrl_write(regs, i, false, EpRxResponse::NAK);
            regs::ep_tx_ctrl_write(regs, i, false, EpTxResponse::NAK);

            // It looks like the HW has a bug, after reset, all EP Tx/Rx
            // are set to enabled. So we disable them here after bus reset.
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::In), false);
            self.endpoint_set_enabled(EndpointAddress::from_parts(i, Direction::Out), false);
        }
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

            let poll_res = {
                let regs = T::regs();
                let interrupt_flags = regs::int_fg(regs);

                // Either Suspend or Resume has happened
                if interrupt_flags.suspend {
                    // Clear suspend flag
                    regs::clear_suspend(regs);

                    if regs::is_suspended(regs) {
                        Poll::Ready(Event::Suspend)
                    } else {
                        Poll::Ready(Event::Resume)
                    }
                } else if interrupt_flags.bus_rst {
                    trace!("bus: reset");
                    self.bus_reset();

                    regs::clear_bus_rst(regs);

                    Poll::Ready(Event::Reset)
                } else {
                    Poll::Pending
                }
            };
            unsafe { T::Interrupt::enable() };
            poll_res
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
        let regs = T::regs();
        let ep = ep_addr.index();

        // Reset data toggle to DATA0 and disarm disabled endpoints to NAK
        match ep_addr.direction() {
            Direction::In => {
                regs::ep_tx_ctrl_set_tog(regs, ep, false);
                if !enabled {
                    regs::ep_tx_ctrl_set_res(regs, ep, EpTxResponse::NAK);
                }
                regs::ep_set_enabled(regs, ep, Direction::In, enabled);
                EP_IN_WAKERS[ep].wake();
            }
            Direction::Out => {
                regs::ep_rx_ctrl_set_tog(regs, ep, false);
                if !enabled {
                    regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::NAK);
                }
                regs::ep_set_enabled(regs, ep, Direction::Out, enabled);
                EP_OUT_WAKERS[ep].wake();
            }
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
