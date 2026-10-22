//! USBHS device mode peripheral driver
//! Note that this currently only implements device mode
//!
//! Every transfer completes in the interrupt handler: it reads the shared status
//! (`INT_ST`, `RX_LEN`) and releases `INT_FG` at once -- isochronous transactions ignore
//! `INT_BUSY` and would overwrite them -- then records the result per endpoint and wakes
//! the task. Bulk / interrupt endpoints keep flow control by answering NAK until their
//! task has taken the data (OUT) or loaded the next packet (IN).
//!
//! Isochronous endpoints run without a task in the loop:
//! - OUT: a ring of [`ISO_OUT_SLOTS`] packet buffers. The interrupt points the endpoint's
//!   DMA at the next free slot after each packet; `read` drains the ring, or
//!   [`iso_out_take`] in place. A full ring drops the newest packet.
//! - IN: a ring of [`ISO_IN_SLOTS`] packets. After each IN transaction the interrupt
//!   loads the next queued packet, or a zero-length one when none is queued. `write`
//!   waits for a free slot, so a writer keeps the ring full and stays at most
//!   [`ISO_IN_SLOTS`] packets ahead; [`iso_in_prime`] queues from an interrupt (e.g. an
//!   explicit feedback value from the SOF callback), [`iso_in_fill`] in place.
//! - Both work from the SOF callback, without a task: one packet each way per microframe.
//! - Each slot is one endpoint buffer from the driver's buffer array, so the array needs
//!   room for the rings: one buffer for EP0 and each bulk / interrupt endpoint, plus
//!   [`ISO_OUT_SLOTS`] per isochronous OUT and [`ISO_IN_SLOTS`] per isochronous IN.
//!
//! [`set_sof_callback`] runs a function from the interrupt on every SOF (125 us at high
//! speed).
//!
//! An endpoint number can be used for both an IN and an OUT endpoint.
//!
//! Not supported: remote wakeup, high-bandwidth isochronous (more than one transaction per
//! microframe), hardware double buffering.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, AtomicUsize, Ordering};
use core::task::Poll;

use bitmaps::Bitmap;
use ch32_metapac::usbhs::regs::{EpBufMod, EpConfig, EpType};
use ch32_metapac::usbhs::vals::{EndpointType as EpKind, EpRxResponse, EpTog, EpTxResponse, SpeedType, UsbToken};
use control::ControlPipe;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event, Unsupported,
};
use endpoint::Endpoint;

use crate::gpio::{AFType, Speed};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointBufferAllocator, EndpointDataBuffer, In, Out};
use crate::{interrupt, Peri, PeripheralType};

pub mod control;
mod endpoint;

const MAX_NR_EP: usize = 16;
const EP_MAX_PACKET_SIZE: u16 = 64;

/// Packet buffers per isochronous OUT endpoint (15 packets can wait for the task).
pub const ISO_OUT_SLOTS: usize = 16;
/// Packets queued per isochronous IN endpoint.
pub const ISO_IN_SLOTS: usize = 2;
// the ring indices are u8s that wrap: a slot count that doesn't divide 256 would skip slots
const _: () = assert!(256 % ISO_OUT_SLOTS == 0 && 256 % ISO_IN_SLOTS == 0);

/// The isochronous IN response: "data ready" (TinyUSB `USBHS_EP_T_RES_NYET`).
const TX_RES_ISO: EpTxResponse = EpTxResponse::_RESERVED_1;

static BUS_WAKER: AtomicWaker = AtomicWaker::new();

/// Per endpoint and direction, shared between the interrupt and the endpoint's task.
pub(crate) struct EpState {
    pub(crate) waker: AtomicWaker,
    /// bulk / interrupt / control: OUT -- a packet of `len` bytes is in the buffer; IN --
    /// the loaded packet went out
    pub(crate) ready: AtomicBool,
    pub(crate) len: AtomicU16,
    /// OUT: a read is waiting (the endpoint answers ACK)
    pub(crate) armed: AtomicBool,
    pub(crate) iso: AtomicBool,
    // isochronous ring: slot i at base + (i % slots) * stride; [tail, head) hold packets
    pub(crate) base: AtomicU32,
    pub(crate) stride: AtomicU16,
    pub(crate) slots: AtomicU8,
    pub(crate) head: AtomicU8,
    pub(crate) tail: AtomicU8,
    /// IN: slot `tail` is loaded in the endpoint (else a zero-length packet is)
    pub(crate) loaded: AtomicBool,
    pub(crate) lens: [AtomicU16; ISO_OUT_SLOTS],
    /// OUT: packets dropped on a full ring
    pub(crate) overruns: AtomicU32,
    /// IN: the frame number of the last iso completion (`NO_FRAME`: none)
    pub(crate) last_frame: AtomicU16,
}

impl EpState {
    #[allow(clippy::declare_interior_mutable_const)]
    const NEW: Self = Self {
        waker: AtomicWaker::new(),
        ready: AtomicBool::new(false),
        len: AtomicU16::new(0),
        armed: AtomicBool::new(false),
        iso: AtomicBool::new(false),
        base: AtomicU32::new(0),
        stride: AtomicU16::new(0),
        slots: AtomicU8::new(1),
        head: AtomicU8::new(0),
        tail: AtomicU8::new(0),
        loaded: AtomicBool::new(false),
        lens: [const { AtomicU16::new(0) }; ISO_OUT_SLOTS],
        overruns: AtomicU32::new(0),
        last_frame: AtomicU16::new(NO_FRAME),
    };

    fn reset(&self) {
        self.last_frame.store(NO_FRAME, Ordering::Relaxed);
        self.ready.store(false, Ordering::Relaxed);
        self.armed.store(false, Ordering::Relaxed);
        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        self.loaded.store(false, Ordering::Relaxed);
    }

    pub(crate) fn count(&self) -> u8 {
        self.head
            .load(Ordering::Acquire)
            .wrapping_sub(self.tail.load(Ordering::Acquire))
    }

    pub(crate) fn slot_addr(&self, i: u8) -> u32 {
        let n = self.slots.load(Ordering::Relaxed);
        self.base.load(Ordering::Relaxed) + (i % n) as u32 * self.stride.load(Ordering::Relaxed) as u32
    }
}

pub(crate) static OUT_STATE: [EpState; MAX_NR_EP] = [EpState::NEW; MAX_NR_EP];
pub(crate) static IN_STATE: [EpState; MAX_NR_EP] = [EpState::NEW; MAX_NR_EP];

/// The setup packet, copied out of the EP0 buffer by the interrupt.
static SETUP: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
static SETUP_READY: AtomicBool = AtomicBool::new(false);
/// EP0's buffer: the DMA address registers read back only their low 16 bits.
static EP0_BUF: AtomicU32 = AtomicU32::new(0);

/// The peripheral's base address, for the free functions (0 until a driver exists).
static REGS: AtomicUsize = AtomicUsize::new(0);
static SOF_CALLBACK: AtomicUsize = AtomicUsize::new(0);

fn dregs() -> Option<crate::pac::usbhs::Usbd> {
    match REGS.load(Ordering::Acquire) {
        0 => None,
        a => Some(unsafe { crate::pac::usbhs::Usbd::from_ptr(a as _) }),
    }
}

/// Call `f` from the USB interrupt on every SOF (every 125 us microframe at high speed).
/// Keep it short: it delays every other USB completion.
pub fn set_sof_callback(f: fn()) {
    SOF_CALLBACK.store(f as usize, Ordering::Release);
    if let Some(d) = dregs() {
        critical_section::with(|_| d.int_en().modify(|w| w.set_sof_act(true)));
    }
}

/// The frame number register: frame (bits 10:0) and microframe (bits 15:13); bits 12:11
/// are always 0, so this is never a frame number.
const NO_FRAME: u16 = 0x1800;

/// The current microframe number, from the frame number register: 8 per frame, wrapping
/// at 2^14. Counts microframes whose SOF interrupt was lost (see the iso IN completion).
pub fn microframe() -> u16 {
    match dregs() {
        Some(d) => {
            let f = d.frame_no().read().frame_no();
            (f & 0x7FF) << 3 | f >> 13
        }
        None => 0,
    }
}

/// Whether the host enabled endpoint `ep` in `direction` (for an isochronous one: selected
/// its interface's streaming alternate setting).
pub fn endpoint_enabled(ep: usize, direction: Direction) -> bool {
    let Some(d) = dregs() else { return false };
    if ep == 0 || ep >= MAX_NR_EP {
        return false;
    }
    match direction {
        Direction::Out => d.ep_config().read().r_en(ep - 1),
        Direction::In => d.ep_config().read().t_en(ep - 1),
    }
}

/// Take the oldest packet waiting on isochronous OUT endpoint `ep`, in place: `f` reads
/// it, then its slot goes back to the ring. `None`: no packet waiting (or not an
/// isochronous OUT endpoint). One reader per endpoint; callable from interrupts.
pub fn iso_out_take<R>(ep: usize, f: impl FnOnce(&[u8]) -> R) -> Option<R> {
    if ep == 0 || ep >= MAX_NR_EP {
        return None;
    }
    let s = &OUT_STATE[ep];
    if !s.iso.load(Ordering::Relaxed) || s.count() == 0 {
        return None;
    }
    let tail = s.tail.load(Ordering::Relaxed);
    let n = s.slots.load(Ordering::Relaxed);
    let len = s.lens[(tail % n) as usize].load(Ordering::Relaxed) as usize;
    // SAFETY: a slot in [tail, head): the DMA is done with it until tail moves
    let r = f(unsafe { core::slice::from_raw_parts(s.slot_addr(tail) as *const u8, len) });
    s.tail.store(tail.wrapping_add(1), Ordering::Release);
    Some(r)
}

/// Wake `waker` when isochronous IN endpoint `ep` sends a packet (a slot frees up), to
/// refill it from a task. One waiter per endpoint, shared with the endpoint's `write`.
pub fn iso_in_register_waker(ep: usize, waker: &core::task::Waker) {
    if ep != 0 && ep < MAX_NR_EP {
        IN_STATE[ep].waker.register(waker);
    }
}

/// Packets queued on isochronous IN endpoint `ep` and not yet taken by the host
/// (including the one loaded in the endpoint).
pub fn iso_in_pending(ep: usize) -> u8 {
    IN_STATE[ep].count()
}

/// Queue one packet on isochronous IN endpoint `ep`; callable from interrupts. `false`:
/// not queued (ring full, endpoint not enabled or not isochronous, packet too long).
pub fn iso_in_prime(ep: usize, data: &[u8]) -> bool {
    let Some(d) = dregs() else { return false };
    if ep == 0 || ep >= MAX_NR_EP || !d.ep_config().read().t_en(ep - 1) {
        return false;
    }
    iso_in_enqueue(ep, data)
}

/// Build one packet on isochronous IN endpoint `ep` in place and queue it: `f` fills the
/// next free slot (one endpoint buffer; keep to the endpoint's max packet size) and
/// returns the packet's length. `false`: not queued and `f` not run (ring full, endpoint
/// not enabled or not isochronous). Callable from interrupts.
pub fn iso_in_fill(ep: usize, f: impl FnOnce(&mut [u8]) -> usize) -> bool {
    let Some(d) = dregs() else { return false };
    if ep == 0 || ep >= MAX_NR_EP || !d.ep_config().read().t_en(ep - 1) {
        return false;
    }
    iso_in_enqueue_with(ep, f)
}

/// Copy `data` into the next free slot and publish it (see [`iso_in_enqueue_with`]).
pub(crate) fn iso_in_enqueue(ep: usize, data: &[u8]) -> bool {
    if data.len() > IN_STATE[ep].stride.load(Ordering::Relaxed) as usize {
        return false;
    }
    iso_in_enqueue_with(ep, |slot| {
        slot[..data.len()].copy_from_slice(data);
        data.len()
    })
}

/// `f` fills the next free slot, then it's published. The slot at `head` is never touched
/// by the interrupt, so `f` runs without a critical section. One producer per endpoint.
///
/// The producer never loads the endpoint itself: the interrupt loads the next packet after
/// every IN, the zero-length one included. Loading while the endpoint answers an IN with
/// the zero-length packet (the ring ran empty) can't be told apart from the packet going
/// out, and that packet would be lost; this way it goes out one microframe later.
fn iso_in_enqueue_with(ep: usize, f: impl FnOnce(&mut [u8]) -> usize) -> bool {
    let s = &IN_STATE[ep];
    let n = s.slots.load(Ordering::Relaxed);
    if !s.iso.load(Ordering::Relaxed) || s.count() >= n {
        return false;
    }
    let head = s.head.load(Ordering::Relaxed);
    let stride = s.stride.load(Ordering::Relaxed) as usize;
    // SAFETY: the slot is inside the ring's buffers and not in [tail, head)
    let len = f(unsafe { core::slice::from_raw_parts_mut(s.slot_addr(head) as *mut u8, stride) }).min(stride);
    s.lens[(head % n) as usize].store(len as u16, Ordering::Relaxed);
    s.head.store(head.wrapping_add(1), Ordering::Release);
    true
}

/// Record an iso IN completion in microframe `frame` (interrupt only); `false` if one is
/// recorded in it already.
fn first_in_frame(s: &EpState, frame: u16) -> bool {
    let first = s.last_frame.load(Ordering::Relaxed) != frame;
    s.last_frame.store(frame, Ordering::Relaxed);
    first
}

/// Load slot `tail` into the endpoint, or a zero-length packet if the ring is empty.
/// Interrupt only (see `iso_in_enqueue_with`).
fn iso_in_load(d: crate::pac::usbhs::Usbd, ep: usize) {
    let s = &IN_STATE[ep];
    if s.count() > 0 {
        let tail = s.tail.load(Ordering::Relaxed);
        let n = s.slots.load(Ordering::Relaxed);
        d.ep_tx_dma(ep - 1).write_value(s.slot_addr(tail));
        d.ep_t_len(ep)
            .write(|w| w.set_len(s.lens[(tail % n) as usize].load(Ordering::Relaxed)));
        s.loaded.store(true, Ordering::Relaxed);
    } else {
        d.ep_t_len(ep).write(|w| w.set_len(0));
        s.loaded.store(false, Ordering::Relaxed);
    }
}

/// The setup packet, if one arrived since the last call.
pub(crate) fn take_setup() -> Option<[u8; 8]> {
    critical_section::with(|_| {
        if SETUP_READY.load(Ordering::Acquire) {
            SETUP_READY.store(false, Ordering::Release);
            let a = SETUP[0].load(Ordering::Relaxed).to_le_bytes();
            let b = SETUP[1].load(Ordering::Relaxed).to_le_bytes();
            Some([a[0], a[1], a[2], a[3], b[0], b[1], b[2], b[3]])
        } else {
            None
        }
    })
}

/// A setup packet arrived and ends whatever control transfer EP0 was in.
pub(crate) fn setup_pending() -> bool {
    SETUP_READY.load(Ordering::Acquire)
}

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let d = T::dregs();
        let flag = r.int_fg().read();

        if flag.bus_rst() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_bus_rst(false));
            BUS_WAKER.wake();
        };
        if flag.suspend() {
            //mask the interrupt and let the main thread handle it
            r.int_en().modify(|w| w.set_suspend(false));
            BUS_WAKER.wake();
        };
        if flag.fifo_ov() {
            r.int_fg().write(|w| w.set_fifo_ov(true));
        }
        if flag.hst_sof() {
            r.int_fg().write(|w| w.set_hst_sof(true));
        }

        if flag.transfer() {
            let st = r.int_st().read();
            let len = r.rx_len().read();
            let ep = st.endp() as usize;
            let token = st.token();

            let iso = match token {
                UsbToken::OUT => OUT_STATE[ep].iso.load(Ordering::Relaxed),
                UsbToken::IN => IN_STATE[ep].iso.load(Ordering::Relaxed),
                _ => true,
            };
            // bulk / interrupt / control: NAK before releasing the flag, so the next
            // transaction waits for the task
            if !iso {
                match token {
                    UsbToken::OUT if st.tog_ok() => {
                        d.ep_rx_ctrl(ep).modify(|w| w.set_mask_uep_r_res(EpRxResponse::NAK))
                    }
                    UsbToken::IN => d.ep_tx_ctrl(ep).modify(|w| w.set_mask_uep_t_res(EpTxResponse::NAK)),
                    _ => {}
                }
            }
            // release early: INT_ST / RX_LEN are read
            r.int_fg().write(|w| w.set_transfer(true));
            let frame = d.frame_no().read().frame_no();

            match token {
                UsbToken::SOF => {
                    let f = SOF_CALLBACK.load(Ordering::Acquire);
                    if f != 0 {
                        let f: fn() = core::mem::transmute(f);
                        f();
                    }
                }
                // a wrong toggle: a retransmit (lost ACK), dropped; the endpoint stays armed
                UsbToken::OUT if !st.tog_ok() => {}
                UsbToken::OUT if iso => {
                    let s = &OUT_STATE[ep];
                    let n = s.slots.load(Ordering::Relaxed);
                    let mut head = s.head.load(Ordering::Relaxed);
                    if head.wrapping_sub(s.tail.load(Ordering::Acquire)) < n - 1 {
                        s.lens[(head % n) as usize].store(len, Ordering::Relaxed);
                        head = head.wrapping_add(1);
                        s.head.store(head, Ordering::Release);
                    } else {
                        // full: the next packet overwrites this slot
                        s.overruns
                            .store(s.overruns.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
                    }
                    d.ep_rx_dma(ep - 1).write_value(s.slot_addr(head));
                    s.waker.wake();
                }
                UsbToken::OUT => {
                    let s = &OUT_STATE[ep];
                    if ep == 0 {
                        d.ep_rx_ctrl(0)
                            .modify(|w| w.set_mask_uep_r_tog(flip(w.mask_uep_r_tog())));
                    }
                    s.len.store(len, Ordering::Relaxed);
                    s.armed.store(false, Ordering::Relaxed);
                    s.ready.store(true, Ordering::Release);
                    s.waker.wake();
                }
                // INT_ST shows a token as soon as it arrives, not when its transaction
                // completes: an entry for the SOF that runs late reads the microframe's iso
                // IN token and looks like its completion, and the real one follows in the
                // same microframe. one iso IN per microframe (no high bandwidth), so a second
                // in the same one is that real completion, already counted: the packet loaded
                // at the first goes out next. (counting both marked that packet sent unsent)
                UsbToken::IN if iso && !first_in_frame(&IN_STATE[ep], frame) => {}
                UsbToken::IN if iso => {
                    let s = &IN_STATE[ep];
                    if s.loaded.load(Ordering::Relaxed) {
                        s.tail
                            .store(s.tail.load(Ordering::Relaxed).wrapping_add(1), Ordering::Release);
                    }
                    iso_in_load(d, ep);
                    s.waker.wake();
                }
                UsbToken::IN => {
                    let s = &IN_STATE[ep];
                    if ep == 0 {
                        d.ep_tx_ctrl(0)
                            .modify(|w| w.set_mask_uep_t_tog(flip(w.mask_uep_t_tog())));
                    }
                    s.ready.store(true, Ordering::Release);
                    s.waker.wake();
                }
                UsbToken::SETUP => {}
            }
        }

        if flag.setup_act() {
            // a new control transfer: whatever EP0 was doing is over
            d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
            d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
            let buf = EP0_BUF.load(Ordering::Relaxed) as *const u32;
            SETUP[0].store(core::ptr::read_volatile(buf), Ordering::Relaxed);
            SETUP[1].store(core::ptr::read_volatile(buf.add(1)), Ordering::Relaxed);
            OUT_STATE[0].ready.store(false, Ordering::Relaxed);
            IN_STATE[0].ready.store(false, Ordering::Relaxed);
            SETUP_READY.store(true, Ordering::Release);
            r.int_fg().write(|w| w.set_setup_act(true));
            OUT_STATE[0].waker.wake();
            IN_STATE[0].waker.wake();
        }
    }
}

fn flip(t: EpTog) -> EpTog {
    match t {
        EpTog::DATA0 => EpTog::DATA1,
        _ => EpTog::DATA0,
    }
}

pub struct WakeupInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::WakeupInterrupt> for WakeupInterruptHandler<T> {
    unsafe fn on_interrupt() {
        // not used by this driver (resume shows up through the SUSPEND flag)
        T::WakeupInterrupt::disable();
    }
}

pub struct Driver<'d, T: Instance, const NR_EP: usize, const SIZE: usize> {
    phantom: PhantomData<&'d T>,
    allocator: EndpointBufferAllocator<'d, NR_EP, SIZE>,
    allocated_in: Bitmap<MAX_NR_EP>,
    allocated_out: Bitmap<MAX_NR_EP>,
}

impl<'d, T: Instance, const NR_EP: usize, const SIZE: usize> Driver<'d, T, NR_EP, SIZE> {
    /// `ep_buffer`: one buffer for EP0 and each bulk / interrupt endpoint, plus the
    /// isochronous rings ([`ISO_OUT_SLOTS`] / [`ISO_IN_SLOTS`] buffers each).
    pub fn new(
        _peri: Peri<'d, T>,
        _irqs: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::WakeupInterrupt, WakeupInterruptHandler<T>>
            + 'd,
        dp: Peri<'d, impl DpPin<T, 0>>,
        dm: Peri<'d, impl DmPin<T, 0>>,
        ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ) -> Self {
        assert!(ep_buffer.len() > 0);

        dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
        dm.set_as_af_output(AFType::OutputPushPull, Speed::High);

        T::enable_and_reset();

        let allocator = EndpointBufferAllocator::new(ep_buffer);

        let r = T::regs();
        let h = T::hregs();

        r.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });

        // Sleep for 10uS from WCH C code
        embassy_time::block_for(embassy_time::Duration::from_micros(10));

        // Following WCH C code.... unclear why clr_all is not also cleared here
        r.ctrl().modify(|w| w.set_reset_sie(false));
        h.ctrl().write(|w| w.set_phy_suspendm(true));

        r.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            // TODO: HS, Configurable?
            w.set_speed_type(SpeedType::HIGHSPEED);
        });

        for s in OUT_STATE.iter().chain(IN_STATE.iter()) {
            s.reset();
            s.iso.store(false, Ordering::Relaxed);
        }
        SETUP_READY.store(false, Ordering::Relaxed);
        REGS.store(r.as_ptr() as usize, Ordering::Release);

        r.int_en().write(|w| {
            w.set_dev_nak(false);
            w.set_fifo_ov(true);
            w.set_setup_act(true);
            w.set_suspend(true);
            w.set_bus_rst(true);
            w.set_transfer(true);
            w.set_sof_act(SOF_CALLBACK.load(Ordering::Relaxed) != 0);
        });

        Self {
            phantom: PhantomData,
            allocator,
            allocated_in: Bitmap::new(),
            allocated_out: Bitmap::new(),
        }
    }

    fn alloc_endpoint<D: Dir>(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
        dir: Direction,
    ) -> Result<Endpoint<'d, T, D, SIZE>, EndpointAllocError> {
        let allocated = match dir {
            Direction::Out => &mut self.allocated_out,
            Direction::In => &mut self.allocated_in,
        };
        let addr = match ep_addr {
            Some(addr) => {
                // Use the provided endpoint address
                if addr.direction() != dir {
                    return Err(EndpointAllocError);
                }

                let ep_num = addr.index();
                if ep_num == 0 || ep_num >= MAX_NR_EP || allocated.get(ep_num) {
                    return Err(EndpointAllocError);
                }

                allocated.set(ep_num, true);
                addr
            }
            None => {
                // Find a free endpoint address; index 0 is the control endpoint
                // endpoint 0 is the control pipe's (claimed in start())
                let ep_num = allocated.next_false_index(1).ok_or(EndpointAllocError)?;
                allocated.set(ep_num, true);
                EndpointAddress::from_parts(ep_num, dir)
            }
        };

        let iso = ep_type == EndpointType::Isochronous;
        let slots = match (iso, dir) {
            (false, _) => 1,
            (true, Direction::Out) => ISO_OUT_SLOTS,
            (true, Direction::In) => ISO_IN_SLOTS,
        };
        // the ring's slots are consecutive buffers of the array
        let data = self.allocator.alloc_endpoint(max_packet_size)?;
        for _ in 1..slots {
            self.allocator.alloc_endpoint(max_packet_size)?;
        }

        let s = state(addr);
        s.iso.store(iso, Ordering::Relaxed);
        s.base.store(data.addr() as u32, Ordering::Relaxed);
        s.stride.store(SIZE as u16, Ordering::Relaxed);
        s.slots.store(slots as u8, Ordering::Relaxed);
        s.reset();

        Ok(Endpoint::new(
            EndpointInfo {
                addr,
                ep_type,
                max_packet_size,
                interval_ms,
            },
            data,
        ))
    }
}

pub(crate) fn state(addr: EndpointAddress) -> &'static EpState {
    match addr.direction() {
        Direction::Out => &OUT_STATE[addr.index()],
        Direction::In => &IN_STATE[addr.index()],
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
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::Out)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.alloc_endpoint(ep_type, ep_addr, max_packet_size, interval_ms, Direction::In)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let d = T::dregs();

        let ep0_buf = self.allocator.alloc_endpoint(control_max_packet_size).unwrap();
        d.ep0_dma().write_value(ep0_buf.addr() as u32);
        EP0_BUF.store(ep0_buf.addr() as u32, Ordering::Relaxed);
        d.ep_max_len(0).write(|w| w.set_len(control_max_packet_size));
        d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));

        let ep0 = Endpoint::new(
            EndpointInfo {
                // todo fix in embassy usb driver ep_addr should be u8 with top bit unset
                addr: EndpointAddress::from_parts(0, Direction::Out),
                ep_type: EndpointType::Control,
                max_packet_size: EP_MAX_PACKET_SIZE,
                interval_ms: 0,
            },
            ep0_buf,
        );

        critical_section::with(|_| {
            T::Interrupt::unpend();
            unsafe { T::Interrupt::enable() };
        });

        (
            Bus {
                _phantom: PhantomData,
                fake_power_on: false,
            },
            ControlPipe::new(ep0),
        )
    }
}

pub struct Bus<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    fake_power_on: bool,
}

impl<'d, T: Instance> Bus<'d, T> {
    fn bus_reset(&mut self) {
        let regs = T::regs();
        let d = T::dregs();

        critical_section::with(|_| {
            // Reset device address
            regs.dev_ad().write(|v| {
                v.set_addr(0);
            });

            // Reset endpoint state on bus reset
            // EP0 get ACK/NAK so it can recieve setup
            d.ep_rx_ctrl(0).write(|v| v.set_mask_uep_r_res(EpRxResponse::ACK));
            d.ep_tx_ctrl(0).write(|v| v.set_mask_uep_t_res(EpTxResponse::NAK));

            // Mark all other EPs as NAK
            for i in 1..=15 {
                d.ep_tx_ctrl(i).write(|v| v.set_mask_uep_t_res(EpTxResponse::NAK));
                d.ep_rx_ctrl(i).write(|v| v.set_mask_uep_r_res(EpRxResponse::NAK));
            }

            // Disable all endpoints [1, 15]
            d.ep_config().write_value(EpConfig::default());
            d.ep_type().write_value(EpType::default());
            d.ep_buf_mod().write_value(EpBufMod::default());

            for s in OUT_STATE.iter().chain(IN_STATE.iter()) {
                s.reset();
            }
            SETUP_READY.store(false, Ordering::Relaxed);
        });
        for s in OUT_STATE.iter().chain(IN_STATE.iter()) {
            s.waker.wake();
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::Bus for Bus<'d, T> {
    async fn enable(&mut self) {
        critical_section::with(|_| T::regs().ctrl().modify(|v| v.set_dev_pu_en(true)));
    }

    async fn disable(&mut self) {}

    async fn poll(&mut self) -> Event {
        // TODO: VBUS detection
        if !self.fake_power_on {
            self.fake_power_on = true;
            return Event::PowerDetected;
        }
        poll_fn(|cx| {
            BUS_WAKER.register(cx.waker());

            let r = T::regs();
            let flag = r.int_fg().read();

            if flag.bus_rst() {
                self.bus_reset();
                r.int_fg().write(|w| w.set_bus_rst(true));
                critical_section::with(|_| {
                    r.int_en().modify(|w| w.set_bus_rst(true));
                });
                Poll::Ready(Event::Reset)
            } else if flag.suspend() {
                let mis_st = r.mis_st().read();
                r.int_fg().write(|w| w.set_suspend(true));
                critical_section::with(|_| {
                    r.int_en().modify(|w| w.set_suspend(true));
                });
                if mis_st.suspend() {
                    Poll::Ready(Event::Suspend)
                } else {
                    Poll::Ready(Event::Resume)
                }
            } else {
                Poll::Pending
            }
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let index = ep_addr.index();
        if index == 0 {
            return;
        }
        let d = T::dregs();
        let s = state(ep_addr);
        let iso = s.iso.load(Ordering::Relaxed);
        let kind = if iso && enabled { EpKind::ISO } else { EpKind::NISO };

        critical_section::with(|_| {
            s.reset();
            if enabled && !iso && ep_addr.direction() == Direction::Out {
                s.armed.store(true, Ordering::Relaxed);
            }
            match ep_addr.direction() {
                Direction::Out => {
                    d.ep_rx_dma(index - 1).write_value(s.slot_addr(0));
                    d.ep_rx_ctrl(index).write(|v| {
                        v.set_mask_uep_r_tog(EpTog::DATA0);
                        // armed as soon as it's enabled: a host may send the first packet
                        // right after SET_CONFIGURATION. the interrupt NAKs after each packet
                        // until the next read()
                        v.set_mask_uep_r_res(match (enabled, iso) {
                            (false, _) => EpRxResponse::NAK,
                            (true, true) => EpRxResponse::NYET, // isochronous: no handshake
                            (true, false) => EpRxResponse::ACK,
                        });
                        v.set_r_tog_auto(!iso);
                    });
                    d.ep_type().modify(|v| v.set_r_type(index - 1, kind));
                    d.ep_config().modify(|v| v.set_r_en(index - 1, enabled));
                }
                Direction::In => {
                    d.ep_tx_dma(index - 1).write_value(s.slot_addr(0));
                    d.ep_t_len(index).write(|v| v.set_len(0));
                    d.ep_tx_ctrl(index).write(|v| {
                        v.set_mask_uep_t_tog(EpTog::DATA0);
                        // isochronous: answers every IN, with a zero-length packet when
                        // nothing is queued
                        v.set_mask_uep_t_res(if iso && enabled { TX_RES_ISO } else { EpTxResponse::NAK });
                        v.set_t_tog_auto(!iso);
                    });
                    d.ep_type().modify(|v| v.set_t_type(index - 1, kind));
                    d.ep_config().modify(|v| v.set_t_en(index - 1, enabled));
                }
            }
        });
        s.waker.wake();
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let index = ep_addr.index();
        let d = T::dregs();
        let s = state(ep_addr);
        let iso = s.iso.load(Ordering::Relaxed);
        critical_section::with(|_| match (ep_addr.direction(), stalled) {
            (Direction::Out, true) => d
                .ep_rx_ctrl(index)
                .modify(|v| v.set_mask_uep_r_res(EpRxResponse::STALL)),
            (Direction::In, true) => {
                d.ep_t_len(index).write(|v| v.set_len(0));
                d.ep_tx_ctrl(index)
                    .modify(|v| v.set_mask_uep_t_res(EpTxResponse::STALL));
            }
            (Direction::Out, false) => d.ep_rx_ctrl(index).write(|v| {
                // a read still waiting stays armed (or it would NAK forever)
                v.set_mask_uep_r_res(if iso {
                    EpRxResponse::NYET
                } else if s.armed.load(Ordering::Relaxed) {
                    EpRxResponse::ACK
                } else {
                    EpRxResponse::NAK
                });
                v.set_mask_uep_r_tog(EpTog::DATA0);
                v.set_r_tog_auto(index != 0 && !iso);
            }),
            (Direction::In, false) => d.ep_tx_ctrl(index).write(|v| {
                v.set_mask_uep_t_res(if iso { TX_RES_ISO } else { EpTxResponse::NAK });
                v.set_mask_uep_t_tog(EpTog::DATA0);
                v.set_t_tog_auto(index != 0 && !iso);
            }),
        });
        if !stalled && ep_addr.direction() == Direction::In && !iso {
            // a write waiting on the stalled endpoint finishes (its data is dropped)
            s.ready.store(true, Ordering::Release);
        }
        s.waker.wake();
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let index = ep_addr.index();
        let d = T::dregs();
        match ep_addr.direction() {
            Direction::Out => d.ep_rx_ctrl(index).read().mask_uep_r_res() == EpRxResponse::STALL,
            Direction::In => d.ep_tx_ctrl(index).read().mask_uep_t_res() == EpTxResponse::STALL,
        }
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

/// USB endpoint.
trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::usbhs::Usb;
    fn dregs() -> crate::pac::usbhs::Usbd;
    fn hregs() -> crate::pac::usbhs::Usbh;
}

/// UsbHs peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    /// Regular interrupt for this instance
    type Interrupt: interrupt::typelevel::Interrupt;
    /// Wakeup interrupt for this instance
    type WakeupInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usbhs, $inst:ident) => {
        use crate::peripherals;
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usbhs::Usb {
                crate::pac::$inst
            }

            fn dregs() -> crate::pac::usbhs::Usbd {
                unsafe {
                    crate::pac::usbhs::Usbd::from_ptr(crate::pac::$inst.as_ptr())
                }
            }

            fn hregs() -> crate::pac::usbhs::Usbh {
                unsafe {
                    crate::pac::usbhs::Usbh::from_ptr(crate::pac::$inst.as_ptr())
                }
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type WakeupInterrupt = crate::_generated::peripheral_interrupts::$inst::WAKEUP;
        }
    };
);

pin_trait!(DmPin, Instance);
pin_trait!(DpPin, Instance);
