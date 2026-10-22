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
// the ring indices are u8s that wrap and are masked: slot counts are powers of two up to 256
const _: () = assert!(256 % ISO_OUT_SLOTS == 0 && 256 % ISO_IN_SLOTS == 0);

/// The pattern an armed isochronous OUT slot is filled with (see `iso_out_check`)
const ISO_OUT_MARK: u32 = 0x5A3C_A5C3;
/// Isochronous OUT endpoints, a bit each
static ISO_OUT_EPS: AtomicU16 = AtomicU16::new(0);
/// Isochronous IN endpoints polled every (micro)frame, a bit each
static ISO_IN_EVERY: AtomicU16 = AtomicU16::new(0);

/// The endpoints of `mask` (bit n: endpoint n) whose IN direction is enabled.
fn in_enabled(d: crate::pac::usbhs::Usbd, mask: u16) -> u16 {
    if mask == 0 {
        return 0;
    }
    // EP_CONFIG bit n: endpoint n's IN enable
    mask & d.ep_config().read().0 as u16
}

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
    /// bulk / interrupt: OUT -- a read is waiting (the endpoint answers ACK); IN -- a packet
    /// is loaded
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
    /// isochronous OUT: the packet size the slots are marked over
    pub(crate) max_len: AtomicU16,
    /// isochronous OUT: the shortest packet since enabled (marks below it get overwritten)
    pub(crate) min_len: AtomicU16,
    /// isochronous OUT: the armed slot's address
    pub(crate) armed_slot: AtomicU32,
    /// bulk / interrupt: R_TOG / T_TOG after the last packet taken / sent (DATA1: true)
    pub(crate) tog: AtomicBool,
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
        max_len: AtomicU16::new(0),
        min_len: AtomicU16::new(0),
        armed_slot: AtomicU32::new(0),
        tog: AtomicBool::new(false),
    };

    fn reset(&self) {
        self.last_frame.store(NO_FRAME, Ordering::Relaxed);
        self.ready.store(false, Ordering::Relaxed);
        self.armed.store(false, Ordering::Relaxed);
        self.head.store(0, Ordering::Relaxed);
        self.tail.store(0, Ordering::Relaxed);
        self.loaded.store(false, Ordering::Relaxed);
        self.tog.store(false, Ordering::Relaxed);
    }

    pub(crate) fn count(&self) -> u8 {
        self.head
            .load(Ordering::Acquire)
            .wrapping_sub(self.tail.load(Ordering::Acquire))
    }

    pub(crate) fn slot_addr(&self, i: u8) -> u32 {
        let n = self.slots.load(Ordering::Relaxed);
        self.base.load(Ordering::Relaxed) + (i & (n - 1)) as u32 * self.stride.load(Ordering::Relaxed) as u32
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
/// The frame number of the last SOF handled
static SOF_FRAME: AtomicU16 = AtomicU16::new(NO_FRAME);
/// Polls of MIS_ST to wait for an iso OUT to finish: longer than a 1024-byte packet
const SIE_WAIT: u32 = 2000;

/// The frame number register: frame (bits 10:0) and microframe (bits 15:13); bits 12:11
/// are always 0, so this is never a frame number.
const NO_FRAME: u16 = 0x1800;
const MICROFRAME_MASK: u16 = (1 << 14) - 1;

/// A frame number as a microframe count
fn microframe_index(frame: u16) -> u16 {
    (frame & 0x7FF) << 3 | frame >> 13
}

fn frame_of_index(i: u16) -> u16 {
    (i >> 3) & 0x7FF | (i & 7) << 13
}

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

/// The current microframe number: 8 per frame, wrapping at 2^14. Read from the frame
/// number register, so it also counts microframes whose SOF interrupt was missed.
pub fn microframe() -> u16 {
    dregs().map_or(0, |d| microframe_index(d.frame_no().read().frame_no()))
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
    let len = s.lens[(tail & (n - 1)) as usize].load(Ordering::Relaxed) as usize;
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
    if ep == 0 || ep >= MAX_NR_EP {
        return 0;
    }
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
    s.lens[(head & (n - 1)) as usize].store(len as u16, Ordering::Relaxed);
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

/// An iso IN completion: the loaded packet went out, load the next. Interrupt only.
fn iso_in_complete(d: crate::pac::usbhs::Usbd, ep: usize) {
    let s = &IN_STATE[ep];
    if s.loaded.load(Ordering::Relaxed) {
        s.tail
            .store(s.tail.load(Ordering::Relaxed).wrapping_add(1), Ordering::Release);
    }
    iso_in_load(d, ep);
    s.waker.wake();
}

/// Bulk / interrupt / control: NAK before releasing the flag, so the next transaction
/// waits for the task.
fn nak_non_iso(d: crate::pac::usbhs::Usbd, st: ch32_metapac::usbhs::regs::IntSt) {
    let ep = st.endp() as usize;
    match st.token() {
        // bulk / interrupt: before knowing whether a packet went through (`toggled`), so the
        // next can't land on it
        UsbToken::OUT if (ep == 0 && st.tog_ok()) || (ep != 0 && !OUT_STATE[ep].iso.load(Ordering::Relaxed)) => {
            d.ep_rx_ctrl(ep).modify(|w| w.set_mask_uep_r_res(EpRxResponse::NAK))
        }
        UsbToken::IN if !IN_STATE[ep].iso.load(Ordering::Relaxed) => {
            d.ep_tx_ctrl(ep).modify(|w| w.set_mask_uep_t_res(EpTxResponse::NAK))
        }
        _ => {}
    }
}

/// A packet went through a bulk / interrupt endpoint (not EP0): the SIE flips its toggle on
/// each one it accepts (OUT) or the host ACKs (IN). INT_ST's TOG_OK can't tell: the flag comes
/// a little before the SIE goes free, and an entry that early reads it before it's final.
/// Waits for the SIE. The entry NAKed the endpoint; a read / write still waiting gets it back.
fn toggled(r: crate::pac::usbhs::Usb, d: crate::pac::usbhs::Usbd, ep: usize, dir: Direction) -> bool {
    let mut n = 0;
    while !r.mis_st().read().sie_free() && n < SIE_WAIT {
        n += 1;
    }
    let (s, tog) = match dir {
        Direction::Out => (&OUT_STATE[ep], d.ep_rx_ctrl(ep).read().mask_uep_r_tog()),
        Direction::In => (&IN_STATE[ep], d.ep_tx_ctrl(ep).read().mask_uep_t_tog()),
    };
    let tog = tog == EpTog::DATA1;
    if s.tog.load(Ordering::Relaxed) == tog {
        if s.armed.load(Ordering::Relaxed) {
            match dir {
                Direction::Out => d.ep_rx_ctrl(ep).modify(|w| w.set_mask_uep_r_res(EpRxResponse::ACK)),
                Direction::In => d.ep_tx_ctrl(ep).modify(|w| w.set_mask_uep_t_res(EpTxResponse::ACK)),
            }
        }
        return false;
    }
    s.tog.store(tog, Ordering::Relaxed);
    s.armed.store(false, Ordering::Relaxed);
    true
}

/// Isochronous OUT arrivals, by the slot's first word: the endpoint's slot gets `ISO_OUT_MARK`
/// when it's armed, so a packet in it shows whatever INT_ST shows -- with the flag one bit, a
/// late entry may stand for an OUT and show the transaction after it. (a packet that starts
/// with the mark is lost; zero-length ones aren't seen.) Interrupt only.
fn iso_out_check(r: crate::pac::usbhs::Usb, d: crate::pac::usbhs::Usbd, entry: Option<(usize, u16)>) {
    let mut eps = ISO_OUT_EPS.load(Ordering::Relaxed);
    while eps != 0 {
        let ep = eps.trailing_zeros() as usize;
        eps &= eps - 1;
        let s = &OUT_STATE[ep];
        let slot = s.armed_slot.load(Ordering::Relaxed);
        // SAFETY: the armed slot, a buffer of this endpoint
        if unsafe { core::ptr::read_volatile(slot as *const u32) } == ISO_OUT_MARK {
            continue;
        }
        let max = s.max_len.load(Ordering::Relaxed) as usize;
        let len = match entry {
            // RX_LEN as this packet's entry read it (releasing the flag clears it)
            Some((e, len)) if e == ep => len,
            _ => {
                // read after the slot: while its OUT is the last token, the packet may still be
                // arriving (or its DMA finishing), and its own entry follows
                let st = r.int_st().read();
                if st.token() == UsbToken::OUT && st.endp() as usize == ep {
                    continue;
                }
                marked_len(slot, max) as u16
            }
        };
        if len != 0 && len < s.min_len.load(Ordering::Relaxed) {
            s.min_len.store(len, Ordering::Relaxed);
        }
        let n = s.slots.load(Ordering::Relaxed);
        let mut head = s.head.load(Ordering::Relaxed);
        // what the slot to arm holds (marks past it are still there)
        let mut dirty = len as usize;
        if head.wrapping_sub(s.tail.load(Ordering::Acquire)) < n - 1 {
            s.lens[(head & (n - 1)) as usize].store(len, Ordering::Relaxed);
            head = head.wrapping_add(1);
            s.head.store(head, Ordering::Release);
            dirty = s.lens[(head & (n - 1)) as usize].load(Ordering::Relaxed) as usize;
        } else {
            // full: the next packet overwrites this slot
            s.overruns
                .store(s.overruns.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
        }
        let from = s.min_len.load(Ordering::Relaxed) as usize;
        iso_out_arm(d, ep, s.slot_addr(head), from, dirty.min(max));
        s.waker.wake();
    }
}

/// The length of a packet in a slot filled with `ISO_OUT_MARK` before it: up to its last
/// byte that differs from the pattern.
fn marked_len(slot: u32, max: usize) -> usize {
    let mark = ISO_OUT_MARK.to_le_bytes();
    let words = slot as *const u32;
    // SAFETY: the slot holds `max` bytes, rounded up to words when marked
    let word = |i: usize| unsafe { core::ptr::read_volatile(words.add(i)) };
    // the first word of the marks after the packet (a packet word that matches the mark can
    // throw this off: 2^-32), then the last byte before them that differs
    let (mut lo, mut hi) = (0, max.div_ceil(4));
    while lo < hi {
        let mid = (lo + hi) / 2;
        if word(mid) == ISO_OUT_MARK {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    let w = lo;
    if w == 0 {
        return 0;
    }
    let last = word(w - 1).to_le_bytes();
    let mut b = 4;
    while last[b - 1] == mark[b - 1] {
        b -= 1;
    }
    ((w - 1) * 4 + b).min(max)
}

/// Point isochronous OUT endpoint `ep` at the slot at `addr`, marked empty: its first word
/// and its bytes `from..dirty` are marked again -- past `dirty` they still are, and below
/// `from` (no packet is that short) the next packet overwrites them.
fn iso_out_arm(d: crate::pac::usbhs::Usbd, ep: usize, addr: u32, from: usize, dirty: usize) {
    let words = addr as *mut u32;
    // SAFETY: a slot of this endpoint, not the reader's, word aligned
    unsafe { core::ptr::write_volatile(words, ISO_OUT_MARK) };
    for i in (from / 4).max(1)..dirty.div_ceil(4) {
        unsafe { core::ptr::write_volatile(words.add(i), ISO_OUT_MARK) };
    }
    OUT_STATE[ep].armed_slot.store(addr, Ordering::Relaxed);
    d.ep_rx_dma(ep - 1).write_value(addr);
}

/// A new microframe's SOF (once each, possibly from a later entry). An iso IN endpoint
/// polled every microframe completes in each: one that didn't in the last went out unseen,
/// its completion merged into another entry's flag (the host repeats a packet otherwise).
/// Not while an IN is on the bus: that may be its packet going out again late.
fn on_sof(r: crate::pac::usbhs::Usb, d: crate::pac::usbhs::Usbd, frame: u16) {
    SOF_FRAME.store(frame, Ordering::Relaxed);
    let prev = microframe_index(frame).wrapping_sub(1) & MICROFRAME_MASK;
    let mut eps = in_enabled(d, ISO_IN_EVERY.load(Ordering::Relaxed));
    let mut checked = false;
    while eps != 0 {
        let ep = eps.trailing_zeros() as usize;
        eps &= eps - 1;
        let s = &IN_STATE[ep];
        let last = s.last_frame.load(Ordering::Relaxed);
        if last != NO_FRAME && microframe_index(last) == prev {
            continue;
        }
        if !checked {
            if !r.mis_st().read().sie_free() && r.int_st().read().token() == UsbToken::IN {
                break;
            }
            checked = true;
        }
        s.last_frame.store(frame_of_index(prev), Ordering::Relaxed);
        iso_in_complete(d, ep);
    }
    let f = SOF_CALLBACK.load(Ordering::Acquire);
    if f != 0 {
        // SAFETY: stored from a `fn()` in `set_sof_callback`
        let f: fn() = unsafe { core::mem::transmute(f) };
        f();
    }
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
            .write(|w| w.set_len(s.lens[(tail & (n - 1)) as usize].load(Ordering::Relaxed)));
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
            // INT_ST shows a token as soon as it arrives, not when its transaction completes:
            // an entry that runs late (the next transaction is already on the bus) reads a
            // token that hasn't completed
            let mut busy = !r.mis_st().read().sie_free();
            let mut st = r.int_st().read();
            let mut len = r.rx_len().read();
            if !busy || (st.token() != UsbToken::SOF && st.endp() != 0) {
                nak_non_iso(d, st);
            }
            // release early: INT_ST / RX_LEN are read
            r.int_fg().write(|w| w.set_transfer(true));
            // an iso OUT on the bus: this entry is its own (raised a little before the SIE goes
            // free) or a late one. take it here once it's in -- left to a later entry, the next
            // microframe's OUT may land on it first. its completion after the release sets
            // RX_LEN (and the flag again: that entry finds it taken)
            let mut out_late = false;
            let mut rx_len_ok = true;
            if busy && st.token() == UsbToken::OUT && OUT_STATE[st.endp() as usize].iso.load(Ordering::Relaxed) {
                let mut n = 0;
                while !r.mis_st().read().sie_free() && n < SIE_WAIT {
                    n += 1;
                }
                let now = r.int_st().read();
                len = r.rx_len().read();
                // 0: it completed before the release, which cleared RX_LEN; another token:
                // the next transaction already. the slot shows the length
                if len == 0 || now.token() != UsbToken::OUT || now.endp() != st.endp() {
                    rx_len_ok = false;
                }
                busy = false;
                out_late = true;
            } else if busy
                && st.endp() != 0
                && match st.token() {
                    UsbToken::OUT => true, // iso: above
                    UsbToken::IN => !IN_STATE[st.endp() as usize].iso.load(Ordering::Relaxed),
                    _ => false,
                }
            {
                // bulk / interrupt raise their flag before the SIE goes free too: this may be the
                // shown transaction's own entry (`toggled` tells). an OUT completing after the
                // release sets RX_LEN again
                let mut n = 0;
                while !r.mis_st().read().sie_free() && n < SIE_WAIT {
                    n += 1;
                }
                busy = false;
                let now = r.rx_len().read();
                if now != 0 {
                    len = now;
                }
            } else if busy && r.mis_st().read().sie_free() && !r.int_fg().read().transfer() {
                // it completed before the release, which took its flag (and RX_LEN): handle it
                // here
                busy = false;
                st = r.int_st().read();
                rx_len_ok = false;
                nak_non_iso(d, st);
            }
            let ep = st.endp() as usize;
            let token = st.token();
            let iso = match token {
                UsbToken::OUT => OUT_STATE[ep].iso.load(Ordering::Relaxed),
                UsbToken::IN => IN_STATE[ep].iso.load(Ordering::Relaxed),
                _ => true,
            };
            let frame = d.frame_no().read().frame_no();
            // an iso IN raises its flag a little before the SIE goes free (its only entry):
            // with this microframe's SOF handled, the entry is that IN's own
            if busy && iso && token == UsbToken::IN && SOF_FRAME.load(Ordering::Relaxed) == frame {
                busy = false;
            }
            // the flag is one bit: a late entry also stands for what completed before the
            // token it shows. the SOF if this microframe's wasn't handled
            if token != UsbToken::SOF && SOF_FRAME.load(Ordering::Relaxed) != frame {
                on_sof(r, d, frame);
            }

            if out_late {
                // what raised a late entry completed before the OUT: the iso INs polled every
                // microframe that haven't completed in it (hosts poll them first; the SOF
                // catches up the ones polled after)
                let mut eps = in_enabled(d, ISO_IN_EVERY.load(Ordering::Relaxed));
                while eps != 0 {
                    let i = eps.trailing_zeros() as usize;
                    eps &= eps - 1;
                    if first_in_frame(&IN_STATE[i], frame) {
                        iso_in_complete(d, i);
                    }
                }
            }
            if !busy {
                match token {
                    UsbToken::SOF => {
                        if SOF_FRAME.load(Ordering::Relaxed) != frame {
                            on_sof(r, d, frame);
                        }
                    }
                    // iso OUT: see iso_out_check
                    UsbToken::OUT if iso => {}
                    // no new packet (a retransmit after a lost ACK, or one NAKed): the endpoint
                    // stays as it is
                    UsbToken::OUT if ep != 0 && !toggled(r, d, ep, Direction::Out) => {}
                    UsbToken::OUT if ep == 0 && !st.tog_ok() => {}
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
                    // one iso IN per microframe (no high bandwidth): a second completion in
                    // one is a stale INT_ST, already counted
                    UsbToken::IN if iso && !first_in_frame(&IN_STATE[ep], frame) => {}
                    UsbToken::IN if iso => iso_in_complete(d, ep),
                    UsbToken::IN if ep != 0 && !toggled(r, d, ep, Direction::In) => {}
                    UsbToken::IN => {
                        let s = &IN_STATE[ep];
                        if ep == 0 {
                            d.ep_tx_ctrl(0)
                                .modify(|w| w.set_mask_uep_t_tog(flip(w.mask_uep_t_tog())));
                            s.armed.store(false, Ordering::Relaxed);
                        }
                        s.ready.store(true, Ordering::Release);
                        s.waker.wake();
                    }
                    UsbToken::SETUP => {}
                }
            }
            let entry = (!busy && rx_len_ok && token == UsbToken::OUT && iso && st.tog_ok()).then_some((ep, len));
            // an OUT completes too early in its microframe to merge into the next SOF's entry,
            // and a busy entry takes its own OUT above
            if token != UsbToken::SOF {
                iso_out_check(r, d, entry);
            }
            if ep == 0 {
                ep0_moved_on(d, token);
            }
        }

        if flag.setup_act() {
            // a new control transfer: whatever EP0 was doing is over
            d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
            d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
            let buf = EP0_BUF.load(Ordering::Relaxed) as *const u32;
            SETUP[0].store(core::ptr::read_volatile(buf), Ordering::Relaxed);
            SETUP[1].store(core::ptr::read_volatile(buf.add(1)), Ordering::Relaxed);
            for s in [&OUT_STATE[0], &IN_STATE[0]] {
                s.ready.store(false, Ordering::Relaxed);
                s.armed.store(false, Ordering::Relaxed);
            }
            SETUP_READY.store(true, Ordering::Release);
            r.int_fg().write(|w| w.set_setup_act(true));
            OUT_STATE[0].waker.wake();
            IN_STATE[0].waker.wake();
        }
    }
}

/// An EP0 stage whose completion no entry saw: the flag is one bit and INT_ST shows the
/// latest token, NAKed ones too, so a host polling a bulk IN right after an EP0 transaction
/// hides it (EP0 has no auto toggle to tell). The host's token for the other direction says
/// it went through: the status stage after the last data packet. (one hidden before the
/// last gets resent with the same toggle, which the host drops, and completes then.)
fn ep0_moved_on(d: crate::pac::usbhs::Usbd, token: UsbToken) {
    match token {
        UsbToken::OUT if IN_STATE[0].armed.load(Ordering::Relaxed) => {
            d.ep_tx_ctrl(0).modify(|w| {
                w.set_mask_uep_t_res(EpTxResponse::NAK);
                w.set_mask_uep_t_tog(flip(w.mask_uep_t_tog()));
            });
            let s = &IN_STATE[0];
            s.armed.store(false, Ordering::Relaxed);
            s.ready.store(true, Ordering::Release);
            s.waker.wake();
        }
        UsbToken::IN if OUT_STATE[0].armed.load(Ordering::Relaxed) => {
            d.ep_rx_ctrl(0).modify(|w| {
                w.set_mask_uep_r_res(EpRxResponse::NAK);
                w.set_mask_uep_r_tog(flip(w.mask_uep_r_tog()));
            });
            // `len`: the packet the stage expects (`ControlPipe` sets it)
            let s = &OUT_STATE[0];
            s.armed.store(false, Ordering::Relaxed);
            s.ready.store(true, Ordering::Release);
            s.waker.wake();
        }
        _ => {}
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
    /// endpoint numbers with an isochronous endpoint in either direction
    iso_nums: Bitmap<MAX_NR_EP>,
    /// endpoint numbers with a bulk / interrupt endpoint in either direction
    other_nums: Bitmap<MAX_NR_EP>,
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
            iso_nums: Bitmap::new(),
            other_nums: Bitmap::new(),
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
        let iso = ep_type == EndpointType::Isochronous;
        // the other direction's type: a PING to a bulk OUT whose number is iso on the IN side
        // hangs the SIE (no endpoint answers after it)
        let taken = if iso { self.other_nums } else { self.iso_nums };
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
                // Find a free endpoint address; endpoint 0 is the control pipe's
                let ep_num = (1..MAX_NR_EP)
                    .find(|&i| !allocated.get(i) && !taken.get(i))
                    .ok_or(EndpointAllocError)?;
                allocated.set(ep_num, true);
                EndpointAddress::from_parts(ep_num, dir)
            }
        };

        if iso {
            self.iso_nums.set(addr.index(), true);
        } else {
            self.other_nums.set(addr.index(), true);
        }
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
        if iso && dir == Direction::In && interval_ms == 0 {
            ISO_IN_EVERY.store(
                ISO_IN_EVERY.load(Ordering::Relaxed) | 1 << addr.index(),
                Ordering::Relaxed,
            );
        }
        s.max_len.store(max_packet_size, Ordering::Relaxed);
        if iso && dir == Direction::Out {
            ISO_OUT_EPS.store(
                ISO_OUT_EPS.load(Ordering::Relaxed) | 1 << addr.index(),
                Ordering::Relaxed,
            );
        }
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
                    if iso {
                        // every slot marked, so arming one marks what its last packet covered
                        let max = s.max_len.load(Ordering::Relaxed) as usize;
                        for k in 0..s.slots.load(Ordering::Relaxed) {
                            s.lens[k as usize].store(max as u16, Ordering::Relaxed);
                        }
                        s.min_len.store(max as u16, Ordering::Relaxed);
                        for k in (0..s.slots.load(Ordering::Relaxed)).rev() {
                            iso_out_arm(d, index, s.slot_addr(k), 0, max);
                        }
                    } else {
                        d.ep_rx_dma(index - 1).write_value(s.slot_addr(0));
                    }
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
            (Direction::Out, false) => {
                s.tog.store(false, Ordering::Relaxed);
                d.ep_rx_ctrl(index).write(|v| {
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
                })
            }
            (Direction::In, false) => {
                s.tog.store(false, Ordering::Relaxed);
                d.ep_tx_ctrl(index).write(|v| {
                    v.set_mask_uep_t_res(if iso { TX_RES_ISO } else { EpTxResponse::NAK });
                    v.set_mask_uep_t_tog(EpTog::DATA0);
                    v.set_t_tog_auto(index != 0 && !iso);
                })
            }
        });
        if !stalled && ep_addr.direction() == Direction::In && !iso {
            // a write waiting on the stalled endpoint finishes (its data is dropped)
            s.armed.store(false, Ordering::Relaxed);
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
