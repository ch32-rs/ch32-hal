use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use super::Instance;

// ============================================================================
// DMA descriptors
// ============================================================================

pub(super) const MTU: usize = 1514;
const TX_BUF_SIZE: usize = 1514;
const RX_BUF_SIZE: usize = 1536;

// TX descriptor word 0 (TDES0) bits
const TDES0_OWN: u32 = 1 << 31;
const TDES0_IC: u32 = 1 << 30; // Interrupt on completion
const TDES0_LS: u32 = 1 << 29; // Last segment
const TDES0_FS: u32 = 1 << 28; // First segment
const TDES0_TCH: u32 = 1 << 20; // Second address chained

// RX descriptor word 0 (RDES0) bits
const RDES0_OWN: u32 = 1 << 31;
const RDES0_FS: u32 = 1 << 9; // First descriptor
const RDES0_LS: u32 = 1 << 8; // Last descriptor
const RDES0_ES: u32 = 1 << 15; // Error summary
const RDES0_FL_MASK: u32 = 0x3FFF0000; // Frame length mask
const RDES0_FL_SHIFT: u32 = 16;

// RX descriptor word 1 (RDES1) bits
const RDES1_RCH: u32 = 1 << 14; // Second address chained

/// A single TX DMA descriptor (4 x 32-bit words).
#[repr(C, align(4))]
pub struct TDes {
    tdes0: u32,
    tdes1: u32,
    tdes2: u32,
    tdes3: u32,
}

impl TDes {
    pub const fn new() -> Self {
        Self {
            tdes0: 0,
            tdes1: 0,
            tdes2: 0,
            tdes3: 0,
        }
    }

    #[inline]
    fn tdes0(&self) -> u32 {
        unsafe { core::ptr::read_volatile(&self.tdes0) }
    }

    #[inline]
    fn set_tdes0(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes0, val) }
    }

    #[inline]
    fn set_tdes1(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes1, val) }
    }

    #[inline]
    fn set_tdes2(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes2, val) }
    }

    #[inline]
    fn set_tdes3(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.tdes3, val) }
    }
}

/// A single RX DMA descriptor (4 x 32-bit words).
#[repr(C, align(4))]
pub struct RDes {
    rdes0: u32,
    rdes1: u32,
    rdes2: u32,
    rdes3: u32,
}

impl RDes {
    pub const fn new() -> Self {
        Self {
            rdes0: 0,
            rdes1: 0,
            rdes2: 0,
            rdes3: 0,
        }
    }

    #[inline]
    fn rdes0(&self) -> u32 {
        unsafe { core::ptr::read_volatile(&self.rdes0) }
    }

    #[inline]
    fn set_rdes0(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes0, val) }
    }

    #[inline]
    fn set_rdes1(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes1, val) }
    }

    #[inline]
    fn set_rdes2(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes2, val) }
    }

    #[inline]
    fn set_rdes3(&mut self, val: u32) {
        unsafe { core::ptr::write_volatile(&mut self.rdes3, val) }
    }
}

// ============================================================================
// DMA descriptor rings
// ============================================================================

/// A TX DMA descriptor ring.
pub struct TDesRing<'d, T: Instance, const N: usize> {
    desc: &'d mut [TDes; N],
    buf: &'d mut [[u8; TX_BUF_SIZE]; N],
    index: usize,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance, const N: usize> TDesRing<'d, T, N> {
    pub(super) fn new(desc: &'d mut [TDes; N], buf: &'d mut [[u8; TX_BUF_SIZE]; N]) -> Self {
        // Set up chained descriptors
        for i in 0..N {
            let next = if i + 1 < N {
                &desc[i + 1] as *const TDes as u32
            } else {
                &desc[0] as *const TDes as u32
            };

            desc[i].set_tdes0(TDES0_TCH | TDES0_IC | TDES0_FS | TDES0_LS);
            desc[i].set_tdes1(0);
            desc[i].set_tdes2(buf[i].as_ptr() as u32);
            desc[i].set_tdes3(next);
        }
        // Last descriptor chains back to first (already set in loop above).

        // Tell DMA where the TX descriptor ring starts
        T::regs()
            .dmatdlar()
            .write(|w| w.set_stl(&desc[0] as *const TDes as u32));

        Self {
            desc,
            buf,
            index: 0,
            _phantom: PhantomData,
        }
    }

    /// Returns a mutable slice of the next available TX buffer, or None if DMA owns it.
    pub fn available(&mut self) -> Option<&mut [u8]> {
        let d = &self.desc[self.index];
        if d.tdes0() & TDES0_OWN == 0 {
            Some(&mut self.buf[self.index][..])
        } else {
            None
        }
    }

    /// Submit the current TX buffer to the DMA for transmission.
    pub fn transmit(&mut self, len: usize) {
        let d = &mut self.desc[self.index];

        // Set buffer pointer and length
        d.set_tdes2(self.buf[self.index].as_ptr() as u32);
        d.set_tdes1(len as u32);

        // Set ownership to DMA with fence
        fence(Ordering::Release);
        let base = d.tdes0() & !TDES0_OWN;
        d.set_tdes0(base | TDES0_OWN);

        // Advance ring index
        self.index = (self.index + 1) % N;

        // Poll transmit demand
        T::regs().dmatpdr().write(|w| w.set_tpd(0));
    }

    /// Number of descriptors in the ring.
    pub fn len(&self) -> usize {
        N
    }
}

/// An RX DMA descriptor ring.
pub struct RDesRing<'d, T: Instance, const N: usize> {
    desc: &'d mut [RDes; N],
    buf: &'d mut [[u8; RX_BUF_SIZE]; N],
    index: usize,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance, const N: usize> RDesRing<'d, T, N> {
    pub(super) fn new(desc: &'d mut [RDes; N], buf: &'d mut [[u8; RX_BUF_SIZE]; N]) -> Self {
        for i in 0..N {
            let next = if i + 1 < N {
                &desc[i + 1] as *const RDes as u32
            } else {
                &desc[0] as *const RDes as u32
            };

            desc[i].set_rdes0(RDES0_OWN); // DMA owns initially
            desc[i].set_rdes1(RDES1_RCH | (RX_BUF_SIZE as u32 & 0x1FFF));
            desc[i].set_rdes2(buf[i].as_ptr() as u32);
            desc[i].set_rdes3(next);
        }
        // Last descriptor chains back to first (already set in loop above).

        // Tell DMA where the RX descriptor ring starts
        T::regs()
            .dmardlar()
            .write(|w| w.set_srl(&desc[0] as *const RDes as u32));

        Self {
            desc,
            buf,
            index: 0,
            _phantom: PhantomData,
        }
    }

    /// Returns a reference to the received frame data if a complete frame is available.
    pub fn available(&mut self) -> Option<&mut [u8]> {
        // If DMA RX is stopped, demand poll to restart
        if T::regs().dmasr().read().rps() == 0b100 {
            self.demand_poll();
        }

        let d = &self.desc[self.index];
        let rdes0 = d.rdes0();

        // Check if DMA still owns this descriptor
        if rdes0 & RDES0_OWN != 0 {
            return None;
        }

        // Check for first+last segment (single descriptor per frame)
        if rdes0 & (RDES0_FS | RDES0_LS) != (RDES0_FS | RDES0_LS) {
            // Not a single-descriptor frame; skip it and give back to DMA
            self.pop_packet();
            return None;
        }

        // Check for errors
        if rdes0 & RDES0_ES != 0 {
            self.pop_packet();
            return None;
        }

        // Extract frame length (includes CRC, subtract 4)
        let len = ((rdes0 & RDES0_FL_MASK) >> RDES0_FL_SHIFT) as usize;
        let len = if len >= 4 { len - 4 } else { len };
        let len = len.min(RX_BUF_SIZE);

        fence(Ordering::Acquire);

        Some(&mut self.buf[self.index][..len])
    }

    /// Release the current RX descriptor back to DMA ownership.
    pub fn pop_packet(&mut self) {
        let d = &mut self.desc[self.index];
        d.set_rdes0(RDES0_OWN);
        self.demand_poll();
        self.index = (self.index + 1) % N;
    }

    fn demand_poll(&self) {
        T::regs().dmarpdr().write(|w| w.set_rpd(0));
    }
}

// ============================================================================
// Packet buffer queue (static allocation)
// ============================================================================

/// Static storage for DMA descriptors and packet buffers.
///
/// `TX` and `RX` are the number of descriptors (and buffers) in each ring.
/// Typical values: 2-4 for TX, 4-8 for RX.
///
/// # Usage
///
/// ```ignore
/// static QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
/// let queue = QUEUE.init(PacketQueue::new());
/// ```
pub struct PacketQueue<const TX: usize, const RX: usize> {
    pub(super) tx_desc: [TDes; TX],
    pub(super) rx_desc: [RDes; RX],
    pub(super) tx_buf: [[u8; TX_BUF_SIZE]; TX],
    pub(super) rx_buf: [[u8; RX_BUF_SIZE]; RX],
}

impl<const TX: usize, const RX: usize> PacketQueue<TX, RX> {
    /// Create a new zeroed packet queue.
    pub const fn new() -> Self {
        Self {
            tx_desc: [const { TDes::new() }; TX],
            rx_desc: [const { RDes::new() }; RX],
            tx_buf: [[0u8; TX_BUF_SIZE]; TX],
            rx_buf: [[0u8; RX_BUF_SIZE]; RX],
        }
    }
}

/// Token for receiving a single packet from the DMA ring.
pub struct RxToken<'a, 'd, T: Instance, const N: usize> {
    pub(super) rx: &'a mut RDesRing<'d, T, N>,
}

/// Token for transmitting a single packet via the DMA ring.
pub struct TxToken<'a, 'd, T: Instance, const N: usize> {
    pub(super) tx: &'a mut TDesRing<'d, T, N>,
}

impl<'a, 'd, T: Instance, const N: usize> embassy_net_driver::RxToken for RxToken<'a, 'd, T, N> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, f: F) -> R {
        let pkt = self.rx.available().expect("RxToken created without available packet");
        let r = f(pkt);
        self.rx.pop_packet();
        r
    }
}

impl<'a, 'd, T: Instance, const N: usize> embassy_net_driver::TxToken for TxToken<'a, 'd, T, N> {
    fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
        let pkt = self.tx.available().expect("TxToken created without available buffer");
        let r = f(&mut pkt[..len]);
        self.tx.transmit(len);
        r
    }
}
