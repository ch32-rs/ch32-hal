//! DMA descriptors and buffers for the CH32H417 gigabit MAC.
//!
//! The H4 DMA moves 256 bits per clock, so both descriptor rings and every
//! buffer must start on a 32-byte boundary (RM §31.8.4.4–31.8.4.5). The
//! descriptor format itself is the same four-word layout the V4x7 uses:
//! `TDES0` carries `OWN/IC/LS/FS/TCH` and `RDES0` carries `OWN/FL/FS/LS`.

use core::cell::UnsafeCell;
use core::sync::atomic::{fence, Ordering};

pub(super) const MTU: usize = 1514;
pub(super) const BUFFER_SIZE: usize = 1536;
const OWN: u32 = 1 << 31;
const TX_CONTROL: u32 = (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20);

/// One four-word DMA descriptor.
///
/// The alignment rounds the size up to 32 bytes, so every element of an array
/// is 32-byte aligned, not just the first one.
#[repr(C, align(32))]
pub(super) struct Descriptor(UnsafeCell<[u32; 4]>);

impl Descriptor {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; 4]))
    }

    pub(super) fn read(&self, index: usize) -> u32 {
        unsafe { self.0.get().cast::<u32>().add(index).read_volatile() }
    }

    pub(super) fn write(&self, index: usize, value: u32) {
        unsafe { self.0.get().cast::<u32>().add(index).write_volatile(value) }
    }

    fn address(&self) -> u32 {
        self.0.get() as usize as u32
    }
}

/// One frame buffer. 1536 bytes keeps a full-MTU frame plus FCS.
#[repr(C, align(32))]
pub(super) struct Buffer(UnsafeCell<[u8; BUFFER_SIZE]>);

impl Buffer {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; BUFFER_SIZE]))
    }

    fn address(&self) -> u32 {
        self.0.get() as usize as u32
    }
}

/// DMA memory must outlive the driver, including if it is forgotten.
#[repr(C)]
pub struct Resources<const TX: usize, const RX: usize> {
    pub(super) tx_descriptors: [Descriptor; TX],
    pub(super) rx_descriptors: [Descriptor; RX],
    pub(super) tx_buffers: [Buffer; TX],
    pub(super) rx_buffers: [Buffer; RX],
}

impl<const TX: usize, const RX: usize> Resources<TX, RX> {
    pub const fn new() -> Self {
        Self {
            tx_descriptors: [const { Descriptor::new() }; TX],
            rx_descriptors: [const { Descriptor::new() }; RX],
            tx_buffers: [const { Buffer::new() }; TX],
            rx_buffers: [const { Buffer::new() }; RX],
        }
    }
}

impl<const TX: usize, const RX: usize> Default for Resources<TX, RX> {
    fn default() -> Self {
        Self::new()
    }
}

pub(super) struct Ring {
    descriptors: &'static mut [Descriptor],
    buffers: &'static mut [Buffer],
    next: usize,
    receive: bool,
    pub(super) errors: u32,
}

impl Ring {
    pub(super) fn new(descriptors: &'static mut [Descriptor], buffers: &'static mut [Buffer], receive: bool) -> Self {
        let mut ring = Self {
            descriptors,
            buffers,
            next: 0,
            receive,
            errors: 0,
        };
        ring.reset();
        ring
    }

    pub(super) fn reset(&mut self) {
        self.next = 0;
        for index in 0..self.descriptors.len() {
            let descriptor = &self.descriptors[index];
            descriptor.write(0, 0);
            descriptor.write(
                1,
                if self.receive {
                    // `RDES1`: second address chained (`RCH`, bit 14) and the
                    // buffer size in `RBS1[12:0]`.
                    (1 << 14) | BUFFER_SIZE as u32
                } else {
                    0
                },
            );
            descriptor.write(2, self.buffers[index].address());
            descriptor.write(3, self.descriptors[(index + 1) % self.descriptors.len()].address());
        }
        fence(Ordering::SeqCst);
        if self.receive {
            for descriptor in self.descriptors.iter() {
                descriptor.write(0, OWN);
            }
        }
        fence(Ordering::SeqCst);
    }

    pub(super) fn address(&self) -> u32 {
        self.descriptors[0].address()
    }

    pub(super) fn len(&self) -> usize {
        self.descriptors.len()
    }

    pub(super) fn tx_ready(&self) -> bool {
        self.descriptors[self.next].read(0) & OWN == 0
    }

    pub(super) fn receive_length(&mut self) -> Option<usize> {
        for _ in 0..self.descriptors.len() {
            let status = self.descriptors[self.next].read(0);
            if status & OWN != 0 {
                return None;
            }
            fence(Ordering::SeqCst);
            // `RDES0.FL[29:16]` counts the CRC, which software strips.
            let length = ((status >> 16) & 0x3fff) as usize;
            if status & ((1 << 15) | (1 << 9) | (1 << 8)) == (1 << 9) | (1 << 8) && (18..=MTU + 4).contains(&length) {
                return Some(length - 4);
            }
            self.errors = self.errors.saturating_add(1);
            self.release();
        }
        None
    }

    fn advance(&mut self) {
        self.next = (self.next + 1) % self.descriptors.len();
    }

    fn release(&mut self) {
        fence(Ordering::SeqCst);
        self.descriptors[self.next].write(0, OWN);
        self.advance();
        super::resume_receive();
    }
}

pub struct RxToken<'a> {
    pub(super) ring: &'a mut Ring,
    pub(super) length: usize,
}

impl embassy_net_driver::RxToken for RxToken<'_> {
    fn consume<Result, Consume>(self, consume: Consume) -> Result
    where
        Consume: FnOnce(&mut [u8]) -> Result,
    {
        let buffer = self.ring.buffers[self.ring.next].0.get();
        consume(unsafe { &mut (&mut *buffer)[..self.length] })
    }
}

impl Drop for RxToken<'_> {
    fn drop(&mut self) {
        self.ring.release();
    }
}

pub struct TxToken<'a> {
    pub(super) ring: &'a mut Ring,
}

impl embassy_net_driver::TxToken for TxToken<'_> {
    fn consume<Result, Consume>(self, length: usize, consume: Consume) -> Result
    where
        Consume: FnOnce(&mut [u8]) -> Result,
    {
        assert!(length <= MTU);
        fence(Ordering::SeqCst);
        let index = self.ring.next;
        let status = self.ring.descriptors[index].read(0);
        if status & (1 << 15) != 0 {
            self.ring.errors = self.ring.errors.saturating_add(1);
        }
        let buffer = self.ring.buffers[index].0.get();
        let result = consume(unsafe { &mut (&mut *buffer)[..length] });
        if length != 0 {
            let length_with_padding = length.max(60);
            unsafe { (&mut *buffer)[length..length_with_padding].fill(0) };
            self.ring.descriptors[index].write(1, length_with_padding as u32);
            fence(Ordering::SeqCst);
            self.ring.descriptors[index].write(0, OWN | TX_CONTROL);
            fence(Ordering::SeqCst);
            self.ring.advance();
            super::resume_transmit();
        }
        result
    }
}
