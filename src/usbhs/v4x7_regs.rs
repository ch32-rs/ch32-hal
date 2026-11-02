use crate::pac::common::{Reg, RW};

/// Indexed device registers. The common USB block and device block share a base.
#[derive(Clone, Copy)]
pub(super) struct Registers(pub crate::pac::usbhs::Usb);

impl Registers {
    fn byte(self, offset: usize) -> Reg<u8, RW> {
        unsafe { Reg::from_ptr(self.0.as_ptr().cast::<u8>().add(offset)) }
    }

    fn halfword(self, offset: usize) -> Reg<u16, RW> {
        unsafe { Reg::from_ptr(self.0.as_ptr().cast::<u8>().add(offset).cast()) }
    }

    fn word(self, offset: usize) -> Reg<u32, RW> {
        unsafe { Reg::from_ptr(self.0.as_ptr().cast::<u8>().add(offset).cast()) }
    }

    pub fn enables(self, incoming: bool) -> Reg<u16, RW> {
        self.halfword(if incoming { 0x10 } else { 0x12 })
    }

    pub fn control(self, index: usize, incoming: bool) -> Reg<u8, RW> {
        self.byte(0x9c + index * 4 + if incoming { 2 } else { 3 })
    }

    pub fn dma(self, index: usize, incoming: bool) -> Reg<u32, RW> {
        self.word(if index == 0 {
            0x20
        } else if incoming {
            0x40 + (index - 1) * 4
        } else {
            0x24 + (index - 1) * 4
        })
    }

    pub fn max_length(self, index: usize) -> Reg<u32, RW> {
        self.word(0x5c + index * 4)
    }

    pub fn tx_length(self, index: usize) -> Reg<u16, RW> {
        self.halfword(0x9c + index * 4)
    }

    pub fn rx_length(self, index: usize) -> Reg<u16, RW> {
        self.halfword(if index == 0 { 0x7c } else { 0x80 + (index - 1) * 4 })
    }

    pub fn iso(self, incoming: bool) -> Reg<u16, RW> {
        self.halfword(if incoming { 0xbc } else { 0xbe })
    }

    pub fn initialize(self) {
        self.halfword(0x10).write_value(0);
        self.halfword(0x12).write_value(0);
        self.halfword(0x14).write_value(0);
        self.halfword(0x16).write_value(0);
        self.byte(0x18).write_value(0);
        self.byte(0x19).write_value(0);
        self.byte(0x1a).write_value(0);
        self.byte(0x1b).write_value(0);
        self.word(0x1c).write_value(0);
        self.iso(true).write_value(0);
        self.iso(false).write_value(0);
        for index in 0..8 {
            self.control(index, true).write_value(0);
            self.control(index, false).write_value(0);
            self.tx_length(index).write_value(0);
        }
    }
}
