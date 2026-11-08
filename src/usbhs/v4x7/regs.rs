use crate::pac::common::{Reg, RW};

#[cfg(usbhs_v4x7)]
type Device = crate::pac::usbhs::UsbDevice;
#[cfg(usbhs_h4)]
type Device = crate::pac::usbhs::UsbhsDevice;

use super::UsbRegs;

/// Indexed device registers. The common USB block and device block share a base.
#[derive(Clone, Copy)]
pub(super) struct Registers(pub UsbRegs);

impl Registers {
    /// The same base viewed as the device-mode register block. Metapac models
    /// the UEP* registers on the device block, and only there are the endpoint
    /// registers addressable by index.
    fn dev(self) -> Device {
        unsafe { Device::from_ptr(self.0.as_ptr()) }
    }

    /// The driver works with raw response codes and lengths, so it takes the
    /// PAC's address and restates the register at the width it was using
    /// before the endpoint registers became arrays.
    fn reg<T: Copy>(ptr: *mut T) -> Reg<T, RW> {
        unsafe { Reg::from_ptr(ptr) }
    }

    pub fn enables(self, incoming: bool) -> Reg<u16, RW> {
        let dev = self.dev();
        let ptr: *mut u16 = if incoming {
            dev.uep_tx_en().as_ptr().cast()
        } else {
            dev.uep_rx_en().as_ptr().cast()
        };
        Self::reg(ptr)
    }

    pub fn control(self, index: usize, incoming: bool) -> Reg<u8, RW> {
        Self::reg(control_ptr(self.dev(), index, incoming).cast())
    }

    pub fn dma(self, index: usize, incoming: bool) -> Reg<u32, RW> {
        Self::reg(dma_ptr(self.dev(), index, incoming).cast())
    }

    pub fn max_length(self, index: usize) -> Reg<u32, RW> {
        Self::reg(max_length_ptr(self.dev(), index).cast())
    }

    pub fn tx_length(self, index: usize) -> Reg<u16, RW> {
        Self::reg(tx_length_ptr(self.dev(), index).cast())
    }

    pub fn rx_length(self, index: usize) -> Reg<u16, RW> {
        Self::reg(rx_length_ptr(self.dev(), index).cast())
    }

    pub fn iso(self, incoming: bool) -> Reg<u16, RW> {
        let dev = self.dev();
        let ptr: *mut u16 = if incoming {
            dev.uep_t_iso().as_ptr().cast()
        } else {
            dev.uep_r_iso().as_ptr().cast()
        };
        Self::reg(ptr)
    }

    pub fn initialize(self) {
        let dev = self.dev();
        self.enables(true).write_value(0);
        self.enables(false).write_value(0);
        Self::reg(dev.uep_t_tog_auto().as_ptr().cast::<u16>()).write_value(0);
        Self::reg(dev.uep_r_tog_auto().as_ptr().cast::<u16>()).write_value(0);
        Self::reg(dev.uep_t_burst().as_ptr().cast::<u8>()).write_value(0);
        Self::reg(dev.uep_t_burst_mode().as_ptr().cast::<u8>()).write_value(0);
        Self::reg(dev.uep_r_burst().as_ptr().cast::<u8>()).write_value(0);
        Self::reg(dev.uep_r_res_mode().as_ptr().cast::<u8>()).write_value(0);
        Self::reg(dev.uep_af_mode().as_ptr().cast::<u32>()).write_value(0);
        self.iso(true).write_value(0);
        self.iso(false).write_value(0);
        for index in 0..8 {
            self.control(index, true).write_value(0);
            self.control(index, false).write_value(0);
            self.tx_length(index).write_value(0);
        }
    }
}

// V4x7 and H4 describe the same device block; only the PAC type name differs
// (`UsbDevice` vs `UsbhsDevice`). Endpoint 0 keeps its own registers; 1-7 come
// from the same array accessors on both.

fn control_ptr(dev: Device, index: usize, incoming: bool) -> *mut u8 {
    // Endpoint 0 has its own narrower fieldsets, so it stays a single
    // register; 1-7 come from the PAC's array accessors.
    match (index, incoming) {
        (0, true) => dev.uep0_tx_ctrl().as_ptr().cast(),
        (0, false) => dev.uep0_rx_ctrl().as_ptr().cast(),
        (n, true) => dev.uep_tx_ctrl(n - 1).as_ptr().cast(),
        (n, false) => dev.uep_rx_ctrl(n - 1).as_ptr().cast(),
    }
}

fn dma_ptr(dev: Device, index: usize, incoming: bool) -> *mut u32 {
    match (index, incoming) {
        (0, _) => dev.uep0_dma().as_ptr().cast(),
        (n, true) => dev.uep_tx_dma(n - 1).as_ptr().cast(),
        (n, false) => dev.uep_rx_dma(n - 1).as_ptr().cast(),
    }
}

fn max_length_ptr(dev: Device, index: usize) -> *mut u32 {
    if index == 0 {
        dev.uep0_max_len().as_ptr().cast()
    } else {
        dev.uep_max_len(index - 1).as_ptr().cast()
    }
}

fn tx_length_ptr(dev: Device, index: usize) -> *mut u16 {
    if index == 0 {
        dev.uep0_t_len().as_ptr().cast()
    } else {
        dev.uep_t_len(index - 1).as_ptr().cast()
    }
}

fn rx_length_ptr(dev: Device, index: usize) -> *mut u16 {
    if index == 0 {
        dev.uep0_rx_len().as_ptr().cast()
    } else {
        dev.uep_rx_len(index - 1).as_ptr().cast()
    }
}
