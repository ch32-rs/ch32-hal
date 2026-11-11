//! `usbhs_v4x7` (CH32V4x7) and `usbhs_h4` (CH32H417) describe the same device
//! block: every endpoint control register carries its own completion flag.
//! `usbhs_v3` (CH32V30x) reports one transaction at a time through the shared
//! `INT_ST`/`RX_LEN` registers, encodes handshakes differently and toggles bulk
//! endpoints in hardware.

use super::Inner;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Res {
    Nak,
    Ack,
    Stall,
}

#[derive(Default)]
pub(crate) struct Wake {
    pub bus: bool,
    pub endpoints: u32,
    pub sof: Option<fn()>,
    #[cfg(not(usbhs_v3))]
    pub reload: u32,
}

impl Wake {
    fn all(&mut self) {
        self.bus = true;
        self.endpoints = u32::MAX;
    }
}

fn isochronous(inner: &Inner, slot: usize) -> bool {
    inner.endpoints[slot].isochronous()
}

#[cfg(any(usbhs_v4x7, usbhs_h4))]
mod imp {
    use embassy_usb_driver::{EndpointType, Speed};

    use super::{isochronous, Res, Wake};
    use crate::pac;
    use crate::pac::common::{Reg, RW};
    use crate::pac::usbhs::regs::{UsbCtrl, UsbDevAd, UsbIntEn, UsbIntFg};
    use crate::usbhs::{iso, slot, EndpointState, Inner};

    /// Common and device-mode registers share one base. V4x7 names the block
    /// `Usb`, H4 names it `Usbhs`; the layout is identical.
    #[cfg(usbhs_v4x7)]
    pub(crate) type Regs = pac::usbhs::Usb;
    #[cfg(usbhs_h4)]
    pub(crate) type Regs = pac::usbhs::Usbhs;
    #[cfg(usbhs_v4x7)]
    type Device = pac::usbhs::UsbDevice;
    #[cfg(usbhs_h4)]
    type Device = pac::usbhs::UsbhsDevice;

    pub(crate) const ENDPOINTS: usize = 8;

    const NAK: u8 = 0;
    const STALL: u8 = 1;
    const ACK: u8 = 2;
    const DONE: u8 = 0x80;
    const PRESERVE_FLAGS: u8 = 0xc0;
    const INTERRUPTS: u8 = 0xd7;
    const SOF: u8 = 0x20;

    fn code(res: Res) -> u8 {
        match res {
            Res::Nak => NAK,
            Res::Ack => ACK,
            Res::Stall => STALL,
        }
    }

    pub(crate) fn clock_ready() -> bool {
        pac::RCC.ctlr().read().usbhs_pllrdy()
    }

    #[cfg(target_arch = "riscv32")]
    pub(crate) fn memory_valid(address: usize, length: usize) -> bool {
        #[cfg(usbhs_h4)]
        let valid = {
            // The USBHS DMA address registers keep bits 23:0 and the engine adds
            // `0x2000_0000` on its own (RM §25.2.1.22–25.2.1.24), so a buffer must
            // lie in the first 16 MB of the memory region. All three H4 RAM blocks
            // — ITCM at `0x200A_0000`, DTCM at `0x200C_0000`, shared SRAM at
            // `0x2010_0000` — do.
            const RAM_START: usize = 0x200A_0000;
            const WINDOW_END: usize = 0x2100_0000;
            address >= RAM_START && address.checked_add(length).is_some_and(|end| end <= WINDOW_END)
        };
        #[cfg(usbhs_v4x7)]
        let valid = {
            // V2/V3 can trade flash for RAM through `RAM_CODE_MOD`; the USB DMA
            // can only reach the configured amount.
            let ram_size = if pac::FLASH.obr().read().ram_code_mod().to_bits() == 0 {
                200 * 1024
            } else {
                136 * 1024
            };
            address >= 0x2000_0000
                && address
                    .checked_add(length)
                    .is_some_and(|end| end <= 0x2000_0000 + ram_size)
        };
        valid
    }

    /// A pair shares its maximum packet length register.
    pub(crate) fn can_pair(other: &embassy_usb_driver::EndpointInfo, _kind: EndpointType, size: u16) -> bool {
        other.max_packet_size == size
    }

    /// The same base viewed as the device-mode register block. Metapac models
    /// the UEP* registers on the device block, and only there are the endpoint
    /// registers addressable by index.
    fn dev(regs: Regs) -> Device {
        unsafe { Device::from_ptr(regs.as_ptr()) }
    }

    /// The driver works with raw response codes and lengths, so it takes the
    /// PAC's address and restates the register at the width it was using
    /// before the endpoint registers became arrays.
    fn reg<T: Copy>(ptr: *mut T) -> Reg<T, RW> {
        unsafe { Reg::from_ptr(ptr) }
    }

    fn enables(regs: Regs, incoming: bool) -> Reg<u16, RW> {
        let dev = dev(regs);
        let ptr: *mut u16 = if incoming {
            dev.uep_tx_en().as_ptr().cast()
        } else {
            dev.uep_rx_en().as_ptr().cast()
        };
        reg(ptr)
    }

    // Endpoint 0 has its own narrower fieldsets, so it stays a single
    // register; 1-7 come from the PAC's array accessors.
    fn control(regs: Regs, index: usize, incoming: bool) -> Reg<u8, RW> {
        let dev = dev(regs);
        reg(match (index, incoming) {
            (0, true) => dev.uep0_tx_ctrl().as_ptr().cast(),
            (0, false) => dev.uep0_rx_ctrl().as_ptr().cast(),
            (n, true) => dev.uep_tx_ctrl(n - 1).as_ptr().cast(),
            (n, false) => dev.uep_rx_ctrl(n - 1).as_ptr().cast(),
        })
    }

    fn dma(regs: Regs, index: usize, incoming: bool) -> Reg<u32, RW> {
        let dev = dev(regs);
        reg(match (index, incoming) {
            (0, _) => dev.uep0_dma().as_ptr().cast(),
            (n, true) => dev.uep_tx_dma(n - 1).as_ptr().cast(),
            (n, false) => dev.uep_rx_dma(n - 1).as_ptr().cast(),
        })
    }

    fn max_length(regs: Regs, index: usize) -> Reg<u32, RW> {
        let dev = dev(regs);
        reg(if index == 0 {
            dev.uep0_max_len().as_ptr().cast()
        } else {
            dev.uep_max_len(index - 1).as_ptr().cast()
        })
    }

    fn tx_length(regs: Regs, index: usize) -> Reg<u16, RW> {
        let dev = dev(regs);
        reg(if index == 0 {
            dev.uep0_t_len().as_ptr().cast()
        } else {
            dev.uep_t_len(index - 1).as_ptr().cast()
        })
    }

    fn rx_length(regs: Regs, index: usize) -> Reg<u16, RW> {
        let dev = dev(regs);
        reg(if index == 0 {
            dev.uep0_rx_len().as_ptr().cast()
        } else {
            dev.uep_rx_len(index - 1).as_ptr().cast()
        })
    }

    fn iso(regs: Regs, incoming: bool) -> Reg<u16, RW> {
        let dev = dev(regs);
        let ptr: *mut u16 = if incoming {
            dev.uep_t_iso().as_ptr().cast()
        } else {
            dev.uep_r_iso().as_ptr().cast()
        };
        reg(ptr)
    }

    pub(crate) fn power_on(regs: Regs, speed: Speed) {
        let dev = dev(regs);
        regs.usb_ctrl().write_value(UsbCtrl(9));
        enables(regs, true).write_value(0);
        enables(regs, false).write_value(0);
        reg(dev.uep_t_tog_auto().as_ptr().cast::<u16>()).write_value(0);
        reg(dev.uep_r_tog_auto().as_ptr().cast::<u16>()).write_value(0);
        reg(dev.uep_t_burst().as_ptr().cast::<u8>()).write_value(0);
        reg(dev.uep_t_burst_mode().as_ptr().cast::<u8>()).write_value(0);
        reg(dev.uep_r_burst().as_ptr().cast::<u8>()).write_value(0);
        reg(dev.uep_r_res_mode().as_ptr().cast::<u8>()).write_value(0);
        reg(dev.uep_af_mode().as_ptr().cast::<u32>()).write_value(0);
        iso(regs, true).write_value(0);
        iso(regs, false).write_value(0);
        for index in 0..ENDPOINTS {
            control(regs, index, true).write_value(0);
            control(regs, index, false).write_value(0);
            tx_length(regs, index).write_value(0);
        }
        regs.usb_base_mode()
            .write(|reg| reg.set_ud_speed_type(if speed == Speed::High { 1 } else { 0 }));
    }

    pub(crate) fn program(regs: Regs, index: usize, incoming: bool, address: u32, max: u16, kind: EndpointType) {
        dma(regs, index, incoming).write_value(address);
        max_length(regs, index).write_value(u32::from(max));
        if kind == EndpointType::Isochronous {
            iso(regs, incoming).modify(|value| *value |= 1 << index);
        }
    }

    pub(crate) fn connect(regs: Regs, sof: bool) {
        regs.usb_int_fg().write_value(UsbIntFg(0xef));
        regs.usb_int_en()
            .write_value(UsbIntEn(INTERRUPTS | if sof { SOF } else { 0 }));
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        regs.usb_ctrl().write_value(UsbCtrl(0x38));
    }

    pub(crate) fn enable_sof(regs: Regs) {
        regs.usb_int_en().modify(|value| value.0 |= SOF);
    }

    pub(crate) fn frame(regs: Regs) -> u16 {
        regs.usb_fram_no().read().0
    }

    pub(crate) fn power_off(regs: Regs) {
        regs.usb_int_en().write_value(UsbIntEn(0));
        regs.usb_ctrl().write_value(UsbCtrl(3));
    }

    pub(crate) fn reset_endpoints(regs: Regs) {
        enables(regs, true).write_value(1);
        enables(regs, false).write_value(1);
        regs.usb_dev_ad().write_value(UsbDevAd(0));
        for index in 0..ENDPOINTS {
            for incoming in [false, true] {
                control(regs, index, incoming).write_value(if index == 0 && !incoming { ACK } else { NAK });
            }
            tx_length(regs, index).write_value(0);
        }
    }

    pub(crate) fn enable(regs: Regs, index: usize, incoming: bool, enabled: bool, _kind: EndpointType, res: Res) {
        enables(regs, incoming).modify(|value| *value &= !(1 << index));
        control(regs, index, incoming).write_value(code(res));
        if enabled {
            enables(regs, incoming).modify(|value| *value |= 1 << index);
        }
    }

    pub(crate) fn restart(regs: Regs, index: usize, incoming: bool, res: Res) {
        control(regs, index, incoming).write_value(code(res));
    }

    pub(crate) fn iso_start(regs: Regs, index: usize, incoming: bool, endpoint: &mut EndpointState) {
        let Some(config) = endpoint.config else {
            return;
        };
        dma(regs, index, incoming).write_value(config.packet(0) as u32);
        if incoming {
            tx_length(regs, index).write_value(0);
        }
        control(regs, index, incoming).write_value(ACK);
    }

    pub(crate) fn iso_resume(regs: Regs, index: usize, incoming: bool) {
        control(regs, index, incoming).write_value(ACK);
    }

    pub(crate) fn iso_reload(regs: Regs, inner: &mut Inner, mut slots: u32) {
        while slots != 0 {
            let s = slots.trailing_zeros() as usize;
            slots &= slots - 1;
            let index = s / 2;
            if control(regs, index, true).read() & DONE != 0 {
                continue;
            }
            if !regs.usb_mis_st().read().udms_sie_free() {
                let st = regs.usb_int_st().read();
                if st.udis_ep_dir() && usize::from(st.udis_ep_id_mask()) == index {
                    continue;
                }
            }
            let endpoint = &mut inner.endpoints[s];
            let Some(config) = endpoint.config else {
                continue;
            };
            if let Some((address, length)) = iso::in_reload(endpoint, &config) {
                dma(regs, index, true).write_value(address as u32);
                tx_length(regs, index).write_value(length);
            }
        }
    }

    pub(crate) fn set_res(regs: Regs, index: usize, incoming: bool, res: Res) {
        control(regs, index, incoming).modify(|value| *value = (*value & 0x0c) | code(res) | PRESERVE_FLAGS);
    }

    pub(crate) fn await_setup(regs: Regs) {
        control(regs, 0, false).modify(|value| *value = (*value & 4) | ACK | PRESERVE_FLAGS);
    }

    pub(crate) fn ep0_prepare(regs: Regs, incoming: bool, first: bool) {
        if first {
            control(regs, 0, incoming).modify(|value| *value = (*value & 3) | 4 | PRESERVE_FLAGS);
        }
        if !incoming {
            control(regs, 0, false).modify(|value| *value = (*value & 4) | ACK | PRESERVE_FLAGS);
        }
    }

    pub(crate) fn setup_pending(regs: Regs) -> bool {
        control(regs, 0, false).read() & 0x88 == 0x88
    }

    pub(crate) fn arm_out(regs: Regs, index: usize) {
        set_res(regs, index, false, Res::Ack);
    }

    pub(crate) fn arm_in(regs: Regs, index: usize, length: u16) {
        tx_length(regs, index).write_value(length);
        set_res(regs, index, true, Res::Ack);
    }

    pub(crate) fn set_address(regs: Regs, address: u8) {
        regs.usb_dev_ad().write(|reg| reg.set_mask_usb_addr(address));
    }

    #[cfg(feature = "embassy")]
    pub(crate) fn remote_wakeup(regs: Regs) -> bool {
        regs.usb_wake_ctrl().write(|reg| reg.set_ud_remote_wkup(true));
        true
    }

    pub(crate) fn interrupt(regs: Regs, inner: &mut Inner) -> Wake {
        let mut wake = Wake::default();
        let flags = regs.usb_int_fg().read().0;
        if flags & 1 != 0 {
            reset_endpoints(regs);
            inner.reset();
            regs.usb_int_fg().write_value(UsbIntFg(1));
            wake.all();
            return wake;
        }
        if flags & 0x80 != 0 {
            inner.overflow();
            wake.endpoints = u32::MAX;
        }
        if flags & SOF != 0 {
            wake.sof = inner.sof;
        }
        let rx0 = control(regs, 0, false).read();
        let setup = rx0 & (DONE | 8) == (DONE | 8);
        if setup {
            let mut bytes = [0; 8];
            let length = rx_length(regs, 0).read();
            if let Some(config) = inner.endpoints[0].config {
                unsafe { config.buffer.read(&mut bytes) };
            }
            for incoming in [false, true] {
                control(regs, 0, incoming).write_value(4);
            }
            if length != 8 {
                control(regs, 0, true).write_value(STALL);
                control(regs, 0, false).write_value(STALL);
            }
            inner.setup((length == 8).then_some(bytes));
            wake.endpoints |= 3;
        }
        for index in 0..ENDPOINTS {
            if index == 0 && setup {
                continue;
            }
            for incoming in [false, true] {
                let control_value = if index == 0 && !incoming {
                    rx0
                } else {
                    control(regs, index, incoming).read()
                };
                if control_value & DONE == 0 {
                    if control_value & 0x40 != 0 {
                        control(regs, index, incoming).write_value((control_value & !0x40) | DONE);
                    }
                    continue;
                }
                let isochronous = isochronous(inner, slot(index, incoming));
                let endpoint = &inner.endpoints[slot(index, incoming)];
                if !endpoint.enabled || endpoint.stalled {
                    control(regs, index, incoming)
                        .write_value((control_value & 0x0c) | if endpoint.stalled { STALL } else { NAK });
                    continue;
                }
                if incoming && isochronous {
                    let endpoint = &mut inner.endpoints[slot(index, true)];
                    if let Some(config) = endpoint.config {
                        match iso::in_next(endpoint, &config) {
                            Some((address, length)) => {
                                dma(regs, index, true).write_value(address as u32);
                                tx_length(regs, index).write_value(length);
                            }
                            None => {
                                tx_length(regs, index).write_value(0);
                                wake.reload |= 1 << slot(index, true);
                            }
                        }
                    }
                    control(regs, index, true).write_value(ACK);
                } else if incoming {
                    inner.in_done(index);
                    control(regs, index, true).write_value(((control_value ^ 4) & 0x0c) | NAK);
                } else if isochronous {
                    let endpoint = &mut inner.endpoints[slot(index, false)];
                    if let Some(config) = endpoint.config {
                        iso::out_push(endpoint, &config, rx_length(regs, index).read());
                        dma(regs, index, false).write_value(config.packet(endpoint.head) as u32);
                    }
                    control(regs, index, false).write_value(ACK);
                } else if control_value & 0x10 != 0 {
                    inner.out_done(index, rx_length(regs, index).read());
                    control(regs, index, false).write_value(((control_value ^ 4) & 0x0c) | NAK);
                } else {
                    control(regs, index, false).write_value((control_value & 0x0c) | ACK);
                }
                wake.endpoints |= 1 << slot(index, incoming);
            }
        }
        let clear = flags & 0xee;
        if clear & 0x46 != 0 {
            let status = regs.usb_mis_st().read();
            wake.bus |= inner.set_suspended(status.udms_suspend() || status.udms_sleep());
        }
        if clear != 0 {
            regs.usb_int_fg().write_value(UsbIntFg(clear));
        }
        wake
    }
}

#[cfg(usbhs_v3)]
mod imp {
    use core::sync::atomic::{AtomicU32, Ordering};

    use embassy_usb_driver::{EndpointType, Speed};

    use super::{isochronous, Res, Wake};
    use crate::pac;
    use crate::pac::usbhs::regs::{EpBufMod, EpConfig, EpType, IntSt};
    use crate::pac::usbhs::vals::{EndpointType as Kind, EpRxResponse, EpTog, EpTxResponse, SpeedType, UsbToken};
    use crate::usbhs::{iso, slot, EndpointState, Inner};

    pub(crate) type Regs = pac::usbhs::Usb;

    pub(crate) const ENDPOINTS: usize = 16;

    /// Polls of MIS_ST to wait for an iso OUT to finish: longer than a 1024-byte packet
    const SIE_WAIT: u32 = 2000;

    /// The frame number register: frame (bits 10:0) and microframe (bits 15:13); bits 12:11
    /// are always 0, so this is never a frame number.
    pub(crate) const NO_FRAME: u16 = 0x1800;

    /// The isochronous IN response: "data ready" (TinyUSB `USBHS_EP_T_RES_NYET`).
    const TX_RES_ISO: EpTxResponse = EpTxResponse::_RESERVED_1;

    /// The pattern an armed isochronous OUT slot is filled with (see `iso_out_check`)
    const ISO_OUT_MARK: u32 = 0x5A3C_A5C3;

    /// bulk / interrupt: R_TOG / T_TOG after the last packet taken / sent (DATA1: true)
    static TOG: AtomicU32 = AtomicU32::new(0);

    fn dev(regs: Regs) -> pac::usbhs::Usbd {
        unsafe { pac::usbhs::Usbd::from_ptr(regs.as_ptr()) }
    }

    fn host(regs: Regs) -> pac::usbhs::Usbh {
        unsafe { pac::usbhs::Usbh::from_ptr(regs.as_ptr()) }
    }

    fn tx(res: Res) -> EpTxResponse {
        match res {
            Res::Nak => EpTxResponse::NAK,
            Res::Ack => EpTxResponse::ACK,
            Res::Stall => EpTxResponse::STALL,
        }
    }

    fn rx(res: Res) -> EpRxResponse {
        match res {
            Res::Nak => EpRxResponse::NAK,
            Res::Ack => EpRxResponse::ACK,
            Res::Stall => EpRxResponse::STALL,
        }
    }

    fn flip(tog: EpTog) -> EpTog {
        match tog {
            EpTog::DATA0 => EpTog::DATA1,
            _ => EpTog::DATA0,
        }
    }

    fn clear_tog(index: usize, incoming: bool) {
        TOG.store(
            TOG.load(Ordering::Relaxed) & !(1 << slot(index, incoming)),
            Ordering::Relaxed,
        );
    }

    pub(crate) fn clock_ready() -> bool {
        pac::RCC.cfgr2().read().usbhs_pllalive()
    }

    #[cfg(target_arch = "riscv32")]
    pub(crate) fn memory_valid(address: usize, length: usize) -> bool {
        address >= 0x2000_0000
            && address
                .checked_add(length)
                .is_some_and(|end| end <= 0x2000_0000 + 128 * 1024)
    }

    /// A PING to a bulk OUT whose number is isochronous on the IN side hangs the SIE.
    pub(crate) fn can_pair(other: &embassy_usb_driver::EndpointInfo, kind: EndpointType, _size: u16) -> bool {
        (other.ep_type == EndpointType::Isochronous) == (kind == EndpointType::Isochronous)
    }

    pub(crate) fn power_on(regs: Regs, speed: Speed) {
        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });
        // Sleep for 10uS from WCH C code
        crate::internal::delay::block_for_us(10);
        regs.ctrl().modify(|w| w.set_reset_sie(false));
        host(regs).ctrl().write(|w| w.set_phy_suspendm(true));
        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_speed_type(if speed == Speed::High {
                SpeedType::HIGHSPEED
            } else {
                SpeedType::FULLSPEED
            });
        });
        dev(regs).ep_buf_mod().write_value(EpBufMod::default());
    }

    pub(crate) fn program(regs: Regs, index: usize, incoming: bool, address: u32, max: u16, _kind: EndpointType) {
        let d = dev(regs);
        match (index, incoming) {
            (0, _) => {
                d.ep0_dma().write_value(address);
                d.ep_max_len(0).write(|w| w.set_len(max));
            }
            // MAX_LEN bounds what an OUT endpoint accepts; IN lengths come from T_LEN
            (n, true) => d.ep_tx_dma(n - 1).write_value(address),
            (n, false) => {
                d.ep_rx_dma(n - 1).write_value(address);
                d.ep_max_len(n).write(|w| w.set_len(max));
            }
        }
    }

    pub(crate) fn connect(regs: Regs, sof: bool) {
        regs.int_fg().write(|w| {
            w.set_setup_act(true);
            w.set_fifo_ov(true);
            w.set_hst_sof(true);
            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });
        regs.int_en().write(|w| {
            w.set_fifo_ov(true);
            w.set_setup_act(true);
            w.set_suspend(true);
            w.set_bus_rst(true);
            w.set_transfer(true);
            w.set_sof_act(sof);
        });
        regs.ctrl().modify(|w| w.set_dev_pu_en(true));
    }

    pub(crate) fn enable_sof(regs: Regs) {
        regs.int_en().modify(|w| w.set_sof_act(true));
    }

    pub(crate) fn frame(regs: Regs) -> u16 {
        regs.frame_no().read().frame_no()
    }

    pub(crate) fn power_off(regs: Regs) {
        regs.int_en().write(|_| {});
        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });
    }

    pub(crate) fn reset_endpoints(regs: Regs) {
        let d = dev(regs);
        regs.dev_ad().write(|w| w.set_addr(0));
        d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::ACK));
        d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
        for index in 1..ENDPOINTS {
            d.ep_tx_ctrl(index).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
            d.ep_rx_ctrl(index).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
        }
        for index in 0..ENDPOINTS {
            d.ep_t_len(index).write(|w| w.set_len(0));
        }
        d.ep_config().write_value(EpConfig::default());
        d.ep_type().write_value(EpType::default());
        TOG.store(0, Ordering::Relaxed);
    }

    pub(crate) fn enable(regs: Regs, index: usize, incoming: bool, enabled: bool, kind: EndpointType, res: Res) {
        let d = dev(regs);
        let iso = kind == EndpointType::Isochronous;
        let kind = if iso && enabled { Kind::ISO } else { Kind::NISO };
        clear_tog(index, incoming);
        if incoming {
            d.ep_t_len(index).write(|w| w.set_len(0));
            d.ep_tx_ctrl(index).write(|w| {
                w.set_mask_uep_t_res(tx(res));
                w.set_t_tog_auto(!iso);
            });
            d.ep_type().modify(|w| w.set_t_type(index - 1, kind));
            d.ep_config().modify(|w| w.set_t_en(index - 1, enabled));
        } else {
            d.ep_rx_ctrl(index).write(|w| {
                w.set_mask_uep_r_res(rx(res));
                w.set_r_tog_auto(!iso);
            });
            d.ep_type().modify(|w| w.set_r_type(index - 1, kind));
            d.ep_config().modify(|w| w.set_r_en(index - 1, enabled));
        }
    }

    pub(crate) fn restart(regs: Regs, index: usize, incoming: bool, res: Res) {
        let d = dev(regs);
        clear_tog(index, incoming);
        if incoming {
            d.ep_tx_ctrl(index).modify(|w| {
                w.set_mask_uep_t_res(tx(res));
                w.set_mask_uep_t_tog(EpTog::DATA0);
            });
        } else {
            d.ep_rx_ctrl(index).modify(|w| {
                w.set_mask_uep_r_res(rx(res));
                w.set_mask_uep_r_tog(EpTog::DATA0);
            });
        }
    }

    pub(crate) fn iso_start(regs: Regs, index: usize, incoming: bool, endpoint: &mut EndpointState) {
        let d = dev(regs);
        let Some(config) = endpoint.config else {
            return;
        };
        if incoming {
            d.ep_tx_dma(index - 1).write_value(config.packet(0) as u32);
            d.ep_t_len(index).write(|w| w.set_len(0));
            d.ep_tx_ctrl(index).write(|w| {
                w.set_mask_uep_t_tog(EpTog::DATA0);
                w.set_t_tog_auto(true);
                w.set_mask_uep_t_res(TX_RES_ISO);
            });
        } else {
            let max = config.info.max_packet_size;
            for k in 0..iso::slots(false) {
                config.set_length(k, max);
            }
            endpoint.min_len = max;
            for k in (0..iso::slots(false)).rev() {
                iso_out_arm(d, index, config.packet(k), 0, usize::from(max));
            }
            d.ep_rx_ctrl(index).write(|w| {
                w.set_mask_uep_r_tog(EpTog::DATA0);
                // no handshake
                w.set_mask_uep_r_res(EpRxResponse::NYET);
            });
        }
    }

    pub(crate) fn iso_resume(regs: Regs, index: usize, incoming: bool) {
        let d = dev(regs);
        if incoming {
            d.ep_tx_ctrl(index).write(|w| {
                w.set_mask_uep_t_tog(EpTog::DATA0);
                w.set_t_tog_auto(true);
                w.set_mask_uep_t_res(TX_RES_ISO);
            });
        } else {
            d.ep_rx_ctrl(index).write(|w| {
                w.set_mask_uep_r_tog(EpTog::DATA0);
                w.set_mask_uep_r_res(EpRxResponse::NYET);
            });
        }
    }

    pub(crate) fn set_res(regs: Regs, index: usize, incoming: bool, res: Res) {
        let d = dev(regs);
        if incoming {
            d.ep_tx_ctrl(index).modify(|w| w.set_mask_uep_t_res(tx(res)));
        } else {
            d.ep_rx_ctrl(index).modify(|w| w.set_mask_uep_r_res(rx(res)));
        }
    }

    /// Setup packets are taken whatever EP0 answers.
    pub(crate) fn await_setup(_regs: Regs) {}

    pub(crate) fn ep0_prepare(regs: Regs, incoming: bool, first: bool) {
        let d = dev(regs);
        if first && incoming {
            d.ep_tx_ctrl(0).modify(|w| w.set_mask_uep_t_tog(EpTog::DATA1));
        }
        if !incoming {
            d.ep_rx_ctrl(0).modify(|w| {
                if first {
                    w.set_mask_uep_r_tog(EpTog::DATA1);
                }
                w.set_mask_uep_r_res(EpRxResponse::ACK);
            });
        }
    }

    pub(crate) fn setup_pending(regs: Regs) -> bool {
        regs.int_fg().read().setup_act()
    }

    pub(crate) fn arm_out(regs: Regs, index: usize) {
        set_res(regs, index, false, Res::Ack);
    }

    pub(crate) fn arm_in(regs: Regs, index: usize, length: u16) {
        dev(regs).ep_t_len(index).write(|w| w.set_len(length));
        set_res(regs, index, true, Res::Ack);
    }

    pub(crate) fn set_address(regs: Regs, address: u8) {
        regs.dev_ad().write(|w| w.set_addr(address));
    }

    #[cfg(feature = "embassy")]
    pub(crate) fn remote_wakeup(_regs: Regs) -> bool {
        false
    }

    pub(crate) fn interrupt(regs: Regs, inner: &mut Inner) -> Wake {
        let d = dev(regs);
        let mut wake = Wake::default();
        let flag = regs.int_fg().read();
        if flag.bus_rst() {
            reset_endpoints(regs);
            inner.reset();
            regs.int_fg().write(|w| w.set_bus_rst(true));
            wake.all();
            return wake;
        }
        if flag.suspend() {
            regs.int_fg().write(|w| w.set_suspend(true));
            wake.bus |= inner.set_suspended(regs.mis_st().read().suspend());
        }
        if flag.fifo_ov() {
            regs.int_fg().write(|w| w.set_fifo_ov(true));
        }
        if flag.hst_sof() {
            regs.int_fg().write(|w| w.set_hst_sof(true));
        }
        if flag.transfer() {
            transfer(regs, d, inner, &mut wake);
        }
        if flag.setup_act() {
            // a new control transfer: whatever EP0 was doing is over
            d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
            d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));
            let mut bytes = [0; 8];
            if let Some(config) = inner.endpoints[0].config {
                unsafe { config.buffer.read(&mut bytes) };
            }
            inner.setup(Some(bytes));
            regs.int_fg().write(|w| w.set_setup_act(true));
            wake.endpoints |= 3;
        }
        wake
    }

    /// Every transfer completes in the interrupt handler: it reads the shared status
    /// (`INT_ST`, `RX_LEN`) and releases `INT_FG` at once -- isochronous transactions ignore
    /// `INT_BUSY` and would overwrite them -- then records the result per endpoint and wakes
    /// the task. Bulk / interrupt endpoints keep flow control by answering NAK until their
    /// task has taken the data (OUT) or loaded the next packet (IN).
    fn transfer(regs: Regs, d: pac::usbhs::Usbd, inner: &mut Inner, wake: &mut Wake) {
        // INT_ST shows a token as soon as it arrives, not when its transaction completes:
        // an entry that runs late (the next transaction is already on the bus) reads a
        // token that hasn't completed
        let mut busy = !regs.mis_st().read().sie_free();
        let mut st = regs.int_st().read();
        let mut len = regs.rx_len().read();
        if !busy || (st.token() != UsbToken::SOF && st.endp() != 0) {
            nak(d, inner, st);
        }
        // release early: INT_ST / RX_LEN are read
        regs.int_fg().write(|w| w.set_transfer(true));
        // an iso OUT on the bus: this entry is its own (raised a little before the SIE goes
        // free) or a late one. take it here once it's in -- left to a later entry, the next
        // microframe's OUT may land on it first. its completion after the release sets
        // RX_LEN (and the flag again: that entry finds it taken)
        let mut rx_len_ok = true;
        if busy && st.token() == UsbToken::OUT && isochronous(inner, slot(st.endp() as usize, false)) {
            wait_sie(regs);
            let now = regs.int_st().read();
            len = regs.rx_len().read();
            // 0: it completed before the release, which cleared RX_LEN; another token:
            // the next transaction already. the slot shows the length
            if len == 0 || now.token() != UsbToken::OUT || now.endp() != st.endp() {
                rx_len_ok = false;
            }
            busy = false;
        } else if busy && st.endp() != 0 && matches!(st.token(), UsbToken::OUT | UsbToken::IN) {
            // bulk / interrupt / iso IN raise their flag before the SIE goes free too: this may
            // be the shown transaction's own entry (`toggled` tells). an OUT completing after
            // the release sets RX_LEN again
            wait_sie(regs);
            busy = false;
            let now = regs.rx_len().read();
            if now != 0 {
                len = now;
            }
        } else if busy && regs.mis_st().read().sie_free() && !regs.int_fg().read().transfer() {
            // it completed before the release, which took its flag (and RX_LEN): handle it
            // here
            busy = false;
            st = regs.int_st().read();
            rx_len_ok = false;
            nak(d, inner, st);
        }
        let ep = st.endp() as usize;
        let token = st.token();
        let iso = match token {
            UsbToken::OUT => isochronous(inner, slot(ep, false)),
            UsbToken::IN => isochronous(inner, slot(ep, true)),
            _ => true,
        };
        let frame = frame(regs);
        // the flag is one bit: a late entry also stands for what completed before the
        // token it shows. the SOF if this microframe's wasn't handled
        if inner.sof_frame != frame && (token != UsbToken::SOF || !busy) {
            inner.sof_frame = frame;
            wake.sof = inner.sof;
        }
        iso_in_sent(regs, d, inner, wake);
        if !busy {
            match token {
                UsbToken::SOF => {}
                // iso OUT: see iso_out_check
                UsbToken::OUT if iso => {}
                // no new packet (a retransmit after a lost ACK, or one NAKed): the endpoint
                // stays as it is
                UsbToken::OUT if ep != 0 && !toggled(regs, d, inner, ep, false) => {}
                UsbToken::OUT if ep == 0 && !st.tog_ok() => {}
                UsbToken::OUT => {
                    if ep == 0 {
                        d.ep_rx_ctrl(0)
                            .modify(|w| w.set_mask_uep_r_tog(flip(w.mask_uep_r_tog())));
                    }
                    inner.out_done(ep, len);
                    wake.endpoints |= 1 << slot(ep, false);
                }
                // iso IN: see iso_in_sent
                UsbToken::IN if iso => {}
                UsbToken::IN if ep != 0 && !toggled(regs, d, inner, ep, true) => {}
                UsbToken::IN => {
                    if ep == 0 {
                        d.ep_tx_ctrl(0)
                            .modify(|w| w.set_mask_uep_t_tog(flip(w.mask_uep_t_tog())));
                    }
                    inner.in_done(ep);
                    wake.endpoints |= 1 << slot(ep, true);
                }
                UsbToken::SETUP => {}
            }
        }
        let entry = (!busy && rx_len_ok && token == UsbToken::OUT && iso && st.tog_ok()).then_some((ep, len));
        // an OUT completes too early in its microframe to merge into the next SOF's entry,
        // and a busy entry takes its own OUT above
        if token != UsbToken::SOF {
            iso_out_check(regs, d, inner, entry, wake);
        }
        if ep == 0 {
            ep0_moved_on(d, inner, token, wake);
        }
    }

    /// An iso IN completion: the loaded packet went out, load the next. Interrupt only.
    fn iso_in_complete(d: pac::usbhs::Usbd, inner: &mut Inner, ep: usize, wake: &mut Wake) {
        let endpoint = &mut inner.endpoints[slot(ep, true)];
        let Some(config) = endpoint.config else {
            return;
        };
        wake.endpoints |= 1 << slot(ep, true);
        match iso::in_next(endpoint, &config) {
            Some((address, length)) => {
                d.ep_tx_dma(ep - 1).write_value(address as u32);
                d.ep_t_len(ep).write(|w| w.set_len(length));
            }
            None => d.ep_t_len(ep).write(|w| w.set_len(0)),
        }
        d.ep_tx_ctrl(ep).modify(|w| w.set_mask_uep_t_tog(EpTog::DATA0));
    }

    /// The iso IN endpoints whose packet went out: with T_TOG_AUTO the SIE moves T_TOG on from
    /// DATA0 once it sends one, so the first entry after the IN loads the next (the flag is
    /// one bit: its completion may show only as a later token). Not while an IN to the
    /// endpoint is on the bus: it is sending that packet again.
    fn iso_in_sent(regs: Regs, d: pac::usbhs::Usbd, inner: &mut Inner, wake: &mut Wake) {
        let mut eps = inner.iso_in;
        while eps != 0 {
            let ep = eps.trailing_zeros() as usize;
            eps &= eps - 1;
            let endpoint = &inner.endpoints[slot(ep, true)];
            if !endpoint.enabled || endpoint.stalled || d.ep_tx_ctrl(ep).read().mask_uep_t_tog() == EpTog::DATA0 {
                continue;
            }
            if !regs.mis_st().read().sie_free() {
                let st = regs.int_st().read();
                if st.token() == UsbToken::IN && st.endp() as usize == ep {
                    continue;
                }
            }
            iso_in_complete(d, inner, ep, wake);
        }
    }

    /// Isochronous OUT arrivals, by the slot's first word: the endpoint's slot gets
    /// `ISO_OUT_MARK` when it's armed, so a packet in it shows whatever INT_ST shows -- with
    /// the flag one bit, a late entry may stand for an OUT and show the transaction after it.
    /// (a packet that starts with the mark is lost; zero-length ones aren't seen.)
    fn iso_out_check(regs: Regs, d: pac::usbhs::Usbd, inner: &mut Inner, entry: Option<(usize, u16)>, wake: &mut Wake) {
        let mut eps = inner.iso_out;
        while eps != 0 {
            let ep = eps.trailing_zeros() as usize;
            eps &= eps - 1;
            let endpoint = &mut inner.endpoints[slot(ep, false)];
            // a disabled endpoint's slot still holds its last packet: it is not an arrival
            let Some(config) = endpoint.config.filter(|_| endpoint.enabled) else {
                continue;
            };
            let armed = config.packet(endpoint.head);
            // SAFETY: the armed slot, a buffer of this endpoint
            if unsafe { core::ptr::read_volatile(armed as *const u32) } == ISO_OUT_MARK {
                continue;
            }
            let max = config.info.max_packet_size;
            let len = match entry {
                // RX_LEN as this packet's entry read it (releasing the flag clears it)
                Some((e, len)) if e == ep => len,
                _ => {
                    // read after the slot: while its OUT is the last token, the packet may
                    // still be arriving (or its DMA finishing), and its own entry follows
                    let st = regs.int_st().read();
                    if st.token() == UsbToken::OUT && st.endp() as usize == ep {
                        continue;
                    }
                    marked_len(armed, usize::from(max)) as u16
                }
            };
            if len != 0 && len < endpoint.min_len {
                endpoint.min_len = len;
            }
            // what the slot to arm holds (marks past it are still there)
            let dirty = if iso::out_push(endpoint, &config, len) {
                config.length(endpoint.head)
            } else {
                // full: the next packet overwrites this slot
                len
            };
            let from = usize::from(endpoint.min_len);
            iso_out_arm(d, ep, config.packet(endpoint.head), from, usize::from(dirty.min(max)));
            wake.endpoints |= 1 << slot(ep, false);
        }
    }

    /// The length of a packet in a slot filled with `ISO_OUT_MARK` before it: up to its last
    /// byte that differs from the pattern.
    fn marked_len(slot: usize, max: usize) -> usize {
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

    /// Point isochronous OUT endpoint `ep` at the slot at `address`, marked empty: its first
    /// word and its bytes `from..dirty` are marked again -- past `dirty` they still are, and
    /// below `from` (no packet is that short) the next packet overwrites them.
    fn iso_out_arm(d: pac::usbhs::Usbd, ep: usize, address: usize, from: usize, dirty: usize) {
        let words = address as *mut u32;
        // SAFETY: a slot of this endpoint, not the reader's, word aligned
        unsafe { core::ptr::write_volatile(words, ISO_OUT_MARK) };
        for i in (from / 4).max(1)..dirty.div_ceil(4) {
            unsafe { core::ptr::write_volatile(words.add(i), ISO_OUT_MARK) };
        }
        d.ep_rx_dma(ep - 1).write_value(address as u32);
    }

    fn wait_sie(regs: Regs) {
        let mut n = 0;
        while !regs.mis_st().read().sie_free() && n < SIE_WAIT {
            n += 1;
        }
    }

    /// Bulk / interrupt / control: NAK before releasing the flag, so the next transaction
    /// waits for the task.
    fn nak(d: pac::usbhs::Usbd, inner: &Inner, st: IntSt) {
        let ep = st.endp() as usize;
        match st.token() {
            // bulk / interrupt: before knowing whether a packet went through (`toggled`), so the
            // next can't land on it
            UsbToken::OUT if (ep == 0 && st.tog_ok()) || (ep != 0 && !isochronous(inner, slot(ep, false))) => {
                d.ep_rx_ctrl(ep).modify(|w| {
                    if w.mask_uep_r_res() == EpRxResponse::ACK {
                        w.set_mask_uep_r_res(EpRxResponse::NAK)
                    }
                })
            }
            UsbToken::IN if !isochronous(inner, slot(ep, true)) => d.ep_tx_ctrl(ep).modify(|w| {
                if w.mask_uep_t_res() == EpTxResponse::ACK {
                    w.set_mask_uep_t_res(EpTxResponse::NAK)
                }
            }),
            _ => {}
        }
    }

    /// A packet went through a bulk / interrupt endpoint (not EP0): the SIE flips its toggle on
    /// each one it accepts (OUT) or the host ACKs (IN). INT_ST's TOG_OK can't tell: the flag comes
    /// a little before the SIE goes free, and an entry that early reads it before it's final.
    /// Waits for the SIE. The entry NAKed the endpoint; a read / write still waiting gets it back.
    fn toggled(regs: Regs, d: pac::usbhs::Usbd, inner: &Inner, ep: usize, incoming: bool) -> bool {
        wait_sie(regs);
        let tog = if incoming {
            d.ep_tx_ctrl(ep).read().mask_uep_t_tog()
        } else {
            d.ep_rx_ctrl(ep).read().mask_uep_r_tog()
        } == EpTog::DATA1;
        let bit = 1 << slot(ep, incoming);
        let last = TOG.load(Ordering::Relaxed);
        if (last & bit != 0) == tog {
            let endpoint = &inner.endpoints[slot(ep, incoming)];
            let armed = if incoming {
                endpoint.queued() != 0
            } else {
                endpoint.armed
            };
            if endpoint.enabled && !endpoint.stalled && armed {
                set_res(regs, ep, incoming, Res::Ack);
            }
            return false;
        }
        TOG.store(last ^ bit, Ordering::Relaxed);
        true
    }

    /// An EP0 stage whose completion no entry saw: the flag is one bit and INT_ST shows the
    /// latest token, NAKed ones too, so a host polling a bulk IN right after an EP0 transaction
    /// hides it (EP0 has no auto toggle to tell). The host's token for the other direction says
    /// it went through: the status stage after the last data packet. (one hidden before the
    /// last gets resent with the same toggle, which the host drops, and completes then.)
    fn ep0_moved_on(d: pac::usbhs::Usbd, inner: &mut Inner, token: UsbToken, wake: &mut Wake) {
        match token {
            UsbToken::OUT if inner.endpoints[slot(0, true)].queued() != 0 => {
                d.ep_tx_ctrl(0).modify(|w| {
                    w.set_mask_uep_t_res(EpTxResponse::NAK);
                    w.set_mask_uep_t_tog(flip(w.mask_uep_t_tog()));
                });
                inner.in_done(0);
                wake.endpoints |= 1 << slot(0, true);
            }
            UsbToken::IN
                if inner.endpoints[0].enabled
                    && inner.endpoints[0].queued() == 0
                    && inner.endpoints[0].rx_wanted > 0 =>
            {
                d.ep_rx_ctrl(0).modify(|w| {
                    w.set_mask_uep_r_res(EpRxResponse::NAK);
                    w.set_mask_uep_r_tog(flip(w.mask_uep_r_tog()));
                });
                let len = inner.endpoints[0].rx_wanted;
                inner.out_done(0, len);
                wake.endpoints |= 1;
            }
            _ => {}
        }
    }
}

pub(crate) use imp::*;
