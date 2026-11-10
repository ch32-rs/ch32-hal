//! Register-view adapter for the OTG-FS register models.
//!
//! `otg_v2` (CH32V4x7), `usbfs_h4` (CH32H417) and `usb_x0fs` (CH32X035)
//! describe the same FS device block at different kind/version pairs. H4
//! keeps the RM's `UC_`/`UIE_`/`UIF_`/`MASK_UEP_` field prefixes and X035
//! groups endpoints into arrays (`UEP0123_DMA`, `UEP567_CTRL`, …), so the
//! driver goes through this module instead of touching the PAC directly.

#[cfg(otg_v2)]
pub(crate) use crate::pac::otg::vals::{EpRxResponse, EpTxResponse, UsbToken};
#[cfg(otg_v2)]
pub(crate) use crate::pac::otg::Usbd as Regs;
#[cfg(usb_x0fs)]
pub(crate) use crate::pac::usb::vals::{EpRxResponse, EpTxResponse, UsbToken};
#[cfg(usb_x0fs)]
pub(crate) use crate::pac::usb::Usbd as Regs;
#[cfg(usbfs_h4)]
pub(crate) use crate::pac::usbfs::vals::{EpRxResponse, EpTxResponse, UsbToken};
#[cfg(usbfs_h4)]
pub(crate) use crate::pac::usbfs::UsbfsDevice as Regs;

/// Interrupt-flag snapshot (`INT_FG`).
#[derive(Copy, Clone)]
pub(crate) struct IntFg {
    pub fifo_ov: bool,
    pub suspend: bool,
    pub transfer: bool,
    pub bus_rst: bool,
    pub hst_sof: bool,
}

/// Interrupt-status snapshot (`INT_ST`): the token and endpoint it refers to.
#[derive(Copy, Clone)]
pub(crate) struct IntSt {
    pub token: UsbToken,
    pub ep: u8,
    /// `TOG_OK`: the received packet's data toggle matched the endpoint.
    /// H4's register view has no such bit, so it always reads as `true`.
    pub tog_ok: bool,
}

#[cfg(otg_v2)]
mod imp {
    use embassy_usb_driver::Direction;

    use super::{IntFg, IntSt, Regs};
    use crate::pac::otg::vals::{EpRxResponse, EpTxResponse};

    pub(crate) fn int_fg(regs: Regs) -> IntFg {
        let v = regs.int_fg().read();
        IntFg {
            fifo_ov: v.fifo_ov(),
            suspend: v.suspend(),
            transfer: v.transfer(),
            bus_rst: v.bus_rst(),
            hst_sof: v.hst_sof(),
        }
    }

    pub(crate) fn clear_transfer(regs: Regs) {
        regs.int_fg().write(|v| v.set_transfer(true));
    }

    pub(crate) fn clear_suspend(regs: Regs) {
        regs.int_fg().write(|v| v.set_suspend(true));
    }

    pub(crate) fn clear_bus_rst(regs: Regs) {
        regs.int_fg().write(|v| v.set_bus_rst(true));
    }

    pub(crate) fn clear_hst_sof(regs: Regs) {
        regs.int_fg().write(|v| v.set_hst_sof(true));
    }

    pub(crate) fn int_st(regs: Regs) -> IntSt {
        let v = regs.int_st().read();
        IntSt {
            token: v.mask_token(),
            ep: v.mask_uis_endp(),
            tog_ok: v.tog_ok(),
        }
    }

    pub(crate) fn reset_sie(regs: Regs) {
        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });
    }

    pub(crate) fn clear_ctrl(regs: Regs) {
        regs.ctrl().write(|_| {});
    }

    pub(crate) fn enable_interrupts(regs: Regs) {
        regs.int_en().write(|w| {
            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });
    }

    pub(crate) fn enter_device_mode(regs: Regs) {
        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            w.set_dev_pu_en(true);
        });
    }

    pub(crate) fn enable_port(regs: Regs) {
        regs.udev_ctrl().write(|w| {
            // Pull-down is host mode only.
            w.set_pd_dis(true);
            w.set_port_en(true);
        });
    }

    pub(crate) fn is_suspended(regs: Regs) -> bool {
        regs.mis_st().read().suspend()
    }

    pub(crate) fn rx_len(regs: Regs) -> u16 {
        regs.rx_len().read().rx_len()
    }

    pub(crate) fn set_ep_dma(regs: Regs, ep: usize, addr: u32) {
        regs.uep_dma(ep).write_value(addr);
    }

    pub(crate) fn set_ep_t_len(regs: Regs, ep: usize, len: u16) {
        regs.uep_t_len(ep).write(|v| v.set_t_len(len));
    }

    pub(crate) fn ep_tx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).write(|v| {
            v.set_t_tog(tog);
            v.set_mask_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).modify(|v| {
            v.set_t_tog(!v.t_tog());
            v.set_mask_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_set_res(regs: Regs, ep: usize, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).modify(|v| v.set_mask_t_res(res));
    }

    pub(crate) fn ep_tx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        regs.uep_tx_ctrl(ep).modify(|v| v.set_t_tog(tog));
    }

    pub(crate) fn ep_rx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).write(|v| {
            v.set_r_tog(tog);
            v.set_mask_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).modify(|v| {
            v.set_r_tog(!v.r_tog());
            v.set_mask_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_set_res(regs: Regs, ep: usize, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).modify(|v| v.set_mask_r_res(res));
    }

    pub(crate) fn ep_rx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        regs.uep_rx_ctrl(ep).modify(|v| v.set_r_tog(tog));
    }

    pub(crate) fn ep_is_enabled(regs: Regs, ep: usize, dir: Direction) -> bool {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().read().uep4_tx_en(),
            (4, Direction::Out) => regs.uep4_1_mod().read().uep4_rx_en(),
            (1, Direction::In) => regs.uep4_1_mod().read().uep1_tx_en(),
            (1, Direction::Out) => regs.uep4_1_mod().read().uep1_rx_en(),
            (2, Direction::In) => regs.uep2_3_mod().read().uep2_tx_en(),
            (2, Direction::Out) => regs.uep2_3_mod().read().uep2_rx_en(),
            (3, Direction::In) => regs.uep2_3_mod().read().uep3_tx_en(),
            (3, Direction::Out) => regs.uep2_3_mod().read().uep3_rx_en(),
            (5, Direction::In) => regs.uep5_6_mod().read().uep5_tx_en(),
            (5, Direction::Out) => regs.uep5_6_mod().read().uep5_rx_en(),
            (6, Direction::In) => regs.uep5_6_mod().read().uep6_tx_en(),
            (6, Direction::Out) => regs.uep5_6_mod().read().uep6_rx_en(),
            (7, Direction::In) => regs.uep7_mod().read().uep7_tx_en(),
            (7, Direction::Out) => regs.uep7_mod().read().uep7_rx_en(),
            _ => panic!("Unsupported EP"),
        }
    }

    pub(crate) fn ep_set_enabled(regs: Regs, ep: usize, dir: Direction, enabled: bool) {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep4_tx_en(enabled)),
            (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep4_rx_en(enabled)),
            (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep1_tx_en(enabled)),
            (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep1_rx_en(enabled)),
            (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep2_tx_en(enabled)),
            (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep2_rx_en(enabled)),
            (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep3_tx_en(enabled)),
            (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep3_rx_en(enabled)),
            (5, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep5_tx_en(enabled)),
            (5, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep5_rx_en(enabled)),
            (6, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep6_tx_en(enabled)),
            (6, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep6_rx_en(enabled)),
            (7, Direction::In) => regs.uep7_mod().modify(|v| v.set_uep7_tx_en(enabled)),
            (7, Direction::Out) => regs.uep7_mod().modify(|v| v.set_uep7_rx_en(enabled)),
            _ => panic!("setting non-existent endpoint"),
        }
    }
}

#[cfg(usbfs_h4)]
mod imp {
    use embassy_usb_driver::Direction;

    use super::{IntFg, IntSt, Regs};
    use crate::pac::usbfs::vals::{EpRxResponse, EpTxResponse};

    pub(crate) fn int_fg(regs: Regs) -> IntFg {
        let v = regs.usb_int_fg().read();
        IntFg {
            fifo_ov: v.uif_fifo_ov(),
            suspend: v.uif_suspend(),
            transfer: v.uif_transfer(),
            bus_rst: v.uif_bus_rst(),
            hst_sof: v.uif_hst_sof(),
        }
    }

    pub(crate) fn clear_transfer(regs: Regs) {
        regs.usb_int_fg().write(|v| v.set_uif_transfer(true));
    }

    pub(crate) fn clear_suspend(regs: Regs) {
        regs.usb_int_fg().write(|v| v.set_uif_suspend(true));
    }

    pub(crate) fn clear_bus_rst(regs: Regs) {
        regs.usb_int_fg().write(|v| v.set_uif_bus_rst(true));
    }

    pub(crate) fn clear_hst_sof(regs: Regs) {
        regs.usb_int_fg().write(|v| v.set_uif_hst_sof(true));
    }

    pub(crate) fn int_st(regs: Regs) -> IntSt {
        let v = regs.usb_int_st().read();
        IntSt {
            token: v.mask_uis_token(),
            ep: v.mask_uis_endp(),
            // H4's USBFS register view has no TOG_OK field.
            tog_ok: true,
        }
    }

    pub(crate) fn reset_sie(regs: Regs) {
        regs.usb_ctrl().write(|w| {
            w.set_uc_clr_all(true);
            w.set_uc_rst_sie(true);
        });
    }

    pub(crate) fn clear_ctrl(regs: Regs) {
        regs.usb_ctrl().write(|_| {});
    }

    pub(crate) fn enable_interrupts(regs: Regs) {
        regs.usb_int_en().write(|w| {
            w.set_uie_suspend(true);
            w.set_uie_transfer(true);
            w.set_uie_bus_rst(true);
        });
    }

    pub(crate) fn enter_device_mode(regs: Regs) {
        regs.usb_ctrl().write(|w| {
            w.set_uc_int_busy(true);
            w.set_uc_dma_en(true);
            w.set_uc_dev_pu_en(true);
        });
    }

    pub(crate) fn enable_port(regs: Regs) {
        regs.udev_ctrl().write(|w| {
            // Pull-down is host mode only.
            w.set_ud_pd_dis(true);
            w.set_ud_port_en(true);
        });
    }

    pub(crate) fn is_suspended(regs: Regs) -> bool {
        regs.usb_mis_st().read().ums_suspend()
    }

    pub(crate) fn rx_len(regs: Regs) -> u16 {
        regs.usb_rx_len().read()
    }

    pub(crate) fn set_ep_dma(regs: Regs, ep: usize, addr: u32) {
        regs.uep_dma(ep).write_value(addr);
    }

    pub(crate) fn set_ep_t_len(regs: Regs, ep: usize, len: u16) {
        regs.uep_t_len(ep).write(|v| v.set_uep_t_len(len));
    }

    pub(crate) fn ep_tx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).write(|v| {
            v.set_uep_t_tog(tog);
            v.set_mask_uep_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).modify(|v| {
            v.set_uep_t_tog(!v.uep_t_tog());
            v.set_mask_uep_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_set_res(regs: Regs, ep: usize, res: EpTxResponse) {
        regs.uep_tx_ctrl(ep).modify(|v| v.set_mask_uep_t_res(res));
    }

    pub(crate) fn ep_tx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        regs.uep_tx_ctrl(ep).modify(|v| v.set_uep_t_tog(tog));
    }

    pub(crate) fn ep_rx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).write(|v| {
            v.set_uep_r_tog(tog);
            v.set_mask_uep_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).modify(|v| {
            v.set_uep_r_tog(!v.uep_r_tog());
            v.set_mask_uep_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_set_res(regs: Regs, ep: usize, res: EpRxResponse) {
        regs.uep_rx_ctrl(ep).modify(|v| v.set_mask_uep_r_res(res));
    }

    pub(crate) fn ep_rx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        regs.uep_rx_ctrl(ep).modify(|v| v.set_uep_r_tog(tog));
    }

    pub(crate) fn ep_is_enabled(regs: Regs, ep: usize, dir: Direction) -> bool {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().read().uep4_tx_en(),
            (4, Direction::Out) => regs.uep4_1_mod().read().uep4_rx_en(),
            (1, Direction::In) => regs.uep4_1_mod().read().uep1_tx_en(),
            (1, Direction::Out) => regs.uep4_1_mod().read().uep1_rx_en(),
            (2, Direction::In) => regs.uep2_3_mod().read().uep2_tx_en(),
            (2, Direction::Out) => regs.uep2_3_mod().read().uep2_rx_en(),
            (3, Direction::In) => regs.uep2_3_mod().read().uep3_tx_en(),
            (3, Direction::Out) => regs.uep2_3_mod().read().uep3_rx_en(),
            (5, Direction::In) => regs.uep5_6_mod().read().uep5_tx_en(),
            (5, Direction::Out) => regs.uep5_6_mod().read().uep5_rx_en(),
            (6, Direction::In) => regs.uep5_6_mod().read().uep6_tx_en(),
            (6, Direction::Out) => regs.uep5_6_mod().read().uep6_rx_en(),
            (7, Direction::In) => regs.uep7_mod().read().uep7_tx_en(),
            (7, Direction::Out) => regs.uep7_mod().read().uep7_rx_en(),
            _ => panic!("Unsupported EP"),
        }
    }

    pub(crate) fn ep_set_enabled(regs: Regs, ep: usize, dir: Direction, enabled: bool) {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep4_tx_en(enabled)),
            (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep4_rx_en(enabled)),
            (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_uep1_tx_en(enabled)),
            (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_uep1_rx_en(enabled)),
            (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep2_tx_en(enabled)),
            (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep2_rx_en(enabled)),
            (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_uep3_tx_en(enabled)),
            (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_uep3_rx_en(enabled)),
            (5, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep5_tx_en(enabled)),
            (5, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep5_rx_en(enabled)),
            (6, Direction::In) => regs.uep5_6_mod().modify(|v| v.set_uep6_tx_en(enabled)),
            (6, Direction::Out) => regs.uep5_6_mod().modify(|v| v.set_uep6_rx_en(enabled)),
            (7, Direction::In) => regs.uep7_mod().modify(|v| v.set_uep7_tx_en(enabled)),
            (7, Direction::Out) => regs.uep7_mod().modify(|v| v.set_uep7_rx_en(enabled)),
            _ => panic!("setting non-existent endpoint"),
        }
    }
}

/// X035's `x0fs` layout groups endpoints into `UEP0123`/`UEP567` arrays and
/// keeps the TX/RX response bits in one 8-bit `UEPn_CTRL` byte.
#[cfg(usb_x0fs)]
mod imp {
    use embassy_usb_driver::Direction;

    use super::{IntFg, IntSt, Regs};
    use crate::pac::usb::regs::UepCtrl;
    use crate::pac::usb::vals::{EpRxResponse, EpTxResponse};

    pub(crate) fn int_fg(regs: Regs) -> IntFg {
        let v = regs.int_fg().read();
        IntFg {
            fifo_ov: v.fifo_ov(),
            suspend: v.suspend(),
            transfer: v.transfer(),
            bus_rst: v.bus_rst(),
            hst_sof: v.hst_sof(),
        }
    }

    pub(crate) fn clear_transfer(regs: Regs) {
        regs.int_fg().write(|v| v.set_transfer(true));
    }

    pub(crate) fn clear_suspend(regs: Regs) {
        regs.int_fg().write(|v| v.set_suspend(true));
    }

    pub(crate) fn clear_bus_rst(regs: Regs) {
        regs.int_fg().write(|v| v.set_bus_rst(true));
    }

    pub(crate) fn clear_hst_sof(regs: Regs) {
        regs.int_fg().write(|v| v.set_hst_sof(true));
    }

    pub(crate) fn int_st(regs: Regs) -> IntSt {
        let v = regs.int_st().read();
        IntSt {
            token: v.mask_token(),
            ep: v.mask_uis_endp(),
            tog_ok: v.tog_ok(),
        }
    }

    pub(crate) fn reset_sie(regs: Regs) {
        regs.ctrl().write(|w| {
            w.set_clr_all(true);
            w.set_reset_sie(true);
        });
    }

    pub(crate) fn clear_ctrl(regs: Regs) {
        regs.ctrl().write(|_| {});
    }

    pub(crate) fn enable_interrupts(regs: Regs) {
        regs.int_en().write(|w| {
            w.set_suspend(true);
            w.set_transfer(true);
            w.set_bus_rst(true);
        });
    }

    pub(crate) fn enter_device_mode(regs: Regs) {
        regs.ctrl().write(|w| {
            w.set_int_busy(true);
            w.set_dma_en(true);
            // `SYS_CTRL=0b10` enables the device controller and its
            // internal pull-up (X035RM `CTLR` bits [5:4]).
            w.set_sys_ctrl(0b10);
        });
    }

    pub(crate) fn enable_port(regs: Regs) {
        regs.udev_ctrl().write(|w| {
            // Pull-down is host mode only.
            w.set_pd_dis(true);
            w.set_port_en(true);
        });
    }

    pub(crate) fn is_suspended(regs: Regs) -> bool {
        regs.mis_st().read().suspend()
    }

    pub(crate) fn rx_len(regs: Regs) -> u16 {
        regs.rx_len().read().rx_len()
    }

    pub(crate) fn set_ep_dma(regs: Regs, ep: usize, addr: u32) {
        match ep {
            0..=3 => regs.uep0123_dma(ep).write(|v| v.set_buf_addr(addr as u16)),
            // EP4 shares EP0's DMA buffer on X0.
            4 => todo!("EP4 shares the EP0 DMA buffer"),
            5..=7 => regs.uep567_dma(ep - 5).write(|v| v.set_buf_addr(addr as u16)),
            _ => panic!("unsupported USBFS endpoint"),
        }
    }

    pub(crate) fn set_ep_t_len(regs: Regs, ep: usize, len: u16) {
        debug_assert!(len <= u8::MAX as u16);
        match ep {
            0..=4 => regs.uep01234_t_len(ep).write(|v| v.set_t_len(len as u8)),
            5..=7 => regs.uep567_t_len(ep - 5).write(|v| v.set_t_len(len as u8)),
            _ => panic!("unsupported USBFS endpoint"),
        }
    }

    pub(crate) fn ep_tx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpTxResponse) {
        modify_ep_ctrl(regs, ep, |v| {
            v.set_t_tog(tog);
            v.set_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpTxResponse) {
        modify_ep_ctrl(regs, ep, |v| {
            v.set_t_tog(!v.t_tog());
            v.set_t_res(res);
        });
    }

    pub(crate) fn ep_tx_ctrl_set_res(regs: Regs, ep: usize, res: EpTxResponse) {
        modify_ep_ctrl(regs, ep, |v| v.set_t_res(res));
    }

    pub(crate) fn ep_tx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        modify_ep_ctrl(regs, ep, |v| v.set_t_tog(tog));
    }

    pub(crate) fn ep_rx_ctrl_write(regs: Regs, ep: usize, tog: bool, res: EpRxResponse) {
        modify_ep_ctrl(regs, ep, |v| {
            v.set_r_tog(tog);
            v.set_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_flip_tog(regs: Regs, ep: usize, res: EpRxResponse) {
        modify_ep_ctrl(regs, ep, |v| {
            v.set_r_tog(!v.r_tog());
            v.set_r_res(res);
        });
    }

    pub(crate) fn ep_rx_ctrl_set_res(regs: Regs, ep: usize, res: EpRxResponse) {
        modify_ep_ctrl(regs, ep, |v| v.set_r_res(res));
    }

    pub(crate) fn ep_rx_ctrl_set_tog(regs: Regs, ep: usize, tog: bool) {
        modify_ep_ctrl(regs, ep, |v| v.set_r_tog(tog));
    }

    pub(crate) fn ep_is_enabled(regs: Regs, ep: usize, dir: Direction) -> bool {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().read().tx_en(0),
            (4, Direction::Out) => regs.uep4_1_mod().read().rx_en(0),
            (1, Direction::In) => regs.uep4_1_mod().read().tx_en(1),
            (1, Direction::Out) => regs.uep4_1_mod().read().rx_en(1),
            (2, Direction::In) => regs.uep2_3_mod().read().tx_en(0),
            (2, Direction::Out) => regs.uep2_3_mod().read().rx_en(0),
            (3, Direction::In) => regs.uep2_3_mod().read().tx_en(1),
            (3, Direction::Out) => regs.uep2_3_mod().read().rx_en(1),
            (5, Direction::In) => regs.uep567_mod().read().tx_en(0),
            (5, Direction::Out) => regs.uep567_mod().read().rx_en(0),
            (6, Direction::In) => regs.uep567_mod().read().tx_en(1),
            (6, Direction::Out) => regs.uep567_mod().read().rx_en(1),
            (7, Direction::In) => regs.uep567_mod().read().tx_en(2),
            (7, Direction::Out) => regs.uep567_mod().read().rx_en(2),
            _ => panic!("Unsupported EP"),
        }
    }

    pub(crate) fn ep_set_enabled(regs: Regs, ep: usize, dir: Direction, enabled: bool) {
        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(0, enabled)),
            (4, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(0, enabled)),
            (1, Direction::In) => regs.uep4_1_mod().modify(|v| v.set_tx_en(1, enabled)),
            (1, Direction::Out) => regs.uep4_1_mod().modify(|v| v.set_rx_en(1, enabled)),
            (2, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(0, enabled)),
            (2, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(0, enabled)),
            (3, Direction::In) => regs.uep2_3_mod().modify(|v| v.set_tx_en(1, enabled)),
            (3, Direction::Out) => regs.uep2_3_mod().modify(|v| v.set_rx_en(1, enabled)),
            (5, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(0, enabled)),
            (5, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(0, enabled)),
            (6, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(1, enabled)),
            (6, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(1, enabled)),
            (7, Direction::In) => regs.uep567_mod().modify(|v| v.set_tx_en(2, enabled)),
            (7, Direction::Out) => regs.uep567_mod().modify(|v| v.set_rx_en(2, enabled)),
            _ => panic!("setting non-existent endpoint"),
        }
    }

    fn modify_ep_ctrl(regs: Regs, ep: usize, f: impl FnOnce(&mut UepCtrl)) {
        match ep {
            0..=4 => regs.uep01234_ctrl(ep).modify(f),
            5..=7 => regs.uep567_ctrl(ep - 5).modify(f),
            _ => panic!("unsupported USBFS endpoint"),
        }
    }
}

pub(crate) use imp::*;

/// Set the device address in `DEV_AD`.
pub(crate) fn set_device_address(regs: Regs, addr: u8) {
    #[cfg(otg_v2)]
    regs.dev_ad().write(|v| v.set_mask_usb_addr(addr));
    #[cfg(usbfs_h4)]
    regs.usb_dev_ad().write(|v| v.set_mask_usb_addr(addr));
    #[cfg(usb_x0fs)]
    regs.dev_ad().write(|v| v.set_mask_usb_addr(addr));
}
