//! Register-view adapter for the two OTG-FS register models.
//!
//! `otg_v2` (CH32V4x7) and `usbfs_h4` (CH32H417) describe the same FS device
//! block at different kind/version pairs. H4 keeps the RM's `UC_`/`UIE_`/
//! `UIF_`/`MASK_UEP_` field prefixes while `otg_v2` uses bare names, so the
//! driver goes through this module instead of touching the PAC directly.

#[cfg(otg_v2)]
pub(crate) use crate::pac::otg::vals::{EpRxResponse, EpTxResponse, UsbToken};
#[cfg(otg_v2)]
pub(crate) use crate::pac::otg::Usbd as Regs;
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
}

#[cfg(otg_v2)]
mod imp {
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
        regs.uep_t_len(ep).write_value(len as u8);
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
}

#[cfg(usbfs_h4)]
mod imp {
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
}

pub(crate) use imp::*;

/// Set the device address in `DEV_AD`.
pub(crate) fn set_device_address(regs: Regs, addr: u8) {
    #[cfg(otg_v2)]
    regs.dev_ad().write(|v| v.set_mask_usb_addr(addr));
    #[cfg(usbfs_h4)]
    regs.usb_dev_ad().write(|v| v.set_mask_usb_addr(addr));
}
