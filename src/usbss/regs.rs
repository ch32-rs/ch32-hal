//! Register-level helpers. Nothing here keeps state.

use core::ops::Deref;

use super::device::ENDPOINTS;
use super::{LMP_LINK_SPEED, LMP_PORT_CAP, LMP_PORT_CFG_RES, NUM_HP_BUF, UP_STREAM};
use crate::pac::usbss::vals::{Ep0RxRes, Ep0TxRes, PdMode, TxDeemph};
use crate::pac::usbss::{regs, EpRx, EpTx, UsbssDevice};

/// Hot Reset bit of `LINK_CTRL.LINK_TX_TS_CFG`.
pub(super) const LINK_HOT_RESET: u8 = 1;

/// The device register block.
#[derive(Clone, Copy)]
pub(super) struct Regs(pub UsbssDevice);

impl Deref for Regs {
    type Target = UsbssDevice;

    fn deref(&self) -> &UsbssDevice {
        &self.0
    }
}

impl Regs {
    /// IN registers of endpoint `n` (1..=7).
    pub fn tx(self, n: usize) -> EpTx {
        self.ep_tx(n - 1)
    }

    /// OUT registers of endpoint `n` (1..=7).
    pub fn rx(self, n: usize) -> EpRx {
        self.ep_rx(n - 1)
    }

    pub fn configure(self, index: usize, incoming: bool, iso: bool) {
        if incoming {
            self.tx(index).cfg().write(|w| {
                w.set_chain_auto(true);
                w.set_erdy_auto(true);
                w.set_seq_auto(true);
                w.set_iso_mode(iso);
            });
        } else {
            self.rx(index).cfg().write(|w| {
                w.set_chain_auto(true);
                w.set_erdy_auto(true);
                w.set_seq_auto(true);
                w.set_iso_mode(iso);
            });
        }
        self.clear(index, incoming);
    }

    /// Drops the endpoint's chain, halt state, sequence number and
    /// `ERDY_NUMP`. The controller ignores a `CR` write right after this one.
    pub fn clear(self, index: usize, incoming: bool) {
        if incoming {
            let tx = self.tx(index);
            tx.cr().write(|w| {
                w.set_clr(true);
                w.set_chain_clr(true);
            });
            tx.st().write(|w| w.set_int_flag(true));
            tx.chain_st().write(|w| w.set_chain_if(true));
        } else {
            let rx = self.rx(index);
            rx.cr().write(|w| {
                w.set_clr(true);
                w.set_chain_clr(true);
            });
            rx.st().write(|w| w.set_int_flag(true));
            rx.chain_st().write(|w| w.set_chain_if(true));
        }
    }

    /// Stops the endpoint's chain. Unlike [`Self::clear`] the sequence number
    /// stays in step with the host.
    pub fn abort(self, index: usize, incoming: bool) {
        // The ERDY is for a host left in flow control without the controller
        // knowing, which then sends none for the next chain.
        if incoming {
            let tx = self.tx(index);
            tx.cr().modify(|w| w.set_chain_clr(true));
            tx.chain_st().write(|w| w.set_chain_if(true));
            tx.st().write(|w| w.set_erdy_req(true));
        } else {
            let rx = self.rx(index);
            rx.cr().modify(|w| w.set_chain_clr(true));
            rx.chain_st().write(|w| w.set_chain_if(true));
            rx.st().write(|w| w.set_erdy_req(true));
        }
    }

    /// Sets `ERDY_NUMP` of OUT endpoint `index`: the host may have
    /// `nump - 1` packets waiting for their ACK.
    pub fn set_out_window(self, index: usize, nump: u8) {
        let cr = self.rx(index).cr();
        if cr.read().erdy_nump() != nump {
            cr.modify(|w| w.set_erdy_nump(nump));
        }
    }

    pub fn halt(self, index: usize, incoming: bool) {
        // ERDY wakes a flow-controlled endpoint so the host sees the STALL.
        if incoming {
            let tx = self.tx(index);
            tx.cr().modify(|w| w.set_halt(true));
            tx.st().write(|w| w.set_erdy_req(true));
        } else {
            let rx = self.rx(index);
            rx.cr().modify(|w| w.set_halt(true));
            rx.st().write(|w| w.set_erdy_req(true));
        }
    }

    /// The endpoint's bit in `UEP_TX_EN` or `UEP_RX_EN`.
    pub fn set_enabled(self, index: usize, incoming: bool, enabled: bool) {
        if incoming {
            self.uep_tx_en().modify(|w| w.set_ep_tx_en(index - 1, enabled));
        } else {
            self.uep_rx_en().modify(|w| w.set_ep_rx_en(index - 1, enabled));
        }
    }

    pub fn ep0_rx_ready(self) {
        self.uep0_rx_ctrl().write(|w| {
            w.set_ep0_rx_res(Ep0RxRes::ACK);
            w.set_ep0_rx_erdy(true);
        });
    }

    pub fn ep0_tx_ready(self, seq: u8, len: u16) {
        self.uep0_tx_ctrl().write(|w| {
            w.set_ep0_tx_res(Ep0TxRes::DPH);
            w.set_ep0_tx_seq(seq);
            w.set_ep0_tx_len(len);
        });
        self.uep0_tx_ctrl().modify(|w| w.set_ep0_tx_erdy(true));
    }

    pub fn ep0_stall(self) {
        self.uep0_tx_ctrl().write(|w| w.set_ep0_tx_res(Ep0TxRes::STALL));
        self.uep0_rx_ctrl().write(|w| {
            w.set_ep0_rx_res(Ep0RxRes::STALL);
            w.set_ep0_rx_erdy(true);
        });
    }

    pub fn phy_write(self, addr: u8, data: u16) {
        self.phy_cfg_cr().write(|w| {
            w.set_phy_cfg_data(data);
            w.set_phy_cfg_addr(addr);
            w.set_phy_cfg_write(true);
        });
        self.phy_cfg_dat().write(|w| w.set_phy_cfg_data(1));
    }

    /// PHY values from the EVT examples. The RM documents neither the indirect
    /// PHY interface nor the `0x5003_C018` write.
    ///
    /// Call this after `LINK_PHY_RESET` has been released. Values written
    /// before it do not survive a cold boot, and the link then drops into
    /// Recovery several times a second.
    pub fn phy_tuning(self) {
        self.phy_write(0x03, 0x7c12);
        self.phy_write(0x0d, 0x79aa);
        self.phy_write(0x15, 0x4430);
        self.phy_write(0x13, 0x0010);
        self.phy_write(0x11, 0x0501);
        unsafe { core::ptr::write_volatile(0x5003_C018 as *mut u32, 0xB005_4000) };
    }

    /// Brings the link layer up as a device. `LINK_IE_RX_SET_FC` exists from
    /// silicon revision 3. The de-emphasis is the 3.5 dB of USB 3.0, where the
    /// EVT examples write the reserved value 3.
    pub fn init(self, revision: u8) {
        self.link_cfg().write(|w| {
            w.set_link_rx_eq_en(true);
            w.set_link_tx_deemph(TxDeemph::DB3_5);
            w.set_link_phy_reset(true);
        });
        self.link_ctrl().write(|w| {
            w.set_link_pd_mode(PdMode::P2);
            w.set_link_go_disabled(true);
        });
        self.link_cfg().write(|w| {
            w.set_link_rx_eq_en(true);
            w.set_link_tx_deemph(TxDeemph::DB3_5);
            w.set_link_ltssm_mode(true);
            w.set_link_tout_mode(true);
        });
        self.link_lpm_cr().modify(|w| w.set_lpm_en(true));
        self.link_cfg().modify(|w| w.set_link_rx_term_en(true));
        self.link_int_ctrl().write(|w| {
            w.set_link_ie_tx_lmp(true);
            w.set_link_ie_rx_lmp(true);
            w.set_link_ie_rx_lmp_tout(true);
            w.set_link_ie_state_chg(true);
            w.set_link_ie_warm_rst(true);
            w.set_link_ie_term_pres(true);
            w.set_link_ie_rx_set_fc(revision >= 3);
        });
        self.link_ctrl().write(|w| w.set_link_pd_mode(PdMode::P2));
        self.link_u1_wkup_tmr().write(|w| w.set_u1_wkup_tmr(120));
        self.link_u1_wkup_filter().write(|w| w.set_u1_wkup_filter(50));
        self.link_u2_wkup_filter().write(|w| w.set_u2_wkup_filter(0));
        self.link_u3_wkup_filter().write(|w| w.set_u3_wkup_filter(0));
        self.phy_tuning();
    }

    /// Takes the device off the bus.
    pub fn shutdown(self) {
        self.usbss_ctrl().write(|w| w.set_force_rst(true));
        self.link_cfg().modify(|w| {
            w.set_link_phy_reset(true);
            w.set_link_reset(true);
        });
        self.link_ctrl().modify(|w| w.set_link_go_disabled(true));
        super::delay_cycles(1000);
        self.usbss_ctrl().modify(|w| w.set_force_rst(false));
        self.link_cfg().modify(|w| {
            w.set_link_phy_reset(false);
            w.set_link_reset(false);
        });
    }

    /// Resets the protocol layer and every endpoint, with EP0 using the 512
    /// bytes at `ep0` and ready for a SETUP.
    pub fn protocol_reset(self, ep0: usize) {
        self.usbss_ctrl().modify(|w| w.set_force_rst(true));
        self.status().write(|w| {
            w.set_uif_transfer(true);
            w.set_udif_setup(true);
            w.set_udif_status(true);
            w.set_uif_rx_ping(true);
            w.set_uif_itp(true);
            w.set_uif_fifo_txov(true);
            w.set_uif_fifo_rxov(true);
        });
        self.usbss_ctrl().write(|w| {
            w.set_uie_transfer(true);
            w.set_udie_setup(true);
            w.set_udie_status(true);
            w.set_dma_en(true);
            w.set_setup_flow(true);
        });

        self.usbss_ctrl().modify(|w| w.set_usb_clr_all(true));
        self.usbss_ctrl().modify(|w| w.set_usb_clr_all(false));
        self.uep_tx_en().write_value(regs::UepTxEn(0));
        self.uep_rx_en().write_value(regs::UepRxEn(0));
        for index in 1..ENDPOINTS {
            self.clear(index, true);
            self.clear(index, false);
        }
        self.uep0_tx_dma().write_value(ep0 as u32);
        self.uep0_rx_dma().write_value(ep0 as u32);
        self.uep0_tx_dma_ofs().write_value(0);
        self.uep0_rx_dma_ofs().write_value(0);
        self.uep0_tx_ctrl().write_value(regs::Uep0TxCtrl(0));
        self.ep0_rx_ready();
        self.usbss_ctrl().modify(|w| w.set_dev_addr(0));
    }

    /// Accept LGO_U1/LGO_U2 when the host forces link power management.
    pub fn link_power_allow(self, allow: bool) {
        self.link_cfg().modify(|w| {
            w.set_link_u1_allow(allow);
            w.set_link_u2_allow(allow);
        });
    }

    pub fn tx_port_cap(self) {
        self.link_lmp_tx_data(0).write_value(LMP_LINK_SPEED | LMP_PORT_CAP);
        self.link_lmp_tx_data(1).write_value(UP_STREAM | NUM_HP_BUF);
        self.link_lmp_tx_data(2).write_value(0);
    }

    pub fn tx_port_cfg_res(self) {
        self.link_lmp_tx_data(0).write_value(LMP_LINK_SPEED | LMP_PORT_CFG_RES);
        self.link_lmp_tx_data(1).write_value(0);
        self.link_lmp_tx_data(2).write_value(0);
        self.link_lmp_port_cap().modify(|w| w.set_link_lmp_tx_cap_vld(true));
    }
}
