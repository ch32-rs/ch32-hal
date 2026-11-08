//! USB 3.0 SuperSpeed device controller (USBSS) — RM §27.
//!
//! The reference manual only documents the register map; the initialization
//! order, PHY tuning values and the link-management handshake follow the WCH
//! EVT `USBSS/DEVICE/CH372Device` example. The controller is a separate IP
//! from `usbhs`: it has its own 125 MHz PLL, PIPE/UTMI gates and a link state
//! machine that must answer Port Capability LMPs before the host will
//! enumerate the device.
//!
//! This module currently brings the link up to U0 and exposes link-state
//! changes. Endpoint and control-transfer support are built on top of it.

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::pac::rcc::vals::UsbsspllRefsel;
use crate::pac::usbss::UsbssDevice;
use crate::{interrupt, Peri, PeripheralType, RccPeripheral};

/// `LINK_STATUS.LINK_STATE` values (RM §27.2.6.5).
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkState {
    U0,
    U1,
    U2,
    U3,
    Disabled,
    RxDetect,
    Inactive,
    Polling,
    Recovery,
    HotReset,
    Compliance,
    Loopback,
    Unknown(u8),
}

impl LinkState {
    fn from_raw(raw: u8) -> Self {
        match raw {
            0 => Self::U0,
            1 => Self::U1,
            2 => Self::U2,
            3 => Self::U3,
            4 => Self::Disabled,
            5 => Self::RxDetect,
            6 => Self::Inactive,
            7 => Self::Polling,
            8 => Self::Recovery,
            9 => Self::HotReset,
            10 => Self::Compliance,
            11 => Self::Loopback,
            other => Self::Unknown(other),
        }
    }
}

const LINK_STATE_UNKNOWN: u8 = 0xff;

// LMP transaction words, from the EVT device example.
const LMP_SUBTYPE_MASK: u32 = 0xf << 5;
const LMP_SET_LINK_FUNC: u32 = 0x1 << 5;
const LMP_U2_INACT_TOUT: u32 = 0x2 << 5;
const LMP_PORT_CAP: u32 = 0x4 << 5;
const LMP_PORT_CFG: u32 = 0x5 << 5;
const LMP_PORT_CFG_RES: u32 = 0x6 << 5;
const LMP_LINK_SPEED: u32 = 1 << 9;
const NUM_HP_BUF: u32 = 4;
const UP_STREAM: u32 = 2 << 16;

struct State {
    link_waker: AtomicWaker,
    link_state: AtomicU8,
    low_power: core::sync::atomic::AtomicBool,
}

impl State {
    const fn new() -> Self {
        Self {
            link_waker: AtomicWaker::new(),
            link_state: AtomicU8::new(LINK_STATE_UNKNOWN),
            low_power: core::sync::atomic::AtomicBool::new(false),
        }
    }
}

trait SealedInstance: RccPeripheral {
    fn device() -> UsbssDevice;
    fn state() -> &'static State;
}

/// USBSS peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType + 'static {
    /// Transaction / endpoint interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
    /// LINK state-machine interrupt.
    type LinkInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usbss, $inst:ident) => {
        use crate::peripherals;

        impl SealedInstance for peripherals::$inst {
            fn device() -> UsbssDevice {
                unsafe { UsbssDevice::from_ptr(crate::pac::$inst.as_ptr()) }
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type LinkInterrupt = crate::_generated::peripheral_interrupts::$inst::LINK;
        }
    };
);

/// GLOBAL interrupt handler (SETUP, STATUS, endpoint transfers).
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        // Endpoint handling is added together with the control pipe.
    }
}

/// LINK interrupt handler.
pub struct LinkInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::LinkInterrupt> for LinkInterruptHandler<T> {
    unsafe fn on_interrupt() {
        handle_link::<T>();
    }
}

/// USBSS device driver.
pub struct Driver<'d, T: Instance> {
    _peri: Peri<'d, T>,
    last_state: u8,
}

impl<'d, T: Instance> Driver<'d, T> {
    /// Initializes the USBSS block and enables both interrupts.
    pub fn new(
        peri: Peri<'d, T>,
        _irqs: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::LinkInterrupt, LinkInterruptHandler<T>>
            + 'd,
    ) -> Self {
        unsafe {
            init::<T>();
            T::Interrupt::enable();
            T::LinkInterrupt::enable();
        }

        let state = T::device().link_status().read().link_state();
        T::state().link_state.store(state, Ordering::Relaxed);

        Self {
            _peri: peri,
            last_state: LINK_STATE_UNKNOWN,
        }
    }

    /// Current link state.
    pub fn link_state(&self) -> LinkState {
        LinkState::from_raw(T::device().link_status().read().link_state())
    }

    /// Waits for the next link-state change.
    pub async fn wait_link_change(&mut self) -> LinkState {
        core::future::poll_fn(|cx| {
            T::state().link_waker.register(cx.waker());
            let raw = T::state().link_state.load(Ordering::Relaxed);
            if raw != self.last_state {
                self.last_state = raw;
                core::task::Poll::Ready(LinkState::from_raw(raw))
            } else {
                core::task::Poll::Pending
            }
        })
        .await
    }
}

unsafe fn init<T: Instance>() {
    // RM §27 has no clock section; the EVT example enables the USBSS PLL,
    // the block clock, the PIPE clock and the UTMI clock in this order.
    crate::rcc::enable_usbss_pll(UsbsspllRefsel::F25MHZ);
    T::enable_and_reset();
    crate::pac::RCC.cfgr0().modify(|w| {
        w.set_pipeon(true);
        w.set_utmion(true);
    });

    let d = T::device();

    // Assert the PHY reset, force DISABLED, then release both and enable
    // terminal detection once the power mode has settled.
    d.link_cfg().write(|w| {
        w.set_link_rx_eq_en(true);
        w.set_link_tx_deemph(3);
        w.set_link_phy_reset(true);
    });
    d.link_ctrl().write(|w| {
        w.set_link_pd_mode(2);
        w.set_link_go_disabled(true);
    });
    d.link_cfg().write(|w| {
        w.set_link_rx_eq_en(true);
        w.set_link_tx_deemph(3);
        w.set_link_ltssm_mode(true);
        w.set_link_tout_mode(true);
    });
    d.link_lpm_cr().modify(|w| w.set_lpm_en(true));
    d.link_cfg().modify(|w| w.set_link_rx_term_en(true));
    d.link_u1_exit_cfg().modify(|w| w.set_u1_exit_time(true));

    d.link_int_ctrl().write(|w| {
        w.set_link_ie_tx_lmp(true);
        w.set_link_ie_rx_lmp(true);
        w.set_link_ie_rx_lmp_tout(true);
        w.set_link_ie_state_chg(true);
        w.set_link_ie_warm_rst(true);
        w.set_link_ie_term_pres(true);
    });

    d.link_ctrl().write(|w| w.set_link_pd_mode(2));

    d.link_u1_wkup_tmr().write(|w| w.set_u1_wkup_tmr(120));
    d.link_u1_wkup_filter().write(|w| w.set_u1_wkup_filter(50));
    d.link_u2_wkup_filter().write(|w| w.set_u2_wkup_filter(0));
    d.link_u3_wkup_filter().write(|w| w.set_u3_wkup_filter(0));

    reset_protocol(d);

    phy_tuning(d);
}

/// Resets the protocol/FIFO layer and re-enables the device interrupts
/// (`USBSS_Device_Endp_Deinit` + control write in the EVT example).
unsafe fn reset_protocol(d: UsbssDevice) {
    d.usbss_ctrl().modify(|w| w.set_force_rst(true));
    d.status().write(|w| w.set_uif_transfer(true));
    d.usbss_ctrl().write(|w| {
        w.set_uie_transfer(true);
        w.set_udie_setup(true);
        w.set_udie_status(true);
        w.set_dma_en(true);
        w.set_setup_flow(true);
    });
}

unsafe fn phy_write(d: UsbssDevice, addr: u8, data: u16) {
    d.phy_cfg_cr().write(|w| {
        w.set_phy_cfg_data(data);
        w.set_phy_cfg_addr(addr);
        w.set_phy_cfg_write(true);
    });
    d.phy_cfg_dat().write(|w| w.set_phy_cfg_data(1));
}

unsafe fn phy_tuning(d: UsbssDevice) {
    // Values from the EVT device example; the RM does not document either the
    // indirect PHY interface or the 0x5003_C018 write.
    phy_write(d, 0x03, 0x7c12);
    phy_write(d, 0x0d, 0x79aa);
    phy_write(d, 0x15, 0x4430);
    phy_write(d, 0x13, 0x0010);
    phy_write(d, 0x11, 0x0501);

    core::ptr::write_volatile(0x5003_C018 as *mut u32, 0xB005_4000);
}

unsafe fn handle_link<T: Instance>() {
    let d = T::device();
    let flags = d.link_int_flag().read();

    if flags.link_if_state_chg() {
        d.link_int_flag().write(|w| w.set_link_if_state_chg(true));
        let state = d.link_status().read().link_state();

        match state {
            3 | 1 | 2 => {
                // U3/U1/U2: power down the PHY receiver.
                if !T::state().low_power.swap(true, Ordering::Relaxed) {
                    phy_write(d, 0x12, 0x67c8 & !(1 << 9));
                }
            }
            8 => {
                // RECOVERY after a low-power state: restore the PHY setting.
                if T::state().low_power.swap(false, Ordering::Relaxed) {
                    crate::internal::delay::block_for_us(100);
                    phy_write(d, 0x12, 0x67c8);
                }
            }
            4 => {
                // DISABLED: release the forced state so the LTSSM re-detects.
                d.link_ctrl().modify(|w| w.set_link_go_disabled(false));
            }
            9 => {
                // HOT_RESET: reset the protocol layer and end the handshake.
                reset_protocol(d);
                let ts = d.link_ctrl().read().link_tx_ts_cfg() & !1;
                d.link_ctrl().modify(|w| w.set_link_tx_ts_cfg(ts));
            }
            _ => {}
        }

        T::state().link_state.store(state, Ordering::Relaxed);
    }

    if flags.link_if_tx_lmp() {
        d.link_int_flag().write(|w| w.set_link_if_tx_lmp(true));
        d.link_lmp_tx_data0()
            .write(|w| w.set_link_lmp_tx_data0(LMP_LINK_SPEED | LMP_PORT_CAP));
        d.link_lmp_tx_data1()
            .write(|w| w.set_link_lmp_tx_data1(UP_STREAM | NUM_HP_BUF));
        d.link_lmp_tx_data2().write(|w| w.set_link_lmp_tx_data2(0));
    }

    if flags.link_if_rx_lmp() {
        d.link_int_flag().write(|w| w.set_link_if_rx_lmp(true));
        let data0 = d.link_lmp_rx_data0().read().0;

        match data0 & LMP_SUBTYPE_MASK {
            LMP_PORT_CAP if d.link_lmp_port_cap().read().link_lmp_rx_cap_vld() => {
                // Upstream port answers the host's PORT_CAP with PORT_CFG_RES.
                d.link_lmp_tx_data0()
                    .write(|w| w.set_link_lmp_tx_data0(LMP_LINK_SPEED | LMP_PORT_CFG_RES));
                d.link_lmp_tx_data1().write(|w| w.set_link_lmp_tx_data1(0));
                d.link_lmp_tx_data2().write(|w| w.set_link_lmp_tx_data2(0));
                // Write-1-clear: marks the local capability as advertised.
                d.link_lmp_port_cap().modify(|w| w.set_link_lmp_tx_cap_vld(true));
            }
            LMP_PORT_CFG if d.link_lmp_port_cap().read().link_lmp_rx_cap_vld() => {
                // Downstream hubs are not supported yet, but answer anyway.
                d.link_lmp_tx_data0()
                    .write(|w| w.set_link_lmp_tx_data0(LMP_LINK_SPEED | LMP_PORT_CFG_RES));
                d.link_lmp_tx_data1().write(|w| w.set_link_lmp_tx_data1(0));
                d.link_lmp_tx_data2().write(|w| w.set_link_lmp_tx_data2(0));
                d.link_lmp_port_cap().modify(|w| w.set_link_lmp_tx_cap_vld(true));
            }
            LMP_U2_INACT_TOUT => {
                let timer = ((data0 >> 9) & 0xff) as u8;
                d.link_u2_inact_timer().write(|w| w.set_u2_inactive_timer(timer));
            }
            LMP_SET_LINK_FUNC => {
                let enable = data0 & (0x02 << 9) != 0;
                d.link_cfg().modify(|w| {
                    w.set_link_u1_allow(enable);
                    w.set_link_u2_allow(enable);
                });
            }
            _ => {}
        }
    }

    if flags.link_if_rx_lmp_tout() {
        d.link_int_flag().write(|w| w.set_link_if_rx_lmp_tout(true));
        d.link_ctrl().modify(|w| {
            w.set_link_go_disabled(true);
            w.set_link_go_rx_det(true);
        });
    }

    if flags.link_if_warm_rst() {
        d.link_int_flag().write(|w| w.set_link_if_warm_rst(true));
        if d.link_status().read().link_rx_warm_rst() {
            reset_protocol(d);
            d.link_ctrl().modify(|w| w.set_link_go_disabled(true));
            for _ in 0..4 {
                core::arch::asm!("nop");
            }
            d.link_ctrl().modify(|w| w.set_link_go_disabled(false));
        }
    }

    if flags.link_if_term_pres() {
        d.link_int_flag().write(|w| w.set_link_if_term_pres(true));
    }

    T::state().link_waker.wake();
}
