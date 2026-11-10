//! USB 3.0 SuperSpeed controller (USBSS), RM §27.
//!
//! [`Driver`] is an `embassy-usb` device driver for SuperSpeed (5 Gbit/s),
//! ported from the TinyUSB `dcd_ch32_usbss` driver. Build the `embassy-usb`
//! configuration with `UsbDeviceSpeed::Super` and a `bcd_usb` of 3.0 or later,
//! and use 1024-byte bulk endpoints, each bursting the `max_burst` of its
//! `EndpointOptions`. The controller has no USB 2.0 fallback.
//!
//! The controller mishandles a link error that finds more than a few header
//! packets pending: an OUT endpoint can stall, a control transfer can fail, or
//! one IN packet can carry wrong data. That takes OUT traffic next to IN
//! traffic, or a control transfer next to bulk traffic. `OUT_WINDOW` in
//! `device.rs` can rule out the first case at the cost of OUT throughput.
//!
//! [`host::HostDriver`] only brings the root-port link up.
//!
//! The block has its own 125 MHz PLL, PIPE/UTMI clocks and link state machine,
//! separate from `usbhs`.

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac::rcc::vals::UsbsspllRefsel;
use crate::pac::usbss::vals::{PdMode, TxDeemph};
use crate::pac::usbss::{UsbssDevice, UsbssHost};
use crate::{interrupt, PeripheralType, RccPeripheral};

mod control;
mod device;
mod endpoint;
pub mod host;
mod regs;

pub use control::ControlPipe;
pub use device::{Bus, Driver, Error};
pub use endpoint::Endpoint;

pub use crate::pac::usbss::vals::LinkState;

const LINK_STATE_UNKNOWN: u8 = 0xff;

/// Operating mode, set when a driver starts. Both modes share the interrupt
/// handlers and register block.
const MODE_DEVICE: u8 = 0;
const MODE_HOST: u8 = 1;

const LMP_SUBTYPE_MASK: u32 = 0xf << 5;
const LMP_SET_LINK_FUNC: u32 = 0x1 << 5;
const LMP_U2_INACT_TOUT: u32 = 0x2 << 5;
const LMP_PORT_CAP: u32 = 0x4 << 5;
const LMP_PORT_CFG: u32 = 0x5 << 5;
const LMP_PORT_CFG_RES: u32 = 0x6 << 5;
const LMP_LINK_SPEED: u32 = 1 << 9;
const NUM_HP_BUF: u32 = 4;
const UP_STREAM: u32 = 2 << 16;
const DOWN_STREAM: u32 = 1 << 16;

struct State {
    mode: AtomicU8,
    link_waker: AtomicWaker,
    link_state: AtomicU8,
    /// Last state reported through the host driver's `wait_link_change`.
    host_reported_state: AtomicU8,
    /// A SuperSpeed device finished the Port Configuration handshake.
    connected: AtomicBool,
    /// The current connection was already reported to `wait_connected`.
    connected_reported: AtomicBool,
    device_waker: AtomicWaker,
    host_buf: HostBuffer,
    device: device::State,
}

impl State {
    const fn new(device: UsbssDevice) -> Self {
        Self {
            mode: AtomicU8::new(MODE_DEVICE),
            link_waker: AtomicWaker::new(),
            link_state: AtomicU8::new(LINK_STATE_UNKNOWN),
            host_reported_state: AtomicU8::new(LINK_STATE_UNKNOWN),
            connected: AtomicBool::new(false),
            connected_reported: AtomicBool::new(false),
            device_waker: AtomicWaker::new(),
            host_buf: HostBuffer::new(),
            device: device::State::new(regs::Regs(device)),
        }
    }
}

/// Host control-transfer buffer. The controller DMAs through it, so it must
/// keep a stable address.
#[repr(align(64))]
struct HostBuffer(UnsafeCell<[u8; 512]>);

// Safety: only the host initialization hands the address to the controller.
unsafe impl Sync for HostBuffer {}

impl HostBuffer {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; 512]))
    }

    fn as_ptr(&self) -> *mut u8 {
        self.0.get().cast()
    }
}

trait SealedInstance: RccPeripheral {
    fn device() -> UsbssDevice;
    fn host() -> UsbssHost;
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

            fn host() -> UsbssHost {
                unsafe { UsbssHost::from_ptr(crate::pac::$inst.as_ptr()) }
            }

            fn state() -> &'static State {
                static STATE: State = State::new(unsafe { UsbssDevice::from_ptr(crate::pac::$inst.as_ptr()) });
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
            type LinkInterrupt = crate::_generated::peripheral_interrupts::$inst::LINK;
        }
    };
);

/// GLOBAL interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        if T::state().mode.load(Ordering::Relaxed) == MODE_DEVICE {
            device::on_interrupt::<T>();
        }
    }
}

/// LINK interrupt handler.
pub struct LinkInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::LinkInterrupt> for LinkInterruptHandler<T> {
    unsafe fn on_interrupt() {
        if T::state().mode.load(Ordering::Relaxed) == MODE_HOST {
            handle_link_host::<T>();
        } else {
            device::on_interrupt::<T>();
        }
    }
}

/// Enables the USBSS PLL, the block clock, the PIPE clock and the UTMI clock.
unsafe fn init_clocks<T: Instance>() {
    crate::rcc::enable_usbss_pll(UsbsspllRefsel::F25MHZ);
    T::enable_and_reset();
    crate::pac::RCC.cfgr0().modify(|w| {
        w.set_pipeon(true);
        w.set_utmion(true);
    });
}

fn delay_cycles(count: u32) {
    for _ in 0..count {
        unsafe { core::arch::asm!("nop") };
    }
}

/// Initializes the block as a SuperSpeed host (`USBSSH_Init` in the EVT
/// example). The PHY/global registers are reached through the device view;
/// both views start at the same base address. The de-emphasis is the 3.5 dB
/// of USB 3.0, where the EVT writes the reserved value 3.
///
/// Safe to call again on a warm reset: the clocks are set up once by
/// `init_clocks`.
unsafe fn init_host<T: Instance>() {
    T::state().mode.store(MODE_HOST, Ordering::Relaxed);
    T::state().connected.store(false, Ordering::Relaxed);
    T::state().connected_reported.store(false, Ordering::Relaxed);

    let h = T::host();
    let d = T::device();

    // Clear the protocol/FIFO layer before programming the link.
    h.usbss_ctrl().modify(|w| w.set_usb_clr_all(true));
    h.usbss_ctrl().modify(|w| w.set_usb_clr_all(false));
    h.uh_tx_ctrl().write(|_| {});
    h.uh_rx_ctrl().write(|_| {});

    // Assert the PHY reset with DOWN_MODE set, force P2, then release it and
    // enable terminal detection once the power mode has settled.
    h.link_cfg().write(|w| {
        w.set_link_rx_eq_en(true);
        w.set_link_tx_deemph(TxDeemph::DB3_5);
        w.set_link_down_mode(true);
        w.set_link_phy_reset(true);
    });
    h.link_ctrl().write(|w| {
        w.set_link_pd_mode(PdMode::P2);
        w.set_link_go_disabled(true);
    });
    h.link_cfg().write(|w| {
        w.set_link_rx_eq_en(true);
        w.set_link_tx_deemph(TxDeemph::DB3_5);
        w.set_link_down_mode(true);
        w.set_link_u2_det_en(true);
        w.set_link_ltssm_mode(true);
    });
    h.link_lpm_cr().modify(|w| w.set_lpm_en(true));

    // The EVT example spins on LINK_BUSY. Bound the wait so a misconfigured
    // PHY cannot hang the firmware forever.
    for _ in 0..1_000_000 {
        if !d.link_status().read().link_busy() {
            break;
        }
    }

    h.link_cfg().modify(|w| w.set_link_rx_term_en(true));
    h.link_int_ctrl().write(|w| {
        w.set_link_ie_tx_lmp(true);
        w.set_link_ie_rx_lmp(true);
        w.set_link_ie_rx_lmp_tout(true);
        w.set_link_ie_state_chg(true);
        w.set_link_ie_warm_rst(true);
    });
    h.link_ctrl().write(|w| w.set_link_pd_mode(PdMode::P2));
    h.link_itp_pre().write(|w| w.set_itp_pre(122));
    h.usbss_ctrl().modify(|w| w.set_force_rst(true));

    // Host mode, ITP generation, DMA and pending-packet control; this write
    // also releases FORCE_RST.
    h.usbss_ctrl().write(|w| {
        w.set_host_mode(true);
        w.set_itp_en(true);
        w.set_dma_en(true);
        w.set_reg_hp_pend(2);
    });
    h.uh_tx_ctrl().write(|_| {});
    h.status().write(|w| w.set_uif_transfer(true));

    let buf = T::state().host_buf.as_ptr() as u32;
    h.uh_tx_dma().write(|w| w.set_uh_tx_dma(buf));
    h.uh_rx_dma().write(|w| w.set_uh_rx_dma(buf));
    // The endpoint enable registers live in the device view at the same
    // offsets 0x80/0x82; host mode drives them too.
    d.uep_tx_en().write(|w| w.set_ep_tx_en(0, true));
    d.uep_rx_en().write(|w| w.set_ep_rx_en(0, true));

    regs::Regs(d).phy_tuning();
}

/// Host-side link management. A downstream port sends PORT_CAP and answers
/// the device's PORT_CAP with PORT_CFG; PORT_CFG_RES means training
/// completed (EVT `USBSS_LINK_Handle`, `LINK_DOWN_MODE` branch).
unsafe fn handle_link_host<T: Instance>() {
    let h = T::host();
    let flags = h.link_int_flag().read();

    if flags.link_if_state_chg() {
        h.link_int_flag().write(|w| w.set_link_if_state_chg(true));
        let state = h.link_status().read().link_state();

        match state {
            LinkState::DISABLED => {
                // Release the forced state so the LTSSM re-detects.
                h.link_ctrl().modify(|w| w.set_link_go_disabled(false));
            }
            LinkState::RXDET => {
                // The cable was unplugged or the link was lost.
                T::state().connected.store(false, Ordering::Relaxed);
                T::state().connected_reported.store(false, Ordering::Relaxed);
                T::state().device_waker.wake();
            }
            LinkState::HOTRST => {
                // End the handshake.
                h.link_ctrl()
                    .modify(|w| w.set_link_tx_ts_cfg(w.link_tx_ts_cfg() & !regs::LINK_HOT_RESET));
            }
            _ => {}
        }

        T::state().link_state.store(state.to_bits(), Ordering::Relaxed);
    }

    if flags.link_if_tx_lmp() {
        h.link_int_flag().write(|w| w.set_link_if_tx_lmp(true));
        h.link_lmp_tx_data(0).write_value(LMP_LINK_SPEED | LMP_PORT_CAP);
        h.link_lmp_tx_data(1).write_value(DOWN_STREAM | NUM_HP_BUF);
        h.link_lmp_tx_data(2).write_value(0);
    }

    if flags.link_if_rx_lmp() {
        h.link_int_flag().write(|w| w.set_link_if_rx_lmp(true));
        let data0 = h.link_lmp_rx_data(0).read();

        match data0 & LMP_SUBTYPE_MASK {
            LMP_PORT_CAP => {
                // The device advertised its capabilities; configure the link.
                h.link_lmp_tx_data(0).write_value(LMP_LINK_SPEED | LMP_PORT_CFG);
                h.link_lmp_tx_data(1).write_value(0);
                h.link_lmp_tx_data(2).write_value(0);
            }
            LMP_PORT_CFG_RES => {
                // The device accepted our PORT_CFG: the SS link is up.
                h.link_lmp_port_cap().modify(|w| w.set_link_lmp_tx_cap_vld(true));
                T::state().connected.store(true, Ordering::Relaxed);
                T::state().device_waker.wake();
            }
            _ => {}
        }
    }

    if flags.link_if_rx_lmp_tout() {
        h.link_int_flag().write(|w| w.set_link_if_rx_lmp_tout(true));
        h.link_ctrl().modify(|w| {
            w.set_link_go_disabled(true);
            w.set_link_go_rx_det(true);
        });
    }

    if flags.link_if_warm_rst() {
        h.link_int_flag().write(|w| w.set_link_if_warm_rst(true));
        if h.link_status().read().link_rx_warm_rst() {
            init_host::<T>();
        }
    }

    T::state().link_waker.wake();
}
