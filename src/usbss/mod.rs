//! Experimental USB 3.0 SuperSpeed controller (USBSS) — RM §27.
//!
//! # Validation status
//!
//! This is an incomplete framework and has not been verified to work. Neither
//! SuperSpeed link training nor enumeration has succeeded in hardware tests.
//! Device bulk transfers and host control transfers, pipes and enumeration are
//! not implemented. Do not rely on this module for USB communication.
//!
//! The reference manual documents the register map; the initialization order,
//! PHY tuning values and link-management handshake follow the WCH EVT USBSS
//! examples. This controller is separate from `usbhs` and has its own 125 MHz
//! PLL, PIPE/UTMI clocks and link state machine.

use core::cell::{RefCell, UnsafeCell};
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;

use crate::interrupt::typelevel::Interrupt;
use crate::pac::rcc::vals::UsbsspllRefsel;
use crate::pac::usbss::{UsbssDevice, UsbssHost};
use crate::{interrupt, Peri, PeripheralType, RccPeripheral};

pub mod descriptors;
pub mod host;

/// SuperSpeed control-endpoint packet size: `2^bMaxPacketSize0`, or 512 bytes.
const EP0_MAX_PACKET: usize = 512;

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

/// Operating mode, set once at initialization. Host and device mode share the
/// same interrupt handlers and register block.
const MODE_DEVICE: u8 = 0;
const MODE_HOST: u8 = 1;

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
const DOWN_STREAM: u32 = 1 << 16;

struct State {
    link_waker: AtomicWaker,
    link_state: AtomicU8,
    /// Last state reported through the host driver's `wait_link_change`.
    host_reported_state: AtomicU8,
    low_power: AtomicBool,
    mode: AtomicU8,
    /// A SuperSpeed device finished the Port Configuration handshake.
    connected: AtomicBool,
    /// The current connection was already reported to `wait_connected`.
    connected_reported: AtomicBool,
    device_waker: AtomicWaker,
    ep0: Mutex<CriticalSectionRawMutex, RefCell<Ep0State>>,
    ep0_buf: Ep0Buffer,
}

impl State {
    const fn new() -> Self {
        Self {
            link_waker: AtomicWaker::new(),
            link_state: AtomicU8::new(LINK_STATE_UNKNOWN),
            host_reported_state: AtomicU8::new(LINK_STATE_UNKNOWN),
            low_power: AtomicBool::new(false),
            mode: AtomicU8::new(MODE_DEVICE),
            connected: AtomicBool::new(false),
            connected_reported: AtomicBool::new(false),
            device_waker: AtomicWaker::new(),
            ep0: Mutex::new(RefCell::new(Ep0State::new())),
            ep0_buf: Ep0Buffer::new(),
        }
    }
}

/// Control-transfer state carried across the SETUP, data and status stages.
struct Ep0State {
    descr: &'static [u8],
    descr_pos: usize,
    /// Bytes left to move in the data stage.
    remaining: u16,
    /// The next packet is already in `ep0_buf` instead of `descr`.
    from_buffer: bool,
    /// Address to commit once the status stage completes.
    pending_address: Option<u8>,
    configured: bool,
}

impl Ep0State {
    const fn new() -> Self {
        Self {
            descr: &[],
            descr_pos: 0,
            remaining: 0,
            from_buffer: false,
            pending_address: None,
            configured: false,
        }
    }
}

/// EP0 packet buffer, shared by both directions like the EVT example. The
/// controller DMAs through this, so it must keep a stable address.
#[repr(align(64))]
struct Ep0Buffer(UnsafeCell<[u8; EP0_MAX_PACKET]>);

// Safety: the buffer is only touched from the GLOBAL interrupt, and from
// `init` before that interrupt is enabled.
unsafe impl Sync for Ep0Buffer {}

impl Ep0Buffer {
    const fn new() -> Self {
        Self(UnsafeCell::new([0; EP0_MAX_PACKET]))
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
        handle_global::<T>();
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
            init_device::<T>();
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

    /// Whether the host has selected a configuration other than zero.
    pub fn configured(&self) -> bool {
        T::state().ep0.lock(|cell| cell.borrow().configured)
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

/// Control-pipe requests this driver answers.
mod request {
    pub const GET_STATUS: u8 = 0x00;
    pub const CLEAR_FEATURE: u8 = 0x01;
    pub const SET_FEATURE: u8 = 0x03;
    pub const SET_ADDRESS: u8 = 0x05;
    pub const GET_DESCRIPTOR: u8 = 0x06;
    pub const GET_CONFIGURATION: u8 = 0x08;
    pub const SET_CONFIGURATION: u8 = 0x09;
    pub const GET_INTERFACE: u8 = 0x0a;
    pub const SET_INTERFACE: u8 = 0x0b;
}

/// Subset of the eight-byte SETUP packet the control pipe needs.
struct Setup {
    request_type: u8,
    request: u8,
    value: u16,
    length: u16,
}

impl Setup {
    unsafe fn read(buf: *const u8) -> Self {
        let word = |offset: usize| u16::from_le_bytes([buf.add(offset).read(), buf.add(offset + 1).read()]);
        Self {
            request_type: buf.read(),
            request: buf.add(1).read(),
            value: word(2),
            length: word(6),
        }
    }
}

/// GLOBAL interrupt: SETUP, STATUS and transfer completions.
unsafe fn handle_global<T: Instance>() {
    let d = T::device();
    let status = d.status().read();

    if status.udif_setup() && !status.udif_status() {
        handle_setup::<T>(d);
        d.status().write(|w| w.set_udif_setup(true));
    } else if status.udif_status() {
        d.status().write(|w| w.set_udif_status(true));

        // The address takes effect only after the status stage, as the USB
        // specification requires.
        let address = T::state().ep0.lock(|cell| cell.borrow_mut().pending_address.take());
        if let Some(address) = address {
            d.usbss_ctrl().modify(|w| w.set_dev_addr(address.into()));
        }

        // Return EP0 to idle for the next transfer.
        d.uep0_tx_ctrl().write(|_| {});
        d.uep0_rx_ctrl().write(|_| {});
    } else if status.uif_transfer() {
        d.status().write(|w| w.set_uif_transfer(true));

        if status.ep_dir() && status.ep_id() == 0 {
            let remaining = T::state().ep0.lock(|cell| cell.borrow().remaining);
            if remaining == 0 {
                // The last packet went out; ready the OUT status stage.
                d.uep0_tx_ctrl().write(|w| w.set_ep0_tx_res(1));
                d.uep0_rx_ctrl().write(|w| {
                    w.set_ep0_rx_res(1);
                    w.set_ep0_rx_erdy(true);
                });
            } else {
                ep0_send_chunk::<T>(d, false);
            }
        }
    } else {
        // Clear every write-1-to-zero flag so a stray interrupt cannot storm
        // the core.
        d.status().write(|w| {
            w.set_uif_transfer(true);
            w.set_udif_setup(true);
            w.set_udif_status(true);
            w.set_uif_rx_ping(true);
            w.set_uif_itp(true);
            w.set_uif_fifo_txov(true);
            w.set_uif_fifo_rxov(true);
        });
    }
}

unsafe fn handle_setup<T: Instance>(d: UsbssDevice) {
    let buf = T::state().ep0_buf.as_ptr();
    let setup = Setup::read(buf);

    // Only standard requests are implemented. Class and vendor requests stall
    // until there is a control trait to dispatch them to.
    let mut stall = setup.request_type & 0x60 != 0;
    let in_dir = setup.request_type & 0x80 != 0;
    let mut length = setup.length;
    let mut descr: &'static [u8] = &[];
    let mut from_buffer = false;
    let mut address = None;
    let mut configuration = None;

    if !stall {
        match setup.request {
            request::GET_DESCRIPTOR => match descriptors::get((setup.value >> 8) as u8, setup.value as u8) {
                Some(found) => {
                    descr = found;
                    length = length.min(found.len() as u16);
                }
                None => stall = true,
            },
            request::SET_ADDRESS => address = Some((setup.value & 0x7f) as u8),
            request::SET_CONFIGURATION => {
                // Only configuration 1 exists, plus the unconfigured state.
                if setup.value <= 1 {
                    configuration = Some(setup.value == 1);
                } else {
                    stall = true;
                }
            }
            request::GET_CONFIGURATION => {
                let configured = T::state().ep0.lock(|cell| cell.borrow().configured);
                buf.write(configured as u8);
                length = length.min(1);
                from_buffer = true;
            }
            request::GET_STATUS => {
                // Bus powered, no remote wakeup, no halted endpoints.
                buf.write(0);
                buf.add(1).write(0);
                length = length.min(2);
                from_buffer = true;
            }
            request::GET_INTERFACE => {
                buf.write(0);
                length = length.min(1);
                from_buffer = true;
            }
            request::CLEAR_FEATURE | request::SET_FEATURE | request::SET_INTERFACE => {}
            _ => stall = true,
        }
    }

    // No other request supplies IN data. Reject malformed direction and length
    // combinations instead of reading past the empty descriptor slice.
    if in_dir && length != 0 && !from_buffer && descr.is_empty() {
        stall = true;
    }

    if stall {
        d.uep0_tx_ctrl().write(|w| w.set_ep0_tx_res(2));
        d.uep0_rx_ctrl().write(|w| {
            w.set_ep0_rx_res(2);
            w.set_ep0_rx_erdy(true);
        });
        return;
    }

    T::state().ep0.lock(|cell| {
        let mut ep0 = cell.borrow_mut();
        ep0.descr = descr;
        ep0.descr_pos = 0;
        ep0.remaining = length;
        ep0.from_buffer = from_buffer;
        if let Some(address) = address {
            ep0.pending_address = Some(address);
        }
        if let Some(configured) = configuration {
            ep0.configured = configured;
        }
    });

    if in_dir {
        if length == 0 {
            // No data stage: ready the OUT status stage directly.
            d.uep0_rx_ctrl().write(|w| {
                w.set_ep0_rx_res(1);
                w.set_ep0_rx_erdy(true);
            });
        } else {
            ep0_send_chunk::<T>(d, true);
        }
    } else if length == 0 {
        // The status stage is an IN zero-length packet.
        d.uep0_tx_ctrl().write(|w| {
            w.set_ep0_tx_res(1);
            w.set_ep0_tx_erdy(true);
        });
        d.uep0_rx_ctrl().write(|w| {
            w.set_ep0_rx_res(1);
            w.set_ep0_rx_erdy(true);
        });
    } else {
        d.uep0_rx_ctrl().write(|w| {
            w.set_ep0_rx_res(1);
            w.set_ep0_rx_erdy(true);
        });
    }
}

/// Pushes the next data-stage packet into the EP0 buffer and arms ERDY.
unsafe fn ep0_send_chunk<T: Instance>(d: UsbssDevice, first: bool) {
    let buf = T::state().ep0_buf.as_ptr();
    let mut seq = 0;

    let len = T::state().ep0.lock(|cell| {
        let mut ep0 = cell.borrow_mut();
        let len = (ep0.remaining as usize).min(EP0_MAX_PACKET);
        if !ep0.from_buffer {
            core::ptr::copy_nonoverlapping(ep0.descr.as_ptr().add(ep0.descr_pos), buf, len);
            ep0.descr_pos += len;
        }
        ep0.remaining -= len as u16;
        if !first {
            // The hardware auto-increments the sequence number; mirror the
            // EVT example, which reads it back and adds one.
            seq = (d.uep0_tx_ctrl().read().ep0_tx_seq() + 1) & 0x1f;
        }
        len
    });

    d.uep0_tx_ctrl().write(|w| {
        w.set_ep0_tx_len(len as u16);
        w.set_ep0_tx_seq(seq);
        w.set_ep0_tx_res(1);
        w.set_ep0_tx_erdy(true);
    });
}

/// Enables the USBSS PLL, the block clock, the PIPE clock and the UTMI clock.
/// RM §27 has no clock section; this order follows the EVT examples.
unsafe fn init_clocks<T: Instance>() {
    crate::rcc::enable_usbss_pll(UsbsspllRefsel::F25MHZ);
    T::enable_and_reset();
    crate::pac::RCC.cfgr0().modify(|w| {
        w.set_pipeon(true);
        w.set_utmion(true);
    });
}

unsafe fn init_device<T: Instance>() {
    init_clocks::<T>();
    T::state().mode.store(MODE_DEVICE, Ordering::Relaxed);

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

    // EP0: both directions share one 512-byte buffer, as in the EVT example.
    let buf = T::state().ep0_buf.as_ptr() as u32;
    d.uep0_tx_ctrl().write(|_| {});
    d.uep0_rx_ctrl().write(|_| {});
    d.u3ep0_tx_dma().write(|w| w.set_u3ep0_tx_dma(buf));
    d.u3ep0_rx_dma().write(|w| w.set_u3ep0_rx_dma(buf));
}

/// Initializes the block as a SuperSpeed host (`USBSSH_Init` in the EVT
/// example). The PHY/global registers are reached through the device view;
/// both views start at the same base address.
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
        w.set_link_tx_deemph(3);
        w.set_link_down_mode(true);
        w.set_link_phy_reset(true);
    });
    h.link_ctrl().write(|w| {
        w.set_link_pd_mode(2);
        w.set_link_go_disabled(true);
    });
    h.link_cfg().write(|w| {
        w.set_link_rx_eq_en(true);
        w.set_link_tx_deemph(3);
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
    h.link_ctrl().write(|w| w.set_link_pd_mode(2));
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

    let buf = T::state().ep0_buf.as_ptr() as u32;
    h.uh_tx_dma().write(|w| w.set_uh_tx_dma(buf));
    h.uh_rx_dma().write(|w| w.set_uh_rx_dma(buf));
    // The endpoint enable registers live in the device view at the same
    // offsets 0x80/0x82; host mode drives them too.
    d.uep_tx_en().write(|w| w.set_ep_tx_en(1));
    d.uep_rx_en().write(|w| w.set_ep_rx_en(1));

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

/// Dispatches a LINK interrupt to the mode that was initialized.
unsafe fn handle_link<T: Instance>() {
    if T::state().mode.load(Ordering::Relaxed) == MODE_HOST {
        handle_link_host::<T>();
    } else {
        handle_link_device::<T>();
    }
}

unsafe fn handle_link_device<T: Instance>() {
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
            4 => {
                // DISABLED: release the forced state so the LTSSM re-detects.
                h.link_ctrl().modify(|w| w.set_link_go_disabled(false));
            }
            5 => {
                // RXDET: the cable was unplugged or the link was lost.
                T::state().connected.store(false, Ordering::Relaxed);
                T::state().connected_reported.store(false, Ordering::Relaxed);
                T::state().device_waker.wake();
            }
            9 => {
                // HOT_RESET: end the handshake.
                let ts = h.link_ctrl().read().link_tx_ts_cfg() & !1;
                h.link_ctrl().modify(|w| w.set_link_tx_ts_cfg(ts));
            }
            _ => {}
        }

        T::state().link_state.store(state, Ordering::Relaxed);
    }

    if flags.link_if_tx_lmp() {
        h.link_int_flag().write(|w| w.set_link_if_tx_lmp(true));
        h.link_lmp_tx_data0()
            .write(|w| w.set_link_lmp_tx_data0(LMP_LINK_SPEED | LMP_PORT_CAP));
        h.link_lmp_tx_data1()
            .write(|w| w.set_link_lmp_tx_data1(DOWN_STREAM | NUM_HP_BUF));
        h.link_lmp_tx_data2().write(|w| w.set_link_lmp_tx_data2(0));
    }

    if flags.link_if_rx_lmp() {
        h.link_int_flag().write(|w| w.set_link_if_rx_lmp(true));
        let data0 = h.link_lmp_rx_data0().read().0;

        match data0 & LMP_SUBTYPE_MASK {
            LMP_PORT_CAP => {
                // The device advertised its capabilities; configure the link.
                h.link_lmp_tx_data0()
                    .write(|w| w.set_link_lmp_tx_data0(LMP_LINK_SPEED | LMP_PORT_CFG));
                h.link_lmp_tx_data1().write(|w| w.set_link_lmp_tx_data1(0));
                h.link_lmp_tx_data2().write(|w| w.set_link_lmp_tx_data2(0));
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
            T::state().connected.store(false, Ordering::Relaxed);
            init_host::<T>();
        }
    }

    T::state().link_waker.wake();
}
