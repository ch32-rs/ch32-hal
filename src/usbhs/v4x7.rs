//! CH32V407/CH32V467 USBHS device driver for Embassy.
//!
//! Endpoints 1–7 support independent IN and OUT allocations. A pair shares its
//! maximum packet length register and therefore must use the same packet size.
//! Endpoint memory stays borrowed for the lifetime of the bus and endpoints;
//! DMA never accesses a caller's temporary read/write slice. Canceling an IN
//! write may leave its owned packet queued until the host consumes it or the
//! endpoint is disabled. A subsequent write drains that packet before reuse.
//!
//! Configure the shared 480 MHz USBHS PLL through RCC before constructing the
//! driver. Full speed is the default. High-speed descriptors and full-speed
//! fallback must be handled by the USB stack/application. No VBUS input is
//! sampled: the bus reports power available and cannot report cable removal.

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event, Unsupported,
};

use crate::gpio::{AnyPin, Pull, SealedPin};
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peri, RccPeripheral};

#[path = "v4x7_control.rs"]
mod control;
#[path = "v4x7_endpoint.rs"]
mod endpoint;
#[path = "v4x7_regs.rs"]
mod registers;

pub use control::ControlPipe;
pub use embassy_usb_driver::Speed;
pub use endpoint::Endpoint;
use registers::Registers;

const ENDPOINTS: usize = 8;
const SLOTS: usize = ENDPOINTS * 2;
const NAK: u8 = 0;
const STALL: u8 = 1;
const ACK: u8 = 2;
const DONE: u8 = 0x80;
const PRESERVE_FLAGS: u8 = 0xc0;
const RESET: u8 = 1;
const SUSPEND: u8 = 2;
const RESUME: u8 = 4;
const POWER: u8 = 8;
const INTERRUPTS: u8 = 0xd7;

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Config {
    pub speed: Speed,
}

impl Default for Config {
    fn default() -> Self {
        Self { speed: Speed::Full }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    ClockNotReady,
    UnsupportedSpeed,
    InvalidMemory,
    BufferTooSmall,
}

#[derive(Clone, Copy)]
struct Buffer {
    address: usize,
    size: u16,
}

impl Buffer {
    unsafe fn read(self, target: &mut [u8]) {
        fence(Ordering::Acquire);
        core::ptr::copy_nonoverlapping(self.address as *const u8, target.as_mut_ptr(), target.len());
    }

    unsafe fn write(self, source: &[u8]) {
        core::ptr::copy_nonoverlapping(source.as_ptr(), self.address as *mut u8, source.len());
        fence(Ordering::Release);
    }
}

#[derive(Clone, Copy)]
struct EndpointConfig {
    info: EndpointInfo,
    buffer: Buffer,
}

#[derive(Clone, Copy)]
struct EndpointState {
    config: Option<EndpointConfig>,
    enabled: bool,
    stalled: bool,
    generation: u32,
    rx_ready: bool,
    rx_length: u16,
    tx_busy: bool,
    tx_done: bool,
    overflow: bool,
}

impl EndpointState {
    const fn new() -> Self {
        Self {
            config: None,
            enabled: false,
            stalled: false,
            generation: 0,
            rx_ready: false,
            rx_length: 0,
            tx_busy: false,
            tx_done: false,
            overflow: false,
        }
    }

    fn invalidate(&mut self) {
        let config = self.config;
        let generation = self.generation.wrapping_add(1);
        *self = Self::new();
        self.config = config;
        self.generation = generation;
    }
}

struct Inner {
    endpoints: [EndpointState; SLOTS],
    powered: bool,
    suspended: bool,
    events: u8,
    setup: Option<[u8; 8]>,
}

impl Inner {
    const fn new() -> Self {
        Self {
            endpoints: [EndpointState::new(); SLOTS],
            powered: false,
            suspended: false,
            events: 0,
            setup: None,
        }
    }
}

struct State {
    inner: Mutex<CriticalSectionRawMutex, RefCell<Inner>>,
    bus_waker: AtomicWaker,
    endpoint_wakers: [AtomicWaker; SLOTS],
}

impl State {
    const fn new() -> Self {
        Self {
            inner: Mutex::new(RefCell::new(Inner::new())),
            bus_waker: AtomicWaker::new(),
            endpoint_wakers: [const { AtomicWaker::new() }; SLOTS],
        }
    }
}

fn slot(index: usize, incoming: bool) -> usize {
    index * 2 + incoming as usize
}

fn packet_size_valid(speed: Speed, kind: EndpointType, size: u16, interval: u8) -> bool {
    match (speed, kind) {
        (Speed::Full, EndpointType::Bulk) => matches!(size, 8 | 16 | 32 | 64),
        (Speed::High, EndpointType::Bulk) => size == 512,
        (Speed::Full, EndpointType::Interrupt) => (1..=64).contains(&size) && interval != 0,
        (Speed::High, EndpointType::Interrupt) => (1..=1024).contains(&size) && (1..=16).contains(&interval),
        (Speed::Full, EndpointType::Isochronous) => (1..=1023).contains(&size) && (1..=16).contains(&interval),
        (Speed::High, EndpointType::Isochronous) => (1..=1024).contains(&size) && (1..=16).contains(&interval),
        _ => false,
    }
}

fn memory_valid(address: usize, length: usize, ram_size: usize) -> bool {
    address >= 0x2000_0000
        && address
            .checked_add(length)
            .is_some_and(|end| end <= 0x2000_0000 + ram_size)
}

pub struct Driver<'d, T: Instance> {
    peri: Peri<'d, T>,
    dp: Peri<'d, AnyPin>,
    dm: Peri<'d, AnyPin>,
    config: Config,
    memory: usize,
    capacity: usize,
    used: usize,
    endpoints: [Option<EndpointConfig>; SLOTS],
    _memory: PhantomData<&'d mut [u8]>,
}

impl<'d, T: Instance> Driver<'d, T> {
    pub fn new(
        peri: Peri<'d, T>,
        dp: Peri<'d, impl DpPin<T>>,
        dm: Peri<'d, impl DmPin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        endpoint_memory: &'d mut [u8],
        config: Config,
    ) -> Result<Self, Error> {
        if config.speed == Speed::Low {
            return Err(Error::UnsupportedSpeed);
        }
        if !pac::RCC.ctlr().read().usbhs_pllrdy() {
            return Err(Error::ClockNotReady);
        }
        let address = endpoint_memory.as_mut_ptr() as usize;
        #[cfg(target_arch = "riscv32")]
        {
            let ram_size = if pac::FLASH.obr().read().ram_code_mod().to_bits() == 0 {
                200 * 1024
            } else {
                136 * 1024
            };
            if !memory_valid(address, endpoint_memory.len(), ram_size) {
                return Err(Error::InvalidMemory);
            }
        }
        let aligned = address.checked_add(3).ok_or(Error::InvalidMemory)? & !3;
        let capacity = endpoint_memory
            .len()
            .checked_sub(aligned - address)
            .ok_or(Error::BufferTooSmall)?;
        if capacity < 64 {
            return Err(Error::BufferTooSmall);
        }
        Ok(Self {
            peri,
            dp: dp.into(),
            dm: dm.into(),
            config,
            memory: aligned,
            capacity,
            used: 64,
            endpoints: [None; SLOTS],
            _memory: PhantomData,
        })
    }

    fn allocate<const IN: bool>(
        &mut self,
        ep_type: EndpointType,
        requested: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d, T, IN>, EndpointAllocError> {
        if !packet_size_valid(self.config.speed, ep_type, max_packet_size, interval_ms) {
            return Err(EndpointAllocError);
        }
        let direction = if IN { Direction::In } else { Direction::Out };
        if requested.is_some_and(|address| address.direction() != direction) {
            return Err(EndpointAllocError);
        }
        let index = (1..ENDPOINTS)
            .find(|index| {
                requested.is_none_or(|address| address.index() == *index)
                    && self.endpoints[slot(*index, IN)].is_none()
                    && self.endpoints[slot(*index, !IN)]
                        .is_none_or(|other| other.info.max_packet_size == max_packet_size)
            })
            .ok_or(EndpointAllocError)?;
        let start = (self.used + 3) & !3;
        let end = start
            .checked_add(usize::from(max_packet_size))
            .ok_or(EndpointAllocError)?;
        if end > self.capacity {
            return Err(EndpointAllocError);
        }
        let info = EndpointInfo {
            addr: EndpointAddress::from_parts(index, direction),
            ep_type,
            max_packet_size,
            interval_ms,
        };
        self.endpoints[slot(index, IN)] = Some(EndpointConfig {
            info,
            buffer: Buffer {
                address: self.memory + start,
                size: max_packet_size,
            },
        });
        self.used = end;
        Ok(Endpoint::new(info))
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, false>;
    type EndpointIn = Endpoint<'d, T, true>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.allocate(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.allocate(ep_type, ep_addr, max_packet_size, interval_ms)
    }

    fn start(mut self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(
            matches!(control_max_packet_size, 8 | 16 | 32 | 64)
                && (self.config.speed != Speed::High || control_max_packet_size == 64)
        );
        for incoming in [false, true] {
            self.endpoints[slot(0, incoming)] = Some(EndpointConfig {
                info: EndpointInfo {
                    addr: EndpointAddress::from_parts(0, if incoming { Direction::In } else { Direction::Out }),
                    ep_type: EndpointType::Control,
                    max_packet_size: control_max_packet_size,
                    interval_ms: 0,
                },
                buffer: Buffer {
                    address: self.memory,
                    size: 64,
                },
            });
        }
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            for (state, config) in inner.endpoints.iter_mut().zip(self.endpoints) {
                state.invalidate();
                state.config = config;
            }
            inner.powered = false;
            inner.events = POWER;
            inner.setup = None;
        });
        (
            Bus {
                _peri: self.peri,
                dp: self.dp,
                dm: self.dm,
                config: self.config,
                _memory: PhantomData,
            },
            ControlPipe::new(control_max_packet_size),
        )
    }
}

pub struct Bus<'d, T: Instance> {
    _peri: Peri<'d, T>,
    dp: Peri<'d, AnyPin>,
    dm: Peri<'d, AnyPin>,
    config: Config,
    _memory: PhantomData<&'d mut [u8]>,
}

impl<T: Instance> Bus<'_, T> {
    fn power_on(&mut self) {
        assert!(pac::RCC.ctlr().read().usbhs_pllrdy(), "USBHS PLL is not ready");
        critical_section::with(|_| {
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                if inner.powered {
                    return;
                }
                T::set_utmi(true);
                T::enable_and_reset();
                self.dp.set_as_input(Pull::None);
                self.dm.set_as_input(Pull::None);
                let regs = Registers(T::REGS);
                T::REGS.usb_ctrl().write_value(pac::usbhs::regs::UsbCtrl(9));
                regs.initialize();
                for index in 0..ENDPOINTS {
                    for incoming in [false, true] {
                        if let Some(config) = inner.endpoints[slot(index, incoming)].config {
                            regs.dma(index, incoming).write_value(config.buffer.address as u32);
                            regs.max_length(index)
                                .write_value(u32::from(config.info.max_packet_size));
                            if config.info.ep_type == EndpointType::Isochronous {
                                regs.iso(incoming).modify(|value| *value |= 1 << index);
                            }
                        }
                    }
                }
                T::REGS
                    .usb_base_mode()
                    .write(|reg| reg.set_ud_speed_type(if self.config.speed == Speed::High { 1 } else { 0 }));
                inner.powered = true;
                inner.suspended = false;
                reset_endpoints::<T>(&mut inner);
                T::REGS.usb_int_fg().write_value(pac::usbhs::regs::UsbIntFg(0xef));
                T::REGS.usb_int_en().write_value(pac::usbhs::regs::UsbIntEn(INTERRUPTS));
                fence(Ordering::SeqCst);
                T::REGS.usb_ctrl().write_value(pac::usbhs::regs::UsbCtrl(0x38));
                T::Interrupt::unpend();
                unsafe { T::Interrupt::enable() };
            });
        });
        wake_all::<T>();
    }

    fn power_off(&mut self) {
        critical_section::with(|_| {
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                if inner.powered {
                    T::REGS.usb_int_en().write_value(pac::usbhs::regs::UsbIntEn(0));
                    T::REGS.usb_ctrl().write_value(pac::usbhs::regs::UsbCtrl(3));
                    T::Interrupt::disable();
                    T::Interrupt::unpend();
                    T::disable();
                    T::set_utmi(false);
                    self.dp.set_as_disconnected();
                    self.dm.set_as_disconnected();
                }
                inner.powered = false;
                inner.suspended = false;
                inner.setup = None;
                for endpoint in &mut inner.endpoints {
                    endpoint.invalidate();
                }
            });
        });
        wake_all::<T>();
    }
}

impl<T: Instance> Drop for Bus<'_, T> {
    fn drop(&mut self) {
        self.power_off();
    }
}

impl<T: Instance> embassy_usb_driver::Bus for Bus<'_, T> {
    async fn enable(&mut self) {
        self.power_on();
    }

    async fn disable(&mut self) {
        self.power_off();
    }

    async fn poll(&mut self) -> Event {
        poll_fn(|context| {
            T::state().bus_waker.register(context.waker());
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                for (flag, event) in [
                    (POWER, Event::PowerDetected),
                    (RESET, Event::Reset),
                    (SUSPEND, Event::Suspend),
                    (RESUME, Event::Resume),
                ] {
                    if inner.events & flag != 0 {
                        inner.events &= !flag;
                        return Poll::Ready(event);
                    }
                }
                Poll::Pending
            })
        })
        .await
    }

    fn endpoint_set_enabled(&mut self, address: EndpointAddress, enabled: bool) {
        change_endpoint::<T>(address, enabled, false);
    }

    fn endpoint_set_stalled(&mut self, address: EndpointAddress, stalled: bool) {
        let index = address.index();
        if index >= ENDPOINTS {
            return;
        }
        let incoming = address.is_in();
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return;
            }
            let endpoint = &mut inner.endpoints[slot(index, incoming)];
            if endpoint.config.is_none() || !endpoint.enabled {
                return;
            }
            endpoint.generation = endpoint.generation.wrapping_add(1);
            endpoint.stalled = stalled;
            endpoint.rx_ready = false;
            endpoint.tx_busy = false;
            endpoint.tx_done = false;
            endpoint.overflow = false;
            Registers(T::REGS).control(index, incoming).write_value(if stalled {
                STALL
            } else if incoming {
                NAK
            } else {
                ACK
            });
        });
        T::state().endpoint_wakers[slot(index, incoming)].wake();
    }

    fn endpoint_is_stalled(&mut self, address: EndpointAddress) -> bool {
        address.index() < ENDPOINTS
            && T::state()
                .inner
                .lock(|inner| inner.borrow().endpoints[slot(address.index(), address.is_in())].stalled)
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        #[cfg(feature = "embassy")]
        {
            embassy_time::Timer::after_millis(5).await;
            T::state().inner.lock(|inner| {
                let inner = inner.borrow();
                if !inner.powered || !inner.suspended {
                    return Err(Unsupported);
                }
                T::REGS.usb_wake_ctrl().write(|reg| reg.set_ud_remote_wkup(true));
                Ok(())
            })
        }
        #[cfg(not(feature = "embassy"))]
        Err(Unsupported)
    }
}

fn change_endpoint<T: Instance>(address: EndpointAddress, enabled: bool, stalled: bool) {
    let index = address.index();
    if index >= ENDPOINTS || index == 0 {
        return;
    }
    let incoming = address.is_in();
    T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        if !inner.powered {
            return;
        }
        let endpoint = &mut inner.endpoints[slot(index, incoming)];
        if endpoint.config.is_none() {
            return;
        }
        let regs = Registers(T::REGS);
        regs.enables(incoming).modify(|value| *value &= !(1 << index));
        endpoint.invalidate();
        endpoint.enabled = enabled;
        endpoint.stalled = stalled;
        regs.control(index, incoming)
            .write_value(if !enabled || incoming { NAK } else { ACK });
        if enabled {
            regs.enables(incoming).modify(|value| *value |= 1 << index);
        }
    });
    T::state().endpoint_wakers[slot(index, incoming)].wake();
}

fn reset_endpoints<T: Instance>(inner: &mut Inner) {
    let regs = Registers(T::REGS);
    regs.enables(true).write_value(1);
    regs.enables(false).write_value(1);
    T::REGS.usb_dev_ad().write_value(pac::usbhs::regs::UsbDevAd(0));
    inner.setup = None;
    inner.suspended = false;
    for index in 0..ENDPOINTS {
        for incoming in [false, true] {
            let endpoint = &mut inner.endpoints[slot(index, incoming)];
            endpoint.invalidate();
            endpoint.enabled = index == 0;
            regs.control(index, incoming)
                .write_value(if index == 0 && !incoming { ACK } else { NAK });
        }
        regs.tx_length(index).write_value(0);
    }
}

fn wake_all<T: Instance>() {
    T::state().bus_waker.wake();
    for waker in &T::state().endpoint_wakers {
        waker.wake();
    }
}

pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let mut wake = 0u16;
        let mut bus = false;
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return;
            }
            let regs = Registers(T::REGS);
            let flags = T::REGS.usb_int_fg().read().0;
            if flags & 1 != 0 {
                reset_endpoints::<T>(&mut inner);
                T::REGS.usb_int_fg().write_value(pac::usbhs::regs::UsbIntFg(1));
                inner.events |= RESET;
                wake = u16::MAX;
                bus = true;
                return;
            }
            if flags & 0x80 != 0 {
                for endpoint in &mut inner.endpoints {
                    if endpoint.enabled {
                        endpoint.overflow = true;
                    }
                }
                wake = u16::MAX;
            }
            let rx0 = regs.control(0, false).read();
            let setup = rx0 & (DONE | 8) == (DONE | 8);
            if setup {
                let mut bytes = [0; 8];
                let length = regs.rx_length(0).read();
                if let Some(config) = inner.endpoints[0].config {
                    config.buffer.read(&mut bytes);
                }
                for incoming in [false, true] {
                    let endpoint = &mut inner.endpoints[slot(0, incoming)];
                    endpoint.invalidate();
                    endpoint.enabled = true;
                    regs.control(0, incoming).write_value(4);
                }
                inner.setup = if length == 8 { Some(bytes) } else { None };
                if length != 8 {
                    regs.control(0, true).write_value(STALL);
                    regs.control(0, false).write_value(STALL);
                }
                wake |= 3;
            }
            for index in 0..ENDPOINTS {
                if index == 0 && setup {
                    continue;
                }
                for incoming in [false, true] {
                    let control = if index == 0 && !incoming {
                        rx0
                    } else {
                        regs.control(index, incoming).read()
                    };
                    if control & DONE == 0 {
                        if control & 0x40 != 0 {
                            regs.control(index, incoming).write_value((control & !0x40) | DONE);
                        }
                        continue;
                    }
                    let endpoint = &mut inner.endpoints[slot(index, incoming)];
                    let isochronous = endpoint
                        .config
                        .is_some_and(|config| config.info.ep_type == EndpointType::Isochronous);
                    if !endpoint.enabled || endpoint.stalled {
                        regs.control(index, incoming)
                            .write_value((control & 0x0c) | if endpoint.stalled { STALL } else { NAK });
                        continue;
                    }
                    if incoming {
                        if endpoint.tx_busy {
                            endpoint.tx_busy = false;
                            endpoint.tx_done = true;
                        }
                        regs.control(index, true)
                            .write_value(if isochronous { 0 } else { (control ^ 4) & 0x0c });
                    } else if control & 0x10 != 0 || isochronous {
                        endpoint.rx_length = regs.rx_length(index).read();
                        endpoint.rx_ready = true;
                        regs.control(index, false)
                            .write_value(if isochronous { 0 } else { (control ^ 4) & 0x0c });
                    } else {
                        regs.control(index, false).write_value((control & 0x0c) | ACK);
                    }
                    wake |= 1 << slot(index, incoming);
                }
            }
            let clear = flags & 0xee;
            if clear & 0x46 != 0 {
                let status = T::REGS.usb_mis_st().read();
                let suspended = status.udms_suspend() || status.udms_sleep();
                if suspended != inner.suspended {
                    inner.suspended = suspended;
                    inner.events |= if suspended { SUSPEND } else { RESUME };
                    bus = true;
                }
            }
            if clear != 0 {
                T::REGS.usb_int_fg().write_value(pac::usbhs::regs::UsbIntFg(clear));
            }
        });
        if bus {
            T::state().bus_waker.wake();
        }
        for index in 0..SLOTS {
            if wake & (1 << index) != 0 {
                T::state().endpoint_wakers[index].wake();
            }
        }
    }
}

trait SealedInstance {
    const REGS: pac::usbhs::Usb;
    fn state() -> &'static State;
    fn set_utmi(enabled: bool);
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType {
    type Interrupt: Interrupt;
}

macro_rules! instance {
    ($name:ident, $set_utmi:ident) => {
        impl SealedInstance for peripherals::$name {
            const REGS: pac::usbhs::Usb = pac::$name;
            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
            fn set_utmi(enabled: bool) {
                pac::RCC.cfgr2().modify(|reg| reg.$set_utmi(enabled));
            }
        }
        impl Instance for peripherals::$name {
            type Interrupt = crate::_generated::peripheral_interrupts::$name::GLOBAL;
        }
    };
}

#[cfg(peri_usbhs1)]
instance!(USBHS1, set_utmi1on);
#[cfg(peri_usbhs2)]
instance!(USBHS2, set_utmi2on);

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);

#[cfg(test)]
#[path = "v4x7_tests.rs"]
mod tests;
