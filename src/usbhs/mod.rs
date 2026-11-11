//! CH32V30x, CH32V407/CH32V467 and CH32H417 USBHS device driver for Embassy.
//!
//! The H4 block keeps the V4x7 register layout; V30x has its own. Endpoints 1–7
//! (1–15 on V30x) support independent IN and OUT allocations. On V4x7/H4 a pair
//! shares its maximum packet length register and therefore must use the same
//! packet size. On V30x isochronous and bulk / interrupt endpoints must use
//! different numbers: a PING to a bulk OUT whose number is isochronous on the
//! IN side hangs the SIE.
//! Endpoint memory stays borrowed for the lifetime of the bus and endpoints;
//! DMA never accesses a caller's temporary read/write slice. Canceling an IN
//! write may leave its owned packet queued until the host consumes it or the
//! endpoint is disabled. A subsequent write drains that packet before reuse.
//!
//! Configure the shared 480 MHz USBHS PLL through RCC before constructing the
//! driver. High speed is the default; full-speed descriptors and fallback must
//! be handled by the USB stack/application. No VBUS input is sampled: the bus
//! reports power available and cannot report cable removal.
//!
//! Isochronous endpoints run without a task in the loop:
//! - OUT: a ring of [`ISO_OUT_SLOTS`] packet buffers. The interrupt points the endpoint's
//!   DMA at the next free slot after each packet; `read` drains the ring, or
//!   [`iso_out_take`] in place. A full ring drops the newest packet, and on V30x
//!   zero-length packets are not delivered (arrivals are detected by a marker in the slot).
//! - IN: a ring of [`ISO_IN_SLOTS`] packets. After each IN transaction the interrupt
//!   loads the next queued packet, or a zero-length one when none is queued. `write`
//!   waits for a free slot, so a writer keeps the ring full and stays at most
//!   [`ISO_IN_SLOTS`] packets ahead; [`iso_in_prime`] queues from an interrupt (e.g. an
//!   explicit feedback value from the SOF callback), [`iso_in_fill`] in place.
//! - Both work from the SOF callback, without a task: one packet each way per microframe.
//! - Each slot takes 4 bytes plus the max packet size, rounded up to 4, of the endpoint
//!   memory.
//!
//! [`set_sof_callback`] runs a function from the interrupt on every SOF (125 us at high
//! speed).
//!
//! Not supported: remote wakeup on V30x, high-bandwidth isochronous (more than one
//! transaction per microframe), hardware double buffering.

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

use crate::gpio::AnyPin;
#[cfg(not(usbhs_h4))]
use crate::gpio::SealedPin;
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peri, RccPeripheral};

mod control;
mod endpoint;
mod iso;
mod regs;

pub use control::ControlPipe;
pub use embassy_usb_driver::Speed;
pub use endpoint::Endpoint;
pub use iso::{
    endpoint_enabled, iso_in_fill, iso_in_pending, iso_in_prime, iso_in_register_waker, iso_out_overruns, iso_out_take,
    microframe, set_sof_callback, ISO_IN_SLOTS, ISO_OUT_SLOTS,
};
use regs::{Res, Wake, ENDPOINTS};

const SLOTS: usize = ENDPOINTS * 2;
const RESET: u8 = 1;
const SUSPEND: u8 = 2;
const RESUME: u8 = 4;
const POWER: u8 = 8;

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Config {
    pub speed: Speed,
}

impl Default for Config {
    fn default() -> Self {
        Self { speed: Speed::High }
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
    rx_wanted: u16,
    overflow: bool,
    head: u8,
    tail: u8,
    length: u16,
    armed: bool,
    /// IN: slot `tail` is loaded in the endpoint (else a zero-length packet is)
    loaded: bool,
    /// OUT: packets dropped on a full ring
    overruns: u32,
    /// isochronous OUT: the shortest packet since enabled (marks below it get overwritten)
    #[cfg(usbhs_v3)]
    min_len: u16,
}

impl EndpointState {
    const fn new() -> Self {
        Self {
            config: None,
            enabled: false,
            stalled: false,
            generation: 0,
            rx_wanted: 0,
            overflow: false,
            head: 0,
            tail: 0,
            length: 0,
            armed: false,
            loaded: false,
            overruns: 0,
            #[cfg(usbhs_v3)]
            min_len: 0,
        }
    }

    fn invalidate(&mut self) {
        let config = self.config;
        let generation = self.generation.wrapping_add(1);
        let overruns = self.overruns;
        *self = Self::new();
        self.config = config;
        self.generation = generation;
        self.overruns = overruns;
    }

    fn isochronous(&self) -> bool {
        self.config
            .is_some_and(|config| config.info.ep_type == EndpointType::Isochronous)
    }

    fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.armed = false;
    }

    fn arm_out(&mut self, regs: regs::Regs, index: usize) {
        if !self.armed && self.queued() == 0 {
            self.armed = true;
            regs::arm_out(regs, index);
        }
    }
}

struct Inner {
    endpoints: [EndpointState; SLOTS],
    powered: bool,
    suspended: bool,
    events: u8,
    setup: Option<[u8; 8]>,
    sof: Option<fn()>,
    /// The frame number of the last SOF handled
    #[cfg(usbhs_v3)]
    sof_frame: u16,
    /// Isochronous IN endpoints, a bit each
    #[cfg(usbhs_v3)]
    iso_in: u16,
    /// Isochronous OUT endpoints, a bit each
    #[cfg(usbhs_v3)]
    iso_out: u16,
}

impl Inner {
    const fn new() -> Self {
        Self {
            endpoints: [EndpointState::new(); SLOTS],
            powered: false,
            suspended: false,
            events: 0,
            setup: None,
            sof: None,
            #[cfg(usbhs_v3)]
            sof_frame: regs::NO_FRAME,
            #[cfg(usbhs_v3)]
            iso_in: 0,
            #[cfg(usbhs_v3)]
            iso_out: 0,
        }
    }

    fn reset(&mut self) {
        self.setup = None;
        self.suspended = false;
        self.events |= RESET;
        for (slot, endpoint) in self.endpoints.iter_mut().enumerate() {
            endpoint.invalidate();
            endpoint.enabled = slot < 2;
        }
    }

    fn setup(&mut self, packet: Option<[u8; 8]>) {
        for endpoint in &mut self.endpoints[..2] {
            endpoint.invalidate();
            endpoint.enabled = true;
        }
        self.setup = packet;
    }

    #[cfg(not(usbhs_v3))]
    fn overflow(&mut self) {
        for endpoint in &mut self.endpoints {
            if endpoint.enabled {
                endpoint.overflow = true;
            }
        }
    }

    fn in_done(&mut self, index: usize) {
        let endpoint = &mut self.endpoints[slot(index, true)];
        if endpoint.queued() != 0 {
            endpoint.tail = endpoint.tail.wrapping_add(1);
        }
    }

    fn out_done(&mut self, index: usize, length: u16) {
        let endpoint = &mut self.endpoints[slot(index, false)];
        endpoint.rx_wanted = 0;
        endpoint.length = length;
        endpoint.head = endpoint.head.wrapping_add(1);
        endpoint.armed = false;
    }

    fn set_suspended(&mut self, suspended: bool) -> bool {
        if suspended == self.suspended {
            return false;
        }
        self.suspended = suspended;
        self.events |= if suspended { SUSPEND } else { RESUME };
        true
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

fn packet_size_valid(speed: Speed, kind: EndpointType, size: u16) -> bool {
    match (speed, kind) {
        (Speed::Full, EndpointType::Bulk) => matches!(size, 8 | 16 | 32 | 64),
        (Speed::High, EndpointType::Bulk) => size == 512,
        (Speed::Full, EndpointType::Interrupt) => (1..=64).contains(&size),
        (Speed::High, EndpointType::Interrupt) => (1..=1024).contains(&size),
        (Speed::Full, EndpointType::Isochronous) => (1..=1023).contains(&size),
        (Speed::High, EndpointType::Isochronous) => (1..=1024).contains(&size),
        _ => false,
    }
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
        if !regs::clock_ready() {
            return Err(Error::ClockNotReady);
        }
        let address = endpoint_memory.as_mut_ptr() as usize;
        #[cfg(target_arch = "riscv32")]
        {
            if !regs::memory_valid(address, endpoint_memory.len()) {
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
        if !packet_size_valid(self.config.speed, ep_type, max_packet_size) {
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
                        .is_none_or(|other| regs::can_pair(&other.info, ep_type, max_packet_size))
            })
            .ok_or(EndpointAllocError)?;
        let end = self.used + buffer_size(ep_type, IN, max_packet_size);
        if end > self.capacity {
            return Err(EndpointAllocError);
        }
        let info = EndpointInfo {
            addr: EndpointAddress::from_parts(index, direction),
            ep_type,
            max_packet_size,
            interval_ms,
        };
        let mut address = self.memory + self.used;
        if ep_type == EndpointType::Isochronous {
            address += 4;
        }
        self.endpoints[slot(index, IN)] = Some(EndpointConfig {
            info,
            buffer: Buffer {
                address,
                size: max_packet_size,
            },
        });
        self.used = end;
        Ok(Endpoint::new(info))
    }
}

fn buffer_size(kind: EndpointType, incoming: bool, max_packet_size: u16) -> usize {
    let size = if kind == EndpointType::Isochronous {
        iso::ring_size(incoming, max_packet_size)
    } else {
        usize::from(max_packet_size)
    };
    (size + 3) & !3
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
                    size: control_max_packet_size,
                },
            });
        }
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            for (state, config) in inner.endpoints.iter_mut().zip(self.endpoints) {
                state.invalidate();
                state.config = config;
            }
            #[cfg(usbhs_v3)]
            {
                inner.iso_in = 0;
                inner.iso_out = 0;
                for config in self.endpoints.iter().flatten() {
                    let info = config.info;
                    if info.ep_type != EndpointType::Isochronous {
                        continue;
                    }
                    if info.addr.is_in() {
                        inner.iso_in |= 1 << info.addr.index();
                    } else {
                        inner.iso_out |= 1 << info.addr.index();
                    }
                }
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
    // Held for ownership only on H4, where the pins connect to the internal
    // PHY without any GPIO configuration.
    #[cfg_attr(usbhs_h4, allow(dead_code))]
    dp: Peri<'d, AnyPin>,
    #[cfg_attr(usbhs_h4, allow(dead_code))]
    dm: Peri<'d, AnyPin>,
    config: Config,
    _memory: PhantomData<&'d mut [u8]>,
}

impl<T: Instance> Bus<'_, T> {
    fn power_on(&mut self) {
        assert!(regs::clock_ready(), "USBHS PLL is not ready");
        critical_section::with(|_| {
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                if inner.powered {
                    return;
                }
                T::set_utmi(true);
                T::enable_and_reset();
                // On H4 the muxed USB pins connect to the internal 480 MHz PHY
                // as soon as the block is clocked (RM table 9-11), so they must
                // be left alone. V4x7 drives them from the UTMI PHY and needs
                // them released to the function.
                #[cfg(usbhs_v4x7)]
                {
                    self.dp.set_as_input(crate::gpio::Pull::None);
                    self.dm.set_as_input(crate::gpio::Pull::None);
                }
                #[cfg(usbhs_v3)]
                {
                    use crate::gpio::{AFType, Speed};
                    self.dp.set_as_af_output(AFType::OutputPushPull, Speed::High);
                    self.dm.set_as_af_output(AFType::OutputPushPull, Speed::High);
                }
                regs::power_on(T::REGS, self.config.speed);
                for index in 0..ENDPOINTS {
                    for incoming in [false, true] {
                        if let Some(config) = inner.endpoints[slot(index, incoming)].config {
                            regs::program(
                                T::REGS,
                                index,
                                incoming,
                                config.buffer.address as u32,
                                config.info.max_packet_size,
                                config.info.ep_type,
                            );
                        }
                    }
                }
                inner.powered = true;
                inner.suspended = false;
                regs::reset_endpoints(T::REGS);
                inner.reset();
                inner.events &= !RESET;
                regs::connect(T::REGS, inner.sof.is_some());
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
                    regs::power_off(T::REGS);
                    T::Interrupt::disable();
                    T::Interrupt::unpend();
                    T::disable();
                    T::set_utmi(false);
                    #[cfg(not(usbhs_h4))]
                    {
                        self.dp.set_as_disconnected();
                        self.dm.set_as_disconnected();
                    }
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
            endpoint.rx_wanted = 0;
            endpoint.overflow = false;
            if endpoint.isochronous() {
                if stalled {
                    regs::restart(T::REGS, index, incoming, Res::Stall);
                } else {
                    regs::iso_resume(T::REGS, index, incoming);
                }
            } else {
                endpoint.clear();
                regs::restart(T::REGS, index, incoming, if stalled { Res::Stall } else { Res::Nak });
                if !stalled && !incoming {
                    endpoint.arm_out(T::REGS, index);
                }
            }
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
                if !inner.powered || !inner.suspended || !regs::remote_wakeup(T::REGS) {
                    return Err(Unsupported);
                }
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
        let Some(config) = endpoint.config else {
            return;
        };
        endpoint.invalidate();
        endpoint.enabled = enabled;
        endpoint.stalled = stalled;
        regs::enable(T::REGS, index, incoming, enabled, config.info.ep_type, Res::Nak);
        if enabled && endpoint.isochronous() {
            regs::iso_start(T::REGS, index, incoming, endpoint);
        } else if enabled && !incoming {
            endpoint.arm_out(T::REGS, index);
        }
    });
    T::state().endpoint_wakers[slot(index, incoming)].wake();
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
        let wake = T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return Wake::default();
            }
            regs::interrupt(T::REGS, &mut inner)
        });
        if let Some(sof) = wake.sof {
            sof();
            // V30x: an iso IN loads only after its IN went out (`T_TOG`)
            #[cfg(not(usbhs_v3))]
            if wake.reload != 0 {
                T::state().inner.lock(|inner| {
                    let mut inner = inner.borrow_mut();
                    if inner.powered {
                        regs::iso_reload(T::REGS, &mut inner, wake.reload);
                    }
                });
            }
        }
        if wake.bus {
            T::state().bus_waker.wake();
        }
        for index in 0..SLOTS {
            if wake.endpoints & (1 << index) != 0 {
                T::state().endpoint_wakers[index].wake();
            }
        }
    }
}

#[cfg(usbhs_v3)]
pub struct WakeupInterruptHandler<T: Instance>(PhantomData<T>);

#[cfg(usbhs_v3)]
impl<T: Instance> interrupt::typelevel::Handler<T::WakeupInterrupt> for WakeupInterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::WakeupInterrupt::disable();
    }
}

trait SealedInstance {
    const REGS: regs::Regs;
    fn state() -> &'static State;
    fn set_utmi(enabled: bool);
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + embassy_hal_internal::PeripheralType {
    type Interrupt: Interrupt;
    #[cfg(usbhs_v3)]
    type WakeupInterrupt: Interrupt;
}

// On V4x7 each instance has its own UTMI gate (`UTMI1ON`/`UTMI2ON` in
// `CFGR2`); H4 has a single one (`UTMION` in `CFGR0`) for the whole block,
// and V30x none.
macro_rules! instance {
    ($name:ident, $set_utmi:ident) => {
        impl SealedInstance for peripherals::$name {
            const REGS: regs::Regs = pac::$name;
            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
            fn set_utmi(_enabled: bool) {
                #[cfg(usbhs_v4x7)]
                pac::RCC.cfgr2().modify(|reg| reg.$set_utmi(_enabled));
                #[cfg(usbhs_h4)]
                pac::RCC.cfgr0().modify(|reg| reg.set_utmion(_enabled));
            }
        }
        impl Instance for peripherals::$name {
            type Interrupt = crate::_generated::peripheral_interrupts::$name::GLOBAL;
            #[cfg(usbhs_v3)]
            type WakeupInterrupt = crate::_generated::peripheral_interrupts::$name::WAKEUP;
        }
    };
}

#[cfg(peri_usbhs1)]
instance!(USBHS1, set_utmi1on);
#[cfg(peri_usbhs2)]
instance!(USBHS2, set_utmi2on);
// The second macro argument only feeds the V4x7 branch above.
#[cfg(peri_usbhs)]
instance!(USBHS, set_utmion);

pin_trait!(DpPin, Instance);
pin_trait!(DmPin, Instance);
