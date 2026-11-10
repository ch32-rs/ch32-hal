use core::cell::RefCell;
use core::future::poll_fn;
use core::sync::atomic::{fence, Ordering};
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointInfo, EndpointType, Event, Unsupported,
};

use super::control::ControlPipe;
use super::endpoint::Endpoint;
use super::regs::{Regs, LINK_HOT_RESET};
use super::{
    delay_cycles, init_clocks, Instance, InterruptHandler, LinkInterruptHandler, LinkState, LMP_PORT_CAP, LMP_PORT_CFG,
    LMP_PORT_CFG_RES, LMP_SET_LINK_FUNC, LMP_SUBTYPE_MASK, LMP_U2_INACT_TOUT, MODE_DEVICE,
};
use crate::interrupt::typelevel::Interrupt;
use crate::pac::usbss::regs;
use crate::pac::usbss::vals::{Ep0RxRes, Ep0TxRes};
use crate::{interrupt, pac, Peri};

pub(super) const ENDPOINTS: usize = 8;
const SLOTS: usize = ENDPOINTS * 2;
pub(super) const EP0_SIZE: usize = 512;

/// Most packets in one chain; the count registers are 8 bits wide. A chain
/// longer than a burst keeps the endpoint out of flow control in between.
const CHAIN_PACKETS: usize = 255;

/// `ERDY_NUMP` of an OUT endpoint while it is the only data endpoint with a
/// transfer queued, and while it is not. The host keeps one packet fewer
/// than this waiting for its ACK.
///
/// The controller buffers only three or four header packets for replay after
/// a link error. With more pending, counting the ACKs of OUT packets and the
/// headers of IN packets, it loses ACKs, which stalls the OUT endpoint, or
/// sends an IN packet with wrong data. `[3, 4]` keeps OUT at full speed next
/// to other traffic, where it exceeds that budget. In case of such issues,
/// change it to `[3, 2]`, which roughly halves OUT throughput while another
/// data endpoint has a transfer queued.
const OUT_WINDOW: [u8; 2] = [3, 4];

const RESET: u8 = 1;
const SUSPEND: u8 = 2;
const RESUME: u8 = 4;
const POWER: u8 = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidMemory,
    BufferTooSmall,
}

/// A piece of the endpoint memory.
#[derive(Clone, Copy)]
pub(super) struct Buffer {
    pub address: usize,
    pub size: usize,
}

impl Buffer {
    pub unsafe fn read(self, target: &mut [u8]) {
        fence(Ordering::Acquire);
        core::ptr::copy_nonoverlapping(self.address as *const u8, target.as_mut_ptr(), target.len());
    }

    pub unsafe fn write(self, source: &[u8]) {
        core::ptr::copy_nonoverlapping(source.as_ptr(), self.address as *mut u8, source.len());
        fence(Ordering::Release);
    }
}

#[derive(Clone, Copy)]
pub(super) struct EndpointConfig {
    pub info: EndpointInfo,
    /// For transfers the controller can't DMA directly.
    pub buffer: Buffer,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Phase {
    Idle,
    /// A transfer is handed to the controller.
    Busy,
    /// The transfer completed and its result is not consumed yet.
    Done,
}

/// A transfer, run one chain at a time by the interrupt handler.
#[derive(Clone, Copy)]
pub(super) struct Transfer {
    pub address: usize,
    /// IN: bytes to send. OUT: room at `address`, in whole packets.
    pub length: usize,
    /// IN: bytes handed to the controller. OUT: bytes received.
    pub progress: usize,
    /// IN: a ZLP is still to be sent.
    pub zlp: bool,
}

impl Transfer {
    /// Packets in the next chain.
    fn chain_packets(&self, mps: usize) -> usize {
        (self.length - self.progress).div_ceil(mps).clamp(1, CHAIN_PACKETS)
    }
}

#[derive(Clone, Copy)]
pub(super) struct EndpointState {
    pub config: Option<EndpointConfig>,
    pub enabled: bool,
    pub stalled: bool,
    pub phase: Phase,
    /// EP0 only uses `progress`, for the OUT length.
    pub transfer: Transfer,
}

impl EndpointState {
    const fn new() -> Self {
        Self {
            config: None,
            enabled: false,
            stalled: false,
            phase: Phase::Idle,
            transfer: Transfer {
                address: 0,
                length: 0,
                progress: 0,
                zlp: false,
            },
        }
    }

    /// Forgets the transfer in flight; its future then fails with `Disabled`.
    fn reset(&mut self, enabled: bool) {
        self.enabled = enabled;
        self.stalled = false;
        self.phase = Phase::Idle;
    }

    /// Hands the next chain of the transfer to the controller.
    pub fn queue(&mut self, regs: Regs) {
        let config = self.config.unwrap();
        let index = config.info.addr.index();
        let mps = usize::from(config.info.max_packet_size);
        let transfer = &mut self.transfer;
        let packets = transfer.chain_packets(mps);
        let remaining = transfer.length - transfer.progress;
        let address = (transfer.address + transfer.progress) as u32;
        if config.info.addr.is_in() {
            let bytes = remaining.min(packets * mps);
            let tx = regs.tx(index);
            if bytes == 0 {
                transfer.zlp = false;
                tx.dma().write_value(config.buffer.address as u32);
            } else {
                tx.dma().write_value(address);
            }
            tx.dma_ofs().write_value(mps as u16);
            tx.chain_len()
                .write(|w| w.set_len((bytes - (packets - 1) * mps) as u16));
            tx.chain_exp_nump().write_value(packets as u8);
            tx.chain_st().write(|w| w.set_chain_if(true));
            transfer.progress += bytes;
        } else {
            let rx = regs.rx(index);
            rx.dma().write_value(address);
            rx.dma_ofs().write_value(mps as u16);
            rx.chain_max_nump().write_value(packets as u8);
            rx.chain_st().write(|w| w.set_chain_if(true));
        }
    }
}

pub(super) struct Inner {
    pub regs: Regs,
    pub endpoints: [EndpointState; SLOTS],
    pub ep0: Buffer,
    pub powered: bool,
    suspended: bool,
    events: u8,
    /// Slots whose tasks the interrupt handler has to wake.
    wake: u16,
    pub setup: Option<[u8; 8]>,
    /// Counts SETUPs and bus resets; a control transfer started under an
    /// older count is dead.
    pub control_generation: u32,
    /// The armed EP0 IN packet ends the data stage.
    pub in_last: bool,
    /// Sequence number of the last EP0 IN packet.
    pub tx_seq: u8,
    /// Address to apply once the status stage completes.
    pub address: Option<u8>,
    revision: u8,
    reset_seen: bool,
    low_power: bool,
}

pub(super) struct State {
    pub inner: Mutex<CriticalSectionRawMutex, RefCell<Inner>>,
    bus_waker: AtomicWaker,
    /// EP0 uses slot 0 for both directions.
    pub endpoint_wakers: [AtomicWaker; SLOTS],
}

impl State {
    pub const fn new(regs: Regs) -> Self {
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                regs,
                endpoints: [EndpointState::new(); SLOTS],
                ep0: Buffer {
                    address: 0,
                    size: EP0_SIZE,
                },
                powered: false,
                suspended: false,
                events: 0,
                wake: 0,
                setup: None,
                control_generation: 0,
                in_last: false,
                tx_seq: 0,
                address: None,
                revision: 0,
                reset_seen: false,
                low_power: false,
            })),
            bus_waker: AtomicWaker::new(),
            endpoint_wakers: [const { AtomicWaker::new() }; SLOTS],
        }
    }

    fn wake(&self, bus: bool, slots: u16) {
        if bus {
            self.bus_waker.wake();
        }
        for (slot, waker) in self.endpoint_wakers.iter().enumerate() {
            if slots & (1 << slot) != 0 {
                waker.wake();
            }
        }
    }
}

pub(super) fn state<T: Instance>() -> &'static State {
    &T::state().device
}

pub(super) fn slot(index: usize, incoming: bool) -> usize {
    index * 2 + incoming as usize
}

fn packet_size_valid(kind: EndpointType, size: u16, interval: u8) -> bool {
    match kind {
        EndpointType::Bulk => size == 1024,
        EndpointType::Interrupt | EndpointType::Isochronous => (1..=1024).contains(&size) && interval != 0,
        EndpointType::Control => false,
    }
}

/// ITCM at `0x200A_0000` through the end of the shared SRAM.
fn dma_range(address: usize, length: usize) -> bool {
    const RAM_START: usize = 0x200A_0000;
    const RAM_END: usize = 0x2018_0000;
    address >= RAM_START && address.checked_add(length).is_some_and(|end| end <= RAM_END)
}

/// Whether the controller can DMA straight to or from `buffer`.
pub(super) fn dma_capable(buffer: &[u8]) -> bool {
    let address = buffer.as_ptr() as usize;
    !buffer.is_empty() && address % 4 == 0 && dma_range(address, buffer.len())
}

/// Silicon revision, `CHIP_ID[7:4]`.
fn revision() -> u8 {
    // The EVT `FLASH_BOOT_GetMode` reads flash before the ESIG area.
    unsafe { core::ptr::read_volatile(0x0800_0000 as *const u32) };
    ((crate::signature::chip_id().dev_id() >> 4) & 0xf) as u8
}

pub struct Driver<'d, T: Instance> {
    peri: Peri<'d, T>,
    memory: usize,
    capacity: usize,
    used: usize,
    endpoints: [Option<EndpointConfig>; SLOTS],
}

impl<'d, T: Instance> Driver<'d, T> {
    /// `endpoint_memory` holds 512 bytes for EP0 plus one burst per endpoint,
    /// used when a transfer buffer is outside the controller's DMA range
    /// (`0x200A_0000..0x2018_0000`) or not 4-byte aligned. A burst is
    /// `max_burst + 1` packets of a bulk endpoint, with the `max_burst` of its
    /// `EndpointOptions`, and one packet of any other endpoint.
    pub fn new(
        peri: Peri<'d, T>,
        _irqs: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>
            + interrupt::typelevel::Binding<T::LinkInterrupt, LinkInterruptHandler<T>>
            + 'd,
        endpoint_memory: &'d mut [u8],
    ) -> Result<Self, Error> {
        let address = endpoint_memory.as_mut_ptr() as usize;
        if !dma_range(address, endpoint_memory.len()) {
            return Err(Error::InvalidMemory);
        }
        let aligned = (address + 3) & !3;
        let capacity = endpoint_memory.len().saturating_sub(aligned - address);
        if capacity < EP0_SIZE {
            return Err(Error::BufferTooSmall);
        }
        Ok(Self {
            peri,
            memory: aligned,
            capacity,
            used: EP0_SIZE,
            endpoints: [None; SLOTS],
        })
    }

    /// Streams and the isochronous `Mult` are not implemented.
    fn allocate<const IN: bool>(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Endpoint<'d, T, IN>, EndpointAllocError> {
        let embassy_usb_driver::EndpointConfig {
            addr: requested,
            ep_type,
            max_packet_size,
            interval_ms,
            max_burst,
            ..
        } = *config;
        if !packet_size_valid(ep_type, max_packet_size, interval_ms)
            || max_burst > 15
            || config.max_streams != 0
            || config.mult != 0
        {
            return Err(EndpointAllocError);
        }
        let direction = if IN { Direction::In } else { Direction::Out };
        if requested.is_some_and(|address| address.direction() != direction) {
            return Err(EndpointAllocError);
        }
        let index = (1..ENDPOINTS)
            .find(|index| {
                requested.is_none_or(|address| address.index() == *index) && self.endpoints[slot(*index, IN)].is_none()
            })
            .ok_or(EndpointAllocError)?;
        let size = usize::from(max_packet_size) * (usize::from(max_burst) + 1);
        let start = (self.used + 3) & !3;
        if start + size > self.capacity {
            return Err(EndpointAllocError);
        }
        let info = EndpointInfo {
            addr: EndpointAddress::from_parts(index, direction),
            ep_type,
            max_packet_size,
            interval_ms,
            max_burst,
        };
        let buffer = Buffer {
            address: self.memory + start,
            size,
        };
        self.endpoints[slot(index, IN)] = Some(EndpointConfig { info, buffer });
        self.used = start + size;
        Ok(Endpoint::new(info, buffer))
    }
}

impl<'d, T: Instance> embassy_usb_driver::Driver<'d> for Driver<'d, T> {
    type EndpointOut = Endpoint<'d, T, false>;
    type EndpointIn = Endpoint<'d, T, true>;
    type ControlPipe = ControlPipe<'d, T>;
    type Bus = Bus<'d, T>;

    fn alloc_endpoint_out(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.allocate(config)
    }

    fn alloc_endpoint_in(
        &mut self,
        config: &embassy_usb_driver::EndpointConfig,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.allocate(config)
    }

    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert_eq!(
            usize::from(control_max_packet_size),
            EP0_SIZE,
            "USBSS needs a SuperSpeed configuration"
        );
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            for (state, config) in inner.endpoints.iter_mut().zip(self.endpoints) {
                state.reset(false);
                state.config = config;
            }
            inner.ep0.address = self.memory;
            inner.powered = false;
            inner.events = POWER;
            inner.setup = None;
        });
        (Bus { _peri: self.peri }, ControlPipe::new())
    }
}

pub struct Bus<'d, T: Instance> {
    _peri: Peri<'d, T>,
}

impl<T: Instance> Bus<'_, T> {
    /// Current link state.
    pub fn link_state(&self) -> LinkState {
        T::device().link_status().read().link_state()
    }

    fn power_on(&mut self) {
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if inner.powered {
                return;
            }
            T::state().mode.store(MODE_DEVICE, Ordering::Relaxed);
            inner.revision = revision();
            unsafe { init_clocks::<T>() };
            inner.regs.init(inner.revision);
            inner.powered = true;
            inner.bus_reset();
            inner.reset_seen = false;
            T::Interrupt::unpend();
            T::LinkInterrupt::unpend();
            unsafe {
                T::Interrupt::enable();
                T::LinkInterrupt::enable();
            }
        });
        state::<T>().wake(true, !0);
    }

    fn power_off(&mut self) {
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if inner.powered {
                T::Interrupt::disable();
                T::LinkInterrupt::disable();
                T::Interrupt::unpend();
                T::LinkInterrupt::unpend();
                inner.regs.shutdown();
                T::disable();
                // UTMI stays on: the gate is shared with USBHS.
                pac::RCC.cfgr0().modify(|w| w.set_pipeon(false));
            }
            inner.powered = false;
            inner.setup = None;
            for endpoint in &mut inner.endpoints {
                endpoint.reset(false);
            }
        });
        state::<T>().wake(true, !0);
    }

    /// Runs `f` on a data endpoint of the powered controller, then wakes its
    /// task.
    fn with_endpoint(address: EndpointAddress, f: impl FnOnce(Regs, &mut EndpointState, EndpointConfig)) {
        if address.index() == 0 || address.index() >= ENDPOINTS {
            return;
        }
        let slot = slot(address.index(), address.is_in());
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let (regs, powered) = (inner.regs, inner.powered);
            let endpoint = &mut inner.endpoints[slot];
            if let (true, Some(config)) = (powered, endpoint.config) {
                f(regs, endpoint, config);
            }
        });
        state::<T>().endpoint_wakers[slot].wake();
    }

    /// Sets or clears the STALL handshake of an endpoint, for the bus and
    /// the endpoint alike.
    pub(super) fn set_endpoint_stalled(address: EndpointAddress, stalled: bool) {
        // An EP0 stall is a protocol stall that the next SETUP clears.
        let (index, incoming) = (address.index(), address.is_in());
        Self::with_endpoint(address, |regs, endpoint, _| {
            if !endpoint.enabled {
                return;
            }
            if endpoint.phase == Phase::Busy {
                regs.abort(index, incoming);
            }
            endpoint.phase = Phase::Idle;
            endpoint.stalled = stalled;
            if stalled {
                regs.halt(index, incoming);
            } else {
                regs.clear(index, incoming);
            }
        });
    }

    pub(super) fn is_endpoint_stalled(address: EndpointAddress) -> bool {
        address.index() < ENDPOINTS
            && state::<T>()
                .inner
                .lock(|inner| inner.borrow().endpoints[slot(address.index(), address.is_in())].stalled)
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
            state::<T>().bus_waker.register(context.waker());
            state::<T>().inner.lock(|inner| {
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
        let (index, incoming) = (address.index(), address.is_in());
        Self::with_endpoint(address, |regs, endpoint, config| {
            endpoint.reset(enabled);
            regs.set_enabled(index, incoming, false);
            if enabled {
                let iso = config.info.ep_type == EndpointType::Isochronous;
                regs.configure(index, incoming, iso);
                regs.set_enabled(index, incoming, true);
            } else {
                regs.clear(index, incoming);
            }
        });
    }

    fn endpoint_set_stalled(&mut self, address: EndpointAddress, stalled: bool) {
        Self::set_endpoint_stalled(address, stalled);
    }

    fn endpoint_is_stalled(&mut self, address: EndpointAddress) -> bool {
        Self::is_endpoint_stalled(address)
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        state::<T>().inner.lock(|inner| {
            let inner = inner.borrow();
            if !inner.powered || !inner.suspended {
                return Err(Unsupported);
            }
            inner.regs.link_ctrl().modify(|w| w.set_link_tx_ux_exit(true));
            Ok(())
        })
    }
}

impl Inner {
    /// `ERDY_NUMP` for the OUT endpoints with a transfer queued.
    fn out_window(&self) -> u8 {
        let queued = self.endpoints[2..].iter().filter(|e| e.phase != Phase::Idle).count();
        OUT_WINDOW[usize::from(queued > 1)]
    }

    /// Hands the first chain of the transfer on `slot` to the controller.
    pub fn start(&mut self, slot: usize) {
        self.endpoints[slot].phase = Phase::Busy;
        let window = self.out_window();
        for index in 1..ENDPOINTS {
            if self.endpoints[self::slot(index, false)].phase == Phase::Busy {
                self.regs.set_out_window(index, window);
            }
        }
        self.endpoints[slot].queue(self.regs);
    }

    /// A reset supersedes pending suspend/resume events, and a suspend and
    /// resume that are both pending cancel out.
    fn raise(&mut self, event: u8) {
        match event {
            RESET => self.events = (self.events & POWER) | RESET,
            SUSPEND if self.events & RESUME != 0 => self.events &= !RESUME,
            RESUME if self.events & SUSPEND != 0 => self.events &= !SUSPEND,
            _ => self.events |= event,
        }
    }

    /// Ends the control transfer in progress.
    fn control_reset(&mut self) {
        self.control_generation = self.control_generation.wrapping_add(1);
        self.address = None;
        self.in_last = false;
        for incoming in [false, true] {
            self.endpoints[slot(0, incoming)].reset(true);
        }
        self.wake |= 1;
    }

    /// Resets the protocol layer, endpoints and address, and drops every
    /// pending transfer.
    fn bus_reset(&mut self) {
        self.regs.protocol_reset(self.ep0.address);
        self.regs.link_power_allow(false);
        self.setup = None;
        self.suspended = false;
        self.low_power = false;
        for endpoint in &mut self.endpoints {
            endpoint.reset(false);
        }
        self.control_reset();
        self.wake = !0;
    }

    /// A HOT_RESET or WARM_RESET from the host.
    fn link_reset(&mut self) {
        self.bus_reset();
        self.reset_seen = true;
        self.raise(RESET);
    }

    /// Returns `false` when the transfer flags must be left for the next
    /// interrupt.
    fn link_irq(&mut self) -> bool {
        let r = self.regs;
        let flags = r.link_int_flag().read();
        let state = r.link_status().read().link_state();

        if self.revision >= 3 && flags.link_if_rx_set_fc() {
            r.link_int_flag().write(|w| w.set_link_if_rx_set_fc(true));
            r.link_power_allow(r.link_lmp_port_cap().read().force_pm());
        }

        if flags.link_if_state_chg() {
            r.link_int_flag().write(|w| w.set_link_if_state_chg(true));
            match state {
                LinkState::RXDET => {
                    let connected = self.reset_seen;
                    self.bus_reset();
                    self.reset_seen = false;
                    if connected {
                        self.raise(SUSPEND);
                    }
                }
                LinkState::RECOVERY => {
                    if self.low_power {
                        self.low_power = false;
                        delay_cycles(4000);
                        r.phy_write(0x12, 0x67c8);
                    }
                }
                LinkState::DISABLED => r.link_ctrl().modify(|w| w.set_link_go_disabled(false)),
                LinkState::U0 => {
                    if !self.reset_seen {
                        // The first U0 is the only sign that link training
                        // finished.
                        self.reset_seen = true;
                        self.raise(RESET);
                    } else if self.suspended {
                        self.suspended = false;
                        self.raise(RESUME);
                    }
                }
                LinkState::U1 | LinkState::U2 | LinkState::U3 => {
                    self.low_power = true;
                    r.phy_write(0x12, 0x67c8 & !(1 << 9));
                    if state == LinkState::U3 && !self.suspended && self.reset_seen {
                        self.suspended = true;
                        self.raise(SUSPEND);
                    }
                }
                LinkState::HOTRST => {
                    self.link_reset();
                    r.link_ctrl()
                        .modify(|w| w.set_link_tx_ts_cfg(w.link_tx_ts_cfg() & !LINK_HOT_RESET));
                }
                _ => {}
            }
        }

        if flags.link_if_term_pres() {
            r.link_int_flag().write(|w| w.set_link_if_term_pres(true));
            if !r.link_status().read().link_rx_term_pres() {
                if self.reset_seen {
                    self.raise(SUSPEND);
                }
                self.reset_seen = false;
            }
        }

        if flags.link_if_tx_lmp() {
            r.link_int_flag().write(|w| w.set_link_if_tx_lmp(true));
            r.tx_port_cap();
        }

        if flags.link_if_rx_lmp() {
            r.link_int_flag().write(|w| w.set_link_if_rx_lmp(true));
            let data0 = r.link_lmp_rx_data(0).read();
            match data0 & LMP_SUBTYPE_MASK {
                LMP_PORT_CAP => r.tx_port_cap(),
                LMP_PORT_CFG => r.tx_port_cfg_res(),
                LMP_PORT_CFG_RES => r.link_lmp_port_cap().modify(|w| w.set_link_lmp_tx_cap_vld(true)),
                LMP_U2_INACT_TOUT => r
                    .link_u2_inact_timer()
                    .write(|w| w.set_u2_inactive_timer((data0 >> 9) as u8)),
                // Revision 3 reports this through RX_SET_FC instead.
                LMP_SET_LINK_FUNC if self.revision < 3 => r.link_power_allow(data0 & (0x02 << 9) != 0),
                _ => {}
            }
        }

        if flags.link_if_rx_lmp_tout() {
            r.link_int_flag().write(|w| w.set_link_if_rx_lmp_tout(true));
            r.link_ctrl().modify(|w| w.set_link_go_disabled(true));
            r.link_ctrl().modify(|w| w.set_link_go_rx_det(true));
            return false;
        }

        if flags.link_if_warm_rst() {
            r.link_int_flag().write(|w| w.set_link_if_warm_rst(true));
            if r.link_status().read().link_rx_warm_rst() {
                self.link_reset();
                r.link_ctrl().modify(|w| w.set_link_go_disabled(true));
                delay_cycles(16);
                r.link_ctrl().modify(|w| w.set_link_go_disabled(false));
            }
        }

        true
    }

    fn transfer_irq(&mut self) {
        let r = self.regs;
        let status = r.status().read();

        if status.udif_setup() && !status.udif_status() {
            r.uep0_tx_ctrl().write_value(regs::Uep0TxCtrl(0));
            r.uep0_rx_ctrl().write_value(regs::Uep0RxCtrl(0));
            r.status().write(|w| w.set_udif_setup(true));
            let mut setup = [0; 8];
            unsafe { self.ep0.read(&mut setup) };
            self.control_reset();
            self.setup = Some(setup);
        } else if status.udif_status() {
            r.status().write(|w| w.set_udif_status(true));
            if let Some(address) = self.address.take() {
                r.usbss_ctrl().modify(|w| w.set_dev_addr(address));
            }
            r.uep0_tx_ctrl().write_value(regs::Uep0TxCtrl(0));
            r.ep0_rx_ready();
            self.complete(slot(0, true));
        } else if status.uif_transfer() {
            match (usize::from(status.ep_id()), status.ep_dir()) {
                (0, incoming) => self.ep0_irq(incoming),
                (index, incoming) if index < ENDPOINTS => self.data_irq(index, incoming),
                _ => {}
            }
        } else {
            r.status().write(|w| {
                w.set_uif_itp(status.uif_itp());
                w.set_uif_rx_ping(status.uif_rx_ping());
                w.set_uif_fifo_txov(status.uif_fifo_txov());
                w.set_uif_fifo_rxov(status.uif_fifo_rxov());
            });
        }
    }

    /// Finishes the slot's transfer, if one is in flight, and wakes its task.
    fn complete(&mut self, slot: usize) {
        if self.endpoints[slot].phase == Phase::Busy {
            self.endpoints[slot].phase = Phase::Done;
            // EP0 waits on slot 0 for both directions.
            self.wake |= 1 << if slot < 2 { 0 } else { slot };
        }
    }

    fn ep0_irq(&mut self, incoming: bool) {
        let r = self.regs;
        if !incoming {
            self.endpoints[0].transfer.progress = usize::from(r.uep0_rx_ctrl().read().ep0_rx_len());
            r.uep0_rx_ctrl().write(|w| w.set_ep0_rx_res(Ep0RxRes::NRDY));
            self.complete(0);
        } else {
            self.tx_seq = r.uep0_tx_ctrl().read().ep0_tx_seq();
            if self.in_last || self.endpoints[1].phase != Phase::Busy {
                // The status stage follows, and its interrupt completes the
                // slot.
                r.uep0_tx_ctrl().write(|w| w.set_ep0_tx_res(Ep0TxRes::DPH));
                r.ep0_rx_ready();
            } else {
                // NRDY until the next data-stage packet is ready.
                r.uep0_tx_ctrl().write(|w| w.set_ep0_tx_res(Ep0TxRes::NRDY));
                self.complete(1);
            }
        }
    }

    /// A chain ended: starts the transfer's next one or completes it.
    fn data_irq(&mut self, index: usize, incoming: bool) {
        let r = self.regs;
        let (packets, last) = if incoming {
            r.tx(index).chain_st().write(|w| w.set_chain_if(true));
            (0, 0)
        } else {
            let rx = r.rx(index);
            let packets = usize::from(rx.chain_nump().read());
            let last = usize::from(rx.chain_len().read().len());
            rx.chain_st().write(|w| w.set_chain_if(true));
            (packets, last)
        };
        let slot = slot(index, incoming);
        let endpoint = &mut self.endpoints[slot];
        let Some(config) = endpoint.config.filter(|_| endpoint.phase == Phase::Busy) else {
            return;
        };
        let mps = usize::from(config.info.max_packet_size);
        let transfer = &mut endpoint.transfer;
        let more = if incoming {
            transfer.progress < transfer.length || transfer.zlp
        } else {
            let expected = transfer.chain_packets(mps);
            let received = (packets.clamp(1, expected) - 1) * mps + last.min(mps);
            transfer.progress = transfer.length.min(transfer.progress + received);
            // Only a short packet ends the transfer early. The chain also ends
            // when the host has no more packets pending.
            last >= mps && transfer.progress < transfer.length
        };
        if more {
            endpoint.queue(r);
            if !incoming {
                r.set_out_window(index, self.out_window());
            }
        } else {
            self.complete(slot);
        }
    }
}

pub(super) unsafe fn on_interrupt<T: Instance>() {
    let (bus, slots) = state::<T>().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        if inner.powered && inner.link_irq() {
            inner.transfer_irq();
        }
        (inner.events != 0, core::mem::take(&mut inner.wake))
    });
    state::<T>().wake(bus, slots);
}
