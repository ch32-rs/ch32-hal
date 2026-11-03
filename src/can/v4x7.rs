//! CH32V407/CH32V467 Classical CAN, RM §25 and EVT CAN examples.
//!
//! Supports both receive FIFOs, all 14 filters, standard/extended data and remote
//! frames, blocking/nonblocking I/O and cancellation-safe asynchronous waits.
//! Async transmit waits for completion. Cancellation requests abort; a frame
//! already transmitted on the bus cannot be withdrawn. Use an external timeout
//! for async operations. No CAN FD or time-triggered mode is configured.

use core::cell::Cell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{AfType, AnyPin, OutputType, Pull, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode, NonBlocking};
use crate::{interrupt, pac, peripherals, Peri, RccPeripheral, RemapPeripheral};

#[path = "v4x7_types.rs"]
mod types;
pub use embedded_can::{ExtendedId, Id, StandardId};
pub use types::{BitTiming, CanFrame, Filter};

const TX_COMPLETED: u32 = 0x0001_0101;
const ERROR_INTERRUPTS: u32 = (1 << 10) | (1 << 11) | (1 << 15);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanFifo {
    Fifo0,
    Fifo1,
}

impl CanFifo {
    fn index(self) -> usize {
        self as usize
    }

    fn interrupts(self) -> u32 {
        (1 << (1 + self.index() * 3)) | (1 << (3 + self.index() * 3))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanMode {
    Normal,
    Silent,
    Loopback,
    SilentLoopback,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CanError {
    Overrun,
    Stuff,
    Form,
    Acknowledge,
    Bit,
    Crc,
    BusOff,
    ArbitrationLost,
    Transmit,
    Timeout,
}

impl embedded_can::Error for CanError {
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            Self::Overrun => embedded_can::ErrorKind::Overrun,
            Self::Stuff => embedded_can::ErrorKind::Stuff,
            Self::Form => embedded_can::ErrorKind::Form,
            Self::Acknowledge => embedded_can::ErrorKind::Acknowledge,
            Self::Bit => embedded_can::ErrorKind::Bit,
            Self::Crc => embedded_can::ErrorKind::Crc,
            _ => embedded_can::ErrorKind::Other,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CanInitError {
    InvalidTimings,
    Timeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidFilter;

/// An opaque mailbox returned by a successful nonblocking submission.
/// Status refers to that submission only until the mailbox is reused or the
/// controller is dropped. Submission order does not guarantee bus order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mailbox(u8);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxStatus {
    Pending,
    Sent,
    Failed(CanError),
}

#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Config {
    pub bitrate: u32,
    pub mode: CanMode,
    /// Optional explicit timing, checked against bitrate and the APB1 clock.
    pub bit_timing: Option<BitTiming>,
    pub automatic_retransmission: bool,
    pub automatic_bus_off_recovery: bool,
    pub receive_fifo_locked: bool,
    pub transmit_fifo_priority: bool,
    /// Register polls per blocking phase, not a duration.
    pub poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bitrate: 500_000,
            mode: CanMode::Normal,
            bit_timing: None,
            automatic_retransmission: true,
            automatic_bus_off_recovery: true,
            receive_fifo_locked: false,
            transmit_fifo_priority: false,
            poll_limit: 1_000_000,
        }
    }
}

pub struct Can<'d, T: Instance, M: Mode = Blocking> {
    _peri: Peri<'d, T>,
    rx: Peri<'d, AnyPin>,
    tx: Peri<'d, AnyPin>,
    config: Config,
    _mode: PhantomData<M>,
}

impl<'d, T: Instance> Can<'d, T, Blocking> {
    pub fn new_blocking<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, rx, tx, config)
    }
}

impl<'d, T: Instance> Can<'d, T, NonBlocking> {
    pub fn new_nb<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, rx, tx, config)
    }
}

impl<'d, T: Instance> Can<'d, T, Async> {
    pub fn new<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::TransmitInterrupt, TransmitInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ReceiveInterrupt0, ReceiveInterruptHandler<T, 0>>
            + interrupt::typelevel::Binding<T::ReceiveInterrupt1, ReceiveInterruptHandler<T, 1>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        config: Config,
    ) -> Result<Self, CanInitError> {
        let this = Self::new_inner(peri, rx, tx, config)?;
        T::TransmitInterrupt::unpend();
        T::ReceiveInterrupt0::unpend();
        T::ReceiveInterrupt1::unpend();
        T::ErrorInterrupt::unpend();
        unsafe {
            T::TransmitInterrupt::enable();
            T::ReceiveInterrupt0::enable();
            T::ReceiveInterrupt1::enable();
            T::ErrorInterrupt::enable();
        }
        Ok(this)
    }

    /// Wait for an actual transmission result, including arbitration/errors.
    pub async fn transmit(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        let mailbox = Cell::new(None);
        let _cleanup = OnDrop::new(|| {
            critical_section::with(|_| {
                T::REGS.intenr().modify(|reg| reg.0 &= !(1 | ERROR_INTERRUPTS));
                if let Some(mailbox) = mailbox.get() {
                    abort::<T>(mailbox);
                }
            });
        });
        poll_fn(|context| {
            T::state().tx_waker.register(context.waker());
            critical_section::with(|_| {
                if mailbox.get().is_none() {
                    match submit::<T>(frame) {
                        Ok(submitted) => mailbox.set(Some(submitted)),
                        Err(nb::Error::Other(error)) => return Poll::Ready(Err(error)),
                        Err(nb::Error::WouldBlock) => {}
                    }
                }
                if let Some(submitted) = mailbox.get() {
                    match transmission_status::<T>(submitted) {
                        TxStatus::Sent => {
                            mailbox.set(None);
                            return Poll::Ready(Ok(()));
                        }
                        TxStatus::Failed(error) => return Poll::Ready(Err(error)),
                        TxStatus::Pending => {}
                    }
                }
                if let Some(error) = take_error(T::REGS) {
                    return Poll::Ready(Err(error));
                }
                T::REGS.intenr().modify(|reg| reg.0 |= 1 | ERROR_INTERRUPTS);
                Poll::Pending
            })
        })
        .await
    }

    pub async fn receive(&mut self, fifo: CanFifo) -> Result<CanFrame, CanError> {
        let _cleanup = OnDrop::new(|| {
            critical_section::with(|_| {
                T::REGS
                    .intenr()
                    .modify(|reg| reg.0 &= !(fifo.interrupts() | ERROR_INTERRUPTS));
            });
        });
        poll_fn(|context| {
            T::state().rx_wakers[fifo.index()].register(context.waker());
            critical_section::with(|_| match receive(T::REGS, fifo) {
                Ok(frame) => Poll::Ready(Ok(frame)),
                Err(nb::Error::Other(error)) => Poll::Ready(Err(error)),
                Err(nb::Error::WouldBlock) => {
                    T::REGS
                        .intenr()
                        .modify(|reg| reg.0 |= fifo.interrupts() | ERROR_INTERRUPTS);
                    Poll::Pending
                }
            })
        })
        .await
    }
}

impl<'d, T: Instance, M: Mode> Can<'d, T, M> {
    fn new_inner<const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T, REMAP>>,
        tx: Peri<'d, impl TxPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        let clock = T::frequency().0;
        let timing = match config.bit_timing {
            Some(timing) => timing,
            None => BitTiming::from_bitrate(clock, config.bitrate).ok_or(CanInitError::InvalidTimings)?,
        };
        let register = timing.register().ok_or(CanInitError::InvalidTimings)?;
        let divisor = u64::from(timing.prescaler) * (1 + u64::from(timing.seg1) + u64::from(timing.seg2));
        if !(1_000..=1_000_000).contains(&config.bitrate) || u64::from(config.bitrate) * divisor != u64::from(clock) {
            return Err(CanInitError::InvalidTimings);
        }
        let this = Self {
            _peri: peri,
            rx: rx.into(),
            tx: tx.into(),
            config,
            _mode: PhantomData,
        };
        T::enable_and_reset();
        T::REGS.intenr().write_value(pac::can::regs::Intenr(0));
        T::state().completed.lock(|completed| completed.set(0));
        T::set_remap(REMAP);
        this.rx.set_as_input(Pull::Up);
        this.tx.set_high();
        this.tx.set_as_af(AfType::output(OutputType::PushPull, Speed::High));
        T::REGS.ctlr().modify(|reg| {
            reg.set_sleep(false);
            reg.set_inrq(true);
        });
        wait_init(T::REGS, true, config.poll_limit)?;
        T::REGS.ctlr().modify(|reg| {
            reg.set_nart(!config.automatic_retransmission);
            reg.set_abom(config.automatic_bus_off_recovery);
            reg.set_rflm(config.receive_fifo_locked);
            reg.set_txfp(config.transmit_fifo_priority);
            reg.set_ttcm(false);
        });
        let mode = match config.mode {
            CanMode::Normal => 0,
            CanMode::Loopback => 1 << 30,
            CanMode::Silent => 1 << 31,
            CanMode::SilentLoopback => (1 << 30) | (1 << 31),
        };
        T::REGS.btimr().write_value(pac::can::regs::Btimr(register | mode));
        T::REGS.fctlr().modify(|reg| reg.set_finit(true));
        T::REGS.fwr().write_value(pac::can::regs::Fwr(0));
        T::REGS.fctlr().modify(|reg| reg.set_finit(false));
        T::REGS.ctlr().modify(|reg| reg.set_inrq(false));
        wait_init(T::REGS, false, config.poll_limit)?;
        Ok(this)
    }

    /// Configure one bank. No register is touched if the bank is out of range.
    pub fn set_filter(&mut self, filter: Filter) -> Result<(), InvalidFilter> {
        if filter.bank >= 14 {
            return Err(InvalidFilter);
        }
        critical_section::with(|_| {
            let regs = T::REGS;
            let bank = usize::from(filter.bank);
            regs.fctlr().modify(|reg| reg.set_finit(true));
            regs.fwr().modify(|reg| reg.set_fact(bank, false));
            regs.fscfgr().modify(|reg| reg.set_fsc(bank, filter.scale_32bit));
            regs.fmcfgr().modify(|reg| reg.set_fbm(bank, filter.list));
            regs.fafifor()
                .modify(|reg| reg.set_ffa(bank, filter.fifo == CanFifo::Fifo1));
            regs.fr(bank * 2).write_value(pac::can::regs::Fr(filter.registers[0]));
            regs.fr(bank * 2 + 1)
                .write_value(pac::can::regs::Fr(filter.registers[1]));
            regs.fwr().modify(|reg| reg.set_fact(bank, true));
            regs.fctlr().modify(|reg| reg.set_finit(false));
        });
        Ok(())
    }

    pub fn disable_filter(&mut self, bank: u8) -> Result<(), InvalidFilter> {
        if bank >= 14 {
            return Err(InvalidFilter);
        }
        critical_section::with(|_| {
            T::REGS.fctlr().modify(|reg| reg.set_finit(true));
            T::REGS.fwr().modify(|reg| reg.set_fact(bank as usize, false));
            T::REGS.fctlr().modify(|reg| reg.set_finit(false));
        });
        Ok(())
    }

    /// Queue a frame without waiting for transmission. Full mailboxes return
    /// WouldBlock; pending lower-priority frames are never replaced.
    pub fn try_transmit(&mut self, frame: &CanFrame) -> nb::Result<Mailbox, CanError> {
        critical_section::with(|_| submit::<T>(frame))
    }

    pub fn transmit_status(&mut self, mailbox: Mailbox) -> TxStatus {
        critical_section::with(|_| transmission_status::<T>(mailbox))
    }

    pub fn abort(&mut self, mailbox: Mailbox) {
        critical_section::with(|_| abort::<T>(mailbox));
    }

    pub fn try_receive(&mut self, fifo: CanFifo) -> nb::Result<CanFrame, CanError> {
        critical_section::with(|_| receive(T::REGS, fifo))
    }

    pub fn blocking_transmit(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        let mut mailbox = None;
        for _poll in 0..self.config.poll_limit {
            match self.try_transmit(frame) {
                Ok(submitted) => {
                    mailbox = Some(submitted);
                    break;
                }
                Err(nb::Error::Other(error)) => return Err(error),
                Err(nb::Error::WouldBlock) => {}
            }
        }
        let mailbox = mailbox.ok_or(CanError::Timeout)?;
        for _poll in 0..self.config.poll_limit {
            match self.transmit_status(mailbox) {
                TxStatus::Sent => return Ok(()),
                TxStatus::Failed(error) => {
                    self.abort(mailbox);
                    return Err(error);
                }
                TxStatus::Pending => {}
            }
            if let Some(error) = critical_section::with(|_| take_error(T::REGS)) {
                self.abort(mailbox);
                return Err(error);
            }
        }
        self.abort(mailbox);
        Err(CanError::Timeout)
    }

    pub fn blocking_receive(&mut self, fifo: CanFifo) -> Result<CanFrame, CanError> {
        for _poll in 0..self.config.poll_limit {
            match self.try_receive(fifo) {
                Ok(frame) => return Ok(frame),
                Err(nb::Error::Other(error)) => return Err(error),
                Err(nb::Error::WouldBlock) => {}
            }
        }
        Err(CanError::Timeout)
    }
}

impl<T: Instance, M: Mode> Drop for Can<'_, T, M> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            T::REGS.intenr().write_value(pac::can::regs::Intenr(0));
            T::REGS.tstatr().write_value(pac::can::regs::Tstatr(0x0080_8080));
            T::REGS.ctlr().modify(|reg| reg.set_sleep(true));
            T::TransmitInterrupt::disable();
            T::ReceiveInterrupt0::disable();
            T::ReceiveInterrupt1::disable();
            T::ErrorInterrupt::disable();
            T::TransmitInterrupt::unpend();
            T::ReceiveInterrupt0::unpend();
            T::ReceiveInterrupt1::unpend();
            T::ErrorInterrupt::unpend();
            self.rx.set_as_disconnected();
            self.tx.set_as_disconnected();
            T::disable();
        });
    }
}

fn wait_init(regs: pac::can::Can, enabled: bool, limit: u32) -> Result<(), CanInitError> {
    for _poll in 0..limit {
        if regs.statr().read().inak() == enabled {
            return Ok(());
        }
    }
    Err(CanInitError::Timeout)
}

fn take_error(regs: pac::can::Can) -> Option<CanError> {
    if regs.statr().read().erri() {
        regs.statr().write(|reg| reg.set_erri(true));
    }
    let status = regs.errsr().read();
    let error = if status.boff() {
        Some(CanError::BusOff)
    } else {
        match status.lec() {
            1 => Some(CanError::Stuff),
            2 => Some(CanError::Form),
            3 => Some(CanError::Acknowledge),
            4 | 5 => Some(CanError::Bit),
            6 => Some(CanError::Crc),
            _ => None,
        }
    };
    if error.is_some() {
        regs.errsr().write(|reg| reg.set_lec(7));
    }
    error
}

fn capture_completions<T: Instance>() {
    let status = T::REGS.tstatr().read().0;
    let mut completed = 0;
    for mailbox in 0..3 {
        if status & (1 << (mailbox * 8)) != 0 {
            completed |= status & (15 << (mailbox * 8));
        }
    }
    T::state().completed.lock(|events| events.set(events.get() | completed));
    T::REGS
        .tstatr()
        .write_value(pac::can::regs::Tstatr(completed & TX_COMPLETED));
}

fn submit<T: Instance>(frame: &CanFrame) -> nb::Result<Mailbox, CanError> {
    if let Some(error) = take_error(T::REGS) {
        return Err(nb::Error::Other(error));
    }
    capture_completions::<T>();
    let status = T::REGS.tstatr().read();
    let index = (0..3).find(|index| status.tme(*index)).ok_or(nb::Error::WouldBlock)?;
    T::state()
        .completed
        .lock(|events| events.set(events.get() & !(15 << (index * 8))));
    T::REGS.txmir(index).write_value(pac::can::regs::Txmir(0));
    T::REGS.txmdtr(index).write(|reg| reg.set_dlc(frame.dlc));
    T::REGS
        .txmdlr(index)
        .write_value(pac::can::regs::Txmdlr(u32::from_le_bytes(
            frame.data[..4].try_into().unwrap(),
        )));
    T::REGS
        .txmdhr(index)
        .write_value(pac::can::regs::Txmdhr(u32::from_le_bytes(
            frame.data[4..].try_into().unwrap(),
        )));
    T::REGS
        .txmir(index)
        .write_value(pac::can::regs::Txmir(frame.identifier() | 1));
    Ok(Mailbox(index as u8))
}

fn transmission_status<T: Instance>(mailbox: Mailbox) -> TxStatus {
    capture_completions::<T>();
    let status = T::state()
        .completed
        .lock(|events| events.get() >> (usize::from(mailbox.0) * 8));
    if status & 1 == 0 {
        TxStatus::Pending
    } else if status & 2 != 0 {
        TxStatus::Sent
    } else if status & 4 != 0 {
        TxStatus::Failed(CanError::ArbitrationLost)
    } else {
        TxStatus::Failed(CanError::Transmit)
    }
}

fn abort<T: Instance>(mailbox: Mailbox) {
    T::REGS
        .tstatr()
        .write_value(pac::can::regs::Tstatr(1 << (usize::from(mailbox.0) * 8 + 7)));
}

fn receive(regs: pac::can::Can, fifo: CanFifo) -> nb::Result<CanFrame, CanError> {
    let index = fifo.index();
    let status = regs.rfifo(index).read();
    if status.fovr() {
        regs.rfifo(index).write(|reg| reg.set_fovr(true));
        return Err(nb::Error::Other(CanError::Overrun));
    }
    if let Some(error) = take_error(regs) {
        return Err(nb::Error::Other(error));
    }
    if status.fmp() == 0 {
        return Err(nb::Error::WouldBlock);
    }
    let frame = CanFrame::decode(
        regs.rxmir(index).read().0,
        regs.rxmdtr(index).read().dlc(),
        regs.rxmdlr(index).read().0,
        regs.rxmdhr(index).read().0,
    );
    regs.rfifo(index).write(|reg| reg.set_rfom(true));
    Ok(frame)
}

/// TX completion handler, preserving per-mailbox results before W1C clearing.
pub struct TransmitInterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> interrupt::typelevel::Handler<T::TransmitInterrupt> for TransmitInterruptHandler<T> {
    unsafe fn on_interrupt() {
        critical_section::with(|_| {
            capture_completions::<T>();
            T::REGS.intenr().modify(|reg| reg.set_tmeie(false));
        });
        T::state().tx_waker.wake();
    }
}

pub struct ReceiveInterruptHandler<T: Instance, const FIFO: usize = 0>(PhantomData<T>);

fn receive_interrupt<T: Instance>(fifo: CanFifo) {
    critical_section::with(|_| T::REGS.intenr().modify(|reg| reg.0 &= !fifo.interrupts()));
    T::state().rx_wakers[fifo.index()].wake();
}

impl<T: Instance> interrupt::typelevel::Handler<T::ReceiveInterrupt0> for ReceiveInterruptHandler<T, 0> {
    unsafe fn on_interrupt() {
        receive_interrupt::<T>(CanFifo::Fifo0);
    }
}

impl<T: Instance> interrupt::typelevel::Handler<T::ReceiveInterrupt1> for ReceiveInterruptHandler<T, 1> {
    unsafe fn on_interrupt() {
        receive_interrupt::<T>(CanFifo::Fifo1);
    }
}

pub struct ErrorInterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        critical_section::with(|_| T::REGS.intenr().modify(|reg| reg.0 &= !ERROR_INTERRUPTS));
        T::state().tx_waker.wake();
        for waker in &T::state().rx_wakers {
            waker.wake();
        }
    }
}

struct State {
    completed: Mutex<CriticalSectionRawMutex, Cell<u32>>,
    tx_waker: AtomicWaker,
    rx_wakers: [AtomicWaker; 2],
}

trait SealedInstance {
    const REGS: pac::can::Can;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + RemapPeripheral + embassy_hal_internal::PeripheralType {
    type TransmitInterrupt: Interrupt;
    type ReceiveInterrupt0: Interrupt;
    type ReceiveInterrupt1: Interrupt;
    type ErrorInterrupt: Interrupt;
}

foreach_peripheral!(
    (can, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            const REGS: pac::can::Can = pac::$inst;
            fn state() -> &'static State {
                static STATE: State = State {
                    completed: Mutex::new(Cell::new(0)),
                    tx_waker: AtomicWaker::new(),
                    rx_wakers: [AtomicWaker::new(), AtomicWaker::new()],
                };
                &STATE
            }
        }
        impl Instance for peripherals::$inst {
            type TransmitInterrupt = crate::_generated::peripheral_interrupts::$inst::TX;
            type ReceiveInterrupt0 = crate::_generated::peripheral_interrupts::$inst::RX0;
            type ReceiveInterrupt1 = crate::_generated::peripheral_interrupts::$inst::RX1;
            type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::SCE;
        }
    };
);

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);

impl<T: Instance> embedded_can::blocking::Can for Can<'_, T, Blocking> {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        self.blocking_transmit(frame)
    }

    fn receive(&mut self) -> Result<CanFrame, CanError> {
        self.blocking_receive(CanFifo::Fifo0)
    }
}

impl<T: Instance> embedded_can::nb::Can for Can<'_, T, NonBlocking> {
    type Frame = CanFrame;
    type Error = CanError;

    fn transmit(&mut self, frame: &CanFrame) -> nb::Result<Option<CanFrame>, CanError> {
        self.try_transmit(frame).map(|_| None)
    }

    fn receive(&mut self) -> nb::Result<CanFrame, CanError> {
        self.try_receive(CanFifo::Fifo0)
    }
}

#[cfg(test)]
#[path = "v4x7_tests.rs"]
mod tests;
