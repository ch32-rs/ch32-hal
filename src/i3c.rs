//! CH32V4 I3C controller with FIFO polling and interrupt-driven transfers.
//!
//! Timings are explicit HCLK counters: board wiring, target capabilities and mixed-bus
//! requirements must be checked against the reference manual.
//! Optional byte DMA handles one buffer per direction in asynchronous private and legacy
//! transactions. Scatter transactions, CCC and ENTDAA use the interrupt FIFO backend.
//! IBI and hot-join are not implemented.
//! FIFO service latency must be verified at the selected bus speed and interrupt load.
//!
//! RM requires STOP on every legacy I2C message. The I2C traits therefore reject
//! direction changes within one transaction, including `write_read`, before bus activity.
//!
//! Failed or cancelled frames require explicit controller reinitialization. Reinitializing
//! the controller does not recover a target holding the bus low or reset target addresses.

#[cfg(feature = "embassy")]
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
#[cfg(feature = "embassy")]
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::i2c::Operation;

use crate::dma::ChannelAndRequest;
use crate::gpio::{AfType, AnyPin, OutputType, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode};
use crate::{interrupt, peripherals, Peri};

#[path = "i3c/transfer.rs"]
mod transfer;
use transfer::{Daa, Engine, Frame, Progress};
#[cfg(feature = "embassy")]
#[path = "i3c/dma.rs"]
mod dma;

const CLEAR_EVENTS: u32 = (1 << 9) | (1 << 10) | (1 << 11) | (1 << 15) | (1 << 19);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    InvalidConfig,
    ClockChanged,
    InvalidAddress,
    ZeroLength,
    TooLong,
    /// ENTDAA found more targets than the supplied address/result capacity.
    Capacity,
    UnsupportedCcc,
    /// RM requires MEND=1 for every legacy I2C message, preventing direction changes within one frame.
    UnsupportedTransaction,
    AddressNack,
    DataNack,
    Protocol,
    Data,
    Fifo,
    Dma,
    /// Controller error code from STATER.CODERR, valid only when EVR.ERRF is set.
    Controller(u8),
    /// The target ended a private read before the requested length.
    ShortRead,
    Timeout,
    NeedsRecovery,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};
        match self {
            Self::AddressNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address),
            Self::DataNack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Self::Fifo => ErrorKind::Overrun,
            Self::Protocol | Self::Controller(_) => ErrorKind::Bus,
            _ => ErrorKind::Other,
        }
    }
}

/// Raw timing fields from RM 19.4.16–19.4.18, bound to one HCLK frequency.
///
/// SCL high/low periods and the AVAL unit use `(counter + 1) / hclk_hz`.
/// FREE is the hardware's MIPI timing parameter, not an I2C baud-rate divider.
#[derive(Clone, Copy, Debug)]
pub struct Timings {
    pub hclk_hz: u32,
    pub pp_low: u8,
    pub i3c_high: u8,
    pub od_low: u8,
    pub i2c_high: u8,
    pub aval: u8,
    pub free: u8,
    pub asncr: u8,
    pub sda_hold_1_5: bool,
}

/// Identification returned by ENTDAA. Entries are valid only after successful completion.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Target {
    pub dynamic_address: u8,
    /// Six PID bytes in bus order, most significant byte first.
    pub pid: [u8; 6],
    pub bcr: u8,
    pub dcr: u8,
}

#[derive(Clone, Copy)]
#[non_exhaustive]
pub struct Config {
    pub timings: Timings,
    pub controller_address: u8,
    pub internal_pullup: bool,
    /// Maximum polling steps per blocking frame; this is not a duration.
    pub poll_limit: u32,
    #[cfg(feature = "embassy")]
    pub timeout: embassy_time::Duration,
}

impl Config {
    pub fn new(timings: Timings) -> Self {
        Self {
            timings,
            controller_address: 0x22,
            internal_pullup: true,
            poll_limit: 1_000_000,
            #[cfg(feature = "embassy")]
            timeout: embassy_time::Duration::from_millis(1000),
        }
    }

    fn validate(&self, hclk: u32) -> Result<(), Error> {
        if hclk == 0
            || self.timings.hclk_hz != hclk
            || self.timings.free > 127
            || self.timings.asncr > 3
            || self.poll_limit == 0
        {
            return Err(Error::InvalidConfig);
        }
        transfer::validate_dynamic_address(self.controller_address)
    }
}

pub struct I3c<'d, T: Instance, M: Mode> {
    scl: Peri<'d, AnyPin>,
    sda: Peri<'d, AnyPin>,
    config: Config,
    needs_recovery: bool,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _phantom: PhantomData<(&'d mut T, M)>,
}

impl<'d, T: Instance> I3c<'d, T, Blocking> {
    pub fn new_blocking<const REMAP: u8>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, Error> {
        Self::new_inner::<REMAP>(scl, sda, config)
    }
}

#[cfg(feature = "embassy")]
impl<'d, T: Instance> I3c<'d, T, Async> {
    /// Enable byte DMA for transactions with at most one buffer per direction.
    ///
    /// Scattered transactions, CCC and ENTDAA retain FIFO interrupt servicing.
    /// Both channels are armed before any message is queued. Control FIFO DMA is unused.
    pub fn new_with_dma<const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        irqs: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Result<Self, Error> {
        let mut driver = Self::new(peri, scl, sda, irqs, config)?;
        driver.tx_dma = new_dma!(tx_dma);
        driver.rx_dma = new_dma!(rx_dma);
        Ok(driver)
    }

    pub fn new<const REMAP: u8>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        _irqs: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>,
        config: Config,
    ) -> Result<Self, Error> {
        let driver = Self::new_inner::<REMAP>(scl, sda, config)?;
        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe {
            T::EventInterrupt::enable();
            T::ErrorInterrupt::enable();
        }
        Ok(driver)
    }

    async fn run(&mut self, frame: &mut impl Engine) -> Result<(), Error> {
        self.begin(frame.address(), frame.legacy())?;
        let cleanup = OnDrop::new(Self::quiesce);
        let transfer = poll_fn(|context| {
            T::state().waker.register(context.waker());
            let regs = T::regs();
            regs.intenr().write(|reg| reg.0 = 0);
            for _ in 0..32 {
                match frame.step(regs) {
                    Ok(Progress::Complete) => return Poll::Ready(Ok(())),
                    Err(error) => return Poll::Ready(Err(error)),
                    Ok(Progress::Advanced) => {}
                    Ok(Progress::Waiting) => break,
                }
            }
            regs.intenr().write(|reg| reg.0 = frame.interrupt_mask());
            Poll::Pending
        });
        let result = embassy_time::with_timeout(self.config.timeout, transfer)
            .await
            .map_err(|_| Error::Timeout)
            .and_then(core::convert::identity);
        if result.is_ok() {
            Self::finish();
            self.needs_recovery = false;
            cleanup.defuse();
        }
        result
    }

    async fn run_frame(&mut self, frame: Frame<'_, '_>) -> Result<(), Error> {
        if self.tx_dma.is_some() && self.rx_dma.is_some() {
            match frame.into_dma() {
                Ok(frame) => self.run_dma(frame).await,
                Err(mut frame) => self.run(&mut frame).await,
            }
        } else {
            self.run(&mut { frame }).await
        }
    }

    /// Transfer to a previously assigned I3C dynamic address. Reads require the full length.
    pub async fn private_transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let frame = Frame::new(address, operations, false)?;
        if frame.is_empty() {
            return Ok(());
        }
        self.run_frame(frame).await
    }

    /// Legacy I2C transaction with one direction; same-direction operations share one message.
    ///
    /// Direction changes require a repeated start, which RM forbids for legacy messages.
    pub async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let frame = Frame::new(address, operations, true)?;
        if frame.is_empty() {
            return Ok(());
        }
        self.run_frame(frame).await
    }

    /// Send a broadcast CCC, excluding ENTDAA and RSTACT, which require dedicated handling.
    pub async fn broadcast_ccc(&mut self, command: u8, payload: &[u8]) -> Result<(), Error> {
        let mut operations = [Operation::Write(payload)];
        self.run(&mut Frame::broadcast(command, &mut operations)?).await
    }

    /// Direct CCC with one nonempty target payload and no defining byte. RSTACT is excluded.
    pub async fn direct_ccc(&mut self, command: u8, address: u8, operation: Operation<'_>) -> Result<(), Error> {
        let mut operations = [operation];
        self.run(&mut Frame::direct(command, address, &mut operations)?).await
    }

    /// Assign supplied addresses to unaddressed targets. Existing dynamic addresses are retained.
    ///
    /// On error, targets may already have accepted addresses; result entries are provisional.
    /// Capacity exhaustion disables the controller and requires restoring an idle bus.
    pub async fn assign_addresses(&mut self, addresses: &[u8], targets: &mut [Target]) -> Result<usize, Error> {
        let mut daa = Daa::new(addresses, targets, self.config.controller_address)?;
        self.run(&mut daa).await?;
        Ok(daa.assigned())
    }
}

impl<'d, T: Instance, M: Mode> I3c<'d, T, M> {
    fn new_inner<const REMAP: u8>(
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        config: Config,
    ) -> Result<Self, Error> {
        config.validate(T::frequency().0)?;
        T::set_remap(REMAP);
        T::enable_and_reset();
        let af = AfType::output(OutputType::PushPull, Speed::High);
        set_as_af!(scl, af);
        set_as_af!(sda, af);
        let mut driver = Self {
            scl: scl.into(),
            sda: sda.into(),
            config,
            needs_recovery: true,
            tx_dma: None,
            rx_dma: None,
            _phantom: PhantomData,
        };
        driver.configure();
        Ok(driver)
    }

    fn configure(&mut self) {
        let regs = T::regs();
        let timing = self.config.timings;
        regs.cfgr().write(|reg| {
            reg.0 = (1 << 1) | (u32::from(self.config.internal_pullup) << 6) | (1 << 19);
        });
        regs.reset().write(|reg| reg.0 = 0);
        regs.timingr0().write(|reg| {
            reg.0 = u32::from(timing.pp_low)
                | (u32::from(timing.i3c_high) << 8)
                | (u32::from(timing.od_low) << 16)
                | (u32::from(timing.i2c_high) << 24);
        });
        regs.timingr1().write(|reg| {
            reg.0 = u32::from(timing.aval)
                | (u32::from(timing.asncr) << 8)
                | (u32::from(timing.free) << 16)
                | (u32::from(timing.sda_hold_1_5) << 28);
        });
        regs.timingr2().write(|reg| reg.0 = 0);
        regs.owndevr()
            .write(|reg| reg.0 = (u32::from(self.config.controller_address) << 1) | 1);
        for index in 0..2 {
            regs.devr(index).write(|reg| reg.0 = 1 << 31);
            regs.devrx(index).write(|reg| reg.0 = 1 << 31);
        }
        regs.intenr().write(|reg| reg.0 = 0);
        regs.cevr().write(|reg| reg.0 = CLEAR_EVENTS);
        regs.cfgr().write(|reg| {
            reg.0 = 1 | (1 << 1) | (u32::from(self.config.internal_pullup) << 6) | (1 << 19);
        });
        self.needs_recovery = false;
        T::state().active.store(true, Ordering::Release);
    }

    /// Reinitialize the controller after the application has restored an idle physical bus.
    ///
    /// This does not generate a bus-clear sequence or reset target dynamic addresses.
    pub fn reinitialize(&mut self) -> Result<(), Error> {
        self.config.validate(T::frequency().0)?;
        T::enable_and_reset();
        self.configure();
        Ok(())
    }

    pub fn needs_recovery(&self) -> bool {
        self.needs_recovery
    }

    fn begin(&mut self, address: u8, legacy: bool) -> Result<(), Error> {
        if self.needs_recovery {
            return Err(Error::NeedsRecovery);
        }
        if self.config.timings.hclk_hz != T::frequency().0 {
            return Err(Error::ClockChanged);
        }
        if address == self.config.controller_address {
            return Err(Error::InvalidAddress);
        }
        self.needs_recovery = true;
        let regs = T::regs();
        regs.cevr().write(|reg| reg.0 = CLEAR_EVENTS);
        regs.cfgr().modify(|reg| {
            reg.0 = (reg.0 & !(1 << 2)) | (u32::from(legacy) << 2) | (1 << 30);
        });
        Ok(())
    }

    fn quiesce() {
        T::state().active.store(false, Ordering::Release);
        let regs = T::regs();
        regs.intenr().write(|reg| reg.0 = 0);
        regs.cfgr().write(|reg| reg.0 = 0);
    }

    fn finish() {
        let regs = T::regs();
        regs.intenr().write(|reg| reg.0 = 0);
        regs.cevr().write(|reg| reg.0 = CLEAR_EVENTS);
    }

    fn blocking_run(&mut self, frame: &mut impl Engine) -> Result<(), Error> {
        self.begin(frame.address(), frame.legacy())?;
        let cleanup = OnDrop::new(Self::quiesce);
        for _ in 0..self.config.poll_limit {
            if frame.step(T::regs())? == Progress::Complete {
                Self::finish();
                self.needs_recovery = false;
                cleanup.defuse();
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    pub fn blocking_private_transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let mut frame = Frame::new(address, operations, false)?;
        if frame.is_empty() {
            return Ok(());
        }
        self.blocking_run(&mut frame)
    }

    pub fn blocking_transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let mut frame = Frame::new(address, operations, true)?;
        if frame.is_empty() {
            return Ok(());
        }
        self.blocking_run(&mut frame)
    }

    pub fn blocking_broadcast_ccc(&mut self, command: u8, payload: &[u8]) -> Result<(), Error> {
        let mut operations = [Operation::Write(payload)];
        self.blocking_run(&mut Frame::broadcast(command, &mut operations)?)
    }

    pub fn blocking_direct_ccc(&mut self, command: u8, address: u8, operation: Operation<'_>) -> Result<(), Error> {
        let mut operations = [operation];
        self.blocking_run(&mut Frame::direct(command, address, &mut operations)?)
    }

    /// Blocking ENTDAA. On error, partial assignments and result entries are provisional.
    pub fn blocking_assign_addresses(&mut self, addresses: &[u8], targets: &mut [Target]) -> Result<usize, Error> {
        let mut daa = Daa::new(addresses, targets, self.config.controller_address)?;
        self.blocking_run(&mut daa)?;
        Ok(daa.assigned())
    }
}

impl<T: Instance, M: Mode> Drop for I3c<'_, T, M> {
    fn drop(&mut self) {
        Self::quiesce();
        T::EventInterrupt::disable();
        T::ErrorInterrupt::disable();
        T::disable();
        self.scl.set_as_disconnected();
        self.sda.set_as_disconnected();
    }
}

impl<T: Instance, M: Mode> embedded_hal::i2c::ErrorType for I3c<'_, T, M> {
    type Error = Error;
}

impl<T: Instance, M: Mode> embedded_hal::i2c::I2c for I3c<'_, T, M> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        self.blocking_transaction(address, operations)
    }
}

#[cfg(feature = "embassy")]
impl<T: Instance> embedded_hal_async::i2c::I2c for I3c<'_, T, Async> {
    async fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        I3c::transaction(self, address, operations).await
    }
}

struct State {
    waker: AtomicWaker,
    active: AtomicBool,
}

pub struct EventInterruptHandler<T: Instance>(PhantomData<T>);
pub struct ErrorInterruptHandler<T: Instance>(PhantomData<T>);

unsafe fn on_interrupt<T: Instance>() {
    if !T::state().active.load(Ordering::Acquire) {
        return;
    }
    T::regs().intenr().write(|reg| reg.0 = 0);
    T::state().waker.wake();
}

impl<T: Instance> interrupt::typelevel::Handler<T::EventInterrupt> for EventInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>();
    }
}

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>();
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::i3c::I3c;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + embassy_hal_internal::PeripheralType + crate::peripheral::RemapPeripheral + 'static
{
    type EventInterrupt: interrupt::typelevel::Interrupt;
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (i3c, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::i3c::I3c { crate::pac::$inst }
            fn state() -> &'static State {
                static STATE: State = State { waker: AtomicWaker::new(), active: AtomicBool::new(false) };
                &STATE
            }
        }
        impl Instance for peripherals::$inst {
            type EventInterrupt = crate::_generated::peripheral_interrupts::$inst::EV;
            type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::ER;
        }
    };
);

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);
