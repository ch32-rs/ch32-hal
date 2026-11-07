//! CH32V407/CH32V467 and CH32H417 Classical CAN, RM §25 and RM §28.
//!
//! Supports both receive FIFOs, every filter bank (14 on V4x7, 42 on H4),
//! standard/extended data and remote frames, blocking/nonblocking I/O and
//! cancellation-safe asynchronous waits. Async transmit waits for completion.
//! Cancellation requests abort; a frame already transmitted on the bus cannot be
//! withdrawn. Use an external timeout for async operations. No CAN FD or
//! time-triggered mode is configured.
//!
//! On CH32H4 the filter table is shared by CAN1/CAN2/CAN3 and only mapped in
//! CAN1 (RM table 28-12), and its clock comes from CAN1 (RM §28.2.1 note).
//! CAN2/CAN3 constructors therefore take `CAN1` as a filter master; the node
//! keeps it for its lifetime and accesses the filter registers through it.

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
use crate::{interrupt, pac, peripherals, Peri, RccPeripheral};

#[path = "v4x7_types.rs"]
mod types;
pub use embedded_can::{ExtendedId, Id, StandardId};
pub use types::{BitTiming, CanFrame, Filter};

const TX_COMPLETED: u32 = 0x0001_0101;
const ERROR_INTERRUPTS: u32 = (1 << 10) | (1 << 11) | (1 << 15);

/// Number of filter banks behind the shared filter table.
#[cfg(can_h4)]
const FILTER_BANKS: u8 = 42;
#[cfg(not(can_h4))]
const FILTER_BANKS: u8 = 14;

/// On H4 banks 0..27 live in `FMCFGR`/... and banks 28..41 in the `*1`
/// registers (RM §28.7.21–28.7.28).
#[cfg(can_h4)]
const FILTER_SPLIT: usize = 28;

/// Register block that owns the filter table. H4 maps it only in CAN1, and
/// CAN2/CAN3 must reach it there (RM table 28-12).
#[cfg(can_h4)]
fn filter_regs<T: Instance>() -> pac::can::Can {
    pac::CAN1
}

#[cfg(not(can_h4))]
fn filter_regs<T: Instance>() -> pac::can::Can {
    T::REGS
}

#[cfg(can_h4)]
fn set_filter_mode(regs: pac::can::Can, bank: usize, filter: &Filter) {
    if bank >= FILTER_SPLIT {
        let bank = bank - FILTER_SPLIT;
        regs.fscfgr1().modify(|reg| reg.set_fsc(bank, filter.scale_32bit));
        regs.fmcfgr1().modify(|reg| reg.set_fbm(bank, filter.list));
        regs.fafifor1()
            .modify(|reg| reg.set_ffa(bank, filter.fifo == CanFifo::Fifo1));
    } else {
        regs.fscfgr().modify(|reg| reg.set_fsc(bank, filter.scale_32bit));
        regs.fmcfgr().modify(|reg| reg.set_fbm(bank, filter.list));
        regs.fafifor()
            .modify(|reg| reg.set_ffa(bank, filter.fifo == CanFifo::Fifo1));
    }
}

#[cfg(not(can_h4))]
fn set_filter_mode(regs: pac::can::Can, bank: usize, filter: &Filter) {
    regs.fscfgr().modify(|reg| reg.set_fsc(bank, filter.scale_32bit));
    regs.fmcfgr().modify(|reg| reg.set_fbm(bank, filter.list));
    regs.fafifor()
        .modify(|reg| reg.set_ffa(bank, filter.fifo == CanFifo::Fifo1));
}

#[cfg(can_h4)]
fn set_filter_active(regs: pac::can::Can, bank: usize, active: bool) {
    if bank >= FILTER_SPLIT {
        regs.fwr1().modify(|reg| reg.set_fact(bank - FILTER_SPLIT, active));
    } else {
        regs.fwr().modify(|reg| reg.set_fact(bank, active));
    }
}

#[cfg(not(can_h4))]
fn set_filter_active(regs: pac::can::Can, bank: usize, active: bool) {
    regs.fwr().modify(|reg| reg.set_fact(bank, active));
}

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
    /// On H4, CAN2/CAN3 keep CAN1 alive because the shared filter table and
    /// its clock belong to CAN1. `None` for CAN1 (and every V4x7 node).
    #[cfg(can_h4)]
    _filter_master: Option<Peri<'d, peripherals::CAN1>>,
    rx: Peri<'d, AnyPin>,
    tx: Peri<'d, AnyPin>,
    config: Config,
    _mode: PhantomData<M>,
}

#[cfg(not(can_h4))]
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

#[cfg(not(can_h4))]
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

#[cfg(not(can_h4))]
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
        enable_async_irqs::<T>();
        Ok(this)
    }
}

#[cfg(can_h4)]
pub trait SlaveNode: Instance {}

#[cfg(can_h4)]
impl SlaveNode for peripherals::CAN2 {}

#[cfg(can_h4)]
impl SlaveNode for peripherals::CAN3 {}

#[cfg(can_h4)]
impl<'d> Can<'d, peripherals::CAN1, Blocking> {
    pub fn new_blocking(
        peri: Peri<'d, peripherals::CAN1>,
        rx: Peri<'d, impl RxPin<peripherals::CAN1>>,
        tx: Peri<'d, impl TxPin<peripherals::CAN1>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, None, rx, tx, config)
    }
}

#[cfg(can_h4)]
impl<'d> Can<'d, peripherals::CAN1, NonBlocking> {
    pub fn new_nb(
        peri: Peri<'d, peripherals::CAN1>,
        rx: Peri<'d, impl RxPin<peripherals::CAN1>>,
        tx: Peri<'d, impl TxPin<peripherals::CAN1>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, None, rx, tx, config)
    }
}

#[cfg(can_h4)]
impl<'d> Can<'d, peripherals::CAN1, Async> {
    pub fn new(
        peri: Peri<'d, peripherals::CAN1>,
        rx: Peri<'d, impl RxPin<peripherals::CAN1>>,
        tx: Peri<'d, impl TxPin<peripherals::CAN1>>,
        _irq: impl interrupt::typelevel::Binding<
                <peripherals::CAN1 as Instance>::TransmitInterrupt,
                TransmitInterruptHandler<peripherals::CAN1>,
            > + interrupt::typelevel::Binding<
                <peripherals::CAN1 as Instance>::ReceiveInterrupt0,
                ReceiveInterruptHandler<peripherals::CAN1, 0>,
            > + interrupt::typelevel::Binding<
                <peripherals::CAN1 as Instance>::ReceiveInterrupt1,
                ReceiveInterruptHandler<peripherals::CAN1, 1>,
            > + interrupt::typelevel::Binding<
                <peripherals::CAN1 as Instance>::ErrorInterrupt,
                ErrorInterruptHandler<peripherals::CAN1>,
            > + 'd,
        config: Config,
    ) -> Result<Self, CanInitError> {
        let this = Self::new_inner(peri, None, rx, tx, config)?;
        enable_async_irqs::<peripherals::CAN1>();
        Ok(this)
    }
}

#[cfg(can_h4)]
impl<'d, T: SlaveNode> Can<'d, T, Blocking> {
    pub fn new_blocking(
        filter_master: Peri<'d, peripherals::CAN1>,
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, Some(filter_master), rx, tx, config)
    }
}

#[cfg(can_h4)]
impl<'d, T: SlaveNode> Can<'d, T, NonBlocking> {
    pub fn new_nb(
        filter_master: Peri<'d, peripherals::CAN1>,
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, Some(filter_master), rx, tx, config)
    }
}

#[cfg(can_h4)]
impl<'d, T: SlaveNode> Can<'d, T, Async> {
    pub fn new(
        filter_master: Peri<'d, peripherals::CAN1>,
        peri: Peri<'d, T>,
        rx: Peri<'d, impl RxPin<T>>,
        tx: Peri<'d, impl TxPin<T>>,
        _irq: impl interrupt::typelevel::Binding<T::TransmitInterrupt, TransmitInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ReceiveInterrupt0, ReceiveInterruptHandler<T, 0>>
            + interrupt::typelevel::Binding<T::ReceiveInterrupt1, ReceiveInterruptHandler<T, 1>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        config: Config,
    ) -> Result<Self, CanInitError> {
        let this = Self::new_inner(peri, Some(filter_master), rx, tx, config)?;
        enable_async_irqs::<T>();
        Ok(this)
    }
}

fn enable_async_irqs<T: Instance>() {
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
}

impl<'d, T: Instance> Can<'d, T, Async> {
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
    fn new_inner<#[cfg(not(can_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        #[cfg(can_h4)] filter_master: Option<Peri<'d, peripherals::CAN1>>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, CanInitError> {
        #[cfg(can_h4)]
        debug_assert_eq!(
            core::ptr::eq(T::REGS.as_ptr(), pac::CAN1.as_ptr()),
            filter_master.is_none(),
            "CAN1 is its own filter master; CAN2/CAN3 need one"
        );

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
        T::enable_and_reset();
        // The filter table and its clock belong to CAN1, which this object owns
        // for CAN2/CAN3. CAN1 is not used as a node in that case, so resetting
        // it here is safe and gives the shared table a clean slate.
        #[cfg(can_h4)]
        if filter_master.is_some() {
            <peripherals::CAN1 as crate::peripheral::SealedRccPeripheral>::enable_and_reset();
        }
        T::REGS.intenr().write_value(pac::can::regs::Intenr(0));
        T::state().completed.lock(|completed| completed.set(0));
        apply_remap!();
        rx.set_as_input(Pull::Up);
        tx.set_high();
        set_as_af!(tx, AfType::output(OutputType::PushPull, Speed::High));
        let this = Self {
            _peri: peri,
            #[cfg(can_h4)]
            _filter_master: filter_master,
            rx: rx.into(),
            tx: tx.into(),
            config,
            _mode: PhantomData,
        };
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
        #[cfg(can_h4)]
        {
            // One shared filter table: program the CAN2/CAN3 start banks and
            // clear every bank before leaving filter init mode.
            let filters = pac::CAN1;
            filters.fctlr().modify(|reg| {
                reg.set_can2sb(14);
                reg.set_can3sb(28);
                reg.set_finit(true);
            });
            filters.fwr().write_value(pac::can::regs::Fwr(0));
            filters.fwr1().write_value(pac::can::regs::Fwr1(0));
            filters.fctlr().modify(|reg| reg.set_finit(false));
        }
        #[cfg(not(can_h4))]
        {
            T::REGS.fctlr().modify(|reg| reg.set_finit(true));
            T::REGS.fwr().write_value(pac::can::regs::Fwr(0));
            T::REGS.fctlr().modify(|reg| reg.set_finit(false));
        }
        T::REGS.ctlr().modify(|reg| reg.set_inrq(false));
        wait_init(T::REGS, false, config.poll_limit)?;
        Ok(this)
    }

    /// Configure one bank. No register is touched if the bank is out of range.
    pub fn set_filter(&mut self, filter: Filter) -> Result<(), InvalidFilter> {
        if filter.bank >= FILTER_BANKS {
            return Err(InvalidFilter);
        }
        critical_section::with(|_| {
            let regs = filter_regs::<T>();
            let bank = usize::from(filter.bank);
            regs.fctlr().modify(|reg| reg.set_finit(true));
            set_filter_active(regs, bank, false);
            set_filter_mode(regs, bank, &filter);
            regs.fr(bank * 2).write_value(pac::can::regs::Fr(filter.registers[0]));
            regs.fr(bank * 2 + 1)
                .write_value(pac::can::regs::Fr(filter.registers[1]));
            set_filter_active(regs, bank, true);
            regs.fctlr().modify(|reg| reg.set_finit(false));
        });
        Ok(())
    }

    pub fn disable_filter(&mut self, bank: u8) -> Result<(), InvalidFilter> {
        if bank >= FILTER_BANKS {
            return Err(InvalidFilter);
        }
        critical_section::with(|_| {
            let regs = filter_regs::<T>();
            regs.fctlr().modify(|reg| reg.set_finit(true));
            set_filter_active(regs, usize::from(bank), false);
            regs.fctlr().modify(|reg| reg.set_finit(false));
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
            // CAN2/CAN3 own CAN1 only to keep the filter table clocked.
            #[cfg(can_h4)]
            if self._filter_master.is_some() {
                <peripherals::CAN1 as crate::peripheral::SealedRccPeripheral>::disable();
            }
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
pub trait Instance:
    SealedInstance + RccPeripheral + crate::peripheral::RemapBound + embassy_hal_internal::PeripheralType
{
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
