//! Touch key detection (TKEY)
//!
//! The TKEY unit is an extension of the ADC. It discharges the pad on an ADC
//! channel, charges it from a current source for a set time, and has the ADC
//! convert the voltage the pad reached. A finger adds capacitance, so a
//! touched key reads lower than an untouched one.
//!
//! The interface follows embassy-stm32's `tsc` driver, in blocking and in
//! async form. Where it differs, the hardware does: there are no pin groups,
//! sampling capacitors or acquisition banks. Every key is one ADC channel, the
//! unit measures one channel per acquisition, and it discharges the pad by
//! itself at the start of each one.
//!
//! # Example
//!
//! ```rust
//! let p = ch32_hal::init(Default::default());
//!
//! let mut key = p.PA6;
//! let mut touch_controller = tkey::Tkey::new_blocking(p.ADC1, tkey::Config::default());
//!
//! loop {
//!     touch_controller.set_active_channel(&mut key);
//!     touch_controller.start();
//!     touch_controller.poll_for_acquisition();
//!     let value = touch_controller.get_value();
//!     // Compare the value with the untouched level of the key
//!     // ...
//! }
//! ```
//!
//! # Async Usage
//!
//! For async operation, use `Tkey::new_async` and `pend_for_acquisition` instead of polling.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::adc::{self, Adc, AdcChannel, Instance, SampleTime};
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::{interrupt, Peri};

/// TKEY configuration. A longer charge gives a higher reading.
pub struct Config {
    /// Configuration of the ADC. Its clock is the unit of the times below.
    pub adc: adc::Config,
    /// Charge time, in ADC clock cycles
    pub charge_time: SampleTime,
    /// Charge time offset, added to `charge_time`
    pub charge_offset: u8,
    /// Discharge time ahead of the charge
    pub discharge_time: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            adc: adc::Config::default(),
            charge_time: SampleTime::CYCLES11,
            charge_offset: 0x10,
            discharge_time: 0x08,
        }
    }
}

/// Peripheral state
#[derive(PartialEq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum State {
    /// Ready to start acquisition
    Ready,
    /// In process of sensor acquisition
    Busy,
}

/// TKEY interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _marker: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        T::regs().ctlr1().modify(|w| w.set_eocie(false));
        T::state().waker.wake();
    }
}

/// TKEY driver
pub struct Tkey<'d, T: Instance, K: PeriMode> {
    adc: Adc<'d, T>,
    state: State,
    value: u16,
    charge_time: SampleTime,
    charge_offset: u8,
    discharge_time: u8,
    _kind: PhantomData<K>,
}

impl<'d, T: Instance, K: PeriMode> Tkey<'d, T, K> {
    fn new_inner(peri: Peri<'d, T>, config: Config) -> Self {
        let adc = Adc::new(peri, config.adc);
        T::regs().ctlr1().modify(|w| w.set_tkenable(true));

        Self {
            adc,
            state: State::Ready,
            value: 0,
            charge_time: config.charge_time,
            charge_offset: config.charge_offset,
            discharge_time: config.discharge_time,
            _kind: PhantomData,
        }
    }

    /// Sets the active channel for the next TKEY acquisitions.
    ///
    /// The unit measures a single channel per acquisition. This also puts the
    /// channel's pin into analog mode.
    pub fn set_active_channel(&mut self, channel: &mut impl AdcChannel<T>) {
        self.adc.configure_channel(channel, 1, self.charge_time);
    }

    /// Start an acquisition on the active channel
    pub fn start(&mut self) {
        self.state = State::Busy;

        // Disable interrupts
        T::regs().ctlr1().modify(|w| w.set_eocie(false));

        // Clear flags
        T::regs().statr().modify(|w| w.set_eoc(false));

        T::regs().tkey_chgoffset().write_value(self.charge_offset);

        // Start the acquisition: the write of the discharge time does it
        T::regs().tkey_act_dcg().write_value(self.discharge_time);
    }

    /// Get current state of acquisition
    pub fn get_state(&mut self) -> State {
        if self.state == State::Busy && T::regs().statr().read().eoc() {
            // Reading the data register clears the end of conversion flag, so
            // the value is taken here, once.
            self.value = T::regs().rdatar().read().data();
            self.state = State::Ready;
        }
        self.state
    }

    /// Get the value of the acquisition, valid once the state is back to `Ready`
    pub fn get_value(&self) -> u16 {
        self.value
    }
}

impl<'d, T: Instance> Tkey<'d, T, Async> {
    /// Create a Tkey instance that can be awaited for completion
    pub fn new_async(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: Config,
    ) -> Self {
        let this = Self::new_inner(peri, config);

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        this
    }

    /// Asynchronously wait for the end of an acquisition
    pub async fn pend_for_acquisition(&mut self) {
        poll_fn(|cx| match self.get_state() {
            State::Busy => {
                T::state().waker.register(cx.waker());
                T::regs().ctlr1().modify(|w| w.set_eocie(true));
                if self.get_state() != State::Busy {
                    T::regs().ctlr1().modify(|w| w.set_eocie(false));
                    return Poll::Ready(());
                }
                Poll::Pending
            }
            _ => {
                T::regs().ctlr1().modify(|w| w.set_eocie(false));
                Poll::Ready(())
            }
        })
        .await;
    }
}

impl<'d, T: Instance> Tkey<'d, T, Blocking> {
    /// Create a Tkey instance that must be polled for completion
    pub fn new_blocking(peri: Peri<'d, T>, config: Config) -> Self {
        Self::new_inner(peri, config)
    }

    /// Wait for end of acquisition
    pub fn poll_for_acquisition(&mut self) {
        while self.get_state() == State::Busy {}
    }
}
