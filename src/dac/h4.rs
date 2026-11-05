//! Digital to Analog Converter (DAC) for CH32H4.
//!
//! H4 gives each channel its own control bits and data registers instead of
//! the array-of-two layout the V3 block uses: `EN1`/`EN2`, `R12BDHR1`/
//! `R12BDHR2`, and so on. The public API mirrors the V3 driver, so a channel
//! is still selected by its `N` const parameter.
#![macro_use]

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU8, Ordering};

use crate::dma::NoDma;
use crate::gpio::{AnyPin, SealedPin};
use crate::internal::drop::OnDrop;
use crate::peripheral::RccPeripheral;
use crate::{peripherals, Peri, PeripheralType};

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The DMA transfer counter cannot represent the buffer length.
    BufferTooLong,
    /// DMA requires an enabled hardware trigger, excluding software trigger.
    TriggerNotConfigured,
    /// Waveform amplitude must be in the range 0..=11.
    InvalidAmplitude,
}

/// DAC trigger source (RM §18.2.2.5).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TriggerSel {
    TIM6_TRGO = 0,
    TIM8_TRGO = 1,
    TIM7_TRGO = 2,
    TIM5_TRGO = 3,
    TIM2_TRGO = 4,
    TIM4_TRGO = 5,
    EXTI9 = 6,
    SOFTWARE = 7,
}

/// Waveform generator selection.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Waveform {
    Disabled = 0,
    Noise = 1,
    Triangle = 2,
}

struct State {
    users: AtomicU8,
}

fn validate_dma(control: &crate::pac::dac::regs::Ctlr, channel: usize, length: usize) -> Result<(), Error> {
    if length == 0 {
        return Ok(());
    }
    if length > u16::MAX as usize {
        return Err(Error::BufferTooLong);
    }
    let (trigger_enabled, trigger_source) = if channel == 0 {
        (control.ten1(), control.tsel1())
    } else {
        (control.ten2(), control.tsel2())
    };
    if !trigger_enabled || trigger_source == TriggerSel::SOFTWARE as u8 {
        return Err(Error::TriggerNotConfigured);
    }
    Ok(())
}

fn acquire<T: Instance>(channels: u8) {
    critical_section::with(|cs| {
        let users = T::state().users.load(Ordering::Relaxed);
        assert!(users + channels <= 2);
        if users == 0 {
            T::enable_and_reset_with_cs(cs);
        }
        T::state().users.store(users + channels, Ordering::Relaxed);
    });
}

fn release<T: Instance>(channel: usize) {
    critical_section::with(|cs| {
        T::regs().ctlr().modify(|reg| {
            if channel == 0 {
                reg.set_en1(false);
                reg.set_dmaen1(false);
                reg.set_ten1(false);
            } else {
                reg.set_en2(false);
                reg.set_dmaen2(false);
                reg.set_ten2(false);
            }
        });
        let users = T::state().users.load(Ordering::Relaxed);
        T::state().users.store(users - 1, Ordering::Relaxed);
        if users == 1 {
            T::disable_with_cs(cs);
        }
    });
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Single 8 or 12 bit value that can be output by the DAC.
///
/// 12-bit values outside the permitted range are silently truncated.
pub enum Value {
    /// 8 bit value
    Bit8(u8),
    /// Unshifted 12-bit value, written through the left-aligned register.
    Bit12Left(u16),
    /// 12 bit value stored in a u16, right-aligned
    Bit12Right(u16),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Dual 8 or 12 bit values that can be output by the DAC channels 1 and 2 simultaneously.
///
/// 12-bit values outside the permitted range are silently truncated.
pub enum DualValue {
    /// 8 bit value
    Bit8(u8, u8),
    /// Unshifted 12-bit values, written through the left-aligned register.
    Bit12Left(u16, u16),
    /// 12 bit value stored in a u16, right-aligned
    Bit12Right(u16, u16),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Array variant of [`Value`].
pub enum ValueArray<'a> {
    /// 8 bit values
    Bit8(&'a [u8]),
    /// DMA words with each 12-bit sample already aligned to bits 15:4.
    Bit12Left(&'a [u16]),
    /// 12 bit values stored in a u16, right-aligned
    Bit12Right(&'a [u16]),
}

/// Driver for a single DAC channel.
///
/// If you want to use both channels, either together or independently,
/// create a [`Dac`] first and use it to access each channel.
pub struct DacChannel<'d, T: Instance, const N: u8, DMA: PeripheralType = NoDma> {
    phantom: PhantomData<&'d mut T>,
    pin: Peri<'d, AnyPin>,
    #[allow(unused)]
    dma: Peri<'d, DMA>,
}

/// DAC channel 1 type alias.
pub type DacCh1<'d, T, DMA = NoDma> = DacChannel<'d, T, 1, DMA>;
/// DAC channel 2 type alias.
pub type DacCh2<'d, T, DMA = NoDma> = DacChannel<'d, T, 2, DMA>;

impl<'d, T: Instance, const N: u8, DMA: PeripheralType> DacChannel<'d, T, N, DMA> {
    const IDX: usize = (N - 1) as usize;

    fn set_enable_bit(on: bool) {
        critical_section::with(|_| {
            T::regs().ctlr().modify(|reg| {
                if N == 1 {
                    reg.set_en1(on);
                } else {
                    reg.set_en2(on);
                }
            });
        });
    }

    fn set_dma_enable_bit(on: bool) {
        T::regs().ctlr().modify(|reg| {
            if N == 1 {
                reg.set_dmaen1(on);
            } else {
                reg.set_dmaen2(on);
            }
        });
    }

    fn write_value(value: Value) {
        match (N, value) {
            (1, Value::Bit8(v)) => T::regs().r8bdhr1().write(|reg| reg.set_dacc1dhr(v)),
            (2, Value::Bit8(v)) => T::regs().r8bdhr2().write(|reg| reg.set_dacc2dhr(v)),
            (1, Value::Bit12Left(v)) => T::regs().l12bdhr1().write(|reg| reg.set_dacc1dhr(v)),
            (2, Value::Bit12Left(v)) => T::regs().l12bdhr2().write(|reg| reg.set_dacc2dhr(v)),
            (1, Value::Bit12Right(v)) => T::regs().r12bdhr1().write(|reg| reg.set_dacc1dhr(v)),
            (2, Value::Bit12Right(v)) => T::regs().r12bdhr2().write(|reg| reg.set_dacc2dhr(v)),
            _ => unreachable!(),
        }
    }

    /// 8-bit holding register of this channel.
    fn dhr8_ptr() -> *mut u8 {
        if N == 1 {
            T::regs().r8bdhr1().as_ptr() as *mut u8
        } else {
            T::regs().r8bdhr2().as_ptr() as *mut u8
        }
    }

    /// 12-bit left-aligned holding register of this channel.
    fn dhr12l_ptr() -> *mut u16 {
        if N == 1 {
            T::regs().l12bdhr1().as_ptr() as *mut u16
        } else {
            T::regs().l12bdhr2().as_ptr() as *mut u16
        }
    }

    /// 12-bit right-aligned holding register of this channel.
    fn dhr12r_ptr() -> *mut u16 {
        if N == 1 {
            T::regs().r12bdhr1().as_ptr() as *mut u16
        } else {
            T::regs().r12bdhr2().as_ptr() as *mut u16
        }
    }

    /// Create a new `DacChannel` instance, consuming the underlying DAC peripheral.
    ///
    /// If you're not using DMA, pass [`dma::NoDma`] for the `dma` argument.
    ///
    /// The channel is enabled on creation and begin to drive the output pin.
    /// Note that methods such as `set_trigger()` will
    /// disable the channel; you must re-enable it with `enable()`.
    ///
    /// By default, triggering is disabled, but it can be enabled using
    /// [`DacChannel::set_trigger()`].
    pub fn new(_peri: Peri<'d, T>, dma: Peri<'d, DMA>, pin: Peri<'d, impl DacPin<T, N> + crate::gpio::Pin>) -> Self {
        assert!(N == 1 || N == 2);
        pin.set_as_analog();
        acquire::<T>(1);
        let mut dac = Self {
            phantom: PhantomData,
            pin: pin.into(),
            dma,
        };
        dac.enable();
        dac
    }

    /// Enable or disable this channel.
    pub fn set_enable(&mut self, on: bool) {
        Self::set_enable_bit(on);
    }

    /// Enable this channel.
    pub fn enable(&mut self) {
        self.set_enable(true)
    }

    /// Disable this channel.
    pub fn disable(&mut self) {
        self.set_enable(false)
    }

    /// Enable the output buffer, or bypass it for a high-impedance load.
    pub fn set_buffering(&mut self, enabled: bool) {
        critical_section::with(|_| {
            T::regs().ctlr().modify(|reg| {
                if N == 1 {
                    reg.set_boff1(!enabled);
                } else {
                    reg.set_boff2(!enabled);
                }
            });
        });
    }

    /// Select noise or triangle generation and its mask/amplitude encoding.
    pub fn set_waveform(&mut self, waveform: Waveform, amplitude: u8) -> Result<(), Error> {
        if amplitude > 11 {
            return Err(Error::InvalidAmplitude);
        }
        critical_section::with(|_| {
            T::regs().ctlr().modify(|reg| {
                if N == 1 {
                    reg.set_wave1(waveform as u8);
                    reg.set_mamp1(amplitude);
                } else {
                    reg.set_wave2(waveform as u8);
                    reg.set_mamp2(amplitude);
                }
            });
        });
        Ok(())
    }

    /// Set the trigger source for this channel.
    ///
    /// This method disables the channel, so you may need to re-enable afterwards.
    pub fn set_trigger(&mut self, source: TriggerSel) {
        critical_section::with(|_| {
            T::regs().ctlr().modify(|reg| {
                if N == 1 {
                    reg.set_en1(false);
                    reg.set_tsel1(source as u8);
                } else {
                    reg.set_en2(false);
                    reg.set_tsel2(source as u8);
                }
            });
        });
    }

    /// Enable or disable triggering for this channel.
    pub fn set_triggering(&mut self, on: bool) {
        critical_section::with(|_| {
            T::regs().ctlr().modify(|reg| {
                if N == 1 {
                    reg.set_ten1(on);
                } else {
                    reg.set_ten2(on);
                }
            });
        });
    }

    /// Software trigger this channel.
    pub fn trigger(&mut self) {
        T::regs().swtr().write(|reg| {
            if N == 1 {
                reg.set_swtrig1(true);
            } else {
                reg.set_swtrig2(true);
            }
        });
    }

    /// Write a new value to this channel.
    ///
    /// If triggering is not enabled, the new value is immediately output; otherwise,
    /// it will be output after the next trigger.
    pub fn set(&mut self, value: Value) {
        Self::write_value(value);
    }

    /// Read the current output value of the DAC.
    pub fn read(&self) -> u16 {
        if N == 1 {
            T::regs().dor1().read().dacc1dor()
        } else {
            T::regs().dor2().read().dacc2dor()
        }
    }
}

macro_rules! impl_dma_methods {
    ($n:literal, $trait:ident) => {
        impl<'d, T: Instance, DMA: PeripheralType> DacChannel<'d, T, $n, DMA>
        where
            DMA: $trait<T>,
        {
            /// Write `data` to this channel via DMA.
            ///
            /// To prevent delays or glitches when outputing a periodic waveform, the `circular`
            /// flag can be set. This configures a circular DMA transfer that continually outputs
            /// `data`. Note that for performance reasons in circular mode the transfer-complete
            /// interrupt is disabled.
            pub async fn write(&mut self, data: ValueArray<'_>, circular: bool) {
                self.try_write(data, circular)
                    .await
                    .expect("invalid DAC DMA configuration");
            }

            /// DMA output, stopping the channel on cancellation.
            ///
            /// Requires triggering enabled with a timer or EXTI source. Circular
            /// transfers run until cancelled. Empty buffers leave the DAC unchanged.
            /// Completion means all samples reached the holding register; the last
            /// sample may need another trigger to reach the output. The channel
            /// stays enabled on completion, with its DMA request disabled.
            pub async fn try_write(&mut self, data: ValueArray<'_>, circular: bool) -> Result<(), Error> {
                let length = match &data {
                    ValueArray::Bit8(buffer) => buffer.len(),
                    ValueArray::Bit12Left(buffer) | ValueArray::Bit12Right(buffer) => buffer.len(),
                };
                if length == 0 {
                    return Ok(());
                }
                let control = T::regs().ctlr().read();
                validate_dma(&control, Self::IDX, length)?;
                let cleanup = OnDrop::new(|| {
                    critical_section::with(|_| {
                        T::regs().ctlr().modify(|w| {
                            if Self::IDX == 0 {
                                w.set_en1(false);
                                w.set_dmaen1(false);
                            } else {
                                w.set_en2(false);
                                w.set_dmaen2(false);
                            }
                        });
                    });
                });

                let tx_request = self.dma.request();

                let tx_options = crate::dma::TransferOptions {
                    circular,
                    half_transfer_ir: false,
                    complete_transfer_ir: !circular,
                    ..Default::default()
                };

                let tx_f = match data {
                    ValueArray::Bit8(buf) => unsafe {
                        crate::dma::Transfer::new_write(
                            self.dma.clone_unchecked(),
                            tx_request,
                            buf,
                            Self::dhr8_ptr(),
                            tx_options,
                        )
                    },
                    ValueArray::Bit12Left(buf) => unsafe {
                        crate::dma::Transfer::new_write(
                            self.dma.clone_unchecked(),
                            tx_request,
                            buf,
                            Self::dhr12l_ptr(),
                            tx_options,
                        )
                    },
                    ValueArray::Bit12Right(buf) => unsafe {
                        crate::dma::Transfer::new_write(
                            self.dma.clone_unchecked(),
                            tx_request,
                            buf,
                            Self::dhr12r_ptr(),
                            tx_options,
                        )
                    },
                };

                critical_section::with(|_| {
                    T::regs().ctlr().modify(|w| {
                        if Self::IDX == 0 {
                            w.set_en1(true);
                            w.set_dmaen1(true);
                        } else {
                            w.set_en2(true);
                            w.set_dmaen2(true);
                        }
                    });
                });

                tx_f.await;
                Self::set_dma_enable_bit(false);
                cleanup.defuse();
                Ok(())
            }
        }
    };
}

impl_dma_methods!(1, DacDma1);
impl_dma_methods!(2, DacDma2);

impl<'d, T: Instance, const N: u8, DMA: PeripheralType> Drop for DacChannel<'d, T, N, DMA> {
    fn drop(&mut self) {
        self.pin.set_as_disconnected();
        release::<T>(Self::IDX);
    }
}

/// DAC driver.
///
/// Use this struct when you want to use both channels, either together or independently.
///
/// # Example
///
/// ```ignore
/// // Pins may need to be changed for your specific device.
/// let (dac_ch1, dac_ch2) = ch32_hal::dac::Dac::new(p.DAC1, dma_ch1, dma_ch2, p.PA4, p.PA5).split();
/// ```
pub struct Dac<'d, T: Instance, DMACh1: PeripheralType = NoDma, DMACh2: PeripheralType = NoDma> {
    ch1: DacChannel<'d, T, 1, DMACh1>,
    ch2: DacChannel<'d, T, 2, DMACh2>,
}

impl<'d, T: Instance, DMACh1: PeripheralType, DMACh2: PeripheralType> Dac<'d, T, DMACh1, DMACh2> {
    /// Create a new `Dac` instance, consuming the underlying DAC peripheral.
    ///
    /// This struct allows you to access both channels of the DAC, where available. You can either
    /// call `split()` to obtain separate `DacChannel`s, or use methods on `Dac` to use
    /// the two channels together.
    ///
    /// The channels are enabled on creation and begin to drive their output pins.
    /// Note that methods such as `set_trigger()` will
    /// disable the channel; you must re-enable them with `enable()`.
    ///
    /// By default, triggering is disabled, but it can be enabled using the `set_trigger()`
    /// method on the underlying channels.
    pub fn new(
        _peri: Peri<'d, T>,
        dma_ch1: Peri<'d, DMACh1>,
        dma_ch2: Peri<'d, DMACh2>,
        pin_ch1: Peri<'d, impl DacPin<T, 1> + crate::gpio::Pin>,
        pin_ch2: Peri<'d, impl DacPin<T, 2> + crate::gpio::Pin>,
    ) -> Self {
        pin_ch1.set_as_analog();
        pin_ch2.set_as_analog();

        acquire::<T>(2);

        let mut ch1 = DacCh1 {
            phantom: PhantomData,
            pin: pin_ch1.into(),
            dma: dma_ch1,
        };
        ch1.enable();

        let mut ch2 = DacCh2 {
            phantom: PhantomData,
            pin: pin_ch2.into(),
            dma: dma_ch2,
        };
        ch2.enable();

        Self { ch1, ch2 }
    }

    /// Split this `Dac` into separate channels.
    ///
    /// You can access and move the channels around separately after splitting.
    pub fn split(self) -> (DacCh1<'d, T, DMACh1>, DacCh2<'d, T, DMACh2>) {
        (self.ch1, self.ch2)
    }

    /// Temporarily access channel 1.
    pub fn ch1(&mut self) -> &mut DacCh1<'d, T, DMACh1> {
        &mut self.ch1
    }

    /// Temporarily access channel 2.
    pub fn ch2(&mut self) -> &mut DacCh2<'d, T, DMACh2> {
        &mut self.ch2
    }

    /// Trigger both channels in the same peripheral write.
    pub fn trigger(&mut self) {
        T::regs().swtr().write(|reg| {
            reg.set_swtrig1(true);
            reg.set_swtrig2(true);
        });
    }

    /// Simultaneously update channels 1 and 2 with a new value.
    ///
    /// If triggering is not enabled, the new values are immediately output;
    /// otherwise, they will be output after the next trigger.
    pub fn set(&mut self, values: DualValue) {
        match values {
            DualValue::Bit8(v1, v2) => T::regs().rd8bdhr().write(|reg| {
                reg.set_dacc1dhr(v1);
                reg.set_dacc2dhr(v2);
            }),
            DualValue::Bit12Left(v1, v2) => T::regs().ld12bdhr().write(|reg| {
                reg.set_dacc1dhr(v1);
                reg.set_dacc2dhr(v2);
            }),
            DualValue::Bit12Right(v1, v2) => T::regs().rd12bdhr().write(|reg| {
                reg.set_dacc1dhr(v1);
                reg.set_dacc2dhr(v2);
            }),
        }
    }
}

trait SealedInstance {
    fn regs() -> &'static crate::pac::dac::Dac;
    fn state() -> &'static State;
}

/// DAC instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + RccPeripheral + 'static {}
dma_trait!(DacDma1, Instance);
dma_trait!(DacDma2, Instance);

/// Marks a pin that can be used with the DAC
pub trait DacPin<T: Instance, const C: u8>: crate::gpio::Pin + 'static {}

foreach_peripheral!(
    (dac, $inst:ident) => {
        impl crate::dac::SealedInstance for peripherals::$inst {
            fn regs() -> &'static crate::pac::dac::Dac {
                &crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State { users: AtomicU8::new(0) };
                &STATE
            }
        }

        impl crate::dac::Instance for peripherals::$inst {}
    };
);

macro_rules! impl_dac_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::dac::DacPin<peripherals::$inst, $ch> for crate::peripherals::$pin {}
    };
}
