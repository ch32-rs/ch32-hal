//! ADC, Analog to Digital Converter

#![macro_use]

use core::marker::PhantomData;

use embassy_sync::waitqueue::AtomicWaker;

use crate::pac::adc::vals;
pub use crate::pac::adc::vals::SampleTime;
use crate::{peripherals, Peri};

/// ADC bit resolution
#[cfg(any(adc_v0, adc_ch641))]
pub const ADC_MAX: u32 = (1 << 10) - 1;
#[cfg(not(any(adc_v0, adc_ch641)))]
pub const ADC_MAX: u32 = (1 << 12) - 1;

// No calibration data, voltage should be 1.2V (1.16 to 1.24)
pub const VREF_INT: u32 = 1200;

pub struct Config {
    /// Div1 to Div16
    // raw values are 0 to 0b111
    pub clkdiv: u8,
    // TODO: handle "-1"
    pub channel_count: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // Use Divide by 8 as default, maximum is 14 MHz
            // Up to 144 MHz system clock can be reached
            // Power on default is Divide by 2,
            clkdiv: 0b11,
            channel_count: 1,
        }
    }
}

pub struct State {
    pub waker: AtomicWaker,
}
impl State {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

/// Analog to Digital driver.
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: Peri<'d, T>,
}

impl<'d, T: Instance> Adc<'d, T> {
    #[allow(unused)]
    pub fn new(adc: Peri<'d, T>, config: Config) -> Self {
        T::enable_and_reset();

        // TODO: ADCPRE
        // CTLR3 not avaiable to CH3V0, CH32V1
        #[cfg(any(adc_v3, adc_x0))]
        T::regs().ctlr3().modify(|w| w.set_clk_div(config.clkdiv));

        // clear, only independent mode is supported
        T::regs().ctlr1().modify(|w| {
            w.0 = 0;
        });

        // no external trigger
        // no continuous mode
        T::regs().ctlr2().modify(|w| {
            w.set_align(false); // right aligned
            w.set_exttrig(true); // external trigger
            w.set_extsel(vals::Extsel::SWSTART); //  SWSTART Software trigger
            w.set_cont(false); // single conversion
        });

        const CHANNEL_COUNT: u8 = 1;
        T::regs().rsqr1().modify(|w| w.set_l(CHANNEL_COUNT - 1));

        // ADC ON
        T::regs().ctlr2().modify(|w| w.set_adon(true));

        Self { adc }
    }

    // regular conversion
    pub fn configure_channel(&mut self, channel: &mut impl AdcChannel<T>, rank: u8, sample_time: SampleTime) {
        channel.set_as_analog();

        let channel = channel.channel();

        // sample time config
        if channel < 10 {
            T::regs().samptr2().modify(|w| w.set_smp(channel as usize, sample_time));
        } else {
            T::regs()
                .samptr1()
                .modify(|w| w.set_smp((channel - 10) as usize, sample_time));
        }

        // regular sequence config
        assert!(rank < 17 || rank > 0);
        if rank < 7 {
            T::regs()
                .rsqr3()
                .modify(|w| w.set_sq((rank - 1) as usize, channel & 0b11111));
        } else if rank < 13 {
            T::regs()
                .rsqr2()
                .modify(|w| w.set_sq((rank - 7) as usize, channel & 0b11111));
        } else {
            T::regs()
                .rsqr1()
                .modify(|w| w.set_sq((rank - 13) as usize, channel & 0b11111));
        }
    }

    // Get_ADC_Val
    pub fn convert(&mut self, channel: &mut impl AdcChannel<T>, sample_time: SampleTime) -> u16 {
        self.configure_channel(channel, 1, sample_time);

        T::regs().ctlr2().modify(|w| w.set_swstart(true));

        // while not end of conversion
        while !T::regs().statr().read().eoc() {}

        T::regs().rdatar().read().data()
    }
}

#[allow(unused)]
trait SealedInstance {
    fn regs() -> crate::pac::adc::Adc;
    fn state() -> &'static State;
}

pub(crate) trait SealedAdcChannel<T: Instance> {
    fn set_as_analog(&mut self) {}

    #[allow(unused)]
    fn channel(&self) -> u8;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + crate::peripheral::RccPeripheral {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// ADC pin.
#[allow(private_bounds)]
pub trait AdcChannel<T: Instance>: SealedAdcChannel<T> + Sized {
    fn degrade_adc(mut self) -> AnyAdcChannel<T> {
        self.set_as_analog();

        AnyAdcChannel {
            channel: self.channel(),
            _phantom: PhantomData,
        }
    }
}

/// A type-erased channel for a given ADC instance.
///
/// This is useful in scenarios where you need the ADC channels to have the same type, such as
/// storing them in an array.
pub struct AnyAdcChannel<T> {
    channel: u8,
    _phantom: PhantomData<T>,
}

impl<T: Instance> AdcChannel<T> for AnyAdcChannel<T> {}
impl<T: Instance> SealedAdcChannel<T> for AnyAdcChannel<T> {
    fn channel(&self) -> u8 {
        self.channel
    }
}

foreach_peripheral!(
    (adc, $inst:ident) => {
        impl crate::adc::SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::adc::Adc {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl crate::adc::Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::adc::AdcChannel<peripherals::$inst> for crate::peripherals::$pin {}

        impl crate::adc::SealedAdcChannel<peripherals::$inst> for crate::peripherals::$pin {
            fn set_as_analog(&mut self) {
                <Self as crate::gpio::SealedPin>::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }

        impl crate::adc::AdcChannel<peripherals::$inst> for crate::Peri<'_, crate::peripherals::$pin> {}

        impl crate::adc::SealedAdcChannel<peripherals::$inst> for crate::Peri<'_, crate::peripherals::$pin> {
            fn set_as_analog(&mut self) {
                <crate::peripherals::$pin as crate::gpio::SealedPin>::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

#[cfg(adc_l1)]
mod ch_internal {
    use super::*;

    pub struct VddaDiv2;
    impl<T: Instance> AdcChannel<T> for VddaDiv2 {}
    impl<T: Instance> SealedAdcChannel<T> for VddaDiv2 {
        fn channel(&self) -> u8 {
            18
        }
    }

    pub struct VrefInt;
    impl<T: Instance> AdcChannel<T> for VrefInt {}
    impl<T: Instance> SealedAdcChannel<T> for VrefInt {
        fn set_as_analog(&mut self) {
            T::regs().ctlr2().modify(|w| w.set_tsvrefe(true));
        }
        fn channel(&self) -> u8 {
            17
        }
    }

    pub struct Temperature;
    impl<T: Instance> AdcChannel<T> for Temperature {}
    impl<T: Instance> SealedAdcChannel<T> for Temperature {
        fn set_as_analog(&mut self) {
            T::regs().ctlr2().modify(|w| w.set_tsvrefe(true));
        }
        fn channel(&self) -> u8 {
            16
        }
    }
}
#[cfg(adc_x0)]
mod ch_internal {
    use super::*;

    pub struct VrefInt;
    impl<T: Instance> AdcChannel<T> for VrefInt {}
    impl<T: Instance> SealedAdcChannel<T> for VrefInt {
        fn channel(&self) -> u8 {
            15
        }
    }
}
#[cfg(any(adc_v1, adc_v3))]
mod ch_internal {
    use super::*;

    pub struct VrefInt;
    impl<T: Instance> AdcChannel<T> for VrefInt {}
    impl<T: Instance> SealedAdcChannel<T> for VrefInt {
        fn set_as_analog(&mut self) {
            T::regs().ctlr2().modify(|w| w.set_tsvrefe(true));
        }
        fn channel(&self) -> u8 {
            17
        }
    }

    pub struct Temperature;
    impl<T: Instance> AdcChannel<T> for Temperature {}
    impl<T: Instance> SealedAdcChannel<T> for Temperature {
        fn set_as_analog(&mut self) {
            T::regs().ctlr2().modify(|w| w.set_tsvrefe(true));
        }
        fn channel(&self) -> u8 {
            16
        }
    }
}

#[cfg(adc_v0)]
mod ch_internal {
    use super::*;

    pub struct Vref;
    impl<T: Instance> AdcChannel<T> for Vref {}
    impl<T: Instance> SealedAdcChannel<T> for Vref {
        fn channel(&self) -> u8 {
            8
        }
    }

    pub struct Vcal;
    impl<T: Instance> AdcChannel<T> for Vcal {}
    impl<T: Instance> SealedAdcChannel<T> for Vcal {
        fn channel(&self) -> u8 {
            9
        }
    }
}

#[cfg(adc_ch641)]
mod ch_internal {
    use super::*;
    use crate::pac::EXTEND;

    // FIXME: move peripherals and pins

    /// Differential input current sampling
    pub struct Isp {
        _marker: core::marker::PhantomData<()>,
    }

    #[derive(Default)]
    pub struct IspConfig {
        /// DC bias is about 400 in raw ADC sample value.
        pub dc_bias: bool,
    }

    impl Isp {
        pub fn new_non_inverting(in_positive: impl NonInvertingPin, config: IspConfig) -> Self {
            in_positive.set_as_isp_pin();
            EXTEND.ctlr2().modify(|w| {
                w.set_isp_ps(in_positive.pin_sel());
                w.set_isp_ns(true); // use GND
                w.set_isp_ae(true);
                w.set_isp_be(config.dc_bias);
            });
            Isp {
                _marker: core::marker::PhantomData,
            }
        }

        // PB6/PA6, PB7
        pub fn new(in_positive: impl NonInvertingPin, in_negative: impl InvertingPin, config: IspConfig) -> Self {
            in_positive.set_as_isp_pin();
            in_negative.set_as_isp_pin();
            EXTEND.ctlr2().modify(|w| {
                w.set_isp_ps(in_positive.pin_sel());
                w.set_isp_ns(false); // true: use GND; false: use PB7
                w.set_isp_ae(true);
                w.set_isp_be(config.dc_bias);
            });
            Isp {
                _marker: core::marker::PhantomData,
            }
        }
    }

    pub trait NonInvertingPin {
        fn set_as_isp_pin(&self) {}
        fn pin_sel(&self) -> bool;
    }
    pub trait InvertingPin {
        fn set_as_isp_pin(&self) {}
    }
    impl NonInvertingPin for peripherals::PA6 {
        fn pin_sel(&self) -> bool {
            true
        }
    }
    // aka. ISP pin
    impl NonInvertingPin for peripherals::PB6 {
        fn pin_sel(&self) -> bool {
            false
        }
    }
    impl InvertingPin for peripherals::PB7 {}

    impl<T: Instance> AdcChannel<T> for Isp {}
    impl<T: Instance> SealedAdcChannel<T> for Isp {
        fn channel(&self) -> u8 {
            8
        }
    }

    pub struct VhvDiv5;
    impl<T: Instance> AdcChannel<T> for VhvDiv5 {}
    impl<T: Instance> SealedAdcChannel<T> for VhvDiv5 {
        fn channel(&self) -> u8 {
            15
        }
    }
}

pub use ch_internal::*;
