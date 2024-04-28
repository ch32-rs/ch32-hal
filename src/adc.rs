//! ADC
//!
//! Max clock: 14MHz for CHFV2x_V3x

#![macro_use]

use ch32_metapac::adc::vals;
use embassy_sync::waitqueue::AtomicWaker;

pub use crate::pac::adc::vals::SampleTime;
use crate::{into_ref, peripherals, Peripheral};

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
    adc: crate::PeripheralRef<'d, T>,
}

impl<'d, T: Instance> Adc<'d, T> {
    pub fn new(adc: impl Peripheral<P = T> + 'd, config: Config) -> Self {
        into_ref!(adc);
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
    pub fn configure_channel(&mut self, channel: &mut impl AdcPin<T>, rank: u8, sample_time: SampleTime) {
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
    pub fn convert(&mut self, channel: &mut impl AdcPin<T>, sample_time: SampleTime) -> u16 {
        self.configure_channel(channel, 1, sample_time);

        T::regs().ctlr2().modify(|w| w.set_swstart(true));

        // while not end of conversion
        while !T::regs().statr().read().eoc() {}

        T::regs().rdatar().read().data()
    }
}

trait SealedInstance {
    #[allow(unused)]
    fn regs() -> crate::pac::adc::Adc;
    fn state() -> &'static State;
}

pub(crate) trait SealedAdcPin<T: Instance> {
    fn set_as_analog(&mut self) {}

    #[allow(unused)]
    fn channel(&self) -> u8;
}

trait SealedInternalChannel<T> {
    #[allow(unused)]
    fn channel(&self) -> u8;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + crate::Peripheral<P = Self> + crate::peripheral::RccPeripheral {
    type Interrupt: crate::interrupt::typelevel::Interrupt;
}

/// ADC pin.
#[allow(private_bounds)]
pub trait AdcPin<T: Instance>: SealedAdcPin<T> {}
/// ADC internal channel.
#[allow(private_bounds)]
pub trait InternalChannel<T>: SealedInternalChannel<T> {}

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
        impl crate::adc::AdcPin<peripherals::$inst> for crate::peripherals::$pin {}

        impl crate::adc::SealedAdcPin<peripherals::$inst> for crate::peripherals::$pin {
            fn set_as_analog(&mut self) {
                <Self as crate::gpio::sealed::Pin>::set_as_analog(self);
            }

            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

// pub struct Vref;
//impl<T: Instance> AdcPin<T> for Vref {}
//impl<T: Instance> sealed::AdcPin<T> for Vref {
//    fn channel(&self) -> u8 {
//        15
//    }
//}
