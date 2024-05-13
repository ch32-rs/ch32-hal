//! Timers, PWM.
//!
//! Instance type:
//!
//! - BasicInstance
//! - GeneralInstance (16-bit or 32-bit)
//! - AdvancedInstance
//!
//! Different vs embassy-stm32:
//!
//! - No too many levels of abstraction
//! - 2CH GPTM instances are also have helper functions defined

use crate::peripheral::RccPeripheral;
use crate::{interrupt, RemapPeripheral};

pub mod complementary_pwm;
pub mod low_level;
pub mod simple_pwm;

/// Timer channel.
#[derive(Clone, Copy, PartialEq)]
pub enum Channel {
    /// Channel 1.
    Ch1,
    /// Channel 2.
    Ch2,
    /// Channel 3.
    Ch3,
    /// Channel 4.
    Ch4,
}

impl Channel {
    /// Get the channel index (0..3)
    pub fn index(&self) -> usize {
        match self {
            Channel::Ch1 => 0,
            Channel::Ch2 => 1,
            Channel::Ch3 => 2,
            Channel::Ch4 => 3,
        }
    }
}

/// Amount of bits of a timer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimerBits {
    /// 16 bits.
    Bits16,
    /// 32 bits.
    #[cfg(any(ch32l1, ch32v208))]
    Bits32,
}

/// Core timer instance.
pub trait CoreInstance: RccPeripheral + RemapPeripheral + 'static {
    /// Update Interrupt for this timer.
    type UpdateInterrupt: interrupt::typelevel::Interrupt;

    /// Amount of bits this timer has.
    const BITS: TimerBits;

    /// Registers for this timer.
    ///
    /// This is a raw pointer to the register block. The actual register block layout varies depending on the timer type.
    fn regs() -> *mut ();
}

/// Basic timer instance, BCTM
pub trait BasicInstance: CoreInstance {}

trait SealedGeneralInstance: BasicInstance {
    fn enable_outputs(&self) {}

    fn get_counting_mode(&self) -> low_level::CountingMode {
        // (vals::Cms::EDGEALIGNED, vals::Dir::UP),
        low_level::CountingMode::EdgeAlignedUp
    }
}

/// General-purpose 16-bit timer with 4 channels instance.
#[allow(private_bounds)]
pub trait GeneralInstance16bit: SealedGeneralInstance {
    /// Capture compare interrupt for this timer.
    type CaptureCompareInterrupt: interrupt::typelevel::Interrupt;

    /// Trigger event interrupt for this timer.
    type TriggerInterrupt: interrupt::typelevel::Interrupt;
}

/// General-purpose 32-bit timer with 4 channels instance.
pub trait GeneralInstance32bit: GeneralInstance16bit {}

/// Advanced 16-bit timer with 4 channels instance.
pub trait AdvancedInstance: GeneralInstance16bit {
    /// Communication interrupt for this timer.
    type CommunicationInterrupt: interrupt::typelevel::Interrupt;
    /// Break input interrupt for this timer.
    type BreakInputInterrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(Channel1Pin, GeneralInstance16bit);
pin_trait!(Channel2Pin, GeneralInstance16bit);
pin_trait!(Channel3Pin, GeneralInstance16bit);
pin_trait!(Channel4Pin, GeneralInstance16bit);
pin_trait!(ExternalTriggerPin, GeneralInstance16bit);

pin_trait!(Channel1ComplementaryPin, AdvancedInstance);
pin_trait!(Channel2ComplementaryPin, AdvancedInstance);
pin_trait!(Channel3ComplementaryPin, AdvancedInstance);
// No Channel4ComplementaryPin for ADTM
// pin_trait!(Channel4ComplementaryPin, AdvancedInstance);

pin_trait!(BreakInputPin, AdvancedInstance);

// Update Event trigger DMA for every timer
dma_trait!(UpDma, BasicInstance);

dma_trait!(Ch1Dma, GeneralInstance16bit);
dma_trait!(Ch2Dma, GeneralInstance16bit);
dma_trait!(Ch3Dma, GeneralInstance16bit);
dma_trait!(Ch4Dma, GeneralInstance16bit);

#[allow(unused)]
macro_rules! impl_core_timer {
    ($inst:ident, $bits:expr) => {
        impl CoreInstance for crate::peripherals::$inst {
            type UpdateInterrupt = crate::_generated::peripheral_interrupts::$inst::UP;

            const BITS: TimerBits = $bits;

            fn regs() -> *mut () {
                crate::pac::$inst.as_ptr()
            }
        }
    };
}

#[allow(unused)]
macro_rules! impl_general_16bit {
    ($inst:ident) => {
        impl GeneralInstance16bit for crate::peripherals::$inst {
            type CaptureCompareInterrupt = crate::_generated::peripheral_interrupts::$inst::CC;
            type TriggerInterrupt = crate::_generated::peripheral_interrupts::$inst::TRG;
        }
    };
}

#[allow(unused)]
macro_rules! impl_advanced {
    ($inst:ident) => {
        impl AdvancedInstance for crate::peripherals::$inst {
            type CommunicationInterrupt = crate::_generated::peripheral_interrupts::$inst::COM;
            type BreakInputInterrupt = crate::_generated::peripheral_interrupts::$inst::BRK;
        }
    };
}

foreach_interrupt! {
    ($inst:ident, timer, BCTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16);
        impl BasicInstance for crate::peripherals::$inst {}
    };

    ($inst:ident, timer, ADTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn enable_outputs(&self) {
                unsafe { crate::pac::timer::Adtm::from_ptr(Self::regs()) }
                    .bdtr()
                    .modify(|w| w.set_moe(true));
            }
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Adtm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst);
        impl_advanced!($inst);
    };
}

// GPTM is 2CH, no CTLR1.CMS and CTLR1.DIR
#[cfg(timer_x0)]
foreach_interrupt! {
    ($inst:ident, timer, GPTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {}
        impl_general_16bit!($inst);
    };
}

#[cfg(not(timer_x0))]
foreach_interrupt! {
    ($inst:ident, timer, GPTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst);
    };

    ($inst:ident, timer, GPTM32, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits32);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst);
        impl GeneralInstance32bit for crate::peripherals::$inst {}
    };
}
