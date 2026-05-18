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
use crate::interrupt;

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
    #[cfg(any(ch32l1, ch32v208, ch32h4))]
    Bits32,
}

/// Core timer instance.
pub trait CoreInstance: RccPeripheral + embassy_hal_internal::PeripheralType + 'static {
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

pin_trait!(Channel1Pin, GeneralInstance16bit, @A);
pin_trait!(Channel2Pin, GeneralInstance16bit, @A);
pin_trait!(Channel3Pin, GeneralInstance16bit, @A);
pin_trait!(Channel4Pin, GeneralInstance16bit, @A);
pin_trait!(ExternalTriggerPin, GeneralInstance16bit, @A);

pin_trait!(Channel1ComplementaryPin, AdvancedInstance, @A);
pin_trait!(Channel2ComplementaryPin, AdvancedInstance, @A);
pin_trait!(Channel3ComplementaryPin, AdvancedInstance, @A);
// No Channel4ComplementaryPin for ADTM
// pin_trait!(Channel4ComplementaryPin, AdvancedInstance, @A);

pin_trait!(BreakInputPin, AdvancedInstance, @A);

// Update Event trigger DMA for every timer
dma_trait!(UpDma, BasicInstance);

dma_trait!(Ch1Dma, GeneralInstance16bit);
dma_trait!(Ch2Dma, GeneralInstance16bit);
dma_trait!(Ch3Dma, GeneralInstance16bit);
dma_trait!(Ch4Dma, GeneralInstance16bit);

// Each macro takes the interrupt-signal *names* (UP / CC / TRG / COM /
// BRK / GLOBAL / TRG_COM) as extra idents so the same body works for
// V0/V1/V2/V3/X0 (per-event signals) and H4 (merged GLOBAL or TRG_COM
// signals). The calling `foreach_interrupt!` arm already knows which
// signal each TIM provides, so it just forwards the right name.

#[allow(unused)]
macro_rules! impl_core_timer {
    ($inst:ident, $bits:expr, $update:ident) => {
        impl CoreInstance for crate::peripherals::$inst {
            type UpdateInterrupt = crate::_generated::peripheral_interrupts::$inst::$update;

            const BITS: TimerBits = $bits;

            fn regs() -> *mut () {
                crate::pac::$inst.as_ptr()
            }
        }
    };
}

#[allow(unused)]
macro_rules! impl_general_16bit {
    ($inst:ident, $cc:ident, $trg:ident) => {
        impl GeneralInstance16bit for crate::peripherals::$inst {
            type CaptureCompareInterrupt = crate::_generated::peripheral_interrupts::$inst::$cc;
            type TriggerInterrupt = crate::_generated::peripheral_interrupts::$inst::$trg;
        }
    };
}

#[allow(unused)]
macro_rules! impl_advanced {
    ($inst:ident, $com:ident, $brk:ident) => {
        impl AdvancedInstance for crate::peripherals::$inst {
            type CommunicationInterrupt = crate::_generated::peripheral_interrupts::$inst::$com;
            type BreakInputInterrupt = crate::_generated::peripheral_interrupts::$inst::$brk;
        }
    };
}

// H4 collapses GPTM / GPTM32 / BCTM IRQs into a single `GLOBAL` signal
// (no separate UP / CC / TRG entries in metadata); earlier families
// keep them as `UP`. ADTM still uses `UP` on both — it has separate
// BRK/UP/TRG_COM/CC vectors on H4 too. Below, the BCTM/GPTM/GPTM32
// arms are duplicated under cfg gates because `foreach_interrupt!`
// doesn't accept per-arm attributes.

#[cfg(not(timer_h4))]
foreach_interrupt! {
    ($inst:ident, timer, BCTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, UP);
        impl BasicInstance for crate::peripherals::$inst {}
    };

    ($inst:ident, timer, ADTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, UP);
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
        impl_general_16bit!($inst, CC, TRG);
        impl_advanced!($inst, COM, BRK);
    };
}

// On H4, ADTM still has separate BRK/UP/TRG_COM/CC vectors (V3-style).
#[cfg(timer_h4)]
foreach_interrupt! {
    ($inst:ident, timer, BCTM, GLOBAL, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, GLOBAL);
        impl BasicInstance for crate::peripherals::$inst {}
    };

    ($inst:ident, timer, ADTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, UP);
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
        // H4 ADTM: CC stays separate, TRG and COM are merged into TRG_COM.
        impl_general_16bit!($inst, CC, TRG_COM);
        impl_advanced!($inst, TRG_COM, BRK);
    };
}

// GPTM is 2CH, no CTLR1.CMS and CTLR1.DIR
#[cfg(timer_x0)]
foreach_interrupt! {
    ($inst:ident, timer, GPTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, UP);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {}
        impl_general_16bit!($inst, CC, TRG);
    };
}

#[cfg(not(any(timer_x0, timer_h4)))]
foreach_interrupt! {
    ($inst:ident, timer, GPTM, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, UP);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst, CC, TRG);
    };

    ($inst:ident, timer, GPTM32, UP, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits32, UP);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst, CC, TRG);
        impl GeneralInstance32bit for crate::peripherals::$inst {}
    };
}

#[cfg(timer_h4)]
foreach_interrupt! {
    ($inst:ident, timer, GPTM, GLOBAL, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits16, GLOBAL);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst, GLOBAL, GLOBAL);
    };

    ($inst:ident, timer, GPTM32, GLOBAL, $irq:ident) => {
        impl_core_timer!($inst, TimerBits::Bits32, GLOBAL);
        impl BasicInstance for crate::peripherals::$inst {}
        impl SealedGeneralInstance for crate::peripherals::$inst {
            fn get_counting_mode(&self) -> low_level::CountingMode {
                let regs = unsafe { crate::pac::timer::Gptm::from_ptr(Self::regs()) };
                let cr1 = regs.ctlr1().read();
                (cr1.cms(), cr1.dir()).into()
            }
        }
        impl_general_16bit!($inst, GLOBAL, GLOBAL);
        impl GeneralInstance32bit for crate::peripherals::$inst {}
    };
}
