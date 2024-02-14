use fugit::HertzU32 as Hertz;

use crate::gpio::sealed::Pin;
use crate::gpio::AnyPin;
use crate::interrupt;

pub fn debug(p: AnyPin) {
    p.set_as_af_output();
}

pub enum ClockSrc {
    /// AHB clock
    CkInt,
    /// TIMx_ETR, external clock input pin
    ETR,
    /// Internal timer clock
    ITRn,
}

pub(crate) mod sealed {
    use super::*;

    pub trait GeneralPurpose16bitInstance {
        /// Interrupt for this timer.
        type Interrupt: interrupt::Interrupt;

        fn regs() -> &'static crate::pac::tim1::RegisterBlock;

        /// Start the timer.
        fn start(&mut self) {
            Self::regs().ctlr1().modify(|_, w| w.cen().set_bit())
        }

        /// Stop the timer.
        fn stop(&mut self) {
            Self::regs().ctlr1().modify(|_, w| w.cen().clear_bit());
        }

        /// Reset the counter value to 0
        fn reset(&mut self) {
            Self::regs().cnt().write(|w| w.cnt().variant(0));
        }

        /// Set the frequency of how many times per second the timer counts up to the max value or down to 0.
        ///
        /// This means that in the default edge-aligned mode,
        /// the timer counter will wrap around at the same frequency as is being set.
        /// In center-aligned mode (which not all timers support), the wrap-around frequency is effectively halved
        /// because it needs to count up and down.
        fn set_frequency(&mut self, frequency: Hertz) {
            let f = frequency.to_Hz();
            let timer_f = crate::rcc::clocks().hclk.to_Hz();
            assert!(f > 0);
            let pclk_ticks_per_timer_period = timer_f / f;
            let psc: u16 = ((pclk_ticks_per_timer_period - 1) / (1 << 16)).try_into().unwrap();
            let divide_by = pclk_ticks_per_timer_period / (u32::from(psc) + 1);

            // the timer counts `0..=arr`, we want it to count `0..divide_by`
            let arr = u16::try_from(divide_by - 1).unwrap();

            let regs = Self::regs();

            // dir and cms is set later
            // TIM_ClockDivision = Div1
            regs.ctlr1().modify(|_, w| w.ckd().variant(0b00));

            regs.psc().write(|w| w.psc().variant(psc));
            regs.atrlr().write(|w| w.atrlr().variant(arr));

            // UEV 事件的源, 1=只有计数器溢出/下溢才产生更新中断或 DMA 请求
            // COUNTERONLY
            regs.ctlr1().modify(|_, w| w.urs().variant(true));

            // reload immediately
            regs.swevgr().write(|w| w.ug().set_bit());
            // ANYEVENT
            regs.ctlr1().modify(|_, w| w.urs().variant(false));
        }

        /// Clear update interrupt.
        ///
        /// Returns whether the update interrupt flag was set.
        fn clear_update_interrupt(&mut self) -> bool {
            todo!()
        }

        /// Enable/disable the update interrupt.
        fn enable_update_interrupt(&mut self, enable: bool) {
            Self::regs().dmaintenr().modify(|_, w| w.uie().variant(enable));
        }

        /// Enable/disable the update dma.
        fn enable_update_dma(&mut self, enable: bool) {
            Self::regs().dmaintenr().modify(|_, w| w.ude().variant(enable));
        }

        /// Get the update dma enable/disable state.
        fn get_update_dma_state(&self) -> bool {
            Self::regs().dmaintenr().read().ude().bit_is_set()
        }

        /// Enable/disable autoreload preload.
        fn set_autoreload_preload(&mut self, enable: bool) {
            Self::regs().ctlr1().modify(|_, w| w.arpe().variant(enable));
        }

        /// Get the timer frequency.
        fn get_frequency(&self) -> Hertz {
            let timer_f = crate::rcc::clocks().hclk.to_Hz();

            let regs = Self::regs();
            let arr = regs.atrlr().read().atrlr().bits() as u32;
            let psc = regs.psc().read().psc().bits() as u32;

            Hertz::from_raw(timer_f / arr / (psc + 1))
        }

        ///  Set clock divider.
        // 0 => Div1
        // 01 => Div2
        // 10 => Div4
        // 11 => reserved
        fn set_clock_division(&mut self, ckd: u8) {
            Self::regs().ctlr1().modify(|_, w| w.ckd().variant(ckd));
        }
    }
}

/// Timer channel.
#[derive(Clone, Copy)]
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

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CountingMode {
    #[default]
    /// The timer counts up to the reload value and then resets back to 0.
    EdgeAlignedUp,
    /// The timer counts down to 0 and then resets back to the reload value.
    EdgeAlignedDown,
    /// The timer counts up to the reload value and then counts back to 0.
    ///
    /// The output compare interrupt flags of channels configured in output are
    /// set when the counter is counting down.
    CenterAlignedDownInterrupts,
    /// The timer counts up to the reload value and then counts back to 0.
    ///
    /// The output compare interrupt flags of channels configured in output are
    /// set when the counter is counting up.
    CenterAlignedUpInterrupts,
    /// The timer counts up to the reload value and then counts back to 0.
    ///
    /// The output compare interrupt flags of channels configured in output are
    /// set when the counter is counting both up or down.
    CenterAlignedBothInterrupts,
}

impl CountingMode {
    fn to_cms_dir(&self) -> (u8, u8) {
        match self {
            CountingMode::EdgeAlignedUp => (0b00, 0),
            CountingMode::EdgeAlignedDown => (0b00, 1),
            CountingMode::CenterAlignedDownInterrupts => (0b01, 1),
            CountingMode::CenterAlignedUpInterrupts => (0b10, 0),
            CountingMode::CenterAlignedBothInterrupts => (0b11, 0),
        }
    }

    fn from_cms_dir(cms: u8, dir: u8) -> Self {
        match (cms, dir) {
            (0b00, 0) => CountingMode::EdgeAlignedUp,
            (0b00, 1) => CountingMode::EdgeAlignedDown,
            (0b01, 1) => CountingMode::CenterAlignedDownInterrupts,
            (0b10, 0) => CountingMode::CenterAlignedUpInterrupts,
            (0b11, 0) => CountingMode::CenterAlignedBothInterrupts,
            _ => unreachable!(),
        }
    }

    /// Return whether this mode is edge-aligned (up or down).
    pub fn is_edge_aligned(&self) -> bool {
        match self {
            CountingMode::EdgeAlignedUp | CountingMode::EdgeAlignedDown => true,
            _ => false,
        }
    }

    /// Return whether this mode is center-aligned.
    pub fn is_center_aligned(&self) -> bool {
        match self {
            CountingMode::CenterAlignedDownInterrupts
            | CountingMode::CenterAlignedUpInterrupts
            | CountingMode::CenterAlignedBothInterrupts => true,
            _ => false,
        }
    }
}
