//! Blocking SysTick delays for CH32V407/CH32V467.

use crate::pac::systick::vals::{Mode, Stclk};
use crate::pac::SYSTICK;

/// Blocking delay that owns SysTick. Do not use alongside Embassy's time driver.
pub struct Delay;

impl Delay {
    pub(crate) unsafe fn init() {
        SYSTICK.ctlr().write(|w| {
            w.set_stclk(Stclk::HCLK);
            w.set_mode(Mode::UPCOUNT);
        });
        SYSTICK.sr().write(|w| w.set_cntif(false));
    }

    fn delay_ticks(&mut self, mut ticks: u64) {
        while ticks != 0 {
            let chunk = ticks.min(u32::MAX as u64);
            SYSTICK.ctlr().write(|w| {
                w.set_stclk(Stclk::HCLK);
                w.set_mode(Mode::UPCOUNT);
            });
            SYSTICK.cmpl().write_value(chunk as u32 - 1);
            SYSTICK.cntl().write_value(0);
            SYSTICK.sr().write(|w| w.set_cntif(false));
            SYSTICK.ctlr().modify(|w| w.set_ste(true));
            while !SYSTICK.sr().read().cntif() {}
            SYSTICK.ctlr().modify(|w| w.set_ste(false));
            ticks -= chunk;
        }
    }

    pub fn delay_us(&mut self, us: u32) {
        self.delay_ticks(
            (u64::from(us) * u64::from(crate::rcc::clocks().sysclk.0))
                .div_ceil(1_000_000 * u64::from(crate::rcc::hclk_divisor())),
        );
    }

    pub fn delay_ms(&mut self, ms: u32) {
        self.delay_ticks(
            (u64::from(ms) * u64::from(crate::rcc::clocks().sysclk.0))
                .div_ceil(1_000 * u64::from(crate::rcc::hclk_divisor())),
        );
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        self.delay_ticks(
            (u64::from(ns) * u64::from(crate::rcc::clocks().sysclk.0))
                .div_ceil(1_000_000_000 * u64::from(crate::rcc::hclk_divisor())),
        );
    }

    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us);
    }

    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms);
    }
}
