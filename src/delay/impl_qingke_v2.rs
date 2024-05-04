//! SYSTICK based delay implementation for Qingke V2

use pac::systick::vals;

use crate::pac;
use crate::pac::SYSTICK;

/// A delay provided by the SysTick core peripheral
///
/// This requires SysTick to be set up and running.
/// Assumes conditions: upcount, hclk.
/// hclk/8 is not accurate enough for ns delays.
pub struct Delay;

static mut P_US: u32 = 0;
static mut P_MS: u32 = 0;

impl Delay {
    /// Init Systick.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it conflicts with embassy's systick time dirver.
    /// Only one of them can be used.
    pub(crate) unsafe fn init() {
        let sysclk = crate::rcc::clocks().hclk.0;

        unsafe {
            P_US = sysclk / 1_000_000;
            P_MS = sysclk / 1_000;
        }
    }

    pub fn delay_us(&mut self, ns: u32) {
        SYSTICK.sr().modify(|w| w.set_cntif(false));

        let i = ns * unsafe { P_US };

        SYSTICK.cmpl().write_value(i);
        SYSTICK.cntl().write_value(0);
        SYSTICK.ctlr().modify(|w| {
            w.set_stclk(vals::Stclk::HCLK);
            w.set_ste(true);
        });

        while SYSTICK.sr().read().cntif() == false {}
        SYSTICK.ctlr().modify(|w| w.set_ste(false));
    }

    #[inline]
    pub fn delay_ms(&mut self, mut ms: u32) {
        // 4294967 is the highest u32 value which you can multiply by 1000 without overflow
        while ms > 4294967 {
            self.delay_us(4294967000u32);
            ms -= 4294967;
        }
        self.delay_us(ms * 1_000);
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        // from the rp2040-hal:
        let us = ns / 1000 + if ns % 1000 == 0 { 0 } else { 1 };
        // With rustc 1.73, this can be replaced by:
        // let us = ns.div_ceil(1000);
        Delay::delay_us(self, us)
    }

    #[inline]
    fn delay_us(&mut self, us: u32) {
        Delay::delay_us(self, us)
    }

    #[inline]
    fn delay_ms(&mut self, ms: u32) {
        Delay::delay_ms(self, ms)
    }
}
