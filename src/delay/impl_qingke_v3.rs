//! SYSTICK based delay implementation for Qingke V3

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
            P_US = sysclk / 8 / 1_000_000;
            P_MS = sysclk / 8 / 1_000;
        }

        SYSTICK.ctlr().modify(|w| w.set_ste(true));
    }

    pub fn delay_us(&mut self, ns: u32) {
        let i = ns * unsafe { P_US };

        let target = SYSTICK.cnt().read().wrapping_add(i as u64);

        while SYSTICK.cnt().read() < target {}
    }

    pub fn delay_ms(&mut self, ms: u32) {
        let i = ms * unsafe { P_MS };

        let target = SYSTICK.cnt().read().wrapping_add(i as u64);

        while SYSTICK.cnt().read() < target {}
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
