//! SYSTICK polling delay for CH32H4 (QingKe V3F + V5F dual-core).
//!
//! H4 ships two independent 32-bit Systick counters — `CTLR_0` / `CNT_0`
//! / `CMP_0` / `ISR.ISR0` route to hart 0 (V3F); `CTLR_1` / `CNT_1` /
//! `CMP_1` / `ISR.ISR1` route to hart 1 (V5F). We hardcode counter 0
//! here; if running on V5F, instantiate `Delay` from V5F-side code and
//! it'll still hit counter 0 unless the user explicitly wants counter
//! 1 (TODO: parameterise by `qingke::pfic::HartId` once both harts
//! actually run different code).

use pac::systick::vals;

use crate::pac;
use crate::pac::SYSTICK;

pub struct Delay;

static mut P_US: u32 = 0;
static mut P_MS: u32 = 0;

impl Delay {
    /// # Safety
    /// Conflicts with embassy's systick time driver — pick one.
    pub(crate) unsafe fn init() {
        let sysclk = crate::rcc::clocks().hclk.0;
        unsafe {
            P_US = sysclk / 1_000_000;
            P_MS = sysclk / 1_000;
        }
    }

    pub fn delay_us(&mut self, us: u32) {
        // Clear pending interrupt flag for counter 0 (write-1-to-clear isn't
        // used here; the H4 ISR field is plain RW so write `false` to clear).
        SYSTICK.isr().modify(|w| w.set_isr0(false));

        let cycles = us * unsafe { P_US };

        SYSTICK.cmp_0().write(|w| w.set_cmp(cycles));
        SYSTICK.cnt_0().write(|w| w.set_cnt(0));
        SYSTICK.ctlr_0().modify(|w| {
            w.set_no_rtc(vals::Stclk::HCLK);
            w.set_down_mode(vals::Mode::UPCOUNT);
            w.set_en(true);
        });

        while !SYSTICK.isr().read().isr0() {}
        SYSTICK.ctlr_0().modify(|w| w.set_en(false));
    }

    #[inline]
    pub fn delay_ms(&mut self, mut ms: u32) {
        // 4294967 is the highest u32 value that can be multiplied by 1000
        // without overflow.
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
        let us = ns / 1000 + if ns % 1000 == 0 { 0 } else { 1 };
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
