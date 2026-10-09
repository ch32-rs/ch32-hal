//! SYSTICK polling delay for CH32H4 (QingKe V3F + V5F dual-core).
//!
//! H4 ships two independent 32-bit Systick counters — `CTLR_0` / `CNT_0`
//! / `CMP_0` / `ISR.ISR0` route to hart 0 (V3F); `CTLR_1` / `CNT_1` /
//! `CMP_1` / `ISR.ISR1` route to hart 1 (V5F). Both harts also have
//! their own core clock (V3F from `FPRE`, V5F from its own prescaler —
//! `rcc::clocks().v3f` / `.v5f`), so the per-tick period here has to come
//! from the *current* hart's frequency, not from the global `hclk`
//! (which is `v3f` on H4). Using `hclk` on the V5F would scale every
//! delay by `v3f/v5f` (4x off on the 400/100 MHz EVT preset).

use pac::systick::vals;
use qingke::pfic::HartId;

use crate::pac;
use crate::pac::SYSTICK;

pub struct Delay;

static mut P_US: u32 = 0;
static mut P_MS: u32 = 0;

impl Delay {
    /// # Safety
    /// Conflicts with embassy's systick time driver — pick one.
    pub(crate) unsafe fn init() {
        let core_hz = match HartId::current() {
            HartId::C0 => crate::rcc::clocks().v3f.0,
            HartId::C1 => crate::rcc::clocks().v5f.0,
        };
        unsafe {
            P_US = core_hz / 1_000_000;
            P_MS = core_hz / 1_000;
        }
    }

    pub fn delay_us(&mut self, us: u32) {
        let hart = HartId::current();
        let on_v5f = matches!(hart, HartId::C1);

        // Clear this counter's pending flag (ISR is plain RW; writing false
        // to one bit leaves the other counter's flag alone).
        SYSTICK
            .isr()
            .modify(|w| if on_v5f { w.set_isr1(false) } else { w.set_isr0(false) });

        let cycles = us * unsafe { P_US };

        if on_v5f {
            SYSTICK.cmp_1().write(|w| w.set_cmp(cycles));
            SYSTICK.cnt_1().write(|w| w.set_cnt(0));
            SYSTICK.ctlr_1().modify(|w| {
                w.set_no_rtc(vals::Stclk::HCLK);
                w.set_down_mode(vals::Mode::UPCOUNT);
                w.set_en(true);
            });
            while !SYSTICK.isr().read().isr1() {}
            SYSTICK.ctlr_1().modify(|w| w.set_en(false));
        } else {
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
