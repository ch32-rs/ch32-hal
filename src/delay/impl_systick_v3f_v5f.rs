//! SYSTICK polling delay for CH32H4 (QingKe V3F + V5F dual-core).
//!
//! H4 ships two independent 32-bit SysTick counters: `CTLR_0` / `CNT_0` /
//! `CMP_0` / `ISR.ISR0` run on hart 0 (V3F), `CTLR_1` / `CNT_1` / `CMP_1` /
//! `ISR.ISR1` run on hart 1 (V5F). Each counter ticks at its own hart's core
//! clock, so the cycles per microsecond depend on which hart calls the delay:
//!
//! - hart 0 (V3F): `rcc::clocks().v3f`
//! - hart 1 (V5F): `rcc::clocks().v5f`
//!
//! Both values are read from the `rcc::clocks()` cache on every call, so the
//! delay follows `rcc::init` / `rcc::refresh` and is never stale. The cache is
//! per image: hart 1's image must call `rcc::refresh(hse)` once before using
//! this delay. That only reads RCC registers and does not reprogram the PLL.

use pac::systick::vals;
use qingke::pfic::HartId;

use crate::pac;
use crate::pac::SYSTICK;

pub struct Delay;

impl Delay {
    /// Nothing to cache: the frequency is read from `rcc::clocks()` per call.
    ///
    /// # Safety
    /// Conflicts with embassy's systick time driver — pick one.
    pub(crate) unsafe fn init() {}

    /// Whether the calling hart is hart 1, and its core clock in Hz.
    #[inline(always)]
    fn current_hart() -> (bool, u32) {
        let clocks = crate::rcc::clocks();
        match HartId::current() {
            HartId::C0 => (false, clocks.v3f.0),
            HartId::C1 => (true, clocks.v5f.0),
        }
    }

    /// Core cycles for `n` units of `1 / unit` seconds at `hz`, rounded up.
    ///
    /// Split as `n * (hz / unit) + ceil(n * (hz % unit) / unit)` so the
    /// programmed wait is never shorter than requested. Every value on this
    /// path is a 32-bit constant division or a 64-bit multiply; the only 64-bit
    /// divide is taken when `hz` is not a whole number of `unit`s. A software
    /// 64-bit divide on every call costs microseconds on the uncached-flash V3F.
    #[inline(always)]
    fn cycles(hz: u32, n: u32, unit: u32) -> u64 {
        let whole = (hz / unit) as u64;
        let rem = (hz % unit) as u64;
        let mut cycles = n as u64 * whole;
        if rem != 0 {
            cycles += (n as u64 * rem).div_ceil(unit as u64);
        }
        cycles
    }

    /// Busy-wait `cycles` core clocks on the calling hart's SysTick counter.
    /// Longer waits are split into chunks that fit the 32-bit `CMP` register.
    #[inline(always)]
    fn wait_cycles(on_v5f: bool, mut cycles: u64) {
        while cycles > 0 {
            let chunk = cycles.min(u32::MAX as u64) as u32;
            Self::wait_chunk(on_v5f, chunk);
            cycles -= chunk as u64;
        }
    }

    #[inline(always)]
    fn wait_chunk(on_v5f: bool, cycles: u32) {
        if on_v5f {
            SYSTICK.isr().modify(|w| w.set_isr1(false));
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
            SYSTICK.isr().modify(|w| w.set_isr0(false));
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

    #[inline(always)]
    pub fn delay_us(&mut self, us: u32) {
        let (on_v5f, hz) = Self::current_hart();
        Self::wait_cycles(on_v5f, Self::cycles(hz, us, 1_000_000));
    }

    #[inline]
    pub fn delay_ms(&mut self, ms: u32) {
        let (on_v5f, hz) = Self::current_hart();
        Self::wait_cycles(on_v5f, Self::cycles(hz, ms, 1_000));
    }
}
