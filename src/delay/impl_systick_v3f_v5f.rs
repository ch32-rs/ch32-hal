//! SYSTICK polling delay for CH32H4 (QingKe V3F + V5F dual-core).
//!
//! H4 has two independent 32-bit counters, indexed by hart: `CTLR(0)` / `CNT(0)`
//! / `CMP(0)` / `SR.CNTIF0` on hart 0 (V3F), and index 1 on hart 1 (V5F). `SR` is
//! shared, so a flag clear only touches the calling hart's bit.
//!
//! RM §4.6 clocks both counters from the V3F HCLK domain, not from each hart's
//! own core clock. The delay therefore always scales from `rcc::clocks().v3f`,
//! read on every call so it follows `rcc::init` / `rcc::refresh`. The cache is
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

    /// The calling hart's counter index, and the V3F HCLK both counters run from.
    #[inline(always)]
    fn current_hart() -> (usize, u32) {
        let index = match HartId::current() {
            HartId::C0 => 0,
            HartId::C1 => 1,
        };
        (index, crate::rcc::clocks().v3f.0)
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

    /// Busy-wait `cycles` V3F HCLK clocks on the calling hart's SysTick counter.
    /// Longer waits are split into chunks that fit the 32-bit `CMP` register.
    #[inline(always)]
    fn wait_cycles(index: usize, mut cycles: u64) {
        while cycles > 0 {
            let chunk = cycles.min(u32::MAX as u64) as u32;
            Self::wait_chunk(index, chunk);
            cycles -= chunk as u64;
        }
    }

    #[inline(always)]
    fn wait_chunk(index: usize, cycles: u32) {
        // `SR` is shared. Clear only this counter's flag; writing the whole
        // register would drop the other hart's pending compare.
        SYSTICK.sr().modify(|w| {
            if index == 0 {
                w.set_cntif0(false);
            } else {
                w.set_cntif1(false);
            }
        });
        SYSTICK.cmp(index).write_value(cycles);
        SYSTICK.cnt(index).write_value(0);
        SYSTICK.ctlr(index).modify(|w| {
            w.set_stclk(vals::Stclk::HCLK);
            w.set_mode(vals::Mode::UPCOUNT);
            w.set_ste(true);
        });
        while {
            let sr = SYSTICK.sr().read();
            if index == 0 {
                !sr.cntif0()
            } else {
                !sr.cntif1()
            }
        } {}
        SYSTICK.ctlr(index).modify(|w| w.set_ste(false));
    }

    #[inline(always)]
    pub fn delay_us(&mut self, us: u32) {
        let (index, hz) = Self::current_hart();
        Self::wait_cycles(index, Self::cycles(hz, us, 1_000_000));
    }

    #[inline]
    pub fn delay_ms(&mut self, ms: u32) {
        let (index, hz) = Self::current_hart();
        Self::wait_cycles(index, Self::cycles(hz, ms, 1_000));
    }
}

impl embedded_hal::delay::DelayNs for Delay {
    #[inline]
    fn delay_ns(&mut self, ns: u32) {
        let (index, hz) = Delay::current_hart();
        Delay::wait_cycles(index, Delay::cycles(hz, ns, 1_000_000_000))
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
