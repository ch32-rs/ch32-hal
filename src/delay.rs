//! Simple busy-loop delay provider

use qingke::riscv;
use qingke_rt::highcode;

use crate::pac;
use crate::rcc::clocks;

/// A delay provided by busy-looping
pub struct CycleDelay;

impl embedded_hal::delay::DelayNs for CycleDelay {
    #[cfg_attr(feature = "highcode", highcode)]
    fn delay_ns(&mut self, ns: u32) {
        let cycles = ns as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }
    #[cfg_attr(feature = "highcode", highcode)]
    fn delay_us(&mut self, us: u32) {
        let cycles = us as u64 * clocks().sysclk.to_Hz() as u64 / 1_500_000;

        unsafe {
            riscv::asm::delay(cycles as u32);
        }
    }

    #[cfg_attr(feature = "highcode", highcode)]
    fn delay_ms(&mut self, ms: u32) {
        let cycles_per_us = clocks().sysclk.to_Hz() as u32 / 1_500_000;

        for _ in 0..ms {
            unsafe {
                riscv::asm::delay(cycles_per_us * 1000);
            }
        }
    }
}

/// A delay provided by the SysTick core peripheral
///
/// This requires SysTick to be set up and running.
/// Assumes conditions: upcount, hclk.
/// hclk/8 is not accurate enough for ns delays.
pub struct SystickDelay;

impl SystickDelay {
    /// Init Systick.
    ///
    /// # Safety
    ///
    /// This function is unsafe because it conflicts with embassy's systick time dirver.
    /// Only one of them can be used.
    pub unsafe fn init() {
        let rb = &*pac::SYSTICK::PTR;
        rb.ctlr().modify(|_, w| {
            w.init()
                .set_bit()
                .mode()
                .upcount()
                .stre()
                .clear_bit()
                .stclk()
                .hclk()
                .ste()
                .set_bit()
        });
    }

    // #[cfg_attr(feature = "highcode", highcode)]
    pub fn delay_ticks(&mut self, n: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };
        let target = rb.cntl().read().bits().wrapping_add(n - 5); // 5 opcodes overhead
        while rb.cntl().read().bits() < target {}
    }
}

impl embedded_hal::delay::DelayNs for SystickDelay {
    fn delay_ns(&mut self, ns: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = ns as u64 * clocks().sysclk.to_Hz() as u64 / 1_000_000_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while rb.cnt().read().bits() < target {}
    }
    fn delay_us(&mut self, us: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = us as u64 * clocks().sysclk.to_Hz() as u64 / 1_000_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while rb.cnt().read().bits() < target {}
    }
    fn delay_ms(&mut self, mut ms: u32) {
        let rb = unsafe { &*pac::SYSTICK::PTR };

        let ticks = ms as u64 * clocks().sysclk.to_Hz() as u64 / 1_000;
        let target = rb.cnt().read().bits().wrapping_add(ticks);

        while ms > 0 {
            while rb.cnt().read().bits() < target {}

            ms -= 1;
        }
    }
}
