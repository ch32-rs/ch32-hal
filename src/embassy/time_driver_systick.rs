//! SysTick-based time driver.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use pac::systick::vals;
use qingke::interrupt::Priority;
use qingke_rt::interrupt;

use crate::pac;

#[interrupt(core)]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub struct SystickDriver {
    cnt_per_tick: AtomicU32,
    // floor((2^64 - 1) / cnt_per_tick), low word then high word. `now()` divides by multiplying
    // with it: a 64-bit division is a software routine on RV32, and its run time depends on the count.
    reciprocal: [AtomicU32; 2],
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    cnt_per_tick: AtomicU32::new(1), // avoid div by zero
    reciprocal: [AtomicU32::new(u32::MAX), AtomicU32::new(u32::MAX)],
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl SystickDriver {
    fn init(&'static self, _cs: critical_section::CriticalSection) {
        let r = &crate::pac::SYSTICK;
        let hclk = crate::rcc::clocks().hclk.0 as u64;

        let cnt_per_second = hclk / 8; // HCLK/8
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        assert!(cnt_per_tick > 0, "SysTick runs slower than TICK_HZ");
        self.cnt_per_tick.store(cnt_per_tick as u32, Ordering::Relaxed);
        let reciprocal = reciprocal(cnt_per_tick as u32);
        self.reciprocal[0].store(reciprocal as u32, Ordering::Relaxed);
        self.reciprocal[1].store((reciprocal >> 32) as u32, Ordering::Relaxed);

        r.ctlr().write(|w| {
            // Everything else is set to default (0)
            w.set_init(true); // Initialize counter
            w.set_ste(true); // Enable counter
        });

        // Write 0 to both halves of the compare register
        r.cmph().write_value(0);
        r.cmpl().write_value(0);

        // Count value compare flag
        r.sr().write(|w| w.set_cntif(false)); // clear

        // Configration: Upcount, No reload, HCLK/8 as clock source
        r.ctlr().modify(|w| {
            w.set_mode(vals::Mode::UPCOUNT); // Counter mode
            w.set_stre(false); // Auto reload count enable bit
            w.set_stclk(vals::Stclk::HCLK_DIV8); // Counter system clock sourse selection bit
        });
    }

    fn on_interrupt(&self) {
        let r = &crate::pac::SYSTICK;
        // Count value compare flag
        r.sr().write(|w| w.set_cntif(false)); // clear IF

        critical_section::with(|cs| {
            self.trigger_alarm(cs);
        });
    }

    #[inline]
    fn raw_cnt(&self) -> u64 {
        let r = crate::pac::SYSTICK;
        loop {
            let high = r.cnth().read();
            let low = r.cntl().read();
            if r.cnth().read() == high {
                return (u64::from(high) << 32) | u64::from(low);
            }
        }
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.raw_cnt());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.raw_cnt());
        }
    }

    fn set_alarm(&self, _cs: CriticalSection, next_alarm_cnt: u64) -> bool {
        let r = &crate::pac::SYSTICK;

        // TODO move this to schedule_wake
        if next_alarm_cnt <= self.raw_cnt() {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            return false;
        }

        // Counter interrupt enable control bit
        r.cmph().write_value((next_alarm_cnt >> 32) as u32);
        r.cmpl().write_value((next_alarm_cnt) as u32);
        r.ctlr().modify(|w| w.set_stie(true));
        r.sr().write(|w| w.set_cntif(false));

        if next_alarm_cnt <= self.raw_cnt() {
            // If alarm timestamp has passed the alarm will not fire.
            // Disarm the alarm and return `false` to indicate that.
            r.ctlr().modify(|w| w.set_stie(false));
            r.sr().write(|w| w.set_cntif(false));
            return false;
        }

        true
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let cnt_per_tick = self.cnt_per_tick.load(Ordering::Relaxed) as u64;
        let reciprocal = u64::from(self.reciprocal[1].load(Ordering::Relaxed)) << 32
            | u64::from(self.reciprocal[0].load(Ordering::Relaxed));
        let cnt = self.raw_cnt();
        // The reciprocal is rounded down, so this is the quotient or one less.
        let ticks = mul_high(cnt, reciprocal);
        if cnt - ticks * cnt_per_tick >= cnt_per_tick {
            ticks + 1
        } else {
            ticks
        }
    }

    fn schedule_wake(&self, ticks: u64, waker: &core::task::Waker) {
        let cnt_per_tick = self.cnt_per_tick.load(Ordering::Relaxed) as u64;
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(ticks * cnt_per_tick, waker) {
                let mut next = queue.next_expiration(self.raw_cnt());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.raw_cnt());
                }
            }
        })
    }
}

/// floor((2^64 - 1) / divisor), by long division: once, at init, without the 64-bit division routine.
fn reciprocal(divisor: u32) -> u64 {
    let divisor = u64::from(divisor);
    let (mut quotient, mut remainder) = (0u64, 0u64);
    for _ in 0..64 {
        remainder = remainder << 1 | 1;
        quotient <<= 1;
        if remainder >= divisor {
            remainder -= divisor;
            quotient |= 1;
        }
    }
    quotient
}

/// The high 64 bits of the 128-bit product, from 32-bit multiplications.
fn mul_high(a: u64, b: u64) -> u64 {
    let (a1, a0) = (a >> 32, a & 0xffff_ffff);
    let (b1, b0) = (b >> 32, b & 0xffff_ffff);
    let middle = ((a0 * b0) >> 32) + ((a1 * b0) & 0xffff_ffff) + ((a0 * b1) & 0xffff_ffff);
    a1 * b1 + ((a1 * b0) >> 32) + ((a0 * b1) >> 32) + (middle >> 32)
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs);
    use qingke_rt::CoreInterrupt;

    // enable interrupt
    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
