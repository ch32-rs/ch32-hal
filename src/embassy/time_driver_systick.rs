//! SysTick-based time driver.

use core::cell::{Cell, RefCell};
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};

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

struct AlarmState {
    timestamp: Cell<u64>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
        }
    }
}

pub struct SystickDriver {
    period: AtomicU32,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarm: Mutex<CriticalSectionRawMutex, AlarmState>,
    queue: Mutex<CriticalSectionRawMutex, RefCell<Queue>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    period: AtomicU32::new(1), // avoid div by zero
    alarm: Mutex::const_new(CriticalSectionRawMutex::new(), AlarmState::new()),
    queue: Mutex::new(RefCell::new(Queue::new()))
});

impl SystickDriver {
    fn init(&'static self, _cs: critical_section::CriticalSection) {
        let r = &crate::pac::SYSTICK;
        let hclk = crate::rcc::clocks().hclk.0 as u64;

        let cnt_per_second = hclk / 8; // HCLK/8
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        self.period.store(cnt_per_tick as u32, Ordering::Relaxed);

        // UNDOCUMENTED:  Avoid initial interrupt
        r.cmp().write(|w| *w = u64::MAX - 1);
        r.cmp().write_value(0);
        critical_section::with(|_| {
            r.sr().write(|w| w.set_cntif(false)); // clear

            // Configration: Upcount, No reload, HCLK as clock source
            r.ctlr().modify(|w| {
                //  w.set_init(true);
                w.set_mode(vals::Mode::UPCOUNT);
                w.set_stre(false);
                w.set_stclk(vals::Stclk::HCLK_DIV8);
                w.set_ste(true);
            });
        })
    }

    #[inline(always)]
    fn on_interrupt(&self) {
        let r = &crate::pac::SYSTICK;
        r.sr().write(|w| w.set_cntif(false)); // clear IF

        let period = self.period.load(Ordering::Relaxed) as u64;

        let next_timestamp = critical_section::with(|cs| {
            let next = self.alarm.borrow(cs).timestamp.get();
            if next > self.now() + 1 {
                return next;
            }
            self.trigger_alarm(cs);
            return u64::MAX;
        });

        let new_cmp = u64::min(next_timestamp * period, self.raw_cnt().wrapping_add(period));
        r.cmp().write_value(new_cmp + 1);
    }

    #[inline]
    fn raw_cnt(&self) -> u64 {
        let r = crate::pac::SYSTICK;
        // Typical implementation of reading 64-bit value from two 32-bit
        // self-incrementing registers: H->L->H loop.
        // See-also: https://github.com/ch32-rs/ch32-hal/issues/4
        loop {
            let cnt_high = r.cnth().read();
            let cnt_low = r.cntl().read();
            if r.cnth().read() == cnt_high {
                return (cnt_high as u64) << 32 | cnt_low as u64;
            }
        }
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let mut next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = self.queue.borrow(cs).borrow_mut().next_expiration(self.now());
        }
    }

    fn set_alarm(&self, cs: CriticalSection, timestamp: u64) -> bool {
        let r = &crate::pac::SYSTICK;

        self.alarm.borrow(cs).timestamp.set(timestamp);

        let period = self.period.load(Ordering::Relaxed) as u64;

            let t = self.raw_cnt();
            let timestamp = timestamp * period;
            if timestamp <= t {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.

            self.alarm.borrow(cs).timestamp.set(u64::MAX);

            return false;
        }

        r.ctlr().modify(|w| w.set_stie(true));

        true
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let r = crate::pac::SYSTICK;
        let period = self.period.load(Ordering::Relaxed) as u64;
        self.raw_cnt() / period
    }

    fn schedule_wake(&self, at: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();

            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
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
