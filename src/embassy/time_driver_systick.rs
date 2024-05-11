//! SysTick-based time driver.

use core::arch::asm;
use core::cell::Cell;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};

use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{AlarmHandle, Driver};
use pac::systick::vals;
use qingke::interrupt::Priority;
#[cfg(feature = "highcode")]
use qingke_rt::highcode;
use qingke_rt::interrupt;

use crate::{pac, println};

pub const ALARM_COUNT: usize = 1;

struct AlarmState {
    timestamp: Cell<u64>,

    // This is really a Option<(fn(*mut ()), *mut ())>
    // but fn pointers aren't allowed in const yet
    callback: Cell<*const ()>,
    ctx: Cell<*mut ()>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            callback: Cell::new(ptr::null()),
            ctx: Cell::new(ptr::null_mut()),
        }
    }
}

pub struct SystickDriver {
    alarm_count: AtomicU8,
    alarms: Mutex<[AlarmState; ALARM_COUNT]>,
    period: AtomicU32,
}

const ALARM_STATE_NEW: AlarmState = AlarmState::new();
embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    period: AtomicU32::new(1), // avoid div by zero
    alarm_count: AtomicU8::new(0),
    alarms: Mutex::new([ALARM_STATE_NEW; ALARM_COUNT]),
});

impl SystickDriver {
    fn init(&'static self) {
        let rb = &crate::pac::SYSTICK;
        let hclk = crate::rcc::clocks().hclk.0 as u64;

        let cnt_per_second = hclk / 8; // HCLK/8
        let cnt_per_tick = cnt_per_second / embassy_time_driver::TICK_HZ;

        self.period.store(cnt_per_tick as u32, Ordering::Relaxed);

        // UNDOCUMENTED:  Avoid initial interrupt
        rb.cmp().write(|w| *w = u64::MAX - 1);
        rb.cmp().write_value(0);
        critical_section::with(|_| {
            rb.sr().write(|w| w.set_cntif(false)); // clear

            // Configration: Upcount, No reload, HCLK as clock source
            rb.ctlr().modify(|w| {
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
        let rb = &crate::pac::SYSTICK;
        rb.sr().write(|w| w.set_cntif(false)); // clear IF

        let period = self.period.load(Ordering::Relaxed) as u64;

        let next_timestamp = critical_section::with(|cs| {
            let next = self.alarms.borrow(cs)[0].timestamp.get();
            if next > self.now() + 1 {
                return next;
            }
            self.trigger_alarm(cs);
            return u64::MAX;
        });

        let new_cmp = u64::min(next_timestamp * period, self.raw_cnt().wrapping_add(period));
        rb.cmp().write_value(new_cmp + 1);
    }

    fn trigger_alarm(&self, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[0];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.

        // safety:
        // - we can ignore the possiblity of `f` being unset (null) because of the safety contract of `allocate_alarm`.
        // - other than that we only store valid function pointers into alarm.callback
        let f: fn(*mut ()) = unsafe { mem::transmute(alarm.callback.get()) };
        f(alarm.ctx.get());
    }

    fn get_alarm<'a>(&'a self, cs: CriticalSection<'a>, alarm: AlarmHandle) -> &'a AlarmState {
        // safety: we're allowed to assume the AlarmState is created by us, and
        // we never create one that's out of bounds.
        unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) }
    }

    #[inline]
    fn raw_cnt(&self) -> u64 {
        let rb = crate::pac::SYSTICK;
        rb.cnt().read()
    }
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        let rb = crate::pac::SYSTICK;
        let period = self.period.load(Ordering::Relaxed) as u64;
        rb.cnt().read() / period
    }
    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        let id = self.alarm_count.fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
            if x < ALARM_COUNT as u8 {
                Some(x + 1)
            } else {
                None
            }
        });

        match id {
            Ok(id) => Some(AlarmHandle::new(id)),
            Err(_) => None,
        }
    }
    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);

            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }
    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let rb = &crate::pac::SYSTICK;

            let _n = alarm.id();

            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);

            let period = self.period.load(Ordering::Relaxed) as u64;

            // See-also: https://github.com/ch32-rs/ch32-hal/issues/4
            let t = self.raw_cnt();
            let timestamp = timestamp * period;
            if timestamp <= t {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.

                alarm.timestamp.set(u64::MAX);

                return false;
            }

            rb.ctlr().modify(|w| w.set_stie(true));

            true
        })
    }
}

#[interrupt(core)]
fn SysTick() {
    DRIVER.on_interrupt();
}

pub(crate) fn init() {
    DRIVER.init();
    use qingke_rt::CoreInterrupt;

    // enable interrupt
    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
