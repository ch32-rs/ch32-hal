//! Time driver implementation for the TIM peripheral.
//!
//! Register renaming:
//!
//! - dmaintenr -> dier
//! - chcvr -> ccr

#![allow(non_snake_case)]

use core::cell::Cell;
use core::sync::atomic::{compiler_fence, AtomicU32, AtomicU8, Ordering};
use core::{mem, ptr};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::{AlarmHandle, Driver, TICK_HZ};
use qingke_rt::interrupt;

use crate::interrupt::typelevel::Interrupt;
use crate::pac::timer::{regs, vals, Gptm};
use crate::peripheral::SealedRccPeripheral;
use crate::peripherals;
// for ::regs()
use crate::timer::{CoreInstance, GeneralInstance16bit};

const ALARM_COUNT: usize = 1;

#[cfg(time_driver_tim1)]
type T = peripherals::TIM1;
#[cfg(time_driver_tim2)]
type T = peripherals::TIM2;
#[cfg(time_driver_tim3)]
type T = peripherals::TIM3;
#[cfg(time_driver_tim4)]
type T = peripherals::TIM4;
#[cfg(time_driver_tim5)]
type T = peripherals::TIM5;
#[cfg(time_driver_tim8)]
type T = peripherals::TIM8;
#[cfg(time_driver_tim9)]
type T = peripherals::TIM9;
#[cfg(time_driver_tim10)]
type T = peripherals::TIM10;

foreach_interrupt! {
    (TIM1, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim1)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn $irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM2, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim2)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn $irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM3, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim3)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn $irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM4, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim4)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn $irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM5, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim5)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn$irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM8, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim8)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn$irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM9, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim9)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn$irq() {
            DRIVER.on_interrupt()
        }
    };
    (TIM10, timer, $block:ident, CC, $irq:ident) => {
        #[cfg(time_driver_tim10)]
        #[cfg(feature = "rt")]
        #[interrupt]
        fn$irq() {
            DRIVER.on_interrupt()
        }
    };

}

fn regs_gp16() -> Gptm {
    unsafe { Gptm::from_ptr(T::regs()) }
}

// Clock timekeeping works with something we call "periods", which are time intervals
// of 2^15 ticks. The Clock counter value is 16 bits, so one "overflow cycle" is 2 periods.
//
// A `period` count is maintained in parallel to the Timer hardware `counter`, like this:
// - `period` and `counter` start at 0
// - `period` is incremented on overflow (at counter value 0)
// - `period` is incremented "midway" between overflows (at counter value 0x8000)
//
// Therefore, when `period` is even, counter is in 0..0x7FFF. When odd, counter is in 0x8000..0xFFFF
// This allows for now() to return the correct value even if it races an overflow.
//
// To get `now()`, `period` is read first, then `counter` is read. If the counter value matches
// the expected range for the `period` parity, we're done. If it doesn't, this means that
// a new period start has raced us between reading `period` and `counter`, so we assume the `counter` value
// corresponds to the next period.
//
// `period` is a 32bit integer, so It overflows on 2^32 * 2^15 / 32768 seconds of uptime, which is 136 years.
fn calc_now(period: u32, counter: u16) -> u64 {
    ((period as u64) << 15) + ((counter as u32 ^ ((period & 1) << 15)) as u64)
}

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

pub(crate) struct RtcDriver {
    /// Number of 2^15 periods elapsed since boot.
    period: AtomicU32,
    alarm_count: AtomicU8,
    /// Timestamp at which to fire alarm. u64::MAX if no alarm is scheduled.
    alarms: Mutex<CriticalSectionRawMutex, [AlarmState; ALARM_COUNT]>,
}

#[allow(clippy::declare_interior_mutable_const)]
const ALARM_STATE_NEW: AlarmState = AlarmState::new();

embassy_time_driver::time_driver_impl!(static DRIVER: RtcDriver = RtcDriver {
    period: AtomicU32::new(0),
    alarm_count: AtomicU8::new(0),
    alarms: Mutex::const_new(CriticalSectionRawMutex::new(), [ALARM_STATE_NEW; ALARM_COUNT]),
});

impl RtcDriver {
    fn init(&'static self, cs: critical_section::CriticalSection) {
        let r = regs_gp16();

        <T as SealedRccPeripheral>::enable_and_reset_with_cs(cs);

        let timer_freq = T::frequency();

        r.ctlr1().modify(|w| w.set_cen(false));
        r.cnt().write_value(0);

        let psc = timer_freq.0 / TICK_HZ as u32 - 1;
        let psc: u16 = match psc.try_into() {
            Err(_) => panic!("psc division overflow: {}", psc),
            Ok(n) => n,
        };

        r.psc().write_value(psc);
        r.atrlr().write_value(u16::MAX);

        // Set URS, generate update and clear URS
        r.ctlr1().modify(|w| w.set_urs(vals::Urs::COUNTERONLY));
        r.swevgr().write(|w| w.set_ug(true));
        r.ctlr1().modify(|w| w.set_urs(vals::Urs::ANYEVENT));

        // Mid-way point. CC1
        r.chcvr(0).write_value(0x8000);

        // Enable overflow and half-overflow interrupts
        r.dmaintenr().write(|w| {
            w.set_uie(true);
            w.set_ccie(0, true);
        });

        <T as GeneralInstance16bit>::CaptureCompareInterrupt::unpend();
        unsafe { <T as GeneralInstance16bit>::CaptureCompareInterrupt::enable() };

        r.ctlr1().modify(|w| w.set_cen(true));
    }

    fn on_interrupt(&self) {
        let r = regs_gp16();

        // XXX: reduce the size of this critical section ?
        critical_section::with(|cs| {
            let sr = r.intfr().read();
            let dier = r.dmaintenr().read();

            // Clear all interrupt flags. Bits in SR are "write 0 to clear", so write the bitwise NOT.
            // Other approaches such as writing all zeros, or RMWing won't work, they can
            // miss interrupts.
            r.intfr().write_value(regs::Intfr(!sr.0));

            // Overflow
            if sr.uif() {
                self.next_period();
            }

            // Half overflow
            if sr.ccif(0) {
                self.next_period();
            }

            for n in 0..ALARM_COUNT {
                if sr.ccif(n + 1) && dier.ccie(n + 1) {
                    self.trigger_alarm(n, cs);
                }
            }
        })
    }

    fn next_period(&self) {
        let r = regs_gp16();

        // We only modify the period from the timer interrupt, so we know this can't race.
        let period = self.period.load(Ordering::Relaxed) + 1;
        self.period.store(period, Ordering::Relaxed);
        let t = (period as u64) << 15;

        critical_section::with(move |cs| {
            r.dmaintenr().modify(move |w| {
                for n in 0..ALARM_COUNT {
                    let alarm = &self.alarms.borrow(cs)[n];
                    let at = alarm.timestamp.get();

                    if at < t + 0xc000 {
                        // just enable it. `set_alarm` has already set the correct CCR val.
                        w.set_ccie(n + 1, true);
                    }
                }
            })
        })
    }

    fn get_alarm<'a>(&'a self, cs: CriticalSection<'a>, alarm: AlarmHandle) -> &'a AlarmState {
        // safety: we're allowed to assume the AlarmState is created by us, and
        // we never create one that's out of bounds.
        unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) }
    }

    fn trigger_alarm(&self, n: usize, cs: CriticalSection) {
        let alarm = &self.alarms.borrow(cs)[n];
        alarm.timestamp.set(u64::MAX);

        // Call after clearing alarm, so the callback can set another alarm.

        // safety:
        // - we can ignore the possibility of `f` being unset (null) because of the safety contract of `allocate_alarm`.
        // - other than that we only store valid function pointers into alarm.callback
        let f: fn(*mut ()) = unsafe { mem::transmute(alarm.callback.get()) };
        f(alarm.ctx.get());
    }
}

impl Driver for RtcDriver {
    fn now(&self) -> u64 {
        let r = regs_gp16();

        let period = self.period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = r.cnt().read();
        calc_now(period, counter)
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        critical_section::with(|_| {
            let id = self.alarm_count.load(Ordering::Relaxed);
            if id < ALARM_COUNT as u8 {
                self.alarm_count.store(id + 1, Ordering::Relaxed);
                Some(AlarmHandle::new(id))
            } else {
                None
            }
        })
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
            let r = regs_gp16();

            let n = alarm.id() as usize;
            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);

            let t = self.now();
            if timestamp <= t {
                // If alarm timestamp has passed the alarm will not fire.
                // Disarm the alarm and return `false` to indicate that.
                r.dmaintenr().modify(|w| w.set_ccie(n + 1, false));

                alarm.timestamp.set(u64::MAX);

                return false;
            }

            // Write the CCR value regardless of whether we're going to enable it now or not.
            // This way, when we enable it later, the right value is already set.
            r.chcvr(n + 1).write_value(timestamp as u16);

            // Enable it if it'll happen soon. Otherwise, `next_period` will enable it.
            let diff = timestamp - t;
            r.dmaintenr().modify(|w| w.set_ccie(n + 1, diff < 0xc000));

            // Reevaluate if the alarm timestamp is still in the future
            let t = self.now();
            if timestamp <= t {
                // If alarm timestamp has passed since we set it, we have a race condition and
                // the alarm may or may not have fired.
                // Disarm the alarm and return `false` to indicate that.
                // It is the caller's responsibility to handle this ambiguity.
                r.dmaintenr().modify(|w| w.set_ccie(n + 1, false));

                alarm.timestamp.set(u64::MAX);

                return false;
            }

            // We're confident the alarm will ring in the future.
            true
        })
    }
}

pub(crate) fn init(cs: CriticalSection) {
    DRIVER.init(cs)
}
