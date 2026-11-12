//! Time driver for the CH32H4 dual-core SysTick, on the boot core.
//!
//! Counter 0 is a free-running 32-bit upcounter clocked from the V3F HCLK
//! domain (RM §4.6), divided by 8.
//!
//! **Only hart 0 can program it.** On CH32H417 the V5F can *read* `CNT0`, but
//! stores from hart 1 to `CTLR0`/`CMP0` are dropped (measured: a raw store
//! leaves the register unchanged, while the same store from hart 0 works).
//! So a wake that hart 1 schedules cannot arm the comparator from hart 1: it
//! only inserts into the shared queue, and hart 0 picks it up on the next
//! re-arm tick. That tick is why the driver keeps a periodic alarm instead of
//! a half-wrap maintenance alarm: nothing else would notice a cross-core
//! insertion. One tick of added latency is the price; once re-armed, the
//! deadline is exact.
//!
//! `SR` is shared by both counters (a plain read-write register), so only
//! `CNTIF0` is cleared, and only on hart 0: clearing it from hart 1 would eat
//! hart 0's pending compare. Counter 1 and `SysTick1` belong to hart 1;
//! `qingke-rt` has no vector for it, so this image never touches them.

use core::cell::RefCell;

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time_driver::Driver;
use embassy_time_queue_utils::Queue;
use qingke::interrupt::Priority;
use qingke_rt::{interrupt, CoreInterrupt};

use crate::pac::systick::vals;
use crate::pac::SYSTICK;

#[path = "systick_time_scale.rs"]
mod time_scale;

/// Counter 0, the V3F (hart 0) counter.
const HART: usize = 0;

/// How often hart 0 re-examines the queue when nothing is about to expire, in
/// counter ticks (set from the clock in [`init`]). Bounds the latency of a
/// wake inserted by hart 1, which cannot arm counter 0 itself.
const REARM_INTERVAL_US: u32 = 1_000;

struct State {
    last_low: u32,
    epoch: u64,
    scale: time_scale::TimeScale,
    queue: Queue,
    /// [`REARM_INTERVAL_US`] converted to counter ticks.
    rearm: u64,
}

impl State {
    fn raw_count(&mut self) -> u64 {
        let low = SYSTICK.cnt(HART).read();
        if low < self.last_low {
            self.epoch += 1 << 32;
        }
        self.last_low = low;
        self.epoch | u64::from(low)
    }

    fn set_alarm(&mut self, deadline: u64) -> bool {
        SYSTICK.ctlr(HART).modify(|w| w.set_stie(false));
        SYSTICK.cmp(HART).write_value(deadline as u32);
        // Shared with counter 1. Clearing `CNTIF0` leaves `CNTIF1` alone.
        SYSTICK.sr().modify(|w| w.set_cntif0(false));
        SYSTICK.ctlr(HART).modify(|w| w.set_stie(true));
        if deadline <= self.raw_count() {
            SYSTICK.ctlr(HART).modify(|w| w.set_stie(false));
            SYSTICK.sr().modify(|w| w.set_cntif0(false));
            return false;
        }
        true
    }

    fn schedule_next(&mut self) {
        loop {
            let now = self.raw_count();
            let expiration = self.queue.next_expiration(now);
            let deadline = expiration
                .min(now.saturating_add(self.rearm.max(1)))
                .min(now | u64::from(u32::MAX));
            if self.set_alarm(deadline) {
                return;
            }
        }
    }
}

pub struct SystickDriver {
    state: Mutex<CriticalSectionRawMutex, RefCell<State>>,
}

embassy_time_driver::time_driver_impl!(static DRIVER: SystickDriver = SystickDriver {
    state: Mutex::new(RefCell::new(State {
        last_low: 0,
        epoch: 0,
        scale: time_scale::TimeScale::new(1, 1),
        queue: Queue::new(),
        rearm: 1,
    }))
});

#[interrupt(core)]
fn SysTick() {
    SYSTICK.sr().modify(|w| w.set_cntif0(false));
    critical_section::with(|cs| DRIVER.state.borrow(cs).borrow_mut().schedule_next());
}

impl Driver for SystickDriver {
    fn now(&self) -> u64 {
        critical_section::with(|cs| {
            let mut state = self.state.borrow(cs).borrow_mut();
            let count = state.raw_count();
            state.scale.ticks_from_counter(count)
        })
    }

    fn schedule_wake(&self, ticks: u64, waker: &core::task::Waker) {
        critical_section::with(|cs| {
            let mut state = self.state.borrow(cs).borrow_mut();
            let deadline = state.scale.counter_from_ticks(ticks);
            if state.queue.schedule_wake(deadline, waker) {
                // Counter 0 is hart 0's: the V5F's stores to `CTLR0`/`CMP0`
                // are dropped. Hart 1 only queues; the next re-arm tick on
                // hart 0 programs the comparator. Touching `SR`/`CTLR0` from
                // here on hart 1 would also clear hart 0's pending compare.
                if qingke::pfic::HartId::current() == qingke::pfic::HartId::C0 {
                    state.schedule_next();
                }
            }
        });
    }
}

pub(crate) fn init(cs: CriticalSection) {
    let mut state = DRIVER.state.borrow(cs).borrow_mut();
    // Both counters run from the V3F HCLK domain, divided by 8 here.
    let counter_hz = crate::rcc::clocks().v3f.0 / 8;
    state.scale = time_scale::TimeScale::new(crate::rcc::clocks().v3f.0, 8);
    state.rearm = (u64::from(counter_hz) * u64::from(REARM_INTERVAL_US) / 1_000_000).max(1);
    SYSTICK.ctlr(HART).write(|w| {
        w.set_mode(vals::Mode::UPCOUNT);
        w.set_stclk(vals::Stclk::HCLK_DIV8);
    });
    SYSTICK.cnt(HART).write_value(0);
    SYSTICK.cmp(HART).write_value(state.rearm as u32);
    SYSTICK.sr().modify(|w| w.set_cntif0(false));
    SYSTICK.ctlr(HART).modify(|w| {
        w.set_ste(true);
        w.set_stie(true);
    });
    unsafe {
        // Same priority as every peripheral interrupt, so the default
        // configuration never nests. `qingke` 0.8.4 restores `mstatus.MPIE`
        // from a critical section, but nothing asks for nesting by default.
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P0 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }
}
