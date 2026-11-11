//! Auto wakeup timer (AWU).
//!
//! A 6-bit up-counter clocked from HSI/1024 (about 46.9 kHz). The counter
//! raises a wakeup event when it reaches `WR.APR` and then starts over;
//! `PSC.TBR` selects the prescaler, from 1 to 61440. The period is
//! `(APR + 1) * prescaler / f_AWU`, so it ranges from about 43 us to 84 s.
//! See RM §4.
//!
//! The AWU is not a time source: its clock is an RC divider, there is no
//! counter to read back, and the period is set in whole counter ticks. It
//! schedules a wakeup, it cannot measure elapsed time, so it is deliberately
//! not wired into the Embassy time driver. With the `embassy` feature,
//! [`Awu::wait`] waits for the next event; the module then owns the `AWU`
//! interrupt and the handler wakes the waiting task.
//!
//! The event also sets EXTI line 27, which is outside the 24 lines the
//! `exti` driver handles and therefore never cleared by it. RM §4.2 asks for
//! a rising edge on that line while WCH's own standby example configures a
//! falling edge; the driver follows the example, and the edge only matters
//! for WFE/standby wakeup, not for the interrupt path.
//!
//! # Time driver interaction
//!
//! The interrupt handler takes a `critical_section` to hand the waker over.
//! On QingKe a second interrupt level that disables interrupts can leave
//! `MIE` clear after `mret`, so the handler runs at the same priority as the
//! Embassy time driver to rule out nesting: P15 for the SysTick driver, P0
//! for the TIM driver, which leaves the PFIC reset priority. The AWU is never
//! a substitute for the time driver, so waking from a stop mode still leaves
//! `embassy-time` stalled until the clock tree is restored.

use crate::pac::awu::vals::Prescaler;
use crate::{pac, peripherals, Peri, PeripheralType};

/// AWU counter clock: HSI 48 MHz / 1024 (RM §2.3.5, §4.1).
const CLOCK_HZ: u32 = 46_875;

/// The compare value is 6 bits wide. The counter runs from 0 to `APR`
/// inclusive, so a period is `APR + 1` counter ticks.
const APR_MAX: u8 = 0x3f;

/// One step is a single counter tick, far below the interrupt entry latency;
/// two keeps the window away from 0, which the RM does not describe.
const STEPS_MIN: u32 = 2;

/// EXTI line fed by the AWU event (RM table 7-2).
const EXTI_LINE: usize = 27;

/// Longest period the 6-bit window and the largest prescaler can express.
const MAX_PERIOD_US: u32 = ((APR_MAX as u64 + 1) * 61_440 * 1_000_000 / CLOCK_HZ as u64) as u32;

/// Prescaler settings in register encoding order.
const PRESCALERS: [(u32, Prescaler); 15] = [
    (1, Prescaler::DIV1),
    (2, Prescaler::DIV2),
    (4, Prescaler::DIV4),
    (8, Prescaler::DIV8),
    (16, Prescaler::DIV16),
    (32, Prescaler::DIV32),
    (64, Prescaler::DIV64),
    (128, Prescaler::DIV128),
    (256, Prescaler::DIV256),
    (512, Prescaler::DIV512),
    (1024, Prescaler::DIV1024),
    (2048, Prescaler::DIV2048),
    (4096, Prescaler::DIV4096),
    (10240, Prescaler::DIV10240),
    (61440, Prescaler::DIV61440),
];

/// Auto wakeup timer driver.
pub struct Awu<'d, T: Instance> {
    _inner: Peri<'d, T>,
}

impl<'d, T: Instance> Awu<'d, T> {
    /// Prepare the AWU and its EXTI line without starting the timer.
    pub fn new(inner: Peri<'d, T>) -> Self {
        T::regs().csr().modify(|w| w.set_en(false));

        let exti = &crate::pac::EXTI;
        exti.intenr().modify(|w| w.set_mr(EXTI_LINE, true));
        exti.ftenr().modify(|w| w.set_tr(EXTI_LINE, true));
        clear_pending();

        #[cfg(feature = "embassy")]
        irq::init();

        Self { _inner: inner }
    }

    /// Start the timer with a period of at least `timeout_us` microseconds.
    ///
    /// Returns the period the hardware actually uses. There is no way to
    /// reset the free-running counter, so the first event after a restart can
    /// arrive earlier than one period.
    ///
    /// Panics if `timeout_us` is too long for the 6-bit window and the
    /// largest prescaler (about 84 s).
    pub fn start(&mut self, timeout_us: u32) -> u32 {
        if timeout_us > MAX_PERIOD_US {
            panic!(
                "AWU timeout of {} us exceeds the {} us maximum",
                timeout_us, MAX_PERIOD_US
            );
        }

        let (prescaler, apr, actual_us) = prescaler_and_window(timeout_us);
        let regs = T::regs();

        // The prescaler and window must be written while the timer is
        // stopped; re-enabling it gates the compare event, not the counter.
        regs.csr().modify(|w| w.set_en(false));
        regs.psc().write(|w| w.set_tbr(prescaler));
        regs.wr().write(|w| w.set_apr(apr));
        clear_pending();
        regs.csr().modify(|w| w.set_en(true));

        actual_us
    }

    /// Stop the timer and clear any event it left pending.
    pub fn stop(&mut self) {
        T::regs().csr().modify(|w| w.set_en(false));
        clear_pending();
    }

    /// Whether the timer is running.
    pub fn is_running(&self) -> bool {
        T::regs().csr().read().en()
    }

    /// Wait for the next wakeup event.
    ///
    /// The timer keeps running, so awaiting again picks up the following
    /// period. An event that arrives before this call is remembered.
    #[cfg(feature = "embassy")]
    pub async fn wait(&mut self) {
        use core::future::poll_fn;
        use core::task::Poll;

        poll_fn(|cx| {
            irq::STATE.lock(|state| {
                let mut state = state.borrow_mut();
                if state.pending {
                    state.pending = false;
                    Poll::Ready(())
                } else {
                    state.waker = Some(cx.waker().clone());
                    Poll::Pending
                }
            })
        })
        .await
    }
}

impl<T: Instance> Drop for Awu<'_, T> {
    fn drop(&mut self) {
        T::regs().csr().modify(|w| w.set_en(false));
        // Drop runs without the borrow checker's awareness of the EXTI line,
        // so leave it enabled only while the driver exists.
        let exti = &crate::pac::EXTI;
        exti.intenr().modify(|w| w.set_mr(EXTI_LINE, false));
        exti.ftenr().modify(|w| w.set_tr(EXTI_LINE, false));
        clear_pending();
    }
}

/// Clear the EXTI pending bit the AWU event sets.
fn clear_pending() {
    crate::pac::EXTI.intfr().write(|w| w.set_if_(EXTI_LINE, true));
}

/// Pick the prescaler and window that land closest to `timeout_us`.
///
/// The counter runs from 0 to `APR` inclusive, so a period is `APR + 1`
/// steps; pick the step count, then store one less.
fn prescaler_and_window(timeout_us: u32) -> (Prescaler, u8, u32) {
    let requested = u64::from(timeout_us.max(1));
    let max_steps = u64::from(APR_MAX) + 1;
    let mut best: Option<(u64, Prescaler, u8, u32)> = None;

    for (divisor, prescaler) in PRESCALERS {
        let span = u64::from(divisor) * 1_000_000;
        let steps = (requested * u64::from(CLOCK_HZ) + span / 2) / span;
        let steps = steps.clamp(u64::from(STEPS_MIN), max_steps);
        let actual_us = (steps * span / u64::from(CLOCK_HZ)) as u32;

        let error = u64::from(actual_us.abs_diff(timeout_us.max(1)));
        match best {
            Some((best_error, ..)) if error >= best_error => {}
            _ => best = Some((error, prescaler, (steps - 1) as u8, actual_us)),
        }
    }

    let (_, prescaler, apr, actual_us) = best.unwrap();
    (prescaler, apr, actual_us)
}

#[cfg(feature = "embassy")]
mod irq {
    use core::cell::RefCell;
    use core::task::Waker;

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::blocking_mutex::Mutex;
    use qingke::interrupt::Priority;
    use qingke_rt::interrupt;

    use super::clear_pending;
    use crate::pac::Interrupt;

    pub(super) struct State {
        pub(super) pending: bool,
        pub(super) waker: Option<Waker>,
    }

    pub(super) static STATE: Mutex<CriticalSectionRawMutex, RefCell<State>> = Mutex::const_new(
        CriticalSectionRawMutex::new(),
        RefCell::new(State {
            pending: false,
            waker: None,
        }),
    );

    #[interrupt]
    unsafe fn AWU() {
        clear_pending();
        let waker = STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.pending = true;
            state.waker.take()
        });
        // Wake outside the critical section: `wake` may run executor code.
        if let Some(waker) = waker {
            waker.wake();
        }
    }

    pub(super) fn init() {
        clear_pending();

        // Match the priority of the active Embassy time driver so the two
        // handlers cannot nest; see the module docs.
        #[cfg(time_driver_timer)]
        const PRIORITY: Priority = Priority::P0;
        #[cfg(not(time_driver_timer))]
        const PRIORITY: Priority = Priority::P15;

        unsafe {
            qingke::pfic::set_priority(Interrupt::AWU as u8, PRIORITY as u8);
            qingke::pfic::enable_interrupt(Interrupt::AWU as u8);
        }
    }
}

trait SealedInstance {
    fn regs() -> pac::awu::Awu;
}

/// AWU instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

foreach_peripheral!(
    (awu, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> pac::awu::Awu {
                pac::$inst
            }
        }

        impl Instance for peripherals::$inst {}
    };
);
