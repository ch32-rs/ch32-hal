//! EXTI interrupt-path self-check through the software trigger (V3F only).
//!
//! H4 collapses the per-line EXTI vectors into `EXTI7_0` and `EXTI15_8`, and
//! `src/exti.rs` gained a branch for that shape. This raises lines through
//! `EXTI.SWIEVR` instead of a pin, so it needs no wiring: if the handler runs,
//! `on_irq()` clears the pending bit in `INTFR` (and the mask bit in
//! `INTENR`). Seeing both clear is proof the merged vector reached the driver.
//! Lines 2, 9 and 15 cover both vectors; each is triggered three times.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::ExtiCheck;

ipc::mailbox!(ExtiCheck);
use panic_halt as _;

const LINES: [usize; 3] = [2, 9, 15];
const ROUNDS: u32 = 3;

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

/// Raise `line` and wait for the handler to clear its pending bit.
fn trigger(line: usize) -> bool {
    let exti = hal::pac::EXTI;
    exti.intenr().modify(|w| w.set_mr(line, true));
    exti.swievr().modify(|w| w.set_swie(line, true));

    let bit = 1u32 << line;
    let mut spins = 0u32;
    while exti.intfr().read().0 & bit != 0 {
        spins += 1;
        if spins > 1_000_000 {
            return false;
        }
    }
    true
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let _p = hal::init(hal::Config::default());

    let mut checks = 0u32;
    let mut failures = 0u32;
    let mut mask = 0u32;

    for line in LINES {
        let mut handled = 0u32;
        for _ in 0..ROUNDS {
            let pending_cleared = trigger(line);
            // `on_irq()` also clears the mask bit for every line it served.
            let mask_cleared = hal::pac::EXTI.intenr().read().0 & (1 << line) == 0;
            check(&mut failures, &mut checks, pending_cleared);
            check(&mut failures, &mut checks, mask_cleared);
            if pending_cleared && mask_cleared {
                handled += 1;
            }
        }
        if handled == ROUNDS {
            mask |= 1 << line;
        }
    }

    let exti = hal::pac::EXTI;
    mailbox.payload.lines.store(mask, Relaxed);
    mailbox.payload.rounds.store(ROUNDS, Relaxed);
    mailbox.payload.intfr.store(exti.intfr().read().0, Relaxed);
    mailbox.payload.intenr.store(exti.intenr().read().0, Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
