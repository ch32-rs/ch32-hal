//! SYSTICK delay frequency check — second core (V5F, hart 1).
//!
//! Software 1-second reference loop, against the V5F's assumed frequency.
//! The V3F half drives the real check (`hal::delay::Delay` running at the
//! hart's core clock); this side only provides a rough, flash-pacing
//! counterexample in `ping`.
//!
//! Hart 1 deliberately does not use `hal::delay::Delay`: `Clocks` lives in
//! the boot core's private ITCM, so on this hart the HAL would read the
//! compile-time `HSI = 25 MHz`, not the post-PLL frequency — the fix is the
//! secondary-core entry in `docs/backlog.md`.

#![no_std]
#![no_main]

use ch32h417_v5f::{cache, mailbox};
use panic_halt as _;

/// V5F core frequency baked in at build time; on the EVT presets (HSI
/// 400/100, HSE 480/120) it is ≈4x the V3F speed.
const V5F_HZ: u32 = 400_000_000;

/// One second of CPU work at `V5F_HZ`. The loop body is ~3 cycles; on
/// uncached flash-resident code it is slower, so `ping` may tick slightly
/// slower than once a second even though hart 1 is healthy.
const LOOPS_PER_SEC: u32 = V5F_HZ / 3;

#[qingke_rt::entry]
fn main() -> ! {
    cache::enable_icache();

    let mailbox = mailbox();
    let mut ticks = 0u32;
    loop {
        for _ in 0..LOOPS_PER_SEC {
            unsafe { core::arch::asm!("nop") };
        }
        ticks = ticks.wrapping_add(1);
        mailbox.ping.store(ticks, core::sync::atomic::Ordering::Relaxed);
    }
}
