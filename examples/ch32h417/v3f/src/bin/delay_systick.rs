//! SYSTICK delay frequency check — boot core (V3F, hart 0).
//!
//! Proves the H4 `hal::delay::Delay` runs at the hart's actual core frequency
//! rather than the global `hclk`. On the wrong clock this half's
//! `delay_ms(1000)` loop would drift visibly from one second per tick.
//!
//! `cargo xtask flash --example delay_systick --dual-core && cargo xtask report --example delay_systick`
//! shows `dualcore_counter` advancing one tick per wall-clock second; the V5F
//! half's `ping` advances at its own (compile-time-fitted) pace for
//! comparison. No console needed.
//!
//! Hart 1's half deliberately keeps its own delay loop instead of
//! `hal::delay::Delay`, so the two cores' paces can be compared. It could use
//! the HAL: `init_secondary` calls `rcc::refresh` and reads the clock tree the
//! boot core programmed rather than the compile-time `HSI` default.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::DelaySystick;
use qingke::pfic;

ipc::mailbox!(DelaySystick);
use ch32_hal as hal;
use panic_halt as _;

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    let _p = hal::init(hal::Config::default());

    let mailbox = mailbox();
    mailbox.clear();

    // Wake the second core.
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    let mut delay = hal::delay::Delay;
    let mut ticks = 0u32;
    loop {
        delay.delay_ms(1000u32);
        ticks = ticks.wrapping_add(1);
        mailbox
            .payload
            .v3f_ticks
            .store(ticks, core::sync::atomic::Ordering::Relaxed);
    }
}
