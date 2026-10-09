//! SYSTICK delay frequency check — boot core (V3F, hart 0).
//!
//! Proves the H4 `hal::delay::Delay` runs at the hart's actual core frequency
//! rather than the global `hclk`. On the wrong clock this half's
//! `delay_ms(1000)` loop would drift visibly from one second per tick.
//!
//! `cargo xtask flash --example delay_systick --dual-core && cargo xtask report`
//! shows `dualcore_counter` advancing one tick per wall-clock second; the V5F
//! half's `ping` advances at its own (compile-time-fitted) pace for
//! comparison. No console needed.
//!
//! Hart 1's half deliberately does not use `hal::delay::Delay`: `Clocks`
//! still lives in the boot core's private ITCM (see `docs/backlog.md`'s
//! secondary-core entry), so on hart 1 the HAL would read the compile-time
//! `HSI = 25 MHz`, not the post-PLL frequency.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    let mailbox = ipc::mailbox();
    mailbox.clear();

    // Hand over before printing anything: without `--enable-sdi-print` the
    // SDI write would spin and hart 1 would never be scheduled. All output
    // is through the mailbox; `report` picks it up.
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    // Lives at the mailbox's marker slot so `xtask report` shows hart 0
    // entered the loop as well.
    mailbox
        .dualcore_marker
        .store(ipc::DUALCORE_MARKER, core::sync::atomic::Ordering::Relaxed);

    let mut delay = hal::delay::Delay;
    let mut ticks = 0u32;
    loop {
        delay.delay_ms(1000u32);
        ticks = ticks.wrapping_add(1);
        mailbox
            .dualcore_counter
            .store(ticks, core::sync::atomic::Ordering::Relaxed);
    }
}
