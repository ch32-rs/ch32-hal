//! Generic V5F launcher — wake hart 1 at its linked entry, then `wfi`.
//!
//! V5F code does not need a bespoke V3F half per example: flash this once, then
//! reflash only the V5F half and let the launcher start it:
//!
//! ```text
//! cargo xtask flash --example launcher          # once
//! cargo xtask flash --example cpuid --core v5f  # iterate on hart 1
//! ```
//!
//! The V5F entry is fixed by `v5f/memory.x` (`FLASH ORIGIN`, 1KB-aligned), so
//! the launcher only has to bring the shared blocks up, write the address to
//! the wake register, and park the V3F in `wfi` — the second core then runs
//! whatever image occupies the V5F region.
//!
//! Deliberately not an embassy application: no executor and no async driver.
//! `hal::init()` only programs the RCC/AFIO/GPIO the V5F halves rely on, and
//! this crate builds with `default-features = false` (without embassy), which
//! the H4 time driver still requires.

#![no_std]
#![no_main]

use hal::println;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    // The shared RCC/AFIO/GPIO blocks are the boot core's job: the V5F halves
    // run with no HAL initialisation of their own.
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    println!(
        "launcher: waking hart 1 at {:#010x}, V3F parks in wfi",
        V5F_ENTRY
    );
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}
