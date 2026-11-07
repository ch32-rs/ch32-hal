//! Generic V5F launcher — wake hart 1 at its linked entry, then `wfi`.
//!
//! V5F code does not need a bespoke V3F half per example: flash this once, then
//! reflash only the V5F half and let the launcher start it:
//!
//! ```text
//! cargo xtask flash --example launcher --v3f-only
//! ```
//!
//! `--v3f-only` writes this image without touching the V5F region, so it starts
//! whatever V5F payload is already programmed — including one built by hand.
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

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::Empty;

// The launcher starts whatever V5F image is already programmed, so it cannot
// know that image's payload. It only stamps the version word.
ipc::mailbox!(Empty);
use ch32_hal as hal;
use panic_halt as _;
use qingke::pfic;

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    // The shared RCC/AFIO/GPIO blocks are the boot core's job: the V5F halves
    // run with no HAL initialisation of their own.
    let _p = hal::init(hal::Config::default());

    // Keep `cargo xtask report` honest: shared RAM survives a reset, so without
    // this it could show fields an earlier example left behind.
    mailbox().clear();

    // Wake the second core.
    unsafe { pfic::wake_other_core(V5F_ENTRY) };
    log!("launcher: woke hart 1 at {:#010x}, V3F parks in wfi", V5F_ENTRY);

    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}
