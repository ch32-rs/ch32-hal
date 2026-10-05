//! SDI + CPU-id demo — boot core (V3F, QingKe hart 0).
//!
//! Answers "which core is running this code?" on hardware: this hart reports
//! itself as `hart=C0`, wakes the second core, and the V5F image
//! (`v5f/src/bin/sdi_cpuid.rs`) reports itself as `hart=C1` over the same SDI
//! console.
//!
//! # One writer at a time
//!
//! `SDIPrint` drives the debug module's `DATA0`/`DATA1`, a single channel
//! shared by both harts with no arbitration. Two harts printing at once
//! interleave 7-byte chunks and corrupt each other's lines — observed on
//! hardware as the V5F banner cutting into this core's "waking the second
//! core" line. So the console is handed over explicitly through the shared
//! RAM mailbox (`RAM_SHARED` at 0x20178000, declared in both crates'
//! `memory.x`):
//!
//! 1. V3F prints its banner while holding the token.
//! 2. V3F gives the token to hart 1 and wakes it, then waits (bounded) for it
//!    to be handed back.
//! 3. V5F prints its banner, hands the token back, and never touches SDI
//!    again — it reports liveness by bumping a counter in shared RAM instead.
//! 4. V3F is the only SDI writer from then on, and prints both counters.
//!
//! # Building and flashing
//!
//! ```text
//! cargo xtask run --example sdi_cpuid
//! ```
//!
//! `xtask` builds both cores and writes this image (flash 0x00000000 ->
//! 0x08000000) before the V5F image at 0x00010000, which is the same 64K split
//! the WCH CSDK uses.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use hal::println;
use qingke::pfic::{self, HartId};
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

/// Cross-core mailbox in shared RAM (`RAM_SHARED`, declared with the same
/// address in both crates' `memory.x` — the CSDK's convention). `PRINT_TURN`
/// is the hart allowed to drive SDI, `V5F_TICKS` is the second core's
/// liveness counter.

/// Bounded spin so a missing/stopped hart 1 cannot wedge this one (~1s).
const HANDOFF_SPINS: u32 = 5_000_000;

#[ch32_hal::entry]
fn main() -> ! {
    // Only the boot core runs the full bring-up: RCC, AFIO/GPIO and EXTI are
    // shared between both harts (see the V5F demo for why hart 1 must not
    // repeat this).
    let _p = hal::init(hal::Config::default());

    unsafe {
        ipc::mailbox().sdi_token.store(0, core::sync::atomic::Ordering::Relaxed);
        ipc::mailbox().sdi_ticks.store(0, core::sync::atomic::Ordering::Relaxed);
    }

    hal::debug::SDIPrint::enable();

    let me = HartId::current();
    println!("[V3F] hart={:?} (mhartid 0) is executing this code", me);
    println!("[V3F] waking the second core at {:#010x}", V5F_ENTRY);

    // Hand the console to hart 1, then start it.
    unsafe { ipc::mailbox().sdi_token.store(1, core::sync::atomic::Ordering::Relaxed) };
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    // Wait for hart 1 to print its banner and hand the token back.
    let mut spins = 0u32;
    while unsafe { ipc::mailbox().sdi_token.load(core::sync::atomic::Ordering::Relaxed) } != 0 {
        spins += 1;
        if spins > HANDOFF_SPINS {
            break;
        }
    }
    unsafe { ipc::mailbox().sdi_token.store(0, core::sync::atomic::Ordering::Relaxed) };

    let mut n = 0u32;
    loop {
        let v5f_ticks = unsafe { ipc::mailbox().sdi_ticks.load(core::sync::atomic::Ordering::Relaxed) };
        println!("[V3F] tick {} | [V5F] ticks {}", n, v5f_ticks);
        n = n.wrapping_add(1);

        hal::delay::Delay.delay_ms(1000);
    }
}
