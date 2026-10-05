//! Ping-pong over shared memory — boot core (V3F, hart 0).
//!
//! The demo answers "how do the two harts talk?" end to end:
//!
//! 1. the boot core clears the shared mailbox and hands over to hart 1;
//! 2. hart 1 probes its own CPU-ID CSRs into that mailbox;
//! 3. the boot core prints what hart 1 reported — the only thing crossing
//!    between the cores is shared memory;
//! 4. from then on the two cores exchange one round per second: hart 1
//!    increments `ping`, this core answers by storing `pong`, and the next
//!    round only starts once the answer has been seen.
//!
//! `ping != pong` therefore means "a round is outstanding", and `xtask report`
//! shows both counters marching while this core is not being observed.
//!
//! # Watching it
//!
//! Printing needs the SDI console, and attaching the console pauses hart 1 —
//! so the two views are mutually exclusive:
//!
//! ```text
//! cargo xtask flash --example pingpong --dual-core   # rounds run; read them back
//! cargo xtask report                                  # ping/pong advancing
//! cargo xtask run   --example pingpong --dual-core    # see hart 1's report printed
//! ```
//!
//! With the console attached the printed counters freeze (hart 1 is held), and
//! without it the prints are dropped rather than stalling the loop: this half
//! uses `try_println!`, the bounded variant, precisely so the ping-pong keeps
//! running when nobody is listening.
//!
//! # Building and flashing
//!
//! ```text
//! cargo xtask flash --example pingpong --dual-core
//! cargo xtask report
//! ```

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use core::sync::atomic::Ordering;
use hal::delay::Delay;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

/// How long to wait for hart 1's report before carrying on without it.
const HANDOFF_SPINS: u32 = 20_000_000;

#[ch32_hal::entry]
fn main() -> ! {
    // The boot core owns bring-up; hart 1 runs with no HAL initialisation.
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    // Clear the mailbox (shared RAM survives a reset) and hand over — before
    // printing anything, because a blocking print would strand hart 1.
    let mailbox = ipc::mailbox();
    mailbox.clear();
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    let mut spins = 0u32;
    while mailbox.cpuid_done.load(Ordering::Relaxed) != ipc::CPUID_DONE {
        spins += 1;
        if spins > HANDOFF_SPINS {
            break;
        }
    }

    if mailbox.cpuid_done.load(Ordering::Relaxed) == ipc::CPUID_DONE {
        hal::try_println!("=== hart 1, reported through shared memory ===");
        let values = mailbox.cpuid_snapshot();
        let present = mailbox.cpuid_present.load(Ordering::Relaxed);
        for (i, name) in ipc::CPUID_CSR_NAMES.iter().enumerate() {
            if present & (1 << i) != 0 {
                hal::try_println!("  {:<18} = {:#010x}", name, values[i]);
            } else {
                hal::try_println!("  {:<18}   absent (read faulted)", name);
            }
        }
        hal::try_println!(
            "  image build features = {:#04x}",
            mailbox.cpuid_buildcfg.load(Ordering::Relaxed)
        );
        if present & (1 << ipc::CPUID_MISA_INDEX) != 0 {
            let misa = values[ipc::CPUID_MISA_INDEX];
            let mut letters = [0u8; 26];
            hal::try_println!(
                "  misa: MXL={} extensions={}",
                (misa >> 30) & 3,
                misa_letters(&mut letters, misa)
            );
        }
    } else {
        hal::try_println!(
            "no report from hart 1 (progress = {})",
            mailbox.cpuid_progress.load(Ordering::Relaxed)
        );
    }

    // Ping-pong: answer hart 1's counter once per second. `ping != pong` means a
    // round is outstanding, so hart 1 only starts the next one after this core
    // has echoed — the exchange cannot run away from the printer.
    let mut echoed = mailbox.pong.load(Ordering::Relaxed);
    loop {
        let ping = mailbox.ping.load(Ordering::Relaxed);
        if ping != echoed {
            hal::try_println!("[V3F] round {} (pong was {})", ping, echoed);
            echoed = ping;
            mailbox.pong.store(ping, Ordering::Relaxed);
        }
        Delay.delay_ms(1000);
    }
}

/// Turn `misa`'s extension bits into letters without an allocator: A..Z map to
/// bits 0..25.
fn misa_letters(out: &mut [u8; 26], misa: u32) -> &str {
    let mut len = 0;
    for (bit, letter) in (b'A'..=b'Z').enumerate() {
        if misa & (1 << bit) != 0 {
            out[len] = letter;
            len += 1;
        }
    }
    core::str::from_utf8(&out[..len]).unwrap_or("")
}
