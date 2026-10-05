//! Cross-core atomicity test — boot core (V3F, hart 0).
//!
//! The litmus test qingke's `unsafe-trust-wch-atomics` asks for: this core waits
//! until hart 1 has started incrementing the shared word, then hammers the same
//! word itself. Both cores do `CAS_INCREMENTS` `fetch_add`s, so the total must be
//! exactly `2 * CAS_INCREMENTS`; anything less means updates were lost, i.e. the
//! atomics are emulated (a critical section here only masks *this* hart's
//! interrupts) rather than performed by the A extension.
//!
//! ```text
//! cargo xtask flash --example atomics --dual-core
//! cargo xtask report
//! cargo xtask run --example atomics --dual-core   # prints the verdict
//! ```

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use core::sync::atomic::Ordering;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

/// Bound on waiting for hart 1 to start, and to finish.
const SPINS: u32 = 200_000_000;

#[ch32_hal::entry]
fn main() -> ! {
    // Same clocks as `dualcore` (rather than the slow default), so the two
    // million increments finish in about a second.
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi();
    let _p = hal::init(config);
    hal::debug::SDIPrint::enable();

    let mailbox = ipc::mailbox();
    mailbox.clear();
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    // Start only once hart 1 is running, so the increments really do contend.
    let mut spins = 0u32;
    while mailbox.cas_total.load(Ordering::Relaxed) == 0 {
        spins += 1;
        if spins > SPINS {
            hal::try_println!("hart 1 never started incrementing");
            break;
        }
    }

    for _ in 0..ipc::CAS_INCREMENTS {
        mailbox.cas_total.fetch_add(1, Ordering::Relaxed);
    }
    mailbox
        .cas_done
        .fetch_or(ipc::CAS_DONE_V3F, Ordering::Release);

    spins = 0;
    while mailbox.cas_done.load(Ordering::Acquire) & ipc::CAS_DONE_V5F == 0 {
        spins += 1;
        if spins > SPINS {
            break;
        }
    }

    let done = mailbox.cas_done.load(Ordering::Acquire);
    let total = mailbox.cas_total.load(Ordering::Relaxed);
    let expected = ipc::CAS_INCREMENTS * 2;
    hal::try_println!("[V3F] done={done:#x} total={total} expected={expected}");
    if total == expected {
        hal::try_println!("[V3F] OK: the A extension did the increments, no updates lost");
    } else {
        hal::try_println!(
            "[V3F] LOST UPDATES: {} increments vanished (atomics are not usable across cores)",
            expected - total
        );
    }

    loop {}
}
