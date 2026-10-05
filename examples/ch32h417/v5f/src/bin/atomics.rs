//! Cross-core atomicity test — second core (V5F, hart 1).
//!
//! `fetch_add`s the same shared word as the boot core, at the same time, and
//! records that it finished. If the hardware only *emulated* CAS (qingke's
//! default critical section masks this hart's interrupts and nothing else), the
//! two cores would lose updates and the boot core's total would fall short of
//! `2 * CAS_INCREMENTS` — which is exactly why both target JSONs say
//! `atomic-cas: true` and both crates enable `qingke/unsafe-trust-wch-atomics`.
//!
//! Metapac only, no `ch32-hal` and no embassy — the boot core owns bring-up.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_v5f::mailbox;
use core::sync::atomic::Ordering;
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    let mailbox = mailbox();

    for _ in 0..ipc::CAS_INCREMENTS {
        mailbox.cas_total.fetch_add(1, Ordering::Relaxed);
    }
    // Release: everything above is visible to the boot core once it observes
    // this bit with an acquire load.
    mailbox
        .cas_done
        .fetch_or(ipc::CAS_DONE_V5F, Ordering::Release);

    loop {}
}
