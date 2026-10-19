//! Cross-core atomicity test — second core (V5F, hart 1).
//!
//! `fetch_add`s the same shared word as the boot core, at the same time, and
//! records that it finished. If the hardware only *emulated* CAS (qingke's
//! default critical section masks this hart's interrupts and nothing else), the
//! two cores would lose updates and the boot core's total would fall short of
//! `2 * CAS_INCREMENTS` — which is exactly why both target JSONs say
//! `atomic-cas: true` and both crates enable `qingke/unsafe-trust-wch-atomics`.
//!
//! No `ch32_hal::init` and no embassy — the boot core owns bring-up, and this
//! half only touches the shared word.
//!
//! It also turns hart 1's instruction cache on before starting: the cache is
//! disabled by reset, and with it off this flash-resident loop runs an order of
//! magnitude slower (see `ch32h417_v5f::cache`).

#![no_std]
#![no_main]

use ch32h417_ipc::protocols::{Atomics, CAS_DONE_V5F, CAS_INCREMENTS};
use ch32h417_v5f::{cache, mailbox};
use core::sync::atomic::Ordering;
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    let mailbox = mailbox::<Atomics>();

    // The I-cache resets to disabled; without this every fetch of this
    // flash-resident image goes to flash.
    cache::enable_icache();

    for _ in 0..CAS_INCREMENTS {
        mailbox.payload.total.fetch_add(1, Ordering::Relaxed);
    }
    // Release: everything above is visible to the boot core once it observes
    // this bit with an acquire load.
    mailbox.payload.done.fetch_or(CAS_DONE_V5F, Ordering::Release);

    loop {}
}
