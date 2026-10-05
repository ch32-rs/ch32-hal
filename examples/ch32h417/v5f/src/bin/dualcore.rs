//! Dual-core demo — V5F half.
//!
//! Woken by `v3f/src/bin/dualcore.rs`, it marks the cross-core mailbox and
//! idles. If `dualcore_marker` reads `0xDEADBEEF` in `cargo xtask report`, the
//! V5F executed.
//! The marker deliberately lives in `RAM_SHARED` (declared with the same
//! address in both crates' `memory.x`, mirroring the CSDK's `RAM_SHARED`
//! section) rather than in ITCM: ITCM is the *V3F's* private RAM, and the
//! V3F's `.bss` gets zeroed at boot, so a marker there cannot distinguish
//! "the V5F never ran" from "the V3F wiped it".

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_v5f::{cache, mailbox};
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    cache::enable_icache();

    mailbox()
        .dualcore_marker
        .store(ipc::DUALCORE_MARKER, core::sync::atomic::Ordering::Relaxed);

    loop {}
}
