//! Minimal V5F probe — marks the cross-core mailbox and loops.
//!
//! If `0x20178008` reads `0xDEADBEEF` after the wake, the V5F executed.
//! The marker deliberately lives in `RAM_SHARED` (declared with the same
//! address in both crates' `memory.x`, mirroring the CSDK's `RAM_SHARED`
//! section) rather than in ITCM: ITCM is the *V3F's* private RAM, and the
//! V3F's `.bss` gets zeroed at boot, so a marker there cannot distinguish
//! "the V5F never ran" from "the V3F wiped it".

#![no_std]
#![no_main]

use panic_halt as _;

/// Cross-core mailbox slot (see the module docs).
const MAILBOX_PROBE: *mut u32 = 0x2017_8008 as *mut u32;

#[qingke_rt::entry]
fn main() -> ! {
    unsafe {
        core::ptr::write_volatile(MAILBOX_PROBE, 0xDEAD_BEEF);
    }
    loop {}
}
