//! Minimal V5F probe — just writes 0xBEEF to ITCM then loops.
//! If 0x200A0100 becomes 0xBEEF after wake, V5F is alive.

#![no_std]
#![no_main]

use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    unsafe {
        // Write magic number to ITCM — V3F can read this
        core::ptr::write_volatile(0x200A0100 as *mut u32, 0xDEAD_BEEF);
    }
    loop {}
}
