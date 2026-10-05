//! Minimal dual-core example — V5F half.
//!
//! Woken by `v3f/src/bin/hello.rs`, it toggles LED1 (PF0) through raw GPIO
//! registers. Uses no ch32-hal APIs (the V3F already configured the clocks);
//! the ch32-hal dependency in Cargo.toml only supplies metapac's
//! device.x/memory.x.

#![no_std]
#![no_main]

use panic_halt as _;

const GPIOF_BASE: u32 = 0x4001_1C00;
const GPIOF_CFGLR: *mut u32 = GPIOF_BASE as *mut u32;
const GPIOF_BSHR: *mut u32 = (GPIOF_BASE + 0x10) as *mut u32;

const PF0_OUTPUT: u32 = 0x03;

#[qingke_rt::entry]
fn main() -> ! {
    // Configure PF0 as push-pull output
    unsafe {
        let cfg = core::ptr::read_volatile(GPIOF_CFGLR);
        core::ptr::write_volatile(GPIOF_CFGLR, (cfg & !0xF) | PF0_OUTPUT);
    }

    loop {
        // Set PF0
        unsafe { core::ptr::write_volatile(GPIOF_BSHR, 1 << 0); }
        for _ in 0..5_000_000 {
            unsafe { core::arch::asm!("nop"); }
        }
        // Reset PF0
        unsafe { core::ptr::write_volatile(GPIOF_BSHR, 1 << 16); }
        for _ in 0..5_000_000 {
            unsafe { core::arch::asm!("nop"); }
        }
    }
}
