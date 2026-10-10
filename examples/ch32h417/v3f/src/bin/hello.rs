//! Minimal dual-core example — V3F half.
//!
//! Reset only starts this core, so every example that runs code on hart 1 needs
//! a V3F half like this one: bring the chip up and wake the V5F. The V5F half
//! (`v5f/src/bin/hello.rs`) then blinks LED1 (PF0) without touching the HAL.
//!
//! ```text
//! cargo xtask run --example hello
//! ```

#![no_std]
#![no_main]

use hal::println;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    // Only the boot core runs the bring-up: RCC, AFIO/GPIO and EXTI are shared
    // between both harts.
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    // Wake hart 1 before printing: `SDIPrint::write_str` spins until the debug
    // module consumes `DATA0`, which only happens once wlink has armed SDI
    // print, so a plain `flash` would block here and never start hart 1.
    unsafe { pfic::wake_other_core(V5F_ENTRY) };
    println!("V3F: woke hart 1 at {:#010x} (V5F blinks LED1/PF0)", V5F_ENTRY);

    loop {
        hal::delay::Delay.delay_ms(1000);
    }
}
