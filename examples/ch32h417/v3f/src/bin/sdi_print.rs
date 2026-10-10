//! SDI debug print for CH32H417 (V3F / boot core).
//!
//! Prints over the WCH-Link SDI virtual serial port. Flash and watch with:
//!
//! ```text
//! wlink flash --enable-sdi-print --watch-serial target/riscv32imafc-unknown-none-elf/release/sdi_print
//! ```

#![no_std]
#![no_main]

use hal::println;
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    // Brings up clocks (POR HSI 25M) and the systick-based delay.
    let _p = hal::init(hal::Config::default());

    hal::debug::SDIPrint::enable();

    println!("hello world from CH32H417 (V3F)!");

    println!("Flash size: {}kb", hal::signature::flash_size_kb());
    println!("Chip UID: {:x?}", hal::signature::unique_id());
    let chip_id = hal::signature::chip_id();
    println!("Chip {}, DevID: 0x{:x}", chip_id.name(), chip_id.dev_id());

    let mut n: u32 = 0;
    loop {
        println!("tick {}", n);
        n = n.wrapping_add(1);

        hal::delay::Delay.delay_ms(1000);
    }
}
