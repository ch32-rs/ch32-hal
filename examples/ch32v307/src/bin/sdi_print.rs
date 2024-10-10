#![no_std]
#![no_main]

//! SDI debug print

use hal::println;
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    unsafe {
        // init delay cfg
        hal::delay::init();
    }

    println!("hello world!");

    println!("Flash size: {}kb", hal::signature::flash_size_kb());
    println!("Chip UID: {:x?}", hal::signature::unique_id());
    let chip_id = hal::signature::chip_id();
    println!("Chip {}, DevID: 0x{:x}", chip_id.name(), chip_id.dev_id());

    loop {
        println!("hello world!");

        unsafe {
            // qingke::riscv::asm::delay(10_000_000);
            hal::delay::Delay.delay_ms(1000);
        }
    }
}
