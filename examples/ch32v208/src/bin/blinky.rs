#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use hal::gpio::{Level, Output};
use hal::println;
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("Hello from ch32v208 blinky!");
    println!("CHIP: {}", hal::signature::chip_id().name());

    let mut led = Output::new(p.PB8, Level::Low, Default::default());
    let mut count = 0u32;

    loop {
        led.toggle();
        count += 1;
        println!("blink: {}", count);

        riscv::asm::delay(1000000);
    }
}
