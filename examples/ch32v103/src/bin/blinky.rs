#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use hal::gpio::{Level, Output};
use hal::println;
use qingke::riscv;

#[qingke_rt::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PA7, Level::Low, Default::default());
    loop {
        led.toggle();

        unsafe {
            riscv::asm::delay(1000000);
        }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    // let _ = println!("\n\n\n{}", info);

    loop {}
}
