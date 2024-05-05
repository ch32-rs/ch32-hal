#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::{pac, println};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_72MHZ_HSE;
    let p = hal::init(config);

    println!("SDI Print enabled");
    let mut led = Output::new(p.PC13, Level::Low, Default::default());

    loop {
        led.toggle();

        Delay.delay_ms(1000);

        let n = pac::SYSTICK.cnt().read();
        println!("systick: {}", n); // should print in 9_000_000 cycles incrementally, 9*8 = 72MHz
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
