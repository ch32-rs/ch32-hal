#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let mut delay = Delay;

    let mut led = Output::new(p.PB17, Level::Low, Default::default());
    loop {
        led.toggle();

        delay.delay_ms(500);
        hal::println!("toggle!");
        let val = hal::pac::SYSTICK.cnt().read();
        hal::println!("systick: {}", val);
    }
}
