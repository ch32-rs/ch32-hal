#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::adc::SampleTime;
use hal::println;
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    //hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let mut delay = Delay;

    let mut led = Output::new(p.PB17, Level::Low, Default::default());

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());
    let mut pin = p.PA1;

    loop {
        led.toggle();
        delay.delay_ms(1000);

        let val = adc.convert(&mut pin, SampleTime::CYCLES9);
        println!("adc: {}", val);
    }
}
