#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::println;
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSE;
    let p = hal::init(config);

    let mut delay = Delay;

    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());
    let mut pin = p.PA1;

    loop {
        led.toggle();
        delay.delay_ms(1000);

        let val = adc.convert(&mut pin, hal::adc::SampleTime::CYCLES73);
        println!("adc: {}", val);
    }
}
