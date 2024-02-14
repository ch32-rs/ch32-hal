#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use hal::gpio::{Level, Output};
use hal::println;
use {ch32v3_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut delay = Delay;

    let mut adc = hal::adc::Adc::new(p.ADC1, &mut delay, Default::default());

    let mut ch = p.PB1;
    adc.configure_channel(&mut ch, 1, hal::adc::SampleTime::Cycles6);

    // GPIO
    let mut led = Output::new(p.PB12, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        println!("Starting conversion!");
        let val = adc.convert(&mut ch, &mut delay);

        println!("val => {}", val);
    }
}
