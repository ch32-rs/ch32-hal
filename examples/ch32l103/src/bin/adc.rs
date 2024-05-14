#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::adc::SampleTime;
use hal::gpio::{Level, Output};
use hal::println;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSE;
    let p = hal::init(config);
    hal::embassy::init();

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut ch = p.PA1;

    // GPIO
    let mut led = Output::new(p.PB0, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        println!("Starting conversion!");
        let val = adc.convert(&mut ch, SampleTime::CYCLES239_5);

        println!("val => {}", val);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
