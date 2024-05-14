#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::{println, Config};



#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSE;
    let p = hal::init(config);
    hal::embassy::init();

    println!("Clocks {:?}", hal::rcc::clocks());


    // GPIO
    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    loop {
        led.toggle();
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
