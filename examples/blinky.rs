#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use {ch32v3_hal as hal, panic_halt as _};

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    // let p = hal::init(Default::default());
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    spawner.spawn(blink(p.PA4.degrade())).unwrap();

    loop {
        Timer::after_millis(2000).await;
        println!("tick in main");
    }
}
