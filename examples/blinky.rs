#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use {ch32v3_hal as hal, panic_halt as _};

#[embassy_executor::task(pool_size = 2)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    spawner.spawn(blink(p.PA15.degrade(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();

    loop {
        Timer::after_millis(2000).await;
        println!("tick in main");
    }
}
