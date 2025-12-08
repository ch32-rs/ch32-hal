#![no_std]
#![no_main]



use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::Peri;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: Peri<'static, AnyPin>, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    // GPIO
    spawner.spawn(blink(p.PA15.into(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.into(), 100)).unwrap();
    // spawner.spawn(blink(p.PB8.into(), 100)).unwrap();
    loop {
        Timer::after_millis(2000).await;
    }
}
