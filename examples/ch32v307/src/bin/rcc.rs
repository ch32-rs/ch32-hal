#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::task(pool_size = 3)]
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
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;

    let p = hal::init(config);
    hal::embassy::init();

    println!("SYS:   {}Hz", hal::rcc::clocks().sysclk.0);
    println!("HCLK:  {}Hz", hal::rcc::clocks().hclk.0);
    println!("PCLK1: {}Hz", hal::rcc::clocks().pclk1.0);
    println!("PCLK2: {}Hz", hal::rcc::clocks().pclk2.0);

    // GPIO
    spawner.spawn(blink(p.PA15.degrade(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();
    spawner.spawn(blink(p.PB8.degrade(), 100)).unwrap();
    let mut tick = 0;
    loop {
        Timer::after_millis(1000).await;

        let t = hal::pac::SYSTICK.cnt().read();
        let dt = t.wrapping_sub(tick);
        println!("systick: {}", dt);
        tick = t;
    }
}
