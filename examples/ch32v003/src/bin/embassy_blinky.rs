#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;

#[embassy_executor::task(pool_size = 2)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after_millis(interval_ms).await;
        led.set_low();
        Timer::after_millis(interval_ms).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    println!("CHIP signature => {}", hal::signature::chip_id().name());
    println!("Clocks {:?}", hal::rcc::clocks());

    // let mut led = Output::new(p.PC4, Level::Low, Default::default());

    spawner.spawn(blink(p.PC4.degrade(), 110)).unwrap();
    spawner.spawn(blink(p.PA2.degrade(), 270)).unwrap();

    loop {
        Timer::after_millis(1000).await;
        println!("tick");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);

    loop {}
}
