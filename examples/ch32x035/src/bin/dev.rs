#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;

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
    println!("clock init");
    let mut config = hal::Config::default();
    config.clock = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    let reason = hal::rcc::x0::reset_resona();

    println!("reset reason: {:?}", reason);

    // GPIO
    spawner.spawn(blink(p.PB12.degrade(), 1000)).unwrap();

    loop {
        Timer::after_millis(1000).await;

        let tick = hal::pac::SYSTICK.cnt().read();
        println!("tick: {:?}", tick);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
