#![no_std]
#![no_main]



use ch32_hal as hal;
use hal::Peri;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;

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

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());


    // GPIO
    spawner.spawn(blink(p.PB12.into(), 1000)).unwrap();

    loop {
        Timer::after_millis(2000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
