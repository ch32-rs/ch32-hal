#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{AnyPin, Level, Output};
use hal::println;
use hal::Peri;

#[embassy_executor::task(pool_size = 2)]
async fn blink(pin: Peri<'static, AnyPin>, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after_millis(interval_ms).await;
        led.set_low();
        Timer::after_millis(interval_ms).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("Hello from ch32v208!");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("Clocks: {:?}", hal::rcc::clocks());

    // Spawn blink tasks with different intervals
    spawner.spawn(blink(p.PB8.into(), 500)).unwrap();
    spawner.spawn(blink(p.PB9.into(), 200)).unwrap();

    let mut tick = 0u32;
    loop {
        Timer::after_millis(1000).await;
        tick += 1;
        println!("tick: {}", tick);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}
