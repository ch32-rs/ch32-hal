#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use hal::println;

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(150)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let mut ei = ExtiInput::new(p.PC7, p.EXTI7, Pull::None);

    println!("Press USER button to start the blink task");
    ei.wait_for_falling_edge().await;

    // GPIO, PA8 and PC9 shares the same pin, use with caution
    spawner.spawn(blink(p.PA8.degrade())).unwrap();

    loop {
        Timer::after(Duration::from_millis(1000)).await;
        println!("tick");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
