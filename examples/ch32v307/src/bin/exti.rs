#![no_std]
#![no_main]



use ch32_hal as hal;
use hal::Peri;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Level, Output, Pin, Pull, Speed};
use hal::println;

#[embassy_executor::task]
async fn blink(pin: Peri<'static, AnyPin>) {
    let mut led = Output::new(pin, Level::Low, Speed::High);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(150)).await;
        led.set_low();
        Timer::after(Duration::from_millis(150)).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let mut ei = ExtiInput::new(p.PB3, p.EXTI3, Pull::Up); // YD-CH32V307VCT6 USER button

    println!("Press USER button to start the blink task");
    ei.wait_for_falling_edge().await;

    // GPIO
    spawner.spawn(blink(p.PA15.into())).unwrap();

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
