#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32v3_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use hal::{peripherals, println};

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low);

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
    hal::embassy::init();

    println!("embassy init ok");

    let input = Input::new(p.PA0, Pull::Up);
    let mut ei = ExtiInput::new(input, p.EXTI0);

    println!("exti init ok");

    ei.wait_for_falling_edge().await;

    println!("await ok");

    // GPIO
    spawner.spawn(blink(p.PA4.degrade())).unwrap();

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
