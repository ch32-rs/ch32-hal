#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use hal::exti::ExtiInput;
use hal::gpio::{Level, Output, Pull};
use hal::println;
use panic_halt as _;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    println!("Wire a button from PC1 to GND. Each press toggles PD6.");

    let mut led = Output::new(p.PD6, Level::Low, Default::default());
    let mut button = ExtiInput::new(p.PC1, p.EXTI1, Pull::Up);

    loop {
        button.wait_for_falling_edge().await;
        led.toggle();
        println!("press");
    }
}
