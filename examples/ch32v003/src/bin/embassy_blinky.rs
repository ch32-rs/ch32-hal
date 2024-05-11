#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embedded_hal::delay::DelayNs;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::println;
use qingke::riscv;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    let mut led = Output::new(p.PC4, Level::Low, Default::default());

    loop {
        led.toggle();

        Timer::after_millis(100).await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);

    loop {}
}
