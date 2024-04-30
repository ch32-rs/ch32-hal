#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal::usart;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::UartTx;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    hal::embassy::init();

    let mut led = Output::new(p.PB8, Level::Low, Default::default());

    let mut cfg = usart::Config::default();
    let mut uart = UartTx::new_blocking(p.USART1, p.PA9, cfg).unwrap();

    loop {
        Timer::after_millis(1000).await;

        uart.blocking_write(b"hello world from embassy main\r\n");

        led.toggle();
    }
}
