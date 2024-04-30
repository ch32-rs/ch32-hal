#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal::usart;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::UartTx;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    spawner.spawn(blink(p.PA0.degrade())).unwrap();

    let mut cfg = usart::Config::default();
    let mut uart = UartTx::new_blocking(p.USART1, p.PA9, cfg).unwrap();

    loop {
        Timer::after_millis(2000).await;

        uart.blocking_write(b"hello world from embassy main\r\n");
    }
}
