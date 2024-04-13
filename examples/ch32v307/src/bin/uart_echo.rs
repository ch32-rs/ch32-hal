#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::{Uart, UartTx};
use hal::{println, usart};
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    let mut led = Output::new(p.PA4, Level::Low, Default::default());

    let mut cfg = usart::Config::default();
    //cfg.baudrate = 1000000;
    let mut uart = Uart::new(p.USART1, p.PA9, p.PA10, cfg).unwrap();

    uart.blocking_write(b"Init ok\r\n");

    loop {
        // Timer::after_millis(2000).await;
        while let Ok(b) = nb::block!(uart.nb_read()) {
            if b == b'\r' {
                uart.blocking_write(b"\r\n");
            } else {
                uart.blocking_write(&[b]);
            }
        }

        led.toggle();

        //uart.blocking_write(b"Hello from serial\r\n");
        // println!("fuck");
    }
}
