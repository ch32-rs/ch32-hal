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
    let mut uart = Uart::new_blocking(p.USART1, p.PA10, p.PA9, cfg).unwrap();

    uart.blocking_write(b"Init ok\r\n");

    // FIXME: no time slice for embassy executor
    let mut buf = [0u8; 1];
    loop {
        uart.blocking_read(&mut buf).unwrap();

        if buf[0] >= b'a' && buf[0] <= b'z' {
            buf[0] -= 32;
        }

        if buf[0] == b'\r' {
            uart.blocking_write(b"\r\n").unwrap();
            led.toggle();
        } else {
            uart.blocking_write(&buf).unwrap();
        }
    }
}
