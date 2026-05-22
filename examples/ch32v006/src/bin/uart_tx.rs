#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use hal::println;
use hal::usart::UartTx;
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSE;
    let p = hal::init(config);

    let mut uart = UartTx::new_blocking(p.USART1, p.PC0, Default::default()).unwrap();

    println!("dev init ok");

    let _ = uart.blocking_write(b"Hello, world!\r\n").unwrap();

    let mut delay = Delay;

    let mut led = Output::new(p.PD6, Level::Low, Default::default());
    loop {
        led.toggle();

        let _ = uart.blocking_write(b"Hello, world!\r\n").unwrap();

        delay.delay_ms(1000);
    }
}
