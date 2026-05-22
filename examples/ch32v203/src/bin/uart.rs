#![no_std]
#![no_main]



use ch32_hal::usart;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::usart::UartTx;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PB8, Level::Low, Default::default());

    let cfg = usart::Config::default();
    let mut uart = UartTx::new_blocking(p.USART1, p.PA9, cfg).unwrap();

    loop {
        Timer::after_millis(1000).await;

        let _ = uart.blocking_write(b"hello world from embassy main\r\n");

        led.toggle();
    }
}
