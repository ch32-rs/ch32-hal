#![no_std]
#![no_main]

use embassy_executor::Spawner;
use hal::gpio::{Level, Output};
use hal::usart;
use hal::usart::Uart;
use {ch32_hal as hal, panic_halt as _};

use ch32_hal::bind_interrupts;

bind_interrupts!(struct Irqs {
    USART4 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART4>;
});

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    // GPIO
    let mut led = Output::new(p.PB4, Level::Low, Default::default());

    let cfg = usart::Config::default();
    let (mut tx_uart, mut rx_uart) = Uart::new(p.USART4, p.PE1, p.PE0, Irqs, p.DMA2_CH5, p.DMA2_CH3, cfg)
        .unwrap()
        .split();

    tx_uart.write(b"Init ok\r\n").await.unwrap();

    let mut buf = [0u8; 255];
    loop {
        match rx_uart.read_until_idle(&mut buf).await {
            Ok(size) => {
                for character in &mut buf[..size] {
                    if *character >= b'a' && *character <= b'z' {
                        *character -= 32;
                    }
                }
                tx_uart.write(&buf[..size]).await.unwrap();
                led.toggle();
            }
            Err(_err) => {}
        }
    }
}
