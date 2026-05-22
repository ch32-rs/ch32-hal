#![no_std]
#![no_main]



use embassy_executor::Spawner;
use hal::gpio::{Level, Output};
use hal::usart::Uart;
use hal::usart;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    // GPIO
    let mut led = Output::new(p.PC9, Level::Low, Default::default());

    let mut cfg = usart::Config::default();
    cfg.baudrate = 115200;
    let mut uart = Uart::new_blocking(p.USART3, p.PB11, p.PB10, cfg).unwrap();

    let _ = uart.blocking_write(b"Init ok\r\n");

    // FIXME: no time slice for embassy executor
    let mut buf = [0u8; 1];
    loop {
        uart.blocking_read(&mut buf).unwrap();

        if buf[0] >= b'a' && buf[0] <= b'z' {
            buf[0] -= 32;
        }

        if buf[0] == b'\r' {
            let _ = uart.blocking_write(b"\r\n").unwrap();
            led.toggle();
        } else {
            let _ = uart.blocking_write(&buf).unwrap();
        }
    }
}
