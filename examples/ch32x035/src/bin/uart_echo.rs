#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use hal::usart::Uart;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(Default::default());
    hal::embassy::init();

    // Connector pinout:
    // GND, VCC, PC17, PC16
    // GND, VCC, SDA, SCL (I2C1) - not working
    // GND, VCC, TX, RX (USART4, remap=5)
    // GND, VCC, RX, TX (USART4, remap=2)

    let mut uart = Uart::new_blocking(p.USART4, p.PC16, p.PC17, Default::default()).unwrap();

    // GPIO
    let mut led = Output::new(p.PB12, Level::High, Default::default());

    uart.blocking_write(b"ready!\r\n").unwrap();

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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
