#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use hal::usart::{UartRx, UartTx};

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

    let mut rx = UartRx::new_blocking(p.USART4, p.PC16, Default::default()).unwrap();

    // GPIO
    let mut led = Output::new(p.PB12, Level::High, Default::default());

    let mut buf = [0u8; 8];
    // FIXME: no time slice for embassy executor
    loop {
        rx.blocking_read(&mut buf).unwrap();

        led.toggle();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
