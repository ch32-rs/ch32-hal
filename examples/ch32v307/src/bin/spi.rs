#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal::spi;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use hal::time::Hertz;
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
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    // GPIO
    spawner.spawn(blink(p.PA0.degrade())).unwrap();
    let (sck, miso, mosi) = (p.PA5, p.PA6, p.PA7);

    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz::khz(375);
    let mut spi = spi::Spi::new_blocking(p.SPI1, sck, mosi, miso, spi_config);

    loop {
        Timer::after_millis(2000).await;
        let mut data: [u8; 4] = [0xde, 0xad, 0xbe, 0xef];
        println!("Transmitting data: {:?}", data);
        let res = spi.blocking_transfer_in_place(&mut data);
        println!("Got result: {:?}", res);
        println!("Data: {:?}", data);
    }
}
