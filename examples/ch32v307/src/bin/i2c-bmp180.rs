#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use hal::i2c::I2c;
use hal::println;
use hal::time::Hertz;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);
    loop {}
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = hal::init(Default::default());
    hal::embassy::init();

    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    println!("init ok");
    let i2c = I2c::new_blocking(p.I2C2, i2c_scl, i2c_sda, Hertz::khz(400), Default::default());

    let mut sensor = edrv_bmp180::blocking::BMP180::new_primary(i2c);

    sensor.init(Default::default()).unwrap();

    loop {
        let m = sensor.read_measurement(&mut Delay).unwrap();

        println!("Measurement: {:?}", m);

        Timer::after_secs(1).await;
    }
}
