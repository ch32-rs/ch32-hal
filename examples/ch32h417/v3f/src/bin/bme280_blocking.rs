//! BME280 temperature/pressure/humidity reading demo for the CH32H417.
//!
//! The sensor sits on **PB6 = SCL**, **PB7 = SDA** (I2C1, AF4) at the
//! primary BME280 address 0x76 (0x77 if SDO is tied high).
//!
//! Run with:
//!
//! ```text
//! cargo run --release --bin bme280_blocking
//! ```

#![no_std]
#![no_main]

use edrv_bme280::blocking::BME280;
use hal::i2c::I2c;
use hal::println;
use hal::time::Hertz;
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    let p = hal::init(hal::Config::default());

    hal::debug::SDIPrint::enable();
    println!("BME280 on I2C1 (SCL=PB6, SDA=PB7) @ 400kHz");

    let i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz::khz(400), Default::default());
    let mut sensor = BME280::new_primary(i2c);
    let mut delay = hal::delay::Delay;

    match sensor.init() {
        Ok(()) => println!("init ok (BME280: {})", sensor.is_bme280),
        Err(e) => {
            println!("init failed: {:?}", e);
            println!("check wiring: SCL=PB6, SDA=PB7, addr 0x76/0x77");
            loop {
                delay.delay_ms(1000);
            }
        }
    }

    loop {
        match sensor.read_measurement(&mut delay) {
            Ok(m) => println!(
                "T={:.2}C  P={:.2}hPa  H={:.2}%",
                m.temperature_celsius(),
                m.pressure_hpa(),
                m.humidity_percent()
            ),
            Err(e) => println!("read failed: {:?}", e),
        }

        delay.delay_ms(2000);
    }
}
