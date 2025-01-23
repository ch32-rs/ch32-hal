//! This example shows how to share (async) I2C buses between multiple devices with embassy.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::cell::RefCell;
use core::fmt::Write;

use ch32_hal::gpio::{AnyPin, Level, Output};
use ch32_hal::i2c::{self, I2c, Instance};
use ch32_hal::mode::Blocking;
// use ch32_hal::pac::I2C1;
use ch32_hal::time::Hertz;
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::{Mutex, NoopMutex};
use embassy_time::Timer;
use panic_halt as _;
use static_cell::StaticCell;

type I2cBus = NoopMutex<I2c<'static, ch32_hal::peripherals::I2C1, Blocking>>;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    ch32_hal::debug::SDIPrint::enable();
    let p = ch32_hal::init(ch32_hal::Config::default());
    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    led.set_high();
    let scl = p.PC2;
    let sda = p.PC1;
    let i2c = I2c::new_blocking(p.I2C1, scl, sda, Hertz::hz(400_000), Default::default());

    // let mut bus: Mutex<NoopRawMutex, _> = Mutex::new(i2c);
    // let bus: I2cBus = ;
    static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();
    let i2c_bus: &'static I2cBus = I2C_BUS.init(Mutex::new(i2c));
    spawner.spawn(i2c_task_one(&i2c_bus)).unwrap();
    // let display_i2c_dev = I2cDevice::new(&bus);
    loop {
        Timer::after_millis(1000).await;
    }
}

/// Well this compiles?
#[embassy_executor::task]
async fn i2c_task_one(i2c_bus: &'static I2cBus) {}
