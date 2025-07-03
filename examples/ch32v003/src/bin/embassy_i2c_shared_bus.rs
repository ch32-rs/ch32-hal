//! This example shows how to share an async I2C bus between multiple devices with embassy.
//! This allows you to have multiple devices on the same bus.
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal::gpio::{Level, Output};
use ch32_hal::i2c::I2c;
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, println};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C1_EV => ch32_hal::i2c::EventInterruptHandler<ch32_hal::peripherals::I2C1>;
    I2C1_ER => ch32_hal::i2c::ErrorInterruptHandler<ch32_hal::peripherals::I2C1>;
});

type I2cBus = Mutex<NoopRawMutex, I2c<'static, ch32_hal::peripherals::I2C1, ch32_hal::mode::Async>>;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    ch32_hal::debug::SDIPrint::enable();
    let p = ch32_hal::init(ch32_hal::Config::default());
    //Check your board for the correct pin. This is for the nanoCH32V003 Development Board
    let mut led = Output::new(p.PD6, Level::Low, Default::default());

    //Sets up an async i2c
    let scl = p.PC2;
    let sda = p.PC1;
    let i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::hz(400_000),
        Default::default(),
    );

    //sets up a i2c bus using the custom type I2cBus at the top
    let i2c_bus: I2cBus = Mutex::new(i2c);

    //Sets up 2 difference embedded_hal devices from the i2c bus
    let i2c_device_1 = I2cDevice::new(&i2c_bus);
    let i2c_device_2 = I2cDevice::new(&i2c_bus);

    //Sets up 2 dummy i2c drivers one for each device
    let mut _dummy_i2c_driver_1 = DummyI2cDeviceDriver::new(i2c_device_1, 0x01);
    let mut _dummy_i2c_driver_2 = DummyI2cDeviceDriver::new(i2c_device_2, 0x02);

    loop {
        led.toggle();
        println!("toggle!");
        Timer::after_millis(1000).await;
    }
}

// Dummy I2C device driver, using `embedded-hal-async`
struct DummyI2cDeviceDriver<I2C: embedded_hal_async::i2c::I2c> {
    _i2c: I2C,
}

impl<I2C: embedded_hal_async::i2c::I2c> DummyI2cDeviceDriver<I2C> {
    fn new(i2c_dev: I2C, _address: u8) -> Self {
        Self { _i2c: i2c_dev }
    }
}
