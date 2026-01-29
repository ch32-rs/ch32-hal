#![no_std]
#![no_main]




use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Delay, Timer};
use hal::i2c::I2c;
use hal::time::Hertz;
use hal::{bind_interrupts, println};
use hal::peripherals;

bind_interrupts!(struct Irqs {
    I2C2_EV => hal::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

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

    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    println!("init ok");
    let i2c = I2c::new(
        p.I2C2,
        i2c_scl,
        i2c_sda,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Hertz::khz(400),
        Default::default(),
    );

    let mut sensor = edrv_bmp180::BMP180::new_primary(i2c);

    sensor.init(Default::default()).await.unwrap();

    loop {
        let m = sensor.read_measurement(&mut Delay).await.unwrap();

        println!("Measurement: {:?}", m);

        Timer::after_secs(1).await;
    }
}
