//! E22-400M22S
//!
//! LoRa SX1268
//!
//! GO FUCK YOURSELF for the unmaintained crates
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use core::fmt::Write;

use ch32_hal as hal;
use ch32x035_examples::sx1268::{regs, Config, SX1268};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiBus;
use hal::dma::NoDma;
use hal::exti::ExtiInput;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::prelude::*;
use hal::spi::Spi;
use hal::{peripherals, println, spi};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.clock = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    let mut delay = Delay;

    // SPI1, remap 0
    let cs = p.PA4;
    let sck = p.PA5;
    let miso = p.PA6;
    let mosi = p.PA7;

    let busy = p.PA0;
    let nrst = p.PA1;
    let rxen = p.PA2;
    let dio1 = p.PA3;

    let mut rxen = Output::new(rxen, Level::Low, Default::default());
    // let mut txen = Output::new(txen, Level::Low, Default::default());
    // let dio1 = ExtiInput::new(dio1, p.EXTI3, hal::gpio::Pull::None);

    rxen.set_high();
    let mut dio1 = ExtiInput::new(dio1, p.EXTI3, hal::gpio::Pull::None);

    let led = p.PB12;
    //    let button = p.PC3;

    //let mut button = ExtiInput::new(button, p.EXTI3, hal::gpio::Pull::None);

    let mut led = Output::new(led, Level::Low, Default::default());

    //let busy = Input::new(busy, hal::gpio::Pull::None);
    let mut busy = ExtiInput::new(busy, p.EXTI0, hal::gpio::Pull::None);

    let cs = Output::new(cs.degrade(), Level::High, Default::default());

    let mut nrst = Output::new(nrst, Level::High, Default::default());

    let mut spi_config = hal::spi::Config::default();
    spi_config.frequency = Hertz::mhz(2);
    spi_config.mode = embedded_hal::spi::MODE_0;

    let spi = Spi::new(p.SPI1, sck, mosi, miso, NoDma, NoDma, spi_config);

    nrst.set_low();
    Timer::after_millis(120).await;
    nrst.set_high();
    Timer::after_millis(20).await;

    let mut sx1268 = SX1268::new(spi, cs, busy);

    sx1268.init(Config {}, &mut delay).unwrap();

    // sx1268.read_register(0x0320, &mut buf[..]).unwrap();
    // println!("buf: {:?}", buf);

    //let irq_status = sx1268.send_command(GetIrqStatus).unwrap();
    //println!("irq_status: {:?}", irq_status);

    println!("init ok");

    //let mut buf = [0u8; 16];
    //sx1268.read_register(regs::CHIP_REV, &mut buf).unwrap();
   // println!("CHIP_REV: {:?}", core::str::from_utf8(&buf[..]).unwrap());

    loop {
        Timer::after_millis(1000).await;
        println!("-------------");

        // active low
        led.set_low();

        sx1268.tx_bytes(b"Hello From ch32-hal!!!\0", &mut delay).unwrap();
        sx1268.busy.wait_for_low().await;
        println!("busy low");

        // dio1.wait_for_high().await;
        let irq_status = sx1268.get_irq_status().unwrap();
        //  println!("dio1 high");
        println!("irq_status: {:?}", irq_status);
        let status = sx1268.get_status().unwrap();
        println!("status: {:?}", status);

        let stats = sx1268.get_stats().unwrap();
        println!("stats: {:?}", stats);

        //println!("TODO: read");
        //println!("busy => {:?}", busy.is_high());
        led.set_high();
    }

    /*
    println!("begin rx");
    sx1268.set_rx(&mut Delay).unwrap();

    loop {
        Timer::after_millis(1000).await;
        println!("irq pin: {:?}", irq.is_high());

        let irq_status = sx1268.send_command(GetIrqStatus).unwrap();
        println!("irq_status: {:?}", irq_status);

        let status = sx1268.send_command(GetStatus).unwrap();
        println!("status: {:?}", status);

        // sx1268.read_register(0x0320, &mut buf[..]).unwrap();
        // println!("buf: {:?}", &buf[4..]);
        led.toggle();
    }
    */
}

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

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
