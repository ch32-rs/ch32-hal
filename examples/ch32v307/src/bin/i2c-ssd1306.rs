//! I2C test with SSD1306
//!
//! Folowing pins are used:
//! SDA     PB11
//! SCL     PB10
//!
//! Depending on your target and the board you are using you have to change the pins.
//!
//! For this example you need to hook up an SSD1306 I2C display.
//! The display will flash black and white.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;

use ch32_hal as hal;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use hal::i2c::I2c;
use hal::println;
use hal::time::Hertz;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = hal::init(Default::default());
    hal::embassy::init();

    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    println!("init ok");
    let i2c = I2c::new_blocking(
        p.I2C2,
        i2c_scl,
        i2c_sda,
        Hertz::khz(400),
        Default::default(),
    );

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    loop {}
}