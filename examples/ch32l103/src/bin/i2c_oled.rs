//! Minimal SSD1306 OLED demo for CH32L103.
//!
//! Pins (I2C2, fixed mapping):
//! - SCL = PB10
//! - SDA = PB11
//!
//! Draws two lines of text so you can confirm the bus and the panel are alive.
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin i2c_oled
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use hal::i2c::I2c;
use hal::time::Hertz;
use panic_halt as _;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("I2C2 OLED (SCL=PB10, SDA=PB11) @ 400kHz");

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz::khz(400), Default::default());

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello CH32L103!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("I2C2 PB10/PB11", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();
    defmt::info!("OLED updated");

    loop {
        Timer::after_millis(1000).await;
    }
}
