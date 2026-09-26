//! SSD1306 128x64 OLED demo for the CH32V208, written in embassy style.
//!
//! Wiring (CH32V208, I2C2):
//!
//! ```text
//!   PB10 -> SCL   (I2C2_SCL, AF open-drain)
//!   PB11 -> SDA   (I2C2_SDA, AF open-drain)
//!   3V3  -> VCC
//!   GND  -> GND
//! ```
//!
//! Both bus lines are open-drain and need external pull-ups to 3.3V (typically
//! 4.7k at 100 kHz, 2.2k at 400 kHz). Most SSD1306 modules carry them already.
//!
//! The panel is driven in [`Async`] mode with the I2C2 event/error interrupts
//! bound and DMA1 channel 4 (TX) / channel 5 (RX), through the `ssd1306` crate
//! plus `embedded-graphics`. The module address is the usual `0x3C`
//! (`i2c_detect` reports it); the alternate `0x3D` is the datasheet default.
//!
//! Draw the frame into the 1 KiB buffer, then `flush().await` it out.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Baseline, Text};
use hal::i2c::{ErrorInterruptHandler, EventInterruptHandler, I2c};
use hal::mode::Async;
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals, println};
use heapless::String;
use panic_halt as _;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306Async};

bind_interrupts!(struct Irqs {
    I2C2_EV => EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => ErrorInterruptHandler<peripherals::I2C2>;
});

/// I2C2 bus: PB10 = SCL, PB11 = SDA, DMA1_CH4 (TX) / DMA1_CH5 (RX).
type I2cBus = I2c<'static, peripherals::I2C2, Async>;

/// Bus speed. The SSD1306 is specified up to 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// 7-bit address of the module (`0x3D` is the alternate).
const SSD1306_ADDR: u8 = 0x3C;

/// Frame pacing for the animation.
const FRAME_PERIOD: Duration = Duration::from_millis(20);

/// Side of the bouncing square, in pixels.
const BOX_SIDE: i32 = 12;

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 SSD1306 128x64 demo");
    println!("  I2C2: SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());

    // REMAP is inferred as 0 from the PB10/PB11 impls (no remap on these pins).
    let i2c: I2cBus = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        I2C_FREQ,
        Default::default(),
    );

    let interface = I2CDisplayInterface::new_custom_address(i2c, SSD1306_ADDR);
    let mut display =
        Ssd1306Async::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    display.init().await.unwrap();
    println!("display init ok");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let box_style = PrimitiveStyle::with_fill(BinaryColor::On);

    // "addr 0x3C" without hand-formatting the constant.
    let mut addr_line = String::<24>::new();
    let _ = write!(addr_line, "addr 0x{:02X}  i2c", SSD1306_ADDR);

    let mut x: i32 = 0;
    let mut dx: i32 = 4;

    loop {
        display.clear(BinaryColor::Off).unwrap();

        Text::with_baseline("CH32V208 OLED", Point::new(0, 0), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline("SSD1306 128x64", Point::new(0, 12), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline("I2C2 PB10/PB11", Point::new(0, 24), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline(addr_line.as_str(), Point::new(0, 36), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Rectangle::new(Point::new(x, 52), Size::new(BOX_SIDE as u32, BOX_SIDE as u32))
            .into_styled(box_style)
            .draw(&mut display)
            .unwrap();

        display.flush().await.unwrap();

        x += dx;
        if x <= 0 || x >= 128 - BOX_SIDE {
            dx = -dx;
        }

        Timer::after(FRAME_PERIOD).await;
    }
}
