//! SSD1306 OLED demo over software I2C for CH32X035.
//!
//! Early CH32X033/X034 parts have no I2C peripheral, so the bus is
//! bit-banged on the pins the hardware I2C1 would normally use:
//! - SCL = PA10
//! - SDA = PA11
//!
//! External pull-ups are required; the bus runs at up to 400 kHz.
//!
//! ```text
//! cd examples/ch32x035 && cargo run --release --bin i2c_oled
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::gpio::Pull;
use ch32_hal::soft_i2c::{Config as SoftI2cConfig, SoftI2c};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use panic_halt as _;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    // Give the debugger a beat to attach and start the RTT session.
    Timer::after_millis(500).await;

    defmt::info!(
        "soft I2C: SCL=PA10 SDA=PA11, target 400 kHz (hclk {} Hz)",
        hal::rcc::clocks().hclk.0
    );

    let mut i2c = SoftI2c::new(
        p.PA10,
        p.PA11,
        SoftI2cConfig {
            frequency: 400_000,
            pull: Pull::None,
        },
    );

    defmt::info!("calibrated half period: {} loop iterations", i2c.half_period_loops());

    let mut oled_address = None;
    for address in 0x08..=0x77 {
        if i2c.blocking_write(address, &[]).is_ok() {
            defmt::info!("I2C device found at 0x{:02x}", address);
            oled_address.get_or_insert(address);
        }
    }

    let Some(address) = oled_address else {
        defmt::error!("no I2C device found; check the pull-ups and SCL/SDA wiring");
        loop {
            Timer::after_secs(1).await;
        }
    };

    if address != 0x3c && address != 0x3d {
        defmt::warn!("found 0x{:02x}, not an SSD1306 address; trying 0x3c", address);
    }

    let interface = if address == 0x3d {
        I2CDisplayInterface::new_alternate_address(i2c)
    } else {
        I2CDisplayInterface::new(i2c)
    };
    let mut display =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello CH32X035!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("soft I2C PA10/PA11", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();
    Text::with_baseline("no hardware I2C", Point::new(0, 32), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();
    defmt::info!("OLED updated");

    loop {
        Timer::after_millis(1000).await;
    }
}
