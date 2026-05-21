//! Minimal blinky for CH32H417 (V3F core, boot hart 0).
//!
//! Verifies the H4 RCC + GPIO + delay chain compiles and produces a
//! plausible bitstream. The pin choice (`PE0`) matches the EVT
//! board's user LED — adjust for custom boards.

#![no_std]
#![no_main]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    // SDK's 400 MHz HSE recipe — V5F = 400 MHz, V3F (us) = 100 MHz.
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PE0, Level::Low, Default::default());
    let mut delay = Delay;

    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
