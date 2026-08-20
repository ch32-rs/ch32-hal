//! Blinky for nanoCH32H417 (V3F core, boot hart 0).
//!
//! LED0 = PF2 (active-high: PF2 → 1K → LED → GND).

#![no_std]
#![no_main]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;

    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
