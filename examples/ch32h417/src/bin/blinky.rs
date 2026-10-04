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
    let config = hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    };
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;

    loop {
        led.toggle();
        delay.delay_ms(500);
    }
}
