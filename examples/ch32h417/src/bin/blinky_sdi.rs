//! Blinky + SDI Print for nanoCH32H417 (V3F core, boot hart 0).
//!
//! LED0 = PF2, SDI output via WCHLink serial.

#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use panic_halt as _;

#[ch32_hal::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    hal::println!("CH32H417 V3F booted");
    let chip = hal::signature::chip_id();
    hal::println!("Chip: {} (dev_id={:04x}, rev={:04x})", chip.name(), chip.dev_id(), chip.rev_id());
    hal::println!("Flash: {}KB", hal::signature::flash_size_kb());

    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    hal::println!("RCC configured — SysClk: Pll400MHse");

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();
        hal::println!("tick {}", counter);
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
