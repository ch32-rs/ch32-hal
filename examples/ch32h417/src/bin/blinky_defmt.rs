//! Blinky with defmt-rtt for nanoCH32H417 (V3F core, boot hart 0).
//!
//! LED0 = PF2. RTT output via WCHLink + probe-rs.

#![no_std]
#![no_main]

use ch32_hal as hal;
use core::panic::PanicInfo;
extern crate defmt_rtt;
use hal::delay::Delay;
use hal::gpio::{Level, Output};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[export_name = "_defmt_panic"]
fn defmt_panic() -> ! {
    loop {}
}

defmt::timestamp! {"{=u32:us}", {
    0 // TODO: use cycle counter once mcycle is verified on V3F
}}

#[ch32_hal::entry]
fn main() -> ! {
    defmt::info!("CH32H417 V3F booted");

    let chip = hal::signature::chip_id();
    defmt::info!("Chip: {} (dev_id={:04x}, rev={:04x})", chip.name(), chip.dev_id(), chip.rev_id());
    defmt::info!("Flash: {}KB", hal::signature::flash_size_kb());

    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    defmt::info!("RCC configured — SysClk: Pll400MHse");

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();
        defmt::info!("tick {}", counter);
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
