//! defmt RTT logging demo for CH32H417 V3F (legacy binary name).
//!
//! `cargo xtask run --example sdi_print`

#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use ch32_hal as hal;
use panic_halt as _;

#[ch32_hal::entry]
fn main() -> ! {
    // Brings up clocks (POR HSI 25M) and the systick-based delay.
    let _p = hal::init(hal::Config::default());

    log!("hello world from CH32H417 (V3F)!");

    log!("Flash size: {}kb", hal::signature::flash_size_kb());
    log!("Chip UID: {:x?}", hal::signature::unique_id());
    let chip_id = hal::signature::chip_id();
    log!("Chip {}, DevID: 0x{:x}", chip_id.name(), chip_id.dev_id());

    let mut n: u32 = 0;
    loop {
        log!("tick {}", n);
        n = n.wrapping_add(1);

        hal::delay::Delay.delay_ms(1000);
    }
}
