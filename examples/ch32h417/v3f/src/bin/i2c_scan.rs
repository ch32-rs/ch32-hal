//! I2C bus scanner for the CH32H417 (V3F core).
//!
//! Probes every 7-bit address and reports which ones ACK. The sensor is
//! wired to **PB6 = SCL**, **PB7 = SDA**, which is I2C1 (AF4) on this part.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --bin i2c_scan
//! ```

#![no_std]
#![no_main]

use hal::i2c::I2c;
use hal::println;
use hal::time::Hertz;
use {ch32_hal as hal, panic_halt as _};

/// First and last valid 7-bit address (0x00..=0x07 and 0x78..=0x7F are
/// reserved by the I2C specification).
const ADDR_FIRST: u8 = 0x08;
const ADDR_LAST: u8 = 0x77;

#[ch32_hal::entry]
fn main() -> ! {
    let p = hal::init(hal::Config::default());

    hal::debug::SDIPrint::enable();
    println!("I2C1 scan (SCL=PB6, SDA=PB7) @ 100kHz");

    let mut i2c = I2c::new_blocking(p.I2C1, p.PB6, p.PB7, Hertz::khz(100), Default::default());

    let mut delay = hal::delay::Delay;
    loop {
        let mut found = 0u32;

        // A zero-length write is a plain START + address + STOP, i.e. the
        // usual "is anything there?" probe (same idiom as embassy's
        // `i2c-scan-blocking` example). A missing device comes back as `Err`.
        for addr in ADDR_FIRST..=ADDR_LAST {
            if i2c.blocking_write(addr, &[]).is_ok() {
                println!("  found 0x{:02x}", addr);
                found += 1;
            }
        }

        match found {
            0 => println!("scan done: no devices"),
            n => println!("scan done: {} device(s)", n),
        }

        delay.delay_ms(2000);
    }
}
