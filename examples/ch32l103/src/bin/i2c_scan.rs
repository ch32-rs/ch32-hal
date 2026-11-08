//! I2C bus scanner for CH32L103.
//!
//! Probes every 7-bit address and reports which ones ACK.
//! Uses **I2C2** on its fixed pins: **PB10 = SCL**, **PB11 = SDA**.
//! (I2C1's default PB6/PB7 share the USBPD CC lines, which reset with
//! Rd enabled and fight external pull-ups.)
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin i2c_scan
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::i2c::I2c;
use hal::time::Hertz;
use panic_halt as _;

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

/// First and last valid 7-bit address (0x00..=0x07 and 0x78..=0x7F are
/// reserved by the I2C specification).
const ADDR_FIRST: u8 = 0x08;
const ADDR_LAST: u8 = 0x77;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    defmt::info!("I2C2 scan (SCL=PB10, SDA=PB11) @ 100kHz");

    let mut i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Hertz::khz(100), Default::default());

    loop {
        let mut found = 0u32;

        // A zero-length write is a plain START + address + STOP, i.e. the
        // usual "is anything there?" probe. A missing device comes back as Err.
        for addr in ADDR_FIRST..=ADDR_LAST {
            if i2c.blocking_write(addr, &[]).is_ok() {
                defmt::info!("  found 0x{:02x}", addr);
                found += 1;
            }
        }

        match found {
            0 => defmt::info!("scan done: no devices"),
            n => defmt::info!("scan done: {} device(s)", n),
        }

        Timer::after_millis(2000).await;
    }
}
