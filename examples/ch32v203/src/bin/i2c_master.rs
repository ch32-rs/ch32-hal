//! I2C master test rig for the CH32L103 slave example.
//!
//! Drives the register device emulated by
//! `examples/ch32l103/src/bin/i2c_slave.rs` at address 0x42 and logs a
//! PASS/FAIL line per check over defmt RTT (probe-rs).
//!
//! Wiring (V203 keeps its default I2C1 mapping; the L103 slave is on I2C2):
//!
//! ```text
//! V203 PB6 (SCL) <-> L103 PB10 (SCL)
//! V203 PB7 (SDA) <-> L103 PB11 (SDA)
//! V203 GND       <-> L103 GND
//! 4.7 kΩ pull-ups from SCL and SDA to 3.3 V
//! ```
//!
//! Each round starts with an address scan, so extra devices on the bus show
//! up by name and an address clash with 0x42/0x43 is visible immediately.
//!
//! ```text
//! cd examples/ch32v203 && cargo run --release --bin i2c_master
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use hal::i2c::{Error, I2c};
use hal::time::Hertz;
use panic_halt as _;
use qingke::riscv;

const SLAVE_ADDR: u8 = 0x42;
const WRONG_ADDR: u8 = 0x43;

const STATUS: u8 = 0x00;
const SCRATCH: u8 = 0x01;
const LED: u8 = 0x02;
const WHO_AM_I: u8 = 0x0F;

#[defmt::panic_handler]
fn panic() -> ! {
    loop {}
}

/// Short name for a transfer error. Formatting `Error` needs the optional
/// `defmt` feature of `ch32-hal`, and turning that on would make every binary
/// of this crate link a defmt logger; the test rig only needs a label.
fn err_name(error: Error) -> &'static str {
    match error {
        Error::Bus => "bus",
        Error::Arbitration => "arbitration",
        Error::Nack => "nack",
        Error::Timeout => "timeout",
        Error::Crc => "crc",
        Error::Overrun => "overrun",
        Error::ZeroLengthTransfer => "zero-length",
        Error::BufferTooLong => "buffer-too-long",
        Error::InvalidAddress => "invalid-address",
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    defmt::info!("I2C master test rig, slave at 0x{:02X}", SLAVE_ADDR);

    // I2C1 default mapping: PB6 = SCL, PB7 = SDA.
    let mut i2c = I2c::new_blocking::<0>(p.I2C1, p.PB6, p.PB7, Hertz::khz(100), Default::default());

    // Give the slave firmware time to arm its first `listen()`.
    riscv::asm::delay(10_000_000);

    loop {
        let mut failed = 0u32;

        // Without pull-ups the bus floats; check the idle levels so a wiring
        // problem is obvious before any transaction.
        let indr = hal::pac::GPIOB.indr().read();
        let (scl, sda) = (indr.idr(6), indr.idr(7));
        defmt::info!(
            "bus idle: SCL={} SDA={}",
            if scl { "high" } else { "low" },
            if sda { "high" } else { "low" }
        );

        // List every address that ACKs: extra devices on the bus are easy to
        // identify, and a clash with 0x42/0x43 shows up before the tests.
        let mut found = 0u32;
        for addr in 0x08..=0x77 {
            if i2c.blocking_write(addr, &[]).is_ok() {
                defmt::info!("  found 0x{:02X}", addr);
                found += 1;
            }
        }
        if found == 0 {
            defmt::info!("scan: no devices");
        }

        // A read must return the slave's WHO_AM_I register.
        let mut who_am_i = [0u8; 1];
        match i2c.blocking_write_read(SLAVE_ADDR, &[WHO_AM_I], &mut who_am_i) {
            Ok(()) => {
                defmt::info!("WHO_AM_I = 0x{:02X}", who_am_i[0]);
                if who_am_i[0] == SLAVE_ADDR {
                    defmt::info!("PASS who_am_i");
                } else {
                    failed += 1;
                    defmt::warn!("FAIL who_am_i");
                }
            }
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL who_am_i: {}", err_name(e));
            }
        }

        // A write must land in the slave's register file.
        match i2c.blocking_write(SLAVE_ADDR, &[SCRATCH, 0x5A]) {
            Ok(()) => defmt::info!("PASS write_scratch"),
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL write_scratch: {}", err_name(e));
            }
        }

        let mut scratch = [0u8; 1];
        match i2c.blocking_write_read(SLAVE_ADDR, &[SCRATCH], &mut scratch) {
            Ok(()) => {
                defmt::info!("SCRATCH = 0x{:02X}", scratch[0]);
                if scratch[0] == 0x5A {
                    defmt::info!("PASS read_scratch");
                } else {
                    failed += 1;
                    defmt::warn!("FAIL read_scratch");
                }
            }
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL read_scratch: {}", err_name(e));
            }
        }

        // The slave serves 8 bytes from the register pointer; asking for 16
        // must yield the surplus as 0x00 padding.
        let mut over_read = [0u8; 16];
        match i2c.blocking_write_read(SLAVE_ADDR, &[STATUS], &mut over_read) {
            Ok(()) => {
                defmt::info!("over-read {:02X}", over_read);
                let padded = over_read[8..] == [0u8; 8];
                let scratch_visible = over_read[1] == 0x5A;
                if padded && scratch_visible {
                    defmt::info!("PASS over_read");
                } else {
                    failed += 1;
                    defmt::warn!("FAIL over_read (pad={}, scratch={})", padded, scratch_visible);
                }
            }
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL over_read: {}", err_name(e));
            }
        }

        // A write of only the register pointer (zero data bytes) is legal.
        match i2c.blocking_write(SLAVE_ADDR, &[STATUS]) {
            Ok(()) => defmt::info!("PASS zero_length_write"),
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL zero_length_write: {}", err_name(e));
            }
        }

        // The slave must not acknowledge addresses it is not configured for.
        match i2c.blocking_write(WRONG_ADDR, &[SCRATCH, 0x00]) {
            Err(Error::Nack) => defmt::info!("PASS address_nack"),
            Ok(()) => {
                failed += 1;
                defmt::warn!("FAIL address_nack (unexpected ACK)");
            }
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL address_nack: {}", err_name(e));
            }
        }

        // The LED register must survive a write/read round trip.
        match i2c.blocking_write(SLAVE_ADDR, &[LED, 0x01]) {
            Ok(()) => match i2c.blocking_write_read(SLAVE_ADDR, &[LED], &mut scratch) {
                Ok(()) => {
                    defmt::info!("LED = 0x{:02X}", scratch[0]);
                    if scratch[0] == 0x01 {
                        defmt::info!("PASS led_register");
                    } else {
                        failed += 1;
                        defmt::warn!("FAIL led_register");
                    }
                }
                Err(e) => {
                    failed += 1;
                    defmt::warn!("FAIL led_register: {}", err_name(e));
                }
            },
            Err(e) => {
                failed += 1;
                defmt::warn!("FAIL led_register: {}", err_name(e));
            }
        }
        let _ = i2c.blocking_write(SLAVE_ADDR, &[LED, 0x00]);

        if failed == 0 {
            defmt::info!("round: all checks passed");
        } else {
            defmt::warn!("round: {} check(s) failed", failed);
        }

        riscv::asm::delay(50_000_000);
    }
}
