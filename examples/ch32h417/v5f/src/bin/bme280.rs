//! BME280 read by hart 1 (V5F), published through the shared mailbox.
//!
//! Woken by `v3f/src/bin/bme280.rs`. This half owns the sensor: it brings up
//! I2C1 on PB6/PB7 itself (metapac only — the boot core's `hal::init()` must
//! not run here) and stores each compensated reading in the mailbox. It never
//! prints. The boot core is the only SDI writer and prints whatever this core
//! published, which is also what `xtask report` reads back.
//!
//! A sample is published by writing the three values first and the sequence
//! number last, so a reader that sees a new sequence number also sees the
//! values that belong to it.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_v5f::{cache, i2c, mailbox};
use core::sync::atomic::Ordering;
use edrv_bme280::blocking::BME280;
use panic_halt as _;

/// How long hart 1 waits between readings. It has no HAL delay (that lives in
/// the boot core's image), so this is a counted spin. The boot core paces the
/// printing; this only has to be slower than one print.
const SAMPLE_SPINS: u32 = 8_000_000;

#[qingke_rt::entry]
fn main() -> ! {
    cache::enable_icache();

    let mailbox = mailbox();
    mailbox.bme280_status.store(ipc::BME280_INIT, Ordering::Relaxed);

    // 100 kHz: the bus clock is derived from the prescalers the boot core left
    // behind, and standard mode tolerates that estimate being a step off.
    // Safety: this half is the only user of I2C1 and of PB6/PB7.
    if unsafe { i2c::init(100_000) }.is_err() {
        mailbox.bme280_status.store(ipc::BME280_FAIL, Ordering::Release);
        loop {}
    }

    let mut sensor = BME280::new_primary(Bus);
    let mut delay = SpinDelay;
    match sensor.init() {
        Ok(()) => mailbox.bme280_status.store(
            if sensor.is_bme280 {
                ipc::BME280_OK
            } else {
                ipc::BME280_BMP
            },
            Ordering::Release,
        ),
        Err(_) => {
            mailbox
                .bme280_status
                .store(ipc::BME280_FAIL, Ordering::Release);
            loop {}
        }
    }

    let mut seq = 0u32;
    loop {
        if let Ok(reading) = sensor.read_measurement(&mut delay) {
            seq = seq.wrapping_add(1);
            // Values before the sequence number: the reader treats a changed
            // sequence as "this sample is complete".
            mailbox
                .bme280_temperature
                .store(reading.temperature as u32, Ordering::Relaxed);
            mailbox
                .bme280_pressure
                .store(reading.pressure, Ordering::Relaxed);
            mailbox
                .bme280_humidity
                .store(reading.humidity, Ordering::Relaxed);
            mailbox.bme280_seq.store(seq, Ordering::Release);
        }
        delay_spins(SAMPLE_SPINS);
    }
}

fn delay_spins(n: u32) {
    for _ in 0..n {
        core::hint::spin_loop();
    }
}

/// The delay the BME280 driver uses to wait out a conversion. A counted spin
/// rather than a timer: hart 1's SysTick is free, but its divider lives in the
/// boot core's clock cache, which this image cannot see.
struct SpinDelay;

impl embedded_hal::delay::DelayNs for SpinDelay {
    fn delay_ns(&mut self, ns: u32) {
        // Roughly one spin per cycle at a few hundred MHz; far more than `ns`
        // asks for, which only makes the conversion wait conservative.
        delay_spins(ns.max(1));
    }
}

/// `edrv-bme280`'s bus, wired to the metapac master in `ch32h417_v5f::i2c`.
struct Bus;

impl embedded_hal::i2c::ErrorType for Bus {
    type Error = i2c::Error;
}

impl embedded_hal::i2c::I2c for Bus {
    fn transaction(
        &mut self,
        address: embedded_hal::i2c::SevenBitAddress,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        // The driver only ever does a register write or a write-then-read.
        match operations {
            [embedded_hal::i2c::Operation::Write(bytes)] => i2c::write(address, bytes),
            [embedded_hal::i2c::Operation::Write(bytes), embedded_hal::i2c::Operation::Read(buf)] => {
                i2c::write_read(address, bytes, buf)
            }
            _ => Err(i2c::Error::Overrun),
        }
    }
}
