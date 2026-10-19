//! BME280 read by hart 1 (V5F), published through the shared mailbox.
//!
//! Woken by `v3f/src/bin/bme280.rs`. This half owns the sensor: it brings up
//! I2C1 on PB6/PB7 itself (`ch32_hal::init_secondary` reads the clock tree
//! back; the boot core's `hal::init()` must not run here) and stores each
//! compensated reading in the mailbox. It never
//! prints. The boot core is the only SDI writer and prints whatever this core
//! published, which is also what `xtask report` reads back.
//!
//! A sample is published by writing the three values first and the sequence
//! number last, so a reader that sees a new sequence number also sees the
//! values that belong to it.

#![no_std]
#![no_main]

use ch32h417_ipc::protocols::{
    Bme280, BME280_BMP, BME280_FAIL, BME280_INIT, BME280_OK,
};
use ch32h417_v5f::{cache, i2c, mailbox};
use core::sync::atomic::Ordering;
use edrv_bme280::blocking::BME280;
use panic_halt as _;

/// How long hart 1 waits between readings. A counted spin rather than a timer:
/// the boot core paces the printing, so this only has to be slower than one
/// print.
const SAMPLE_SPINS: u32 = 8_000_000;

#[qingke_rt::entry]
fn main() -> ! {
    cache::enable_icache();

    // The boot core programs the clock tree; this only reads it back. The
    // default config runs from HSI, so no HSE frequency is needed, and the
    // defaults leave the AFIO reset and the DMA interrupts to the boot core.
    let _p = unsafe { ch32_hal::init_secondary(Default::default()) };

    let mailbox = mailbox::<Bme280>();
    mailbox.payload.status.store(BME280_INIT, Ordering::Relaxed);

    // 100 kHz. `i2c::init` derives the bus clock from `rcc::clocks()`, which
    // `init_secondary` filled by reading the prescalers the boot core left.
    // Safety: this half is the only user of I2C1 and of PB6/PB7.
    if unsafe { i2c::init(100_000) }.is_err() {
        mailbox.payload.status.store(BME280_FAIL, Ordering::Release);
        loop {}
    }

    let mut sensor = BME280::new_primary(Bus);
    let mut delay = SpinDelay;
    match sensor.init() {
        Ok(()) => mailbox.payload.status.store(
            if sensor.is_bme280 { BME280_OK } else { BME280_BMP },
            Ordering::Release,
        ),
        Err(_) => {
            mailbox.payload.status.store(BME280_FAIL, Ordering::Release);
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
                .payload
                .temperature
                .store(reading.temperature as u32, Ordering::Relaxed);
            mailbox
                .payload
                .pressure
                .store(reading.pressure, Ordering::Relaxed);
            mailbox
                .payload
                .humidity
                .store(reading.humidity, Ordering::Relaxed);
            mailbox.payload.seq.store(seq, Ordering::Release);
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
/// rather than a timer: it only has to outlast one conversion, so a generous
/// overestimate is enough.
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
