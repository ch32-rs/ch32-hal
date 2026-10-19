//! BME280 printed by the boot core, read by hart 1.
//!
//! The sensor sits on PB6/PB7, which hart 1 drives itself — this half never
//! touches I2C. It brings the chip up, wakes hart 1 at the address
//! `v5f/memory.x` links it to, and prints the samples hart 1 publishes in the
//! shared mailbox. The mailbox is also what `xtask report` reads, so the
//! readings survive a run with no console attached:
//!
//! ```text
//! cargo xtask flash --example bme280 --dual-core
//! cargo xtask report --example bme280
//! cargo xtask run   --example bme280 --dual-core
//! ```
//!
//! Attaching the SDI console holds hart 1, so `run` shows only this core. The
//! samples it prints are the ones hart 1 stored before the console attached.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::{Bme280, BME280_FAIL, BME280_IDLE, BME280_NO_SAMPLE};
use core::sync::atomic::Ordering;

ipc::mailbox!(Bme280);
use hal::delay::Delay;
use qingke::pfic;
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    // The boot core owns the clock tree. Hart 1 reads the prescalers back and
    // must not see them change underneath it, so this runs before the wake.
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    // Shared RAM survives a reset; clear it so the report describes this run.
    let mailbox = mailbox();
    mailbox.clear();
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    hal::try_println!("BME280: hart 1 reads I2C1 (PB6/PB7), hart 0 prints");

    let mut printed = BME280_NO_SAMPLE;
    loop {
        let status = mailbox.payload.status.load(Ordering::Acquire);
        let seq = mailbox.payload.seq.load(Ordering::Acquire);

        if seq != printed && seq != BME280_NO_SAMPLE {
            // The sequence number is stored last, so these three belong to it.
            let temperature = mailbox.payload.temperature.load(Ordering::Relaxed) as i32;
            let pressure = mailbox.payload.pressure.load(Ordering::Relaxed);
            let humidity = mailbox.payload.humidity.load(Ordering::Relaxed);
            hal::try_println!(
                "[V5F] T={:.2}C  P={:.2}hPa  H={:.2}%",
                temperature as f32 / 100.0,
                pressure as f32 / 10_000.0,
                humidity as f32 / 100.0
            );
            printed = seq;
        } else if seq == BME280_NO_SAMPLE {
            // Nothing to print yet. Keep saying why, and keep re-issuing the
            // wake: hart 1's startup races with the debug link attaching.
            match status {
                BME280_FAIL => {
                    hal::try_println!("hart 1: sensor did not answer (PB6/PB7, addr 0x76)")
                }
                BME280_IDLE => {
                    unsafe { pfic::wake_other_core(V5F_ENTRY) };
                    hal::try_println!("waiting for hart 1");
                }
                _ => hal::try_println!("hart 1: sensor init in progress"),
            }
        }

        Delay.delay_ms(1000);
    }
}
