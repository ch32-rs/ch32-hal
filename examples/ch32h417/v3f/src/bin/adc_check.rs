//! ADC1 blocking self-check (V3F only).
//!
//! Calibrates ADC1 and reads **PA1 = IN1** with the longest sample time, then
//! the internal VREFINT and temperature channels. Mailbox (`AdcCheck`): stage
//! 1 = init returned, 2 = calibration and every read completed; `failures`
//! counts failed checks and the last three fields hold the raw 12-bit results.
//!
//! No analog source is required: the check only proves that calibration
//! finishes and each conversion returns a value inside the 12-bit range.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::AdcCheck;
use hal::adc::{Adc, SampleTime, ADC_MAX};

ipc::mailbox!(AdcCheck);
use panic_halt as _;

fn check(failures: &mut u32, condition: bool) {
    *failures += u32::from(!condition);
}

fn in_range(value: u16) -> bool {
    u32::from(value) <= ADC_MAX
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut p = hal::init(hal::Config::default());

    let mut failures = 0u32;
    let mut adc = match Adc::new(p.ADC1, hal::adc::Config::default()) {
        Ok(adc) => adc,
        Err(_) => {
            mailbox.payload.failures.store(1, Relaxed);
            loop {
                hal::delay::Delay.delay_ms(1000);
            }
        }
    };

    let pa1 = match adc.blocking_read(&mut p.PA1, SampleTime::S239_5) {
        Ok(value) => value,
        Err(_) => {
            failures += 1;
            0
        }
    };
    check(&mut failures, in_range(pa1));

    let mut vref = adc.enable_vref();
    let vref_value = match adc.blocking_read(&mut vref, SampleTime::S239_5) {
        Ok(value) => value,
        Err(_) => {
            failures += 1;
            0
        }
    };
    check(&mut failures, in_range(vref_value));

    let mut temperature = adc.enable_temperature();
    let temperature_value = match adc.blocking_read(&mut temperature, SampleTime::S239_5) {
        Ok(value) => value,
        Err(_) => {
            failures += 1;
            0
        }
    };
    check(&mut failures, in_range(temperature_value));

    mailbox.payload.pa1.store(u32::from(pa1), Relaxed);
    mailbox.payload.vref.store(u32::from(vref_value), Relaxed);
    mailbox.payload.temperature.store(u32::from(temperature_value), Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);

    loop {
        hal::delay::Delay.delay_ms(1000);
    }
}
