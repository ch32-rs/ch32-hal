//! VL53L0X time-of-flight distance sensor demo, **blocking** flavour, for the
//! CH32V208.
//!
//! Sync counterpart of the `vl53l0x` example: same sensor, same `edrv-vl53l0x`
//! driver, but the blocking API and a polled I2C bus - no embassy executor, no
//! async, no DMA.
//!
//! Wiring is identical to `vl53l0x`:
//!
//! ```text
//!   VL53L0X breakout     CH32V208
//!   VIN / VCC  <----  3V3     (3.3V part - NOT 5V)
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   XSHUT      ----   must be HIGH, or every access NACKs
//! ```
//!
//! Everything about identifying the chip, the three ST calibrations, the 0x28
//! offset register and what the numbers mean is documented once in the
//! `vl53l0x` example. This file shows the sync call pattern and, deliberately,
//! the same raw-register offset write - the blocking driver exposes
//! `read_reg` / `write_reg` too.
//!
//! # The tradeoff
//!
//! Blocking I2C is a good fit for a single-shot ranger: start, wait ~33 ms, read
//! is a naturally sequential operation, and the driver does the waiting
//! internally. The cost is that a missing sensor blocks forever instead of being
//! reported as a timeout.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_vl53l0x::blocking::VL53L0X;
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, print, println};
use panic_halt as _;
use qingke::riscv;

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Blocking>;
type Sensor = VL53L0X<I2cBus>;

/// The VL53L0X handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// Pace between single-shot measurements.
const SAMPLE_PERIOD_MS: u32 = 200;

/// Anything at or above this is the sensor's "no target" marker, not a distance.
const NO_TARGET_MM: u16 = 8000;

/// The model ID a real VL53L0X returns from register 0xC0.
const VL53L0X_MODEL_ID: u8 = 0xEE;

/// The model ID of the lookalike that shares the address but not the registers.
const VL53L1X_MODEL_ID: u8 = 0xEA;

/// `VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM`, a 16-bit register.
const REG_RANGE_OFFSET_MM: u8 = 0x28;

/// Correction subtracted from every reading, in mm. **This is the knob to set.**
///
/// See `vl53l0x` for how to measure it and why it lives in firmware rather than
/// only in the sensor.
const RANGE_OFFSET_MM: i32 = 30;

/// Also program `RANGE_OFFSET_MM` into the sensor's own offset register.
const WRITE_OFFSET_TO_SENSOR: bool = false;

/// Rough proximity band, purely for readability.
fn proximity(mm: u16) -> &'static str {
    match mm {
        0..=50 => "touching",
        51..=150 => "near",
        151..=500 => "mid",
        501..=1200 => "far",
        _ => "edge of range",
    }
}

/// Encode a millimetre offset the way ST's API does: 12-bit two's complement in
/// units of 0.25 mm, with the sign flipped relative to `RANGE_OFFSET_MM`.
fn encode_offset(offset_mm: i32) -> u16 {
    let quarter_mm = (offset_mm * 4) as u32;
    (quarter_mm & 0x0FFF) as u16
}

/// Send `offset_mm` to the sensor's offset register.
fn write_offset_register(sensor: &mut Sensor, offset_mm: i32) -> Result<(), edrv_vl53l0x::Error<hal::i2c::Error>> {
    let encoded = encode_offset(-offset_mm);
    sensor.write_reg(REG_RANGE_OFFSET_MM, (encoded & 0x00FF) as u8)?;
    sensor.write_reg(REG_RANGE_OFFSET_MM + 1, (encoded >> 8) as u8)?;

    // The register stores `actual - measured`, i.e. the opposite sign to
    // `RANGE_OFFSET_MM`; print it as the sensor sees it.
    println!(
        "  register 0x{:02X} <- 0x{:04X} (stored as {:+} mm)",
        REG_RANGE_OFFSET_MM,
        encoded,
        -offset_mm
    );
    Ok(())
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 VL53L0X ToF distance sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  3V3 part - do not power it from 5V");
    println!("  driver: edrv-vl53l0x (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2cBus = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, I2C_FREQ, Default::default());
    let mut sensor = VL53L0X::new_primary(i2c);
    let mut delay = Delay;

    // `init` reads model register 0xC0 and runs the standard init sequence,
    // which includes the reference (VHV + phase) calibration.
    match sensor.init() {
        Ok(()) => {
            println!(
                "model register 0xC0 = 0x{:02X} -> VL53L0X detected, init done",
                VL53L0X_MODEL_ID
            );
            println!("");
        }
        Err(edrv_vl53l0x::Error::InvalidDevice(id)) => {
            println!(
                "model register 0xC0 = 0x{:02X}, expected 0x{:02X}.",
                id, VL53L0X_MODEL_ID
            );
            println!("");
            if id == VL53L1X_MODEL_ID {
                println!("That is a **VL53L1X**, not a VL53L0X: same address, different");
                println!("register map and init sequence. It needs its own driver.");
            } else {
                println!("Not a VL53L0X. Check SDA/SCL are not swapped and that XSHUT is");
                println!("pulled HIGH.");
            }
            park();
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            println!("Check 3V3, GND, SDA/SCL orientation and that XSHUT is HIGH.");
            park();
        }
    }

    if WRITE_OFFSET_TO_SENSOR {
        if let Err(e) = write_offset_register(&mut sensor, RANGE_OFFSET_MM) {
            println!("could not program the offset register: {:?}", e);
        }
    }

    if RANGE_OFFSET_MM != 0 {
        println!(
            "applying a software offset of {:+} mm to every reading",
            RANGE_OFFSET_MM
        );
        println!("");
    }

    let mut count: u32 = 0;
    let mut closest: Option<u16> = None;
    let mut misses: u32 = 0;

    loop {
        count += 1;

        match sensor.read_range_single_millimeters() {
            Ok(mm) if mm >= NO_TARGET_MM => {
                misses += 1;
                println!(
                    "#{:<4} no target in range (raw {}, the {}-mm marker)   [{} misses]",
                    count, mm, NO_TARGET_MM, misses
                );
            }
            Ok(raw) => {
                // Same correction the sensor's own offset register would apply.
                let mm = (raw as i32 - RANGE_OFFSET_MM).max(0) as u16;
                closest = Some(closest.map_or(mm, |c| c.min(mm)));
                print!(
                    "#{:<4} {:>4} mm  ({:>3} cm)  {:<14} ",
                    count,
                    mm,
                    mm / 10,
                    proximity(mm)
                );
                match closest {
                    Some(c) => print!("closest {} mm   ", c),
                    None => print!("closest --       "),
                }
                println!("[{} ok / {} miss]", count - misses, misses);
            }
            Err(e) => {
                misses += 1;
                println!("#{:<4} read error: {:?}   [{} misses]", count, e, misses);
            }
        }

        delay.delay_ms(SAMPLE_PERIOD_MS);
    }
}

/// Park forever after a fatal setup problem.
fn park() -> ! {
    loop {
        riscv::asm::delay(50_000_000);
    }
}
