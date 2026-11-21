//! VL53L0X time-of-flight distance sensor demo for the CH32V208.
//!
//! Uses **our own** `edrv-vl53l0x` driver, in its async flavour over ch32-hal's
//! async I2C. The blocking flavour is exercised by the companion
//! `vl53l0x_blocking` example.
//!
//! Wiring (I2C2):
//!
//! ```text
//!   VL53L0X breakout     CH32V208
//!   VIN / VCC  <----  3V3     (this is a 3.3V part - NOT 5V)
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   XSHUT      ----   must be HIGH (most breakouts pull it up; if in doubt,
//!                     wire it to 3V3 - held low, the chip stays in reset and
//!                     every access NACKs)
//! ```
//!
//! Pull-ups on SCL/SDA are needed, as always; most VL53L0X breakouts carry them.
//!
//! # Identifying the chip
//!
//! `0x29` is shared by several parts, so the address alone does not identify
//! this sensor - see `i2c_identify`. `init` validates model register `0xC0` and
//! refuses to run if it is not `0xEE`, which is what makes an accidental
//! **VL53L1X** (`0xEA`) fail loudly instead of misbehaving: the L1X has a
//! different register map and init sequence.
//!
//! # Calibration
//!
//! The VL53L0X supports three calibrations, and ST's API defines all three:
//!
//! | calibration | fixes | API |
//! |---|---|---|
//! | reference (VHV + phase) | the device's own reference | `VL53L0X_PerformRefCalibration` |
//! | **offset** | a **fixed** range error in mm | `VL53L0X_PerformOffsetCalibration` |
//! | crosstalk | reflections from a **cover glass** | `VL53L0X_PerformXTalkCalibration` |
//!
//! The reference calibration is already run by the driver during init. For a
//! reading that is consistently *too large*, the relevant one is the **offset**.
//!
//! Two things matter before reaching for it:
//!
//! 1. **Offset calibration only fixes a constant error.** If the sensor reads
//!    30 mm high at both 200 mm and 1000 mm, that is an offset. If it reads 5%
//!    high at every distance, that is the part's normal spread (the datasheet
//!    quotes a few percent) and no offset can remove it. Measure at two
//!    distances before concluding.
//! 2. **The offset does not persist in the sensor.** ST's own API documentation
//!    requires the application to read it back on first power-up and re-apply it
//!    on every later init, so it has to live in your firmware either way.
//!
//! Register `0x0028` (`VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM`) holds the
//! value as a **12-bit two's complement number in 10.2 fixed point**, i.e. in
//! units of 0.25 mm, covering about +/-511 mm - it is not a plain millimetre
//! count. ST encodes it as `offset_um / 250` (adding 4096 first when negative)
//! and only the low 12 bits are read back.
//!
//! Note the sign if you ever write that register yourself: ST stores
//! `offset = actual - measured`, so a sensor reading *high* needs a **negative**
//! register value. That is the opposite sign to `RANGE_OFFSET_MM` below.
//!
//! ## Our driver can do what the third-party crate could not
//!
//! Because the host must re-apply the value on every boot regardless, a software
//! subtraction is numerically equivalent for a simple distance reading (the
//! device-side version additionally feeds the chip's internal threshold checks),
//! so this demo does the subtraction in firmware via `RANGE_OFFSET_MM`.
//!
//! Writing the register is nevertheless possible here, unlike with the
//! third-party `vl53l0x` crate this example used to use: `edrv-vl53l0x` exposes
//! `read_reg` / `read_regs` / `write_reg`, so `write_offset_register` below can
//! program register `0x28` directly. Set `WRITE_OFFSET_TO_SENSOR` to `true` to
//! send the same constant to the chip as well.
//!
//! ## What the other drivers actually do
//!
//! Before investing in calibration code, it is worth knowing that **no
//! mainstream third-party driver implements offset calibration**: Adafruit's C++
//! library, Pololu's Arduino library and ESPHome's component never touch the
//! offset register, and CircuitPython only declares the `0x28` constant without
//! ever writing it. Only ST's full ULD (`vl53l0x_api_calibration.c`) does, via a
//! procedure that zeroes the offset, disables the TCC sequence step and the
//! range-ignore threshold, averages 50 measurements that report
//! `RangeStatus == 0`, and then stores `(cal_distance - mean) * 1000` microns.
//!
//! ## Applying the offset
//!
//! `RANGE_OFFSET_MM` is the single knob: it is subtracted from every reading.
//! It defaults to **30** because the module used to develop this example reads
//! about 3 cm high, constant across the working range.
//!
//! ## Measuring the offset
//!
//! Set `OFFSET_CHECK_DISTANCE_MM` to a distance you can set up accurately, put a
//! **flat, matte, light-coloured** target exactly that far from the sensor face,
//! then run. The demo averages 50 readings and prints the offset to enter; it
//! measures the raw sensor output, so `RANGE_OFFSET_MM` does not have to be
//! zeroed first. Precautions: no direct sunlight or incandescent light on the
//! target, target perpendicular to the sensor, and the module at least ~30 mm
//! away.
//!
//! # What the numbers mean
//!
//! * Readings are millimetres, roughly 30 mm to ~1200 mm; past that, accuracy
//!   depends heavily on the target's reflectivity.
//! * `8190` is the "no target / out of range" marker, not a distance. It appears
//!   when nothing is in view, in bright sunlight, or against a dark angled
//!   target.
//! * The measurement is single-shot: start, wait ~33 ms, read. The loop paces
//!   itself at 200 ms.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_vl53l0x::{Error as Vl53Error, VL53L0X};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::i2c::I2c;
use hal::mode::Async;
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals, print, println};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C2_EV => hal::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Async>;
type Sensor = VL53L0X<I2cBus>;

/// The VL53L0X handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// Pace between single-shot measurements.
const SAMPLE_PERIOD: Duration = Duration::from_millis(200);

/// Anything at or above this is the sensor's "no target" marker, not a distance.
const NO_TARGET_MM: u16 = 8000;

/// The model ID a real VL53L0X returns from register 0xC0.
const VL53L0X_MODEL_ID: u8 = 0xEE;

/// The model ID of the lookalike that shares the address but not the registers.
const VL53L1X_MODEL_ID: u8 = 0xEA;

/// `VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM`, a 16-bit register.
///
/// `edrv-vl53l0x` does not name this one because the driver itself never touches
/// it, so the example declares it - which is the point: the raw register access
/// it needs is public.
const REG_RANGE_OFFSET_MM: u8 = 0x28;

/// Correction subtracted from every reading, in mm. **This is the knob to set.**
///
/// Positive means "the sensor reads this much too high". The unit used for this
/// example reads about **+30 mm** across the working range, so 30 is the default
/// here; 3 cm is a typical part-to-part offset and it is constant, which is
/// exactly what a single constant can remove.
///
/// This lives in firmware rather than only in the sensor's register on purpose:
/// ST's API requires the host to re-apply the offset after every power-up
/// anyway, so the register alone would not save the firmware from carrying the
/// value (see `WRITE_OFFSET_TO_SENSOR`).
///
/// If your error is not constant in the range you care about, this will not fix
/// it - that is proportional spread, not an offset.
///
/// Find the value for your own unit by setting `OFFSET_CHECK_DISTANCE_MM` below
/// and running once. That check always reports the sensor's *raw* offset, so
/// this constant does not need to be zeroed first.
const RANGE_OFFSET_MM: i32 = 30;

/// Also program `RANGE_OFFSET_MM` into the sensor's own offset register.
///
/// Off by default: the firmware subtraction below already applies exactly the
/// same correction, and doing it in software is far easier to inspect. Turning
/// this on additionally feeds the value into the chip's internal threshold
/// checks, which is what ST's own API does.
const WRITE_OFFSET_TO_SENSOR: bool = false;

/// Set to a known target distance (mm) to run an offset measurement instead of
/// the normal loop. `None` is normal operation.
///
/// The target must be flat, matte and light-coloured, exactly this far from the
/// sensor face, perpendicular, with no sunlight or incandescent light on it.
const OFFSET_CHECK_DISTANCE_MM: Option<u16> = None;

/// Readings averaged by the offset check.
const OFFSET_CHECK_SAMPLES: u32 = 50;

/// A spread wider than this makes an offset measurement untrustworthy.
const OFFSET_CHECK_MAX_SPREAD_MM: u16 = 20;

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

/// Encode a millimetre offset the way ST's API does.
///
/// The register holds a 12-bit two's complement value in units of 0.25 mm, so
/// millimetres are multiplied by 4 and wrapped into 12 bits. The sign convention
/// is inverted relative to `RANGE_OFFSET_MM`: the register stores
/// `actual - measured`, so a sensor that reads *high* needs a *negative* value.
fn encode_offset(offset_mm: i32) -> u16 {
    let quarter_mm = (offset_mm * 4) as u32;
    (quarter_mm & 0x0FFF) as u16
}

/// Send `offset_mm` to the sensor's offset register.
///
/// `edrv-vl53l0x` exposes byte-granular register access, so the 16-bit write is
/// two `write_reg` calls; only the low 12 bits are meaningful.
async fn write_offset_register(sensor: &mut Sensor, offset_mm: i32) -> Result<(), Vl53Error<hal::i2c::Error>> {
    // ST's convention is the opposite of `RANGE_OFFSET_MM`: a sensor reading
    // high needs a negative register value, hence the negation.
    let encoded = encode_offset(-offset_mm);
    let low = (encoded & 0x00FF) as u8;
    let high = (encoded >> 8) as u8;

    sensor.write_reg(REG_RANGE_OFFSET_MM, low).await?;
    sensor.write_reg(REG_RANGE_OFFSET_MM + 1, high).await?;

    // The register stores `actual - measured`, so the value that lands there is
    // the opposite sign to `RANGE_OFFSET_MM`; print it as the sensor sees it.
    println!(
        "  register 0x{:02X} <- 0x{:04X} (stored as {:+} mm)",
        REG_RANGE_OFFSET_MM,
        encoded,
        -offset_mm
    );
    Ok(())
}

/// Park forever after a fatal setup problem or a finished one-shot mode.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Average a number of readings against a target at a known distance and report
/// the constant offset to enter in `RANGE_OFFSET_MM`.
async fn offset_check(sensor: &mut Sensor, actual_mm: u16) {
    println!("---- offset check ----");
    println!("Target should be exactly {} mm from the sensor face.", actual_mm);
    println!("Averaging {} readings...", OFFSET_CHECK_SAMPLES);
    println!(
        "(measures the sensor's raw output, so the {:+} mm applied via RANGE_OFFSET_MM",
        RANGE_OFFSET_MM
    );
    println!(" is deliberately not subtracted here)");
    println!("");

    let mut sum: u64 = 0;
    let mut valid: u32 = 0;
    let mut min = u16::MAX;
    let mut max = 0u16;

    for _ in 0..OFFSET_CHECK_SAMPLES {
        match sensor.read_range_single_millimeters().await {
            Ok(mm) if mm < NO_TARGET_MM => {
                sum += mm as u64;
                valid += 1;
                min = min.min(mm);
                max = max.max(mm);
            }
            Ok(_) => {}
            Err(e) => println!("  read error: {:?}", e),
        }
        Timer::after(SAMPLE_PERIOD).await;
    }

    if valid == 0 {
        println!("No valid readings. Check the target is in view and lit normally.");
        return;
    }

    let mean = (sum / valid as u64) as u16;
    let offset = mean as i32 - actual_mm as i32;
    let spread = max.saturating_sub(min);

    println!("  valid readings : {}", valid);
    println!("  mean           : {} mm  (min {} / max {})", mean, min, max);
    println!("  actual         : {} mm", actual_mm);
    println!("  offset         : {:+} mm", offset);
    println!("");

    if spread > OFFSET_CHECK_MAX_SPREAD_MM {
        println!(
            "  Spread is {} mm, too noisy to calibrate. Steady the target, remove",
            spread
        );
        println!("  ambient IR, and make sure the target fills the field of view.");
    } else if offset.abs() <= 5 {
        println!("  Within a few mm of nominal - no offset correction needed.");
    } else {
        println!("  Set `const RANGE_OFFSET_MM: i32 = {};` to correct it.", offset);
        println!("  Re-check at a second distance first: if the error grows with");
        println!("  distance it is proportional spread, not an offset, and this will");
        println!("  not help.");
    }
    println!("");
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 VL53L0X ToF distance sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  3V3 part - do not power it from 5V");
    println!("  driver: edrv-vl53l0x (async)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2cBus = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        I2C_FREQ,
        Default::default(),
    );
    let mut sensor = VL53L0X::new_primary(i2c);

    // `init` reads model register 0xC0 and runs the standard init sequence,
    // which includes the reference (VHV + phase) calibration.
    match sensor.init().await {
        Ok(()) => {
            println!(
                "model register 0xC0 = 0x{:02X} -> VL53L0X detected, init done",
                VL53L0X_MODEL_ID
            );
            println!("");
        }
        Err(Vl53Error::InvalidDevice(id)) => {
            println!(
                "model register 0xC0 = 0x{:02X}, expected 0x{:02X}.",
                id, VL53L0X_MODEL_ID
            );
            println!("");
            if id == VL53L1X_MODEL_ID {
                println!("That is a **VL53L1X**, not a VL53L0X. They share address 0x29 but");
                println!("have different register maps and init sequences, so this driver");
                println!("cannot drive it - swapping in an L1X driver is the fix, not rewiring.");
            } else {
                println!("Not a VL53L0X. Check SDA/SCL are not swapped and that XSHUT is");
                println!("pulled HIGH: held low, the chip sits in reset and NACKs everything.");
            }
            park().await;
        }
        Err(Vl53Error::Timeout) => {
            println!("init timed out. The chip answered but did not complete its start-up;");
            println!("this is usually a marginal 3V3 supply or missing pull-ups.");
            park().await;
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            println!("Check 3V3, GND, SDA/SCL orientation and that XSHUT is HIGH.");
            park().await;
        }
    }

    // One-shot offset measurement mode.
    if let Some(actual_mm) = OFFSET_CHECK_DISTANCE_MM {
        offset_check(&mut sensor, actual_mm).await;
        park().await;
    }

    if WRITE_OFFSET_TO_SENSOR {
        if let Err(e) = write_offset_register(&mut sensor, RANGE_OFFSET_MM).await {
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

        match sensor.read_range_single_millimeters().await {
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

        Timer::after(SAMPLE_PERIOD).await;
    }
}
