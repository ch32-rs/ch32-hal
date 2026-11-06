//! BME280 / BMP280 environmental sensor demo for the CH32V208, in embassy style.
//!
//! Uses **our own** `edrv-bme280` driver, in its async flavour. The blocking
//! flavour is exercised by the companion `bme280_blocking` example.
//!
//! Wiring (I2C2):
//!
//! ```text
//!   BME280 breakout      CH32V208
//!   VCC        <----  3V3     (3.3V part; do not feed it 5V)
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   SDO        ----   address strap: GND = 0x76, VCC = 0x77
//! ```
//!
//! `0x76`/`0x77` is shared by the whole Bosch pressure line, so the address does
//! not identify the part. The driver validates register `0xD0` during `init`:
//! `0x60` is a BME280, `0x58` a BMP280. Both are accepted - the difference is
//! that a BMP280 has no humidity sensor at all.
//!
//! # Three readings, and the one that is easy to get wrong
//!
//! Temperature, pressure and humidity are all compensated by the driver. The
//! returned integers are in 1/100 units: 1/100 degC, 1/100 pascal and 1/100 %RH.
//!
//! **Humidity is the interesting one on this driver.** Bosch's BME280 only
//! applies a write to `CTRL_HUM` *after* a subsequent write to `CTRL_MEAS`. A
//! driver that writes them the other way round - as `edrv-bme280` 0.1.0 did -
//! silently leaves the humidity oversampling at its reset value, which is
//! "skipped", so the humidity data registers keep reading their `0x8000`
//! "not measured" marker.
//!
//! **That marker does not compensate to zero.** It decodes to a *believable*
//! humidity in the sixties or low seventies, and it barely moves: worked through
//! the compensation with typical calibration coefficients, the same `0x8000`
//! gives about **71 %RH at 25 degC and 71 %RH at 37 degC**. So the broken driver
//! reports a plausible, near-constant humidity that has nothing to do with the
//! room, with no error and no other symptom - temperature and pressure stay
//! correct.
//!
//! That makes it impossible to spot from the numbers alone, so **test it by
//! changing the humidity**: breathe gently on the sensor. A working BME280 moves
//! by tens of %RH within a second; under this bug the value does not budge.
//! If it does not move, check which `edrv-bme280` version you resolved - 0.1.1
//! fixed the ordering.
//!
//! # The first reading used to be wrong too
//!
//! Until a conversion completes, the data registers hold `0x80000` - the
//! datasheet's 20-bit "measurement skipped" marker - and those bytes compensate
//! to values that look like *real weather*: roughly 24 degC instead of 26, and a
//! pressure about a **third** low, because `t_fine` comes from the bogus
//! temperature. `read_measurement` used to read them immediately after `init`,
//! so the first line of output was silently wrong by ~30% on pressure and looked
//! like high-altitude weather rather than an error.
//!
//! 0.1.1 makes `read_measurement` trigger a forced conversion, wait for it, and
//! then read the whole pressure/temperature/humidity block in one burst, so it
//! cannot return the placeholder. That is why the call below takes a `delay`:
//! the wait is the point.
//!
//! # Floats cost flash, so nothing here formats one
//!
//! `riscv32imc` is soft-float, and `println!("{}", some_f32)` alone pulls in the
//! float formatter for about **12.5 KB** of flash - measured, not guessed. The
//! driver exposes the integer fields as well as `f32` accessors, so the decimal
//! point is split by hand here.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_bme280::{Error as BmeError, BME280};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
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
type Sensor = BME280<I2cBus>;

/// The BME280 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// The address this board's sensor answers on. **Change this if yours differs.**
///
/// `ADDRESS` is `0x76` (`SDO` to GND), which is what most GY-BME280 modules use;
/// `0x77` (`SDO` to VCC) is the other option and is what Adafruit's breakouts
/// ship with. Bosch calls `0x76` the *primary* address, the opposite convention
/// to the SPL06 - do not carry the habit across.
const SENSOR_ADDRESS: u8 = edrv_bme280::ADDRESS;

/// Pace between measurements.
const SAMPLE_PERIOD: Duration = Duration::from_secs(3);


/// Park forever after a fatal setup problem.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Rough pressure band, for eyeballing whether a reading is plausible.
fn pressure_hint(pa: u32) -> &'static str {
    match pa {
        0..=30_000 => "implausibly low - the calibration read looks wrong",
        30_001..=90_000 => "above ~1 km altitude",
        90_001..=100_000 => "high altitude or low-pressure weather",
        100_001..=103_000 => "plausible sea-level pressure",
        103_001..=110_000 => "high-pressure weather",
        _ => "implausibly high - the calibration read looks wrong",
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 BME280 / BMP280 sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  address 0x{:02X} (SDO strap: GND=0x76, VCC=0x77)", SENSOR_ADDRESS);
    println!("  driver: edrv-bme280 (async)");
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
    let mut sensor: Sensor = BME280::new(i2c, SENSOR_ADDRESS);

    // Read the chip ID first so the output always says what is actually on the
    // bus, even when it is not what we expect.
    let chip_id = match sensor.read_reg(edrv_bme280::regs::CHIP_ID).await {
        Ok(id) => id,
        Err(e) => {
            println!("no response at 0x{:02X}: {:?}", SENSOR_ADDRESS, e);
            println!("");
            println!("Nothing acknowledged. Check 3V3, GND, the pull-ups, and that SDO");
            println!("is strapped to one rail rather than floating.");
            park().await;
        }
    };

    let is_bme280 = match chip_id {
        edrv_bme280::CHIP_ID_BME280 => {
            println!("chip ID 0xD0 = 0x60 -> BME280 (temperature, pressure, humidity)");
            true
        }
        edrv_bme280::CHIP_ID_BMP280 => {
            println!("chip ID 0xD0 = 0x58 -> BMP280 (temperature and pressure only)");
            println!("  there is no humidity sensor in this part, so humidity reads 0");
            false
        }
        other => {
            println!("chip ID 0xD0 = 0x{:02X}, expected 0x60 (BME280) or 0x58 (BMP280).", other);
            println!("");
            println!("Not a BME280/BMP280. That address is shared by several parts;");
            println!("flash `i2c_identify` with the matching TARGET to name it.");
            park().await;
        }
    };

    // `init` deliberately does not reset (matching the other edrv drivers), so
    // reset explicitly first and let it wait out the start-up.
    if let Err(e) = sensor.reset(&mut Delay).await {
        println!("soft reset failed: {:?}", e);
        park().await;
    }

    if let Err(e) = sensor.init().await {
        println!("init failed: {:?}", e);
        park().await;
    }

    println!("calibration read; x1 oversampling; each read triggers one forced");
    println!("conversion and waits for it before reading the result.");
    println!("");

    let mut count: u32 = 0;

    loop {
        count += 1;

        match sensor.read_measurement(&mut Delay).await {
            Ok(m) => {
                // Split every value at its decimal point by hand; see the module
                // docs for why no float formatter is used.
                let t = m.temperature;
                let t_whole = t.unsigned_abs() / 100;
                let t_frac = t.unsigned_abs() % 100;

                // pressure is in 1/100 Pa; 1 hPa is 10000 of those.
                let p_pa = m.pressure / 100;
                let p_whole = m.pressure / 10_000;
                let p_frac = (m.pressure % 10_000) / 100;

                let h_whole = m.humidity / 100;
                let h_frac = m.humidity % 100;

                print!(
                    "#{:<4} {}{}.{:02} C   {}.{:02} hPa ({} Pa)   ",
                    count,
                    if t < 0 { "-" } else { "" },
                    t_whole,
                    t_frac,
                    p_whole,
                    p_frac,
                    p_pa
                );

                if is_bme280 {
                    println!("{}.{:02} %RH   [{}]", h_whole, h_frac, pressure_hint(p_pa));
                } else {
                    println!("n/a (BMP280)   [{}]", pressure_hint(p_pa));
                }

                // Values outside the part's range mean the calibration block did
                // not read correctly, not that the weather is strange.
                if !(30_000..=110_000).contains(&p_pa) {
                    println!("      !! {} Pa is outside the BME280's range - suspect the", p_pa);
                    println!("         38-byte calibration read at 0x88.");
                }
            }
            Err(BmeError::UnsupportedMeasurement) => {
                println!("#{:<4} the driver rejected this measurement.", count);
            }
            Err(e) => println!("#{:<4} read error: {:?}", count, e),
        }

        Timer::after(SAMPLE_PERIOD).await;
    }
}
