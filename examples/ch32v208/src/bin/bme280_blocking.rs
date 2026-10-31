//! BME280 / BMP280 environmental sensor demo, **blocking** flavour, for the
//! CH32V208.
//!
//! Sync counterpart of the `bme280` example: same sensor, same `edrv-bme280`
//! driver, but the blocking API and a polled I2C bus - no embassy executor, no
//! async, no DMA.
//!
//! Wiring is identical to `bme280`:
//!
//! ```text
//!   BME280 breakout      CH32V208
//!   VCC        <----  3V3
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   SDO        ----   address strap: GND = 0x76, VCC = 0x77
//! ```
//!
//! The three readings, their units, and the humidity-oversampling trap that this
//! example exists to catch are all documented once in the `bme280` example.
//!
//! # The tradeoff
//!
//! A BME280 measurement is naturally sequential - trigger, wait, read - so
//! blocking is a good fit and the driver does the waiting internally. The cost is
//! that a missing or wedged sensor blocks forever instead of being reported.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_bme280::{blocking::BME280, Error as BmeError};
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, print, println};
use panic_halt as _;
use qingke::riscv;

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Blocking>;
type Sensor = BME280<I2cBus>;

/// The BME280 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// The address this board's sensor answers on. **Change this if yours differs.**
///
/// `ADDRESS` is `0x76` (`SDO` to GND); `0x77` (`SDO` to VCC) is the alternative
/// and is what Adafruit's breakouts use.
const SENSOR_ADDRESS: u8 = edrv_bme280::ADDRESS;

/// Pace between measurements.
const SAMPLE_PERIOD_MS: u32 = 3000;


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

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 BME280 / BMP280 sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  address 0x{:02X} (SDO strap: GND=0x76, VCC=0x77)", SENSOR_ADDRESS);
    println!("  polled (no DMA, no executor)");
    println!("  driver: edrv-bme280 (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2cBus = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, I2C_FREQ, Default::default());
    let mut sensor: Sensor = BME280::new(i2c, SENSOR_ADDRESS);
    let mut delay = Delay;

    let chip_id = match sensor.read_reg(edrv_bme280::regs::CHIP_ID) {
        Ok(id) => id,
        Err(e) => {
            println!("no response at 0x{:02X}: {:?}", SENSOR_ADDRESS, e);
            println!("");
            println!("Nothing acknowledged. Check 3V3, GND, the pull-ups, and that SDO");
            println!("is strapped to one rail rather than floating.");
            park();
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
            println!("chip ID 0xD0 = 0x{:02X}, expected 0x60 or 0x58.", other);
            println!("Not a BME280/BMP280; run `i2c_identify` to name what is there.");
            park();
        }
    };

    // `init` does not reset; do it explicitly so the calibration is certainly
    // loaded, and let reset wait out the start-up.
    if let Err(e) = sensor.reset(&mut delay) {
        println!("soft reset failed: {:?}", e);
        park();
    }
    if let Err(e) = sensor.init() {
        println!("init failed: {:?}", e);
        park();
    }

    println!("calibration read; x1 oversampling; each read triggers one forced");
    println!("conversion and waits for it before reading the result.");
    println!("");
    println!("Note: this blocking demo has no timeout, so it stops here if the");
    println!("sensor is silent. The `bme280` example reports that case instead.");
    println!("");

    let mut count: u32 = 0;

    loop {
        count += 1;

        match sensor.read_measurement(&mut delay) {
            Ok(m) => {
                // Decimal points are split by hand; see the `bme280` example.
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

                if !(30_000..=110_000).contains(&p_pa) {
                    println!("      !! {} Pa is outside the BME280's range.", p_pa);
                }
            }
            Err(BmeError::UnsupportedMeasurement) => {
                println!("#{:<4} the driver rejected this measurement.", count);
            }
            Err(e) => println!("#{:<4} read error: {:?}", count, e),
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
