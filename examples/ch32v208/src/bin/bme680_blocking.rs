//! BME680 environmental sensor demo, **blocking** flavour, for the CH32V208.
//!
//! Sync counterpart of the `bme680` example: same sensor, same `edrv-bme680`
//! driver, but the blocking API and a polled I2C bus - no embassy executor, no
//! async, no DMA.
//!
//! Wiring is identical to `bme680`:
//!
//! ```text
//!   BME680 breakout      CH32V208
//!   VCC        <----  3V3
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   SDO        ----    GND for 0x76, or 3V3 for 0x77
//! ```
//!
//! What the four quantities mean, why gas resistance is not an air-quality
//! index, and why nothing here formats a float are all documented once in the
//! `bme680` example.
//!
//! # The tradeoff
//!
//! A forced-mode BME680 measurement is naturally sequential - trigger, wait for
//! the heater and the conversion, then read - so blocking is a good fit and the
//! driver does the waiting internally. The cost is that a missing or wedged
//! sensor blocks forever instead of being reported as a timeout.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_bme680::{blocking::BME680, Error as BmeError};
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, println};
use panic_halt as _;
use qingke::riscv;

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Blocking>;
type Sensor = BME680<I2cBus>;

/// The BME680 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// Pace between measurements. A forced-mode measurement including the gas
/// heater takes roughly 200 ms at these oversampling settings.
const SAMPLE_PERIOD_MS: u32 = 3000;

/// The variant ID of a BME688, whose gas path differs from a BME680's.
const VARIANT_BME688: u8 = 0x01;

/// Rough gas-resistance band, purely for readability. **Not** an air-quality
/// index, and not calibrated to your sensor.
fn gas_band(ohms: u32) -> &'static str {
    match ohms {
        0..=5_000 => "very low - VOCs present, or heater not settled",
        5_001..=20_000 => "low",
        20_001..=50_000 => "typical indoor",
        50_001..=200_000 => "clean",
        _ => "very clean (or the sensor has not been exposed yet)",
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 BME680 T/P/H/gas sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  polled (no DMA, no executor)");
    println!("  driver: edrv-bme680 (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2cBus = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, I2C_FREQ, Default::default());
    let mut sensor: Sensor = BME680::new_primary(i2c);
    let mut delay = Delay;

    // `init` does NOT reset the chip - `reset` is a separate step, as in the
    // `bme280` driver. Bosch's own `bme68x_init` soft-resets first, so the
    // reference sequence is reproduced by calling both, in this order.
    if let Err(e) = sensor.reset(&mut delay) {
        println!("soft reset failed: {:?}", e);
        println!("Check 3V3, GND and that SDA/SCL are not swapped.");
        park();
    }

    match sensor.init() {
        Ok(()) => {}
        Err(BmeError::InvalidDevice(id)) => {
            println!("chip ID 0xD0 = 0x{:02X}, expected 0x61 -> not a BME680.", id);
            println!("");
            println!("That address is shared by the whole Bosch line: 0x55 BMP180,");
            println!("0x58 BMP280, 0x60 BME280, 0x61 BME680/688. Flash `i2c_identify`");
            println!("with TARGET = 0x77 to name the part.");
            park();
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            println!("Check 3V3, GND, SDA/SCL orientation and the pull-ups.");
            park();
        }
    }

    println!("chip ID 0xD0 = 0x61, calibration read, defaults programmed");
    match sensor.variant_id {
        VARIANT_BME688 => {
            println!("variant_id = 0x01 -> **BME688**. Its gas readings need the");
            println!("  high-range formula, which this driver does not implement, so the");
            println!("  gas resistance below will be WRONG. T/P/H are still valid.");
        }
        other => println!("variant_id = 0x{:02X} -> BME680 gas path", other),
    }
    println!(
        "heater: {} C for {} ms (ambient assumed {} C)",
        edrv_bme680::DEFAULT_HEATER_TARGET_CELSIUS,
        edrv_bme680::DEFAULT_HEATER_DURATION_MS,
        edrv_bme680::DEFAULT_AMBIENT_CELSIUS
    );
    println!("");
    println!("Note: this blocking demo has no timeout, so it stops here if the");
    println!("sensor is silent. The `bme680` example reports that case instead.");
    println!("");

    let mut count: u32 = 0;
    let mut unstable: u32 = 0;

    loop {
        count += 1;

        match sensor.measure(&mut delay) {
            Ok(m) => {
                // Decimal points are split by hand; see the `bme680` example for
                // why a float formatter is avoided on this target.
                let t = m.temperature;
                let t_whole = t.unsigned_abs() / 100;
                let t_frac = t.unsigned_abs() % 100;
                let p_whole = m.pressure / 100;
                let p_frac = (m.pressure % 100) / 10;
                let h_whole = m.humidity / 1000;
                let h_frac = m.humidity % 1000;
                let g_k = m.gas_resistance / 1000;
                let g_frac = (m.gas_resistance % 1000) / 100;

                println!(
                    "#{:<4} {}{}.{:02} C   {}.{} hPa ({} Pa)   {}.{:03} %RH",
                    count,
                    if t < 0 { "-" } else { "" },
                    t_whole,
                    t_frac,
                    p_whole,
                    p_frac,
                    m.pressure,
                    h_whole,
                    h_frac
                );

                if m.gas_valid && m.heat_stable {
                    println!(
                        "      gas {}.{} kOhm  ({})   [raw {} range {}]",
                        g_k,
                        g_frac,
                        gas_band(m.gas_resistance),
                        m.raw_gas_resistance(),
                        m.gas_range()
                    );
                } else {
                    unstable += 1;
                    println!(
                        "      gas not usable yet (valid {}, heater stable {})   [{} so far]",
                        m.gas_valid, m.heat_stable, unstable
                    );
                    println!("      This is normal for the first measurements after power-up.");
                }
            }
            Err(BmeError::Timeout) => {
                println!("#{:<4} measurement timed out - the sensor stopped responding.", count);
                println!("      Suspect the 3V3 supply or the pull-ups.");
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
