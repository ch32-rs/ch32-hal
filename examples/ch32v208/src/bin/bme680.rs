//! BME680 environmental sensor demo for the CH32V208, in embassy style.
//!
//! Uses **our own** `edrv-bme680` driver, in its async flavour. The blocking
//! flavour is exercised by the companion `bme680_blocking` example.
//!
//! Wiring (I2C2):
//!
//! ```text
//!   BME680 breakout      CH32V208
//!   VCC        <----  3V3     (3.3V part; most breakouts are 3.3V only)
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   SDO        ----    GND for 0x76, or 3V3 for 0x77
//! ```
//!
//! `0x76`/`0x77` is shared by the whole Bosch pressure-sensor line - BMP180,
//! BMP280, BME280 and BME680 all answer there - so the address alone does not
//! identify the part. This driver validates the chip ID at register `0xD0`
//! (`0x61`) and refuses to run otherwise; run `i2c_identify` with `TARGET = 0x77`
//! if you are not sure what is on the bus.
//!
//! # What the four numbers mean
//!
//! * **Temperature / pressure / humidity** are ordinary compensated readings.
//! * **Gas resistance** is *not* an air-quality index. It is the resistance of a
//!   metal-oxide (MOX) sensor in ohms, and it is only meaningful as a *relative*
//!   signal for one specific sensor: it drifts, it needs burn-in of hours to
//!   days, and every unit reads differently. Rough field guidance is that clean
//!   air sits in the tens of kOhm and the resistance falls as reducing gases
//!   (VOCs) are present. Bosch's actual air-quality *index* (IAQ) is computed by
//!   their **BSEC** library, which is closed-source and not part of this driver -
//!   so this demo deliberately stops at resistance.
//! * `heat_stable` must be true for the resistance to be trustworthy: it says
//!   the heater reached its set point during the measurement. Until the heater
//!   has settled, the reading is meaningless.
//! * The BME688 answers the same chip ID but reports a different variant
//!   (`variant_id == 0x01`); its gas readings need the high-range formula, which
//!   this driver does not implement. The variant is printed below so a BME688 is
//!   not silently misread.
//!
//! # Floats cost flash, so nothing here formats one
//!
//! `edrv-bme680` exposes the compensated values as integers (`temperature` in
//! 0.01 C, `humidity` in 0.001 %RH, `pressure` in Pa, `gas_resistance` in ohms)
//! *and* as `f32` convenience accessors. On this soft-float `riscv32imc` target,
//! `println!("{}", some_f32)` alone pulls in the float formatter and costs about
//! **12.5 KB** of flash - measured on the `mpu6050` example, not guessed.
//!
//! So this demo prints the integer fields directly, split at the decimal point
//! by hand. The `f32` accessors exist and are fine to use; they are just
//! expensive to *print*.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_bme680::{Error as BmeError, BME680};
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use hal::i2c::I2c;
use hal::mode::Async;
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals, println};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C2_EV => hal::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Async>;
type Sensor = BME680<I2cBus>;

/// The BME680 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// Pace between measurements.
///
/// A forced-mode BME680 measurement takes roughly 200 ms at these oversampling
/// settings, including the gas heater, so there is no point going much faster.
const SAMPLE_PERIOD: Duration = Duration::from_secs(3);

/// The variant ID of a BME688, whose gas path differs from a BME680's.
const VARIANT_BME688: u8 = 0x01;

/// Rough gas-resistance band, purely for readability. **Not** an air-quality
/// index, and not calibrated to your sensor - see the module docs.
fn gas_band(ohms: u32) -> &'static str {
    match ohms {
        0..=5_000 => "very low - VOCs present, or heater not settled",
        5_001..=20_000 => "low",
        20_001..=50_000 => "typical indoor",
        50_001..=200_000 => "clean",
        _ => "very clean (or the sensor has not been exposed yet)",
    }
}

/// Park forever after a fatal setup problem.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 BME680 T/P/H/gas sensor");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  driver: edrv-bme680 (async)");
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
    let mut sensor: Sensor = BME680::new_primary(i2c);

    // `init` does NOT reset the chip - `reset` is a separate step, exactly as in
    // the `bme280` driver. Bosch's own `bme68x_init` does soft-reset first, so
    // the reference sequence is reproduced here by calling both.
    if let Err(e) = sensor.reset(&mut Delay).await {
        println!("soft reset failed: {:?}", e);
        println!("Check 3V3, GND and that SDA/SCL are not swapped.");
        park().await;
    }

    match sensor.init().await {
        Ok(()) => {}
        Err(BmeError::InvalidDevice(id)) => {
            println!("chip ID 0xD0 = 0x{:02X}, expected 0x61 -> not a BME680.", id);
            println!("");
            println!("That address is shared by the whole Bosch line, and register 0xD0");
            println!("is what separates them: 0x55 BMP180, 0x58 BMP280, 0x60 BME280,");
            println!("0x61 BME680/688. Flash `i2c_identify` with TARGET = 0x77 to name it.");
            park().await;
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            println!("Check 3V3, GND, SDA/SCL orientation and the pull-ups.");
            park().await;
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
    println!("oversampling: T x2, P x16, H x1; IIR filter 3");
    println!(
        "heater: {} C for {} ms (ambient assumed {} C)",
        edrv_bme680::DEFAULT_HEATER_TARGET_CELSIUS,
        edrv_bme680::DEFAULT_HEATER_DURATION_MS,
        edrv_bme680::DEFAULT_AMBIENT_CELSIUS
    );
    println!("");
    println!("The gas heater needs to settle before its reading means anything,");
    println!("and a new sensor needs hours of burn-in. Watch the heat_stable flag.");
    println!("");

    let mut count: u32 = 0;
    let mut unstable: u32 = 0;

    loop {
        count += 1;

        match sensor.measure(&mut Delay).await {
            Ok(m) => {
                // Split every value at its decimal point by hand: formatting an
                // `f32` here would pull in the soft-float formatter. See the
                // module docs.
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

        Timer::after(SAMPLE_PERIOD).await;
    }
}
