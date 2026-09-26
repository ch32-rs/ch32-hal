//! SPL06-001 barometric pressure sensor demo for the CH32V208, in embassy style.
//!
//! Uses **our own** `edrv-spl06` driver, in its async flavour. The blocking
//! flavour is exercised by the companion `spl06_blocking` example.
//!
//! Wiring (I2C2):
//!
//! ```text
//!   SPL06-001 breakout   CH32V208
//!   VDD        <----  3V3
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   CSB        ----   must be HIGH for I2C (low selects SPI)
//!   SDO        ----   address strap: low = 0x76, high = 0x77
//! ```
//!
//! # The address is a board property, so this example does not assume one
//!
//! `SDO` selects the 7-bit address: **low gives `0x76`, high gives `0x77`**, and
//! the SPL06-001 datasheet gives `0x77` as the default. Note this is the
//! **opposite** naming to Bosch's barometers, where the "primary" address is
//! `0x76` - so `SPL06::new_primary()` (which uses `0x76`) is *not* necessarily
//! the one your board answers on.
//!
//! This example therefore passes the address explicitly, and `SENSOR_ADDRESS`
//! below is the knob to change. If you are unsure, run `i2c_detect` and then
//! `i2c_identify` with `TARGET` set to whatever address it reports.
//!
//! The driver validates register `0x0D` (`PROD_ID[7:4]` / `REV_ID[3:0]`, `0x10`
//! on a SPL06-001/-007) during `init` and returns `InvalidDevice` otherwise, so
//! a wrong part fails loudly instead of producing plausible nonsense.
//!
//! # Floats cost flash, so nothing here formats one
//!
//! `edrv-spl06` returns the compensated values as integers (`pressure` in Pa,
//! `temperature` in 0.01 C) *and* as `f32` convenience accessors. On this
//! soft-float `riscv32imc` target, `println!("{}", some_f32)` alone pulls in the
//! float formatter and costs about **12.5 KB** of flash - measured on the
//! `mpu6050` example, not guessed. So this demo splits the decimal point by hand.
//!
//! # What the numbers mean
//!
//! * Pressure is in pascal; sea level is ~101325 Pa and it falls with altitude.
//! * The sensor is a *relative* device: its absolute accuracy is a few hundred
//!   pascal unless you calibrate it, but it resolves height changes of well under
//!   a metre, which is what it is normally used for.
//! * Temperature is the die temperature, and it is what the pressure
//!   compensation depends on. It reads a little warm because it is measuring the
//!   chip, not the air.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_spl06::{Error as Spl06Error, Oversampling, SPL06};
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
type Sensor = SPL06<I2cBus>;

/// The SPL06 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// The address this board's sensor answers on. **Change this if yours differs.**
///
/// `ADDRESS` is `0x77` (`SDO` high), which the datasheet gives as the default;
/// `ADDRESS_ALT` is `0x76` (`SDO` pulled to GND). This board answers on `0x77`.
const SENSOR_ADDRESS: u8 = edrv_spl06::ADDRESS;

/// Pace between measurements. A single-shot measurement at 8x oversampling takes
/// a few tens of milliseconds; there is nothing to gain from hammering it.
const SAMPLE_PERIOD: Duration = Duration::from_secs(2);

/// Print the raw ADC values alongside the compensated ones.
const PRINT_RAW: bool = true;

/// Park forever after a fatal setup problem.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Rough pressure band, for eyeballing whether a reading is plausible.
fn pressure_hint(pa: i32) -> &'static str {
    match pa {
        i32::MIN..=30_000 => "implausibly low - check the calibration read",
        30_001..=90_000 => "above ~1 km altitude",
        90_001..=100_000 => "high altitude or low-pressure weather",
        100_001..=103_000 => "plausible sea-level pressure",
        103_001..=110_000 => "high-pressure weather",
        _ => "implausibly high - check the calibration read",
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 SPL06-001 barometer");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  address 0x{:02X} (SDO strap: low=0x76, high=0x77)", SENSOR_ADDRESS);
    println!("  driver: edrv-spl06 (async)");
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
    let mut sensor: Sensor = SPL06::new(i2c, SENSOR_ADDRESS);

    // Read the ID before `init` so the actual value is visible even when it does
    // not match: on a mismatch `init` reports it, but showing it here means the
    // output says what *is* on the bus, not just what is not.
    match sensor.read_reg(edrv_spl06::regs::ID).await {
        Ok(id) => println!(
            "PROD_ID/REV_ID (0x0D) = 0x{:02X}  -> product {}, revision {}",
            id,
            id >> 4,
            id & 0x0F
        ),
        Err(e) => {
            println!("no response at 0x{:02X}: {:?}", SENSOR_ADDRESS, e);
            println!("");
            println!("Nothing acknowledged. Check 3V3, GND, pull-ups, and that CSB is");
            println!("HIGH (low selects SPI). If the bus scans clean at the other address");
            println!("then `SDO` is strapped the other way - try changing SENSOR_ADDRESS.");
            park().await;
        }
    }

    // Soft reset first: `init` deliberately does not reset (matching bme280 and
    // bme680), and a reset is the only way to be sure the calibration registers
    // are loaded. `reset` already waits out TCoef_rdy.
    if let Err(e) = sensor.reset(&mut Delay).await {
        println!("soft reset failed: {:?}", e);
        park().await;
    }

    match sensor.init().await {
        Ok(()) => {
            println!("calibration read, default oversampling programmed");
            println!(
                "oversampling: pressure {:?}, temperature {:?}",
                edrv_spl06::Config::default().pressure,
                edrv_spl06::Config::default().temperature
            );
            println!("");
        }
        Err(Spl06Error::InvalidDevice(id)) => {
            println!("");
            println!("0x0D read 0x{:02X}, expected 0x10 -> not an SPL06-001/-007.", id);
            println!("Product ID 1 is the SPL06 family. Something else is at this address.");
            park().await;
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            park().await;
        }
    }

    // 8x on both is a reasonable accuracy/speed compromise; the driver applies
    // the matching kP/kT scaling itself.
    let config = edrv_spl06::Config {
        pressure: Oversampling::X8,
        temperature: Oversampling::X8,
    };
    if let Err(e) = sensor.set_oversampling(config.pressure, config.temperature).await {
        println!("could not set oversampling: {:?}", e);
    }

    let mut count: u32 = 0;
    let mut implausible: u32 = 0;

    loop {
        count += 1;

        match sensor.measure(&mut Delay).await {
            Ok(m) => {
                // Split every value at its decimal point by hand; see the module
                // docs for why no float formatter is used here.
                let t = m.temperature;
                let t_whole = t.unsigned_abs() / 100;
                let t_frac = t.unsigned_abs() % 100;
                let p = m.pressure;
                let p_whole = p / 100;
                let p_frac = p % 100;

                println!(
                    "#{:<4} {:>6}.{:02} hPa   {}{}.{:02} C   [{}]",
                    count,
                    p_whole,
                    p_frac,
                    if t < 0 { "-" } else { "" },
                    t_whole,
                    t_frac,
                    pressure_hint(p)
                );

                if PRINT_RAW {
                    println!(
                        "      raw P {}  raw T {}   ({} Pa)",
                        m.raw_pressure(),
                        m.raw_temperature(),
                        m.pressure
                    );
                }

                // A reading outside the part's own range means the calibration
                // coefficients were not read correctly - not that the weather is
                // strange.
                if !(30_000..=110_000).contains(&p) || !(-4_000..=8_500).contains(&t) {
                    implausible += 1;
                    if implausible == 3 {
                        println!("      !! {} implausible readings in a row. The SPL06's", implausible);
                        println!("         compensation depends on the 18-byte coefficient block at");
                        println!("         0x10; if it read as zeros the results will be garbage.");
                        println!("         Check that PROD_ID is 0x10 and the bus is stable.");
                    }
                } else {
                    implausible = 0;
                }
            }
            Err(Spl06Error::Timeout) => {
                println!("#{:<4} measurement timed out - no new data appeared.", count);
                println!("      Suspect the supply or the pull-ups.");
            }
            Err(e) => println!("#{:<4} read error: {:?}", count, e),
        }

        Timer::after(SAMPLE_PERIOD).await;
    }
}
