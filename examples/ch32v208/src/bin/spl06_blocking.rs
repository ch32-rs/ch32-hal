//! SPL06-001 barometric pressure sensor demo, **blocking** flavour, for the
//! CH32V208.
//!
//! Sync counterpart of the `spl06` example: same sensor, same `edrv-spl06`
//! driver, but the blocking API and a polled I2C bus - no embassy executor, no
//! async, no DMA.
//!
//! Wiring is identical to `spl06`:
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
//! The address-strap discussion, what the readings mean, and why nothing here
//! formats a float are all documented once in the `spl06` example.
//!
//! # The tradeoff
//!
//! A single-shot measurement is naturally sequential - configure, trigger, wait
//! for the conversion, read - so blocking is a good fit and the driver does the
//! waiting internally. The cost is that a missing or wedged sensor blocks forever
//! instead of being reported as a timeout.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_spl06::{blocking::SPL06, Config, Error as Spl06Error, Oversampling};
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, println};
use panic_halt as _;
use qingke::riscv;

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Blocking>;
type Sensor = SPL06<I2cBus>;

/// The SPL06 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// The address this board's sensor answers on. **Change this if yours differs.**
///
/// `ADDRESS` is `0x77` (`SDO` high), which the datasheet gives as the default;
/// `ADDRESS_ALT` is `0x76` (`SDO` pulled to GND). This board answers on `0x77`.
const SENSOR_ADDRESS: u8 = edrv_spl06::ADDRESS;

/// Pace between measurements.
const SAMPLE_PERIOD_MS: u32 = 2000;

/// Print the raw ADC values alongside the compensated ones.
const PRINT_RAW: bool = true;

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

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 SPL06-001 barometer");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  address 0x{:02X} (SDO strap: low=0x76, high=0x77)", SENSOR_ADDRESS);
    println!("  polled (no DMA, no executor)");
    println!("  driver: edrv-spl06 (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2cBus = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, I2C_FREQ, Default::default());
    let mut sensor: Sensor = SPL06::new(i2c, SENSOR_ADDRESS);
    let mut delay = Delay;

    // Read the ID before `init` so the actual value is visible even when it does
    // not match.
    match sensor.read_reg(edrv_spl06::regs::ID) {
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
            println!("then `SDO` is strapped the other way - try 0x76.");
            park();
        }
    }

    // `init` deliberately does not reset (matching bme280/bme680); reset first so
    // the calibration registers are certainly loaded. `reset` waits out TCoef_rdy.
    if let Err(e) = sensor.reset(&mut delay) {
        println!("soft reset failed: {:?}", e);
        park();
    }

    match sensor.init() {
        Ok(()) => {
            println!("calibration read");
            println!("");
        }
        Err(Spl06Error::InvalidDevice(id)) => {
            println!("");
            println!("0x0D read 0x{:02X}, expected 0x10 -> not an SPL06-001/-007.", id);
            println!("Product ID 1 is the SPL06 family. Something else is at this address.");
            park();
        }
        Err(e) => {
            println!("init failed: {:?}", e);
            park();
        }
    }

    // 8x on both is a reasonable accuracy/speed compromise; the driver applies
    // the matching kP/kT scaling itself.
    let config = Config {
        pressure: Oversampling::X8,
        temperature: Oversampling::X8,
    };
    if let Err(e) = sensor.set_oversampling(config.pressure, config.temperature) {
        println!("could not set oversampling: {:?}", e);
    }

    println!("Note: this blocking demo has no timeout, so it stops here if the");
    println!("sensor is silent. The `spl06` example reports that case instead.");
    println!("");

    let mut count: u32 = 0;
    let mut implausible: u32 = 0;

    loop {
        count += 1;

        match sensor.measure(&mut delay) {
            Ok(m) => {
                // Decimal points are split by hand; see the `spl06` example.
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

                if !(30_000..=110_000).contains(&p) || !(-4_000..=8_500).contains(&t) {
                    implausible += 1;
                    if implausible == 3 {
                        println!("      !! {} implausible readings in a row. The SPL06's", implausible);
                        println!("         compensation depends on the 18-byte coefficient block at");
                        println!("         0x10; if it read as zeros the results will be garbage.");
                    }
                } else {
                    implausible = 0;
                }
            }
            Err(Spl06Error::Timeout) => {
                println!("#{:<4} measurement timed out - no new data appeared.", count);
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
