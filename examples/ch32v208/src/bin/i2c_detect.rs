//! I2C bus scanner / detector for the CH32V208, written in embassy style.
//!
//! Wiring (CH32V208, I2C2):
//!
//! ```text
//!   PB10 -> SCL   (I2C2_SCL, AF open-drain)
//!   PB11 -> SDA   (I2C2_SDA, AF open-drain)
//!   GND  -> GND
//! ```
//!
//! Both lines are open-drain and need external pull-ups to 3.3V (typically
//! 4.7k, or 2.2k at 400 kHz). Many breakout boards already have them.
//!
//! The scan is fully async: the I2C2 driver runs in [`Async`] mode with the
//! I2C2 event/error interrupts bound and DMA1 channel 4 (TX) / channel 5 (RX),
//! and each address is probed by an awaited one-byte read.
//!
//! Every 7-bit address in `0x08..=0x77` is probed. A slave that acknowledges
//! its address is reported; a missing slave answers with a NACK.
//!
//! # The names below are guesses, and the address is not an identity
//!
//! This program can only tell you that *something* answered on an address. The
//! name it prints comes from a static table, and most of those addresses carry
//! several unrelated parts - `0x76`/`0x77` alone is shared by the whole Bosch
//! barometer line **and** the SPL06-001, which do not even use the same ID
//! register. Treating "device at 0x77" as "it is a BME680" is exactly the
//! mistake this comment exists to prevent.
//!
//! Where a real ID register can be read, the list below marks the address and
//! names the follow-up: run `i2c_identify` with `TARGET` set to that address and
//! it will interrogate the silicon instead of guessing.

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::i2c::{Error, ErrorInterruptHandler, EventInterruptHandler, I2c};
use hal::mode::Async;
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals, print, println};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C2_EV => EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => ErrorInterruptHandler<peripherals::I2C2>;
});

/// I2C2 bus: PB10 = SCL, PB11 = SDA, DMA1_CH4 (TX) / DMA1_CH5 (RX).
type I2cBus = I2c<'static, peripherals::I2C2, Async>;

/// Bus speed. 100 kHz is the safe default; use 400 kHz for fast devices.
const I2C_FREQ: Hertz = Hertz::khz(100);

/// 7-bit addresses worth scanning (0x00..0x07 and 0x78..0x7F are reserved).
const SCAN_FIRST: u8 = 0x08;
const SCAN_LAST: u8 = 0x77;
const SCAN_COUNT: usize = (SCAN_LAST - SCAN_FIRST + 1) as usize;

/// How long to wait between full scans.
const RESCAN_PERIOD: Duration = Duration::from_secs(5);

/// Result of probing a single address.
enum Probe {
    /// The address was acknowledged: a slave is present.
    Ack,
    /// The address was not acknowledged: nothing there.
    Nack,
    /// The bus itself misbehaved (arbitration lost, bus error, timeout...).
    Fault(Error),
}

/// Probe one 7-bit address.
///
/// This issues `START + (addr << 1 | R)` and reads a single byte. A slave that
/// is present acknowledges its address; a missing one NACKs. Reading (rather
/// than writing) keeps the probe free of side effects on the slave's registers.
async fn probe(i2c: &mut I2cBus, addr: u8) -> Probe {
    let mut byte = [0u8; 1];

    match i2c.read(addr, &mut byte).await {
        Ok(()) => Probe::Ack,
        Err(Error::Nack) => Probe::Nack,
        Err(e) => Probe::Fault(e),
    }
}

/// Best-effort name lookup for well-known addresses.
///
/// **These are guesses.** The address alone cannot identify a part; see the
/// module docs.
fn device_name(addr: u8) -> &'static str {
    match addr {
        0x0C | 0x0D => "AK8975 compass",
        0x0E => "MAG3110 magnetometer",
        0x18..=0x1A => "LIS3DH / LSM303 accelerometer",
        0x1C | 0x1D => "ADXL345 / LSM303",
        0x1E => "HMC5883L / LSM303 magnetometer",
        0x20..=0x27 => "PCF8574 I/O expander / LCD backpack",
        // Deliberately lists the candidates instead of picking one: 0x29 is
        // shared by a light sensor, an RGB colour sensor and a ToF ranger.
        0x29 => "TSL2561/TSL2591 light, TCS34725 RGB, or VL53L0X ToF",
        0x38 | 0x39 => "AHT10/AHT20 / APDS-9960",
        0x3C | 0x3D => "SSD1306 / SH1106 OLED",
        0x3E => "SH1106 OLED (alt)",
        0x40..=0x47 => "INA219 / HTU21D / Si7021 / PCA9685",
        0x48..=0x4F => "ADS1115 / LM75 / PCF8591",
        0x50..=0x57 => "24Cxx EEPROM / AT24Cxx",
        0x5A => "MLX90614 IR thermometer",
        0x5C | 0x5D => "BH1750 / CCS811",
        0x60..=0x67 => "MPL3115A2 / MCP4725 DAC",
        0x68 | 0x69 => "DS1307/DS3231 RTC / MPU6050",
        0x6A | 0x6B => "LSM6DS3 / L3GD20 gyro",
        0x70..=0x75 => "TCA9548A mux / PCA9685",
        // 0x76/0x77 is a barometer address with two unrelated families on it:
        //
        // * Bosch, ID in register 0xD0 - BMP180 (0x55), BMP280 (0x58),
        //   BME280 (0x60), BME680/688 (0x61)
        // * Goertek SPL06-001 / Infineon SPL06-007, ID in register 0x0D (0x10)
        //
        // plus the MS5611, which has no ID register at all. The address alone
        // tells you nothing, so `i2c_identify` reads both ID registers.
        0x76 | 0x77 => "BMP180/280, BME280, BME680 or SPL06 barometer",
        _ => "unknown device",
    }
}

/// Addresses `i2c_identify` has a real ID-register probe for.
///
/// Used to point the reader at the tool that can give an answer instead of a
/// guess. Extend this when a probe is added to `i2c_identify`.
fn identify_supported(addr: u8) -> bool {
    matches!(addr, 0x29 | 0x68 | 0x69 | 0x76 | 0x77)
}

/// Scan the whole bus once and print a table plus a list of found devices.
async fn scan(i2c: &mut I2cBus) {
    println!("");
    println!("Scanning 0x{:02X}..0x{:02X} ...", SCAN_FIRST, SCAN_LAST);

    // Column header, aligned with the 3-char cells used below.
    print!("   ");
    for col in 0..16u8 {
        print!(" {:X} ", col);
    }
    println!("");

    let mut found_addrs = [0u8; SCAN_COUNT];
    let mut found: usize = 0;
    let mut faults: u32 = 0;
    let mut last_fault: Option<Error> = None;

    for row in (0x00u8..=0x70).step_by(16) {
        print!("{:02X}:", row);

        for col in 0..16u8 {
            let addr = row + col;

            if addr < SCAN_FIRST || addr > SCAN_LAST {
                print!("   ");
                continue;
            }

            match probe(i2c, addr).await {
                Probe::Ack => {
                    print!(" {:02X}", addr);
                    found_addrs[found] = addr;
                    found += 1;
                }
                Probe::Nack => print!(" --"),
                Probe::Fault(e) => {
                    print!(" ??");
                    faults += 1;
                    last_fault = Some(e);
                }
            }
        }

        println!("");
    }

    // Human-readable list of everything that answered.
    if found > 0 {
        let mut probeable = 0usize;

        println!("Found {} device(s):", found);
        for &addr in &found_addrs[..found] {
            print!("  0x{:02X}  {}", addr, device_name(addr));

            if identify_supported(addr) {
                probeable += 1;
                print!("   <- run i2c_identify, TARGET = 0x{:02X}", addr);
            }

            println!("");
        }

        if probeable > 0 {
            println!("");
            println!("The names above are looked up from the address alone, which is only a");
            println!("guess - most of these addresses carry several unrelated parts.");
            println!("Addresses marked above have a real ID-register probe in `i2c_identify`,");
            println!("which reads the chip itself and can tell you which part it actually is.");
        }
    } else {
        println!("No I2C devices found.");
        println!("Check: pull-ups on PB10/PB11, common GND, slave powered, wiring.");
    }

    if faults > 0 {
        println!(
            "Bus faults on {} address(es), e.g. {:?} (check wiring / pull-ups).",
            faults,
            last_fault.unwrap_or(Error::Bus)
        );
    }

    println!("");
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 I2C bus scanner");
    println!("  I2C2: SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());

    // REMAP is inferred as 0 from the PB10/PB11 impls (no remap on these pins).
    let mut i2c = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        I2C_FREQ,
        Default::default(),
    );

    loop {
        scan(&mut i2c).await;
        Timer::after(RESCAN_PERIOD).await;
    }
}
