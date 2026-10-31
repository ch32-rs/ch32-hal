//! Identify an unknown I2C device by reading its chip-ID register.
//!
//! An address alone does not identify a part. `i2c_detect` labels `0x29` as a
//! light sensor because that is the most common part there, but the address is
//! shared by several unrelated chips:
//!
//! | part | kind | ID location | expected |
//! |------|------|-------------|----------|
//! | TSL2561      | ambient light   | reg `0x0A`, cmd `0x8A` | `0x5x` |
//! | TSL2591      | ambient light   | reg `0x12`, cmd `0xB2` | `0x50` |
//! | TCS34725     | RGB colour      | reg `0x12`, cmd `0x92` | `0x44` |
//! | TCS34727     | RGB colour      | reg `0x12`, cmd `0x92` | `0x4D` |
//! | VL53L0X      | ToF distance    | 16-bit reg `0x00C0`    | `0xEE` |
//! | VL53L1X      | ToF distance    | 16-bit reg `0x00C0`    | `0xEA` |
//!
//! At `0x68` / `0x69` the InvenSense IMU family answers `WHO_AM_I` (`0x75`):
//!
//! | part | `WHO_AM_I` |
//! |------|------------|
//! | MPU6050 / MPU6000 / MPU9150 | `0x68` |
//! | MPU6500 class               | `0x70` |
//! | MPU9250                     | `0x71` |
//! | MPU9255                     | `0x73` |
//!
//! That address is shared with the **DS1307/DS3231 RTC**, which has no ID
//! register at all, so `i2c_detect` labelling `0x68` as "RTC / MPU6050" is a
//! genuine ambiguity rather than a missing entry.
//!
//! At `0x76` / `0x77` two unrelated barometer families share the address, and
//! they do not even use the same ID register:
//!
//! | part | ID register | value |
//! |------|-------------|-------|
//! | BMP180 / BMP085 | `0xD0` | `0x55` (documented as "version") |
//! | BMP280          | `0xD0` | `0x58` |
//! | BME280          | `0xD0` | `0x60` |
//! | BME680 / BME688 | `0xD0` | `0x61` |
//! | SPL06-001 / SPL06-007 | **`0x0D`** | `0x10` (high nibble product, low revision) |
//! | MS5611          | - | no ID register; uses PROM reads, not registers |
//!
//! That is why both `0xD0` and `0x0D` are read below: probing only one of them
//! silently misses whichever family uses the other. Reads are harmless on all of
//! these parts, so the probe order does not matter.
//!
//! `0x77` is what a part answers on with `SDO` tied **high**, and `0x76` with it
//! tied low - so seeing `0x77` is not evidence of anything until an ID register
//! has been read.
//!
//! This demo reads those registers and prints what answered, so the part is
//! identified from silicon rather than guessed from a table.
//!
//! Wiring is the usual I2C2 pair: PB10 = SCL, PB11 = SDA, with pull-ups.
//!
//! Two caveats worth knowing:
//!
//! * The 16-bit probe (`0x00C0`) writes two bytes, which on a TSL/TCS part is
//!   interpreted as a command plus a data byte rather than an address. It runs
//!   last, and the worst case is clearing an interrupt or changing a power
//!   state - nothing permanent.
//! * Some parts only NACK a command they do not implement, some answer
//!   nonsense. Both are informative: the raw values are printed either way.

#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::i2c::{Error, I2c};
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, println};
use panic_halt as _;
use qingke::riscv;

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Blocking>;

/// Address reported by `i2c_detect`. Change this to probe something else.
///
/// Five families are handled so far:
///
/// * `0x29` - the light/colour/ToF ID registers.
/// * `0x1E` - the HMC5883L magnetometer, whose ID is three ASCII registers
///   spelling `H43` rather than one byte.
/// * `0x1C` / `0x1D` / `0x53` - the ADXL345 accelerometer, which answers on
///   `0x1D` or `0x53` depending on its `ALT-ADDRESS` pin. `0x53` overlaps the
///   24Cxx EEPROM range, which is why a scanner guesses wrong there.
/// * `0x68` / `0x69` - **three** different parts with three different ID
///   registers: InvenSense IMUs at `0x75`, the ST L3G4200D gyro at `0x0F`
///   (reading `0xD3`), and the DS1307/DS3231 RTC, which has no ID register at
///   all. Reading only `0x75` silently misses the ST part.
/// * `0x76` / `0x77` - **both** Bosch-style registers: `0xD0` for the Bosch line
///   (BMP085/180, BMP280, BME280, BME680/688) and `0x0D` for the
///   Goertek/Infineon SPL06-001/007. Different registers, so probing only one
///   misses the other family.
///
/// A GY-80 module carries HMC5883L (`0x1E`), ADXL345 (`0x53`), L3G4200D
/// (`0x69`) and BMP085 (`0x77`) - so all four of its addresses are covered.
const TARGET: u8 = 0x29;

/// Bus speed; 100 kHz is safe for every candidate part.
const I2C_FREQ: Hertz = Hertz::khz(100);

/// Write a command (one or two register-address bytes) and read one byte back.
///
/// Prints the outcome either way and returns the byte when the part answered.
fn read_register(i2c: &mut I2cBus, what: &str, command: &[u8]) -> Option<u8> {
    let mut value = [0u8; 1];

    match i2c.blocking_write_read(TARGET, command, &mut value) {
        Ok(()) => {
            println!("  {:<38} -> 0x{:02X}", what, value[0]);
            Some(value[0])
        }
        Err(Error::Nack) => {
            println!("  {:<38} -> no ack", what);
            None
        }
        Err(e) => {
            println!("  {:<38} -> error {:?}", what, e);
            None
        }
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 I2C chip identification");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");
    println!("Reading ID registers of the parts that share 0x{:02X}:", TARGET);
    println!("");

    let mut i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, I2C_FREQ, Default::default());

    // Different address families need different ID registers, so dispatch on the
    // address the scanner reported.
    match TARGET {
        // MPU6050 and friends, the ST gyro that shares the address, and the
        // DS1307/DS3231 RTC - three different ID registers on one address.
        0x68 | 0x69 => probe_imu(&mut i2c),
        // The Bosch pressure-sensor line (and the SPL06, which uses a different
        // register entirely).
        0x76 | 0x77 => probe_pressure(&mut i2c),
        // Magnetometer and accelerometer addresses, including the alternates an
        // ADXL345 answers on when its ALT-ADDRESS pin is strapped high.
        0x1E => probe_hmc5883l(&mut i2c),
        0x1C | 0x1D | 0x53 => probe_adxl345(&mut i2c),
        _ => probe_0x29(&mut i2c),
    }

    println!("");
    println!("Done. Nothing was written except the register addresses above.");

    loop {
        riscv::asm::delay(50_000_000);
    }
}

/// Parts that share `0x29`: light sensors, an RGB colour sensor and a ToF ranger.
fn probe_0x29(i2c: &mut I2cBus) {
    // Read-only-ish single-byte-command probes first.
    let tcs = read_register(i2c, "TCS3472x ID   (reg 0x12, cmd 0x92)", &[0x92]);
    let tsl2591 = read_register(i2c, "TSL2591 ID    (reg 0x12, cmd 0xB2)", &[0xB2]);
    let tsl2561 = read_register(i2c, "TSL2561 ID    (reg 0x0A, cmd 0x8A)", &[0x8A]);

    // The 16-bit-address probe runs last; see the caveat in the module docs.
    let vl_model = read_register(i2c, "VL53L0X model (16-bit reg 0x00C0)", &[0xC0, 0x00]);
    let vl_rev = read_register(i2c, "VL53L0X rev   (16-bit reg 0x00C2)", &[0xC2, 0x00]);

    println!("");
    println!("---- identification ----");

    let mut found = false;

    match vl_model {
        Some(0xEE) => {
            println!("  VL53L0X time-of-flight distance sensor (not a light sensor!)");
            if let Some(rev) = vl_rev {
                println!("    revision 0x{:02X}", rev);
            }
            found = true;
        }
        Some(0xEA) => {
            println!("  VL53L1X time-of-flight distance sensor (not a light sensor!)");
            found = true;
        }
        _ => {}
    }

    match tcs {
        Some(0x44) => {
            println!("  TCS34725 RGB colour sensor (ID 0x44)");
            found = true;
        }
        Some(0x4D) => {
            println!("  TCS34727 RGB colour sensor (ID 0x4D)");
            found = true;
        }
        _ => {}
    }

    if tcs == Some(0x50) || tsl2591 == Some(0x50) {
        println!("  TSL2591 ambient light sensor (ID 0x50)");
        found = true;
    }

    // TSL2561 puts the part number in the high nibble: 0x5x.
    if let Some(id) = tsl2561 {
        if id & 0xF0 == 0x50 {
            println!(
                "  TSL2561 ambient light sensor (ID 0x{:02X}: part 0x5, rev 0x{:X})",
                id,
                id & 0x0F
            );
            found = true;
        }
    }

    if !found {
        println!("  No known part matched. The raw values above are still useful:");
        println!("  a distinct, stable byte for one command is the chip's ID even if");
        println!("  it is not in this table - paste the output and we can look it up.");
    }
}

/// Barometers at `0x76`/`0x77`. Two unrelated families share the address and
/// use **different** ID registers, so both are read.
fn probe_pressure(i2c: &mut I2cBus) {
    // Bosch: chip ID in 0xD0. The BMP180 documents the very same register as
    // its "version" byte, which is why it reads 0x55 there.
    let bosch = read_register(i2c, "Bosch chip ID     (reg 0xD0)", &[0xD0]);
    // Goertek SPL06-001 / Infineon SPL06-007: product and revision ID in 0x0D,
    // high nibble product (1), low nibble revision. Nothing to do with 0xD0.
    let spl06 = read_register(i2c, "SPL06 PROD_ID/REV (reg 0x0D)", &[0x0D]);

    println!("");
    println!("---- identification ----");

    match bosch {
        Some(0x55) => {
            println!("  BMP180 / BMP085 pressure sensor (0xD0 = 0x55)");
            read_register(i2c, "CTRL_MEAS  (reg 0xF4)", &[0xF4]);
            return;
        }
        Some(0x58) => {
            println!("  BMP280 pressure sensor (0xD0 = 0x58)");
            read_register(i2c, "CTRL_MEAS  (reg 0xF4)", &[0xF4]);
            return;
        }
        Some(0x60) => {
            println!("  BME280 temperature / humidity / pressure (0xD0 = 0x60)");
            read_register(i2c, "CTRL_MEAS  (reg 0xF4)", &[0xF4]);
            return;
        }
        Some(0x61) => {
            println!("  BME680 temperature / humidity / pressure / gas (0xD0 = 0x61)");
            println!("    (a BME688 answers the same ID; same die, different gas-sensor");
            println!("     variant, and its VOC index needs Bosch's BSEC library)");
            read_register(i2c, "CTRL_MEAS  (reg 0x74)", &[0x74]);
            read_register(i2c, "CTRL_HUM   (reg 0x72)", &[0x72]);
            read_register(i2c, "CTRL_GAS_1 (reg 0x71)", &[0x71]);
            return;
        }
        _ => {}
    }

    // Not a Bosch part. Check the SPL06, which does not use 0xD0 at all - this
    // is the case a 0xD0-only probe gets wrong.
    match spl06 {
        Some(id) if id >> 4 == 0x1 => {
            println!(
                "  SPL06-001 / SPL06-007 barometric pressure sensor (0x0D = 0x{:02X})",
                id
            );
            println!(
                "    product ID {}, revision {} - the two parts share this register map",
                id >> 4,
                id & 0x0F
            );
            read_register(i2c, "MEAS_CFG (reg 0x08)", &[0x08]);
            read_register(i2c, "CFG_REG  (reg 0x09)", &[0x09]);
            return;
        }
        Some(other) => println!(
            "  Unrecognised 0x0D = 0x{:02X} (expected product nibble 1 for an SPL06).",
            other
        ),
        None => println!("  No ID response at 0x0D either."),
    }

    match bosch {
        Some(other) => println!("  0xD0 = 0x{:02X} matched no known part.", other),
        None => {
            println!("  No response at 0xD0.");
            println!("  The MS5611 also sits at 0x76/0x77 but has no register file: it is");
            println!("  read through PROM commands (0xA0..0xAE), so it answers neither");
            println!("  register above.");
        }
    }
}

/// HMC5883L magnetometer at `0x1E`: three ASCII identification registers.
fn probe_hmc5883l(i2c: &mut I2cBus) {
    // Unlike most parts, Honeywell puts the ID in three consecutive registers
    // and it spells "H43" rather than being a single byte.
    let id_a = read_register(i2c, "IDENT_A (reg 0x0A)", &[0x0A]);
    let id_b = read_register(i2c, "IDENT_B (reg 0x0B)", &[0x0B]);
    let id_c = read_register(i2c, "IDENT_C (reg 0x0C)", &[0x0C]);

    println!("");
    println!("---- identification ----");

    match (id_a, id_b, id_c) {
        (Some(0x48), Some(0x34), Some(0x33)) => {
            println!("  HMC5883L 3-axis magnetometer (ID registers spell \"H43\")");
            read_register(i2c, "MODE      (reg 0x02)", &[0x02]);
            read_register(i2c, "CONFIG_A  (reg 0x00)", &[0x00]);
        }
        (Some(a), Some(b), Some(c)) => println!(
            "  ID registers read 0x{:02X} 0x{:02X} 0x{:02X}, expected 0x48 0x34 0x33 (\"H43\")",
            a, b, c
        ),
        _ => {
            println!("  Not all three identification registers answered.");
            println!("  An LSM303 also sits at 0x1E, but its magnetometer has a different");
            println!("  register map.");
        }
    }
}

/// ADXL345 accelerometer, which answers on `0x53` or `0x1D` depending on the
/// `ALT-ADDRESS` pin.
fn probe_adxl345(i2c: &mut I2cBus) {
    let devid = read_register(i2c, "DEVID (reg 0x00)", &[0x00]);

    println!("");
    println!("---- identification ----");

    match devid {
        Some(0xE5) => {
            println!("  ADXL345 3-axis accelerometer (DEVID 0xE5)");
            println!(
                "    ALT-ADDRESS {} -> this is the {} address",
                if TARGET == 0x53 { "high" } else { "low" },
                if TARGET == 0x53 { "0x53" } else { "0x1D" }
            );
            read_register(i2c, "POWER_CTL  (reg 0x2D)", &[0x2D]);
            read_register(i2c, "DATA_FORMAT(reg 0x31)", &[0x31]);
        }
        Some(other) => println!("  DEVID 0x{:02X}, expected 0xE5 - not an ADXL345.", other),
        None => {
            println!("  No DEVID response.");
            println!("  0x53 is also inside the 24Cxx EEPROM range, so a small EEPROM on");
            println!("  a board like this looks the same to a scanner.");
        }
    }
}

/// InvenSense IMU family at `0x68`/`0x69`, the ST gyro that shares it, plus the
/// RTC that shares it too.
fn probe_imu(i2c: &mut I2cBus) {
    // WHO_AM_I is the definitive test: the DS1307/DS3231 RTCs on the same
    // address have no ID register at all.
    let who = read_register(i2c, "WHO_AM_I   (reg 0x75)", &[0x75]);
    // 0x40 after reset on an MPU6050: the sleep bit is set, which is also a
    // useful proof of life when the ID read is ambiguous.
    let pwr = read_register(i2c, "PWR_MGMT_1 (reg 0x6B)", &[0x6B]);
    // ST's L3G4200D sits on the same address and puts its WHO_AM_I somewhere
    // else entirely: register 0x0F, and it reads 0xD3. This is what the GY-80
    // module's gyro answers, and what a scanner calling 0x69 an MPU6050 misses.
    let st_who = read_register(i2c, "L3G4200D WHO_AM_I (reg 0x0F)", &[0x0F]);

    println!("");
    println!("---- identification ----");

    // The ST gyro answers 0xD3 in its own register; check it first, because on
    // this address an MPU6050-style read can return something plausible too.
    if st_who == Some(0xD3) {
        println!("  L3G4200D 3-axis gyroscope (WHO_AM_I reg 0x0F = 0xD3)");
        println!("    not an InvenSense part: this one is ST, with a different");
        println!("    register map, so it needs `edrv-l3g4200d` rather than `edrv-mpu6050`.");
        read_register(i2c, "CTRL_REG1  (reg 0x20)", &[0x20]);
        read_register(i2c, "CTRL_REG4  (reg 0x23)", &[0x23]);
        return;
    }

    match who {
        // 0x68 is shared by these; they differ in which axes/sensors they carry,
        // not in the basic register map this example uses.
        Some(0x68) => println!("  MPU6050 / MPU6000 / MPU9150  (WHO_AM_I 0x68)"),
        Some(0x70) => println!("  MPU6500-class                (WHO_AM_I 0x70)"),
        Some(0x71) => println!("  MPU9250                      (WHO_AM_I 0x71)"),
        Some(0x73) => println!("  MPU9255                      (WHO_AM_I 0x73)"),
        Some(other) => println!(
            "  Unrecognised WHO_AM_I 0x{:02X} - not an InvenSense 6-axis part?",
            other
        ),
        None => {
            println!("  No WHO_AM_I response.");
            println!("  If register 0x00 returns a changing BCD value instead, this is a");
            println!("  DS1307/DS3231 RTC, which has no ID register and does not ACK 0x75.");
        }
    }

    if let Some(value) = pwr {
        println!(
            "  PWR_MGMT_1 = 0x{:02X} (0x40 is the MPU6050 reset default: asleep)",
            value
        );
    }
}
