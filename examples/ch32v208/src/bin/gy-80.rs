//! GY-80 10-DOF module demo for the CH32V208 - four sensors on one I2C bus.
//!
//! ```text
//!   GY-80 module         CH32V208     I2C address
//!   VCC_IN   <----  3V3 or 5V         (the board has its own regulator)
//!   GND      <----  GND
//!   SCL      <----  PB10              0x1E  HMC5883L  3-axis magnetometer
//!   SDA      <----  PB11              0x53  ADXL345   3-axis accelerometer
//!                                     0x69  L3G4200D  3-axis gyroscope
//!                                     0x77  BMP085    barometric pressure
//! ```
//!
//! The GY-80 is a fixed combination of four separate chips, none of which share
//! an ID register, a bus protocol quirk or a driver. What they *do* share is the
//! bus - which is the actual subject of this example.
//!
//! # Sharing one bus between four drivers
//!
//! Every edrv driver takes ownership of its transport, so four drivers need four
//! views of one `I2c`. That is what `embassy_embedded_hal`'s `I2cDevice` is for:
//! one `Mutex` owns the bus, and each driver is handed a handle that locks it for
//! the duration of a transaction. It is a *lock*, not a separate bus, so a
//! driver that forgets to release would wedge the others - which is also why the
//! bus is held for one I2C transaction at a time rather than for a whole read.
//!
//! # Addresses are straps, not identities
//!
//! Three of these four sit on addresses that depend on a pin, and two of them
//! collide with unrelated parts:
//!
//! | address | chip | why that address |
//! |---|---|---|
//! | `0x1E` | HMC5883L | fixed |
//! | `0x53` | ADXL345 | `ALT-ADDRESS` high; low gives `0x1D`. Also the 24Cxx EEPROM range |
//! | `0x69` | L3G4200D | `SDO` high; low gives `0x68`. Also MPU6050 and the DS1307 RTC |
//! | `0x77` | BMP085 | fixed; also BMP180/BMP280/BME280/BME680/SPL06 and the MS5611 |
//!
//! Every driver validates its own ID register in `init` and returns
//! `InvalidDevice` otherwise, so a wrong strap shows up as a failed `init` for
//! that one device rather than as plausible-looking numbers. `i2c_identify` will
//! name them from the silicon if the addresses here do not match your board.
//!
//! # Units
//!
//! * accelerometer - g, converted here to thousandths of a g
//! * magnetometer - **milligauss**, per the HMC5883L's `mG/LSB` resolution table
//! * gyroscope - milli-degrees per second on the integer path
//! * barometer - pascals, and degrees Celsius
//!
//! # The gyro driver offers both a float and an exact integer path
//!
//! `edrv-l3g4200d` returns the angular rate two ways, and this example uses the
//! integer one:
//!
//! * `read_raw()` - the output registers, in LSB
//! * `read_gyro()` - `f32` degrees per second, matching `read_accel` elsewhere
//! * `read_gyro_mdps()` - `i32` **milli**-degrees per second, no floating point
//!
//! The integer path exists because the datasheet sensitivities (8.75 / 17.5 /
//! 70 mdps per LSB) are not whole numbers: it works in quarters and divides once,
//! so nothing is lost but a sub-mdps remainder.
//!
//! This replaced a `read_gyro` that returned **whole** degrees per second by
//! casting `raw * dps_per_digit` straight to `i16`. At the default +/-2000 dps
//! that made one output step 14 LSB, so ordinary sub-degree readings - which is
//! all a still board ever produces - collapsed to `0`.
//!
//! # Floats cost flash, so nothing here formats one
//!
//! `riscv32imc` is soft-float and `println!("{}", some_f32)` alone pulls in the
//! float formatter for about **12.5 KB** of flash. Every value below is scaled to
//! an integer and the decimal point is placed by hand.

#![no_std]
#![no_main]

use ch32_hal as hal;
use edrv_adxl345::{Config as AccelConfig, ADXL345};
use edrv_bmp180::{BMP180, Config as BaroConfig};
use edrv_hmc5883l::{Config as MagConfig, HMC5883L};
use edrv_l3g4200d::{Config as GyroConfig, L3G4200D};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
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

/// The bus, owned by exactly one mutex and lent to each driver in turn.
///
/// The `I2cDevice` handles are deliberately *not* behind a `'static` alias: the
/// mutex lives on `main`'s stack, so each handle borrows it for as long as the
/// driver that owns it. Naming the handle type with `'static` would demand a
/// `static` mutex for no reason.
type I2cBus = Mutex<NoopRawMutex, I2c<'static, peripherals::I2C2, Async>>;

/// The GY-80 runs its I2C at 400 kHz; the HMC5883L is the slowest part here but
/// still handles it.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// Pace between full sweeps of the four sensors.
const SAMPLE_PERIOD: Duration = Duration::from_millis(500);

/// Park forever after a fatal bus problem.
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
    println!("  CH32V208 GY-80 10-DOF module");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  0x1E HMC5883L  0x53 ADXL345  0x69 L3G4200D  0x77 BMP085");
    println!("  drivers: edrv-adxl345 / -hmc5883l / -l3g4200d / -bmp180");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    // REMAP is inferred as 0 from the PB10/PB11 impls.
    let i2c: I2c<'static, peripherals::I2C2, Async> = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        I2C_FREQ,
        Default::default(),
    );

    // One bus, four handles. Each handle locks the mutex for a single I2C
    // transaction, so the drivers interleave safely.
    let i2c_bus: I2cBus = Mutex::new(i2c);

    let mut accel = ADXL345::new(
        I2cDevice::new(&i2c_bus),
        edrv_adxl345::PRIMARY_ADDRESS, // 0x53: ALT-ADDRESS strapped high
    );
    let mut mag = HMC5883L::new(I2cDevice::new(&i2c_bus), edrv_hmc5883l::ADDRESS);
    let mut gyro = L3G4200D::new(
        I2cDevice::new(&i2c_bus),
        edrv_l3g4200d::ADDRESS, // 0x69: SDO strapped high
    );
    let mut baro = BMP180::new(I2cDevice::new(&i2c_bus), edrv_bmp180::ADDRESS);

    // ---- bring-up -------------------------------------------------------
    // Every `init` checks that device's ID register, so a failure here names the
    // one device that did not answer rather than poisoning the whole demo. A
    // GY-80 with one dead chip still runs the other three.
    println!("---- bring-up ----");

    let accel_ok = match accel.init(AccelConfig::default(), &mut Delay).await {
        Ok(()) => {
            // `init` waits for the first sample, so the first read is real.
            println!("  ADXL345   0x53  init ok");
            true
        }
        Err(e) => {
            println!("  ADXL345   0x53  FAILED: {:?}", e);
            println!("             expected DEVID 0xE5; try 0x1D if ALT-ADDRESS is low");
            false
        }
    };

    let mag_ok = match mag.init(MagConfig::default(), &mut Delay).await {
        Ok(()) => {
            println!("  HMC5883L  0x1E  init ok");
            true
        }
        Err(e) => {
            println!("  HMC5883L  0x1E  FAILED: {:?}", e);
            println!("             expected the ID registers to spell \"H43\"");
            false
        }
    };

    let gyro_ok = match gyro.init(GyroConfig::default(), &mut Delay).await {
        Ok(()) => {
            println!("  L3G4200D  0x69  init ok");
            true
        }
        Err(e) => {
            println!("  L3G4200D  0x69  FAILED: {:?}", e);
            println!("             expected WHO_AM_I (0x0F) = 0xD3; try 0x68 if SDO is low");
            false
        }
    };

    let baro_ok = match baro.init(BaroConfig::default()).await {
        Ok(()) => {
            println!("  BMP085    0x77  init ok");
            true
        }
        Err(e) => {
            println!("  BMP085    0x77  FAILED: {:?}", e);
            println!("             expected version register 0xD0 = 0x55");
            false
        }
    };

    let present = [accel_ok, mag_ok, gyro_ok, baro_ok].iter().filter(|ok| **ok).count();
    println!("");
    println!("  {} of 4 sensors answered.", present);
    if present == 0 {
        println!("");
        println!("  Nothing on the bus. Check 3V3/GND, the pull-ups, and that the");
        println!("  module's own regulator is fed (VCC_IN, not the 3V3 pin, on some");
        println!("  GY-80 revisions).");
        park().await;
    }
    println!("");

    let mut sweep: u32 = 0;

    loop {
        sweep += 1;
        print!("#{:<4} ", sweep);

        // ---- accelerometer: g -> thousandths of a g ----
        if accel_ok {
            match accel.read_accel().await {
                Ok((x, y, z)) => {
                    let (mx, my, mz) = (
                        (x * 1000.0) as i32,
                        (y * 1000.0) as i32,
                        (z * 1000.0) as i32,
                    );
                    let magnitude = {
                        // Integer sqrt so the magnitude needs no libm.
                        let sq = (mx as i64) * (mx as i64)
                            + (my as i64) * (my as i64)
                            + (mz as i64) * (mz as i64);
                        isqrt(sq as u64) as i32
                    };
                    print!("a[{:>5} {:>5} {:>5}]mg ", mx, my, mz);
                    // At rest the magnitude is ~1 g whatever the orientation is,
                    // which is the cheapest "is this part alive and sane" check.
                    if !(850..=1150).contains(&magnitude) {
                        print!("(|a| {}mg !) ", magnitude);
                    }
                }
                Err(e) => print!("accel err {:?} ", e),
            }
        } else {
            // Keep the column so a missing sensor is obvious on every line
            // rather than silently shortening it.
            print!("a[    -    -    -]mg ");
        }

        // ---- magnetometer: already milligauss ----
        if mag_ok {
            match mag.read_measurement().await {
                Ok((x, y, z)) => print!(
                    "m[{:>5} {:>5} {:>5}]mG ",
                    x as i32, y as i32, z as i32
                ),
                Err(e) => print!("mag err {:?} ", e),
            }
        } else {
            print!("m[    -    -    -]mG ");
        }

        // ---- gyroscope: milli-degrees per second, integer path ----
        if gyro_ok {
            match gyro.read_gyro_mdps().await {
                // 0.001 dps resolution: a still board's noise is a few tens of
                // mdps, so it is visible here rather than rounded to zero.
                Ok((x, y, z)) => print!("g[{:>7} {:>7} {:>7}]mdps ", x, y, z),
                Err(e) => print!("gyro err {:?} ", e),
            }
        } else {
            print!("g[      -      -      -]mdps ");
        }

        // ---- barometer: Pa and degC ----
        if baro_ok {
            match baro.read_measurement(&mut Delay).await {
                Ok(m) => {
                    let t = (m.temperature * 100.0) as i32;
                    print!(
                        "{}Pa {}{}.{:02}C ",
                        m.pressure,
                        if t < 0 { "-" } else { "" },
                        t.unsigned_abs() / 100,
                        t.unsigned_abs() % 100
                    );
                }
                Err(e) => print!("baro err {:?} ", e),
            }
        } else {
            print!("-----Pa --.--C ");
        }

        println!("");
        Timer::after(SAMPLE_PERIOD).await;
    }
}

/// Integer square root, so the acceleration magnitude needs no `libm`.
fn isqrt(value: u64) -> u64 {
    if value < 2 {
        return value;
    }
    let mut x = value;
    let mut y = x.div_ceil(2);
    while y < x {
        x = y;
        y = (x + value / x) / 2;
    }
    x
}
