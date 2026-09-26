//! MPU6050 6-axis IMU demo for the CH32V208.
//!
//! Uses **our own** `edrv-mpu6050` driver rather than a third-party crate: the
//! only embedded-hal 1.0 crate for this part on crates.io is an unmaintained
//! 0.1.1 upload, while `edrv-mpu6050` is the one we maintain.
//!
//! Wiring (I2C2):
//!
//! ```text
//!   MPU6050 / GY-521      CH32V208
//!   VCC        <----  3V3      (GY-521 has its own regulator, so 5V is fine
//!                               on that board; the bare chip is 3.3V)
//!   GND        <----  GND
//!   SCL        <----  PB10
//!   SDA        <----  PB11
//!   AD0        <----  GND for 0x68, or 3V3 for 0x69
//!   INT        ----   not used here
//! ```
//!
//! # Printing floats is what costs flash, not the driver
//!
//! `edrv-mpu6050` returns `f32` (g, deg/s, Celsius). On this soft-float
//! `riscv32imc` target, `println!("{}", some_f32)` alone pulls in the float
//! formatter and costs about **12.5 KB** of flash - measured, not guessed: the
//! same program was 29.2 KB printing `f32` and 16.3 KB printing fixed-point
//! integers.
//!
//! So the conversions below multiply the `f32` by a scale and print an integer.
//! The driver API is untouched; only the display path avoids floats.
//!
//! # What the numbers mean
//!
//! * Acceleration in **g**, gyro in **degrees/second**, die temperature in
//!   **Celsius**.
//! * At rest the acceleration *magnitude* is ~1 g regardless of orientation,
//!   which is why the self check reports it: it does not depend on how the board
//!   happens to be lying.
//! * A non-zero gyro reading at rest is **bias**, not a fault. Removing it is
//!   what gyro calibration does.
//! * `0x68` is shared with the DS1307/DS3231 RTC, which has no ID register, so
//!   the `WHO_AM_I` read below is what tells them apart.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use ch32_hal as hal;
use edrv_mpu6050::{regs, AccelRange, Config, DlpfCfg, GyroRange, MPU6050};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::i2c::I2c;
use hal::mode::Async;
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals, print, println};
use heapless::String;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    I2C2_EV => hal::i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

/// I2C2 bus: PB10 = SCL, PB11 = SDA.
type I2cBus = I2c<'static, peripherals::I2C2, Async>;
type Imu = MPU6050<I2cBus>;

/// The MPU6050 handles 400 kHz.
const I2C_FREQ: Hertz = Hertz::khz(400);

/// `WHO_AM_I` for an MPU6050, and for the MPU6000/MPU9150 that share its map.
const WHO_AM_I_MPU6050: u8 = 0x68;

/// The MPU6500-class parts answer 0x70 with a compatible register map.
const WHO_AM_I_MPU6500: u8 = 0x70;

/// Pacing between printed samples.
const SAMPLE_PERIOD: Duration = Duration::from_millis(200);

/// Samples averaged by the start-up self check.
const SELF_CHECK_SAMPLES: u32 = 20;

/// Fixed-point scales used for printing.
const SCALE_G: f32 = 1000.0;
const SCALE_DPS: f32 = 1000.0;
const SCALE_C: f32 = 100.0;

/// Scale an `f32` reading to a fixed-point integer for display.
fn fixed(value: f32, scale: f32) -> i32 {
    (value * scale) as i32
}

/// Format a value scaled by 1000 as fixed point: `-1234` -> `-1.234`.
fn fx3(value: i32) -> String<12> {
    let mut text = String::new();
    let magnitude = value.unsigned_abs();
    let _ = write!(
        text,
        "{}{}.{:03}",
        if value < 0 { "-" } else { "" },
        magnitude / 1000,
        magnitude % 1000
    );
    text
}

/// Format a value scaled by 100 as fixed point: `3653` -> `36.53`.
fn fx2(value: i32) -> String<12> {
    let mut text = String::new();
    let magnitude = value.unsigned_abs();
    let _ = write!(
        text,
        "{}{}.{:02}",
        if value < 0 { "-" } else { "" },
        magnitude / 100,
        magnitude % 100
    );
    text
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

/// `|a|` in thousandths of a g, from the fixed-point components.
fn magnitude_mg(accel_g: (f32, f32, f32)) -> i32 {
    let (x, y, z) = accel_g;
    let x = fixed(x, SCALE_G) as i64;
    let y = fixed(y, SCALE_G) as i64;
    let z = fixed(z, SCALE_G) as i64;
    isqrt((x * x + y * y + z * z) as u64) as i32
}

/// Plain-language reading of the acceleration magnitude (argument in mg).
fn gravity_hint(magnitude_mg: i32) -> &'static str {
    if (magnitude_mg - 1000).abs() < 150 {
        "~1 g, at rest"
    } else if magnitude_mg < 500 {
        "well under 1 g - check wiring and the range setting"
    } else {
        "not 1 g - accelerating, or the sensor needs calibration"
    }
}

/// Park forever after a fatal setup problem.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

/// Averaged read while the board is still: proves the part is alive and sane.
///
/// An MPU6050 at rest gives ~1 g of magnitude and a small, roughly constant gyro
/// reading. A dead, mis-wired or fake part tends to give zeros or wild values.
async fn self_check(imu: &mut Imu) {
    println!("---- self check: keep the board still ----");

    let mut samples = 0u32;
    let mut accel_sum = [0i64; 3];
    let mut gyro_sum = [0i64; 3];
    let mut magnitude_sum = 0i64;
    let mut temp_sum = 0i64;

    for _ in 0..SELF_CHECK_SAMPLES {
        if let Ok((ax, ay, az)) = imu.read_accel().await {
            accel_sum[0] += fixed(ax, SCALE_G) as i64;
            accel_sum[1] += fixed(ay, SCALE_G) as i64;
            accel_sum[2] += fixed(az, SCALE_G) as i64;
            magnitude_sum += magnitude_mg((ax, ay, az)) as i64;

            if let Ok((gx, gy, gz)) = imu.read_gyro().await {
                gyro_sum[0] += fixed(gx, SCALE_DPS) as i64;
                gyro_sum[1] += fixed(gy, SCALE_DPS) as i64;
                gyro_sum[2] += fixed(gz, SCALE_DPS) as i64;
            }
            if let Ok(temp) = imu.read_temperature().await {
                temp_sum += fixed(temp, SCALE_C) as i64;
                samples += 1;
            }
        }
        Timer::after(Duration::from_millis(50)).await;
    }

    if samples == 0 {
        println!("  no readable samples - check the wiring.");
        return;
    }

    let n = samples as i64;
    let magnitude_mg = (magnitude_sum / n) as i32;

    println!("  samples   : {}", samples);
    println!("  mean |a|  : {} g   {}", fx3(magnitude_mg), gravity_hint(magnitude_mg));
    println!(
        "  mean accel: x {}  y {}  z {} g",
        fx3((accel_sum[0] / n) as i32),
        fx3((accel_sum[1] / n) as i32),
        fx3((accel_sum[2] / n) as i32)
    );
    println!(
        "  gyro bias : x {}  y {}  z {} deg/s",
        fx3((gyro_sum[0] / n) as i32),
        fx3((gyro_sum[1] / n) as i32),
        fx3((gyro_sum[2] / n) as i32)
    );
    println!("  die temp  : {} C", fx2((temp_sum / n) as i32));
    println!("  (a small non-zero gyro bias is normal and is removed by calibration,");
    println!("   not a fault)");
    println!("");
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 MPU6050 6-axis IMU");
    println!("  I2C2  SCL=PB10  SDA=PB11  @ {} Hz", I2C_FREQ.0);
    println!("  driver: edrv-mpu6050");
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
    let mut imu = MPU6050::new_primary(i2c);

    // Identify before configuring: writing config registers to whatever else
    // might live at 0x68 (an RTC, for instance) is not harmless. The driver
    // checks this inside `init` too, but reading it here lets us name the chip.
    match imu.read_reg(regs::WHO_AM_I).await {
        Ok(WHO_AM_I_MPU6050) => println!("WHO_AM_I = 0x68 -> MPU6050 (or MPU6000/MPU9150)"),
        Ok(WHO_AM_I_MPU6500) => {
            println!("WHO_AM_I = 0x70 -> MPU6500-class; the register map used here is compatible")
        }
        Ok(other) => {
            println!(
                "WHO_AM_I = 0x{:02X}, expected 0x{:02X} - not an MPU6050.",
                other, WHO_AM_I_MPU6050
            );
            println!("Nothing was configured. Check AD0 (it selects 0x68 vs 0x69).");
            park().await;
        }
        Err(e) => {
            println!("No WHO_AM_I response: {:?}", e);
            println!("A DS1307/DS3231 RTC also answers at 0x68 but has no ID register;");
            println!("run `i2c_identify` if you are not sure what is on the bus.");
            park().await;
        }
    }
    println!("");

    let config = Config {
        lpf: DlpfCfg::Hz94,
        gyro_range: GyroRange::Deg250,
        accel_range: AccelRange::G2,
    };

    if let Err(e) = imu.init(config).await {
        println!("configuration failed: {:?}", e);
        println!("Check 3V3, GND and that SDA/SCL are not swapped.");
        park().await;
    }

    println!("accel +/-2 g   gyro +/-250 deg/s   dlpf 94 Hz");
    println!("");

    self_check(&mut imu).await;

    let mut count: u32 = 0;
    loop {
        count += 1;

        match imu.read_accel().await {
            Ok((ax, ay, az)) => {
                let magnitude = magnitude_mg((ax, ay, az));
                print!(
                    "#{:<4} a[{:>8} {:>8} {:>8}] g  ",
                    count,
                    fx3(fixed(ax, SCALE_G)),
                    fx3(fixed(ay, SCALE_G)),
                    fx3(fixed(az, SCALE_G))
                );

                match imu.read_gyro().await {
                    Ok((gx, gy, gz)) => print!(
                        "gyro[{:>8} {:>8} {:>8}] deg/s  ",
                        fx3(fixed(gx, SCALE_DPS)),
                        fx3(fixed(gy, SCALE_DPS)),
                        fx3(fixed(gz, SCALE_DPS))
                    ),
                    Err(e) => print!("gyro err {:?}  ", e),
                }

                match imu.read_temperature().await {
                    Ok(temp) => println!("{:>6} C  |a| {} g", fx2(fixed(temp, SCALE_C)), fx3(magnitude)),
                    Err(e) => println!("temp err {:?}", e),
                }
            }
            Err(e) => println!("#{:<4} read error: {:?}", count, e),
        }

        Timer::after(SAMPLE_PERIOD).await;
    }
}
