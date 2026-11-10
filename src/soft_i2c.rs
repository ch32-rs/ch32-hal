//! Software (bit-banged) I2C master.
//!
//! Needed on chips without an I2C peripheral (early CH32X033/X034) and on
//! pins that do not carry an I2C alternate function.
//!
//! CH32X0 GPIO has no general-purpose open-drain output: the only open-drain
//! mode is the I2C alternate function (CH32X035RM §8.2.4). This driver
//! therefore emulates an open-drain bus by switching each pin between
//! "input, externally pulled up" and "push-pull output low". It never drives
//! the bus high, so it is safe to share with slave devices.
//!
//! The bus must have external pull-ups. Internal pull-ups are available but
//! weak and not characterised; at 400 kHz use 4.7 kOhm or stronger.
//!
//! Timing is an instruction-count loop calibrated against the Embassy time
//! driver at construction time, because a loop executing from flash runs far
//! slower than one cycle per instruction. Treat [`Config::frequency`] as an
//! upper bound: the driver rounds the half period up, so SCL never exceeds
//! the requested rate. Without the `embassy` feature the loop assumes two
//! cycles per iteration and the bus comes out slower; that is harmless.

use core::arch::asm;

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c as I2cTrait, NoAcknowledgeSource, Operation};

use crate::gpio::{Flex, Pin, Pull, SealedPin, Speed};
use crate::Peri;

/// Software I2C configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Target SCL frequency in Hz. Upper bound, see the module docs.
    pub frequency: u32,
    /// Pull setting used while the pin is released.
    ///
    /// Use [`Pull::None`] when the bus has external pull-ups; use
    /// [`Pull::Up`] only for short, slow buses without them.
    pub pull: Pull,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 100_000,
            pull: Pull::Up,
        }
    }
}

/// Software I2C error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The slave did not acknowledge the address or a data byte.
    Nack,
    /// SCL stayed low past the clock-stretching timeout.
    Bus,
    /// The address is not a valid 7-bit address.
    InvalidAddress,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Nack => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            Self::Bus => ErrorKind::Bus,
            Self::InvalidAddress => ErrorKind::Other,
        }
    }
}

/// Bit-banged I2C master.
pub struct SoftI2c<'d> {
    scl: Flex<'d>,
    sda: Flex<'d>,
    pull: Pull,
    half_loops: u32,
}

impl<'d> SoftI2c<'d> {
    /// Create a software I2C bus.
    ///
    /// The pins must be usable as GPIO and the bus must be externally pulled
    /// up when `config.pull` is [`Pull::None`].
    pub fn new(scl: Peri<'d, impl Pin>, sda: Peri<'d, impl Pin>, config: Config) -> Self {
        let frequency = config.frequency.max(10_000);

        let mut bus = Self {
            scl: Flex::new(scl),
            sda: Flex::new(sda),
            pull: config.pull,
            half_loops: half_period_loops(frequency),
        };
        bus.release_scl();
        bus.release_sda();
        if bus.sda.is_low() {
            bus.recover();
        }
        bus
    }

    /// Number of delay-loop iterations used for one half period.
    ///
    /// Exposed for bring-up so the actual bus speed can be sanity-checked.
    pub fn half_period_loops(&self) -> u32 {
        self.half_loops
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error> {
        self.check_address(address)?;
        self.start()?;

        if !self.write_byte(address << 1)? {
            self.stop();
            return Err(Error::Nack);
        }
        for &byte in bytes {
            if !self.write_byte(byte)? {
                self.stop();
                return Err(Error::Nack);
            }
        }

        self.stop();
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), Error> {
        self.check_address(address)?;
        self.start()?;

        if !self.write_byte((address << 1) | 1)? {
            self.stop();
            return Err(Error::Nack);
        }
        self.read_bytes(bytes)?;

        self.stop();
        Ok(())
    }

    /// Blocking write followed by a repeated start and a read.
    pub fn blocking_write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.check_address(address)?;
        self.start()?;

        if !self.write_byte(address << 1)? {
            self.stop();
            return Err(Error::Nack);
        }
        for &byte in write {
            if !self.write_byte(byte)? {
                self.stop();
                return Err(Error::Nack);
            }
        }

        self.start()?;
        if !self.write_byte((address << 1) | 1)? {
            self.stop();
            return Err(Error::Nack);
        }
        self.read_bytes(read)?;

        self.stop();
        Ok(())
    }

    fn check_address(&self, address: u8) -> Result<(), Error> {
        if address > 0x7f {
            Err(Error::InvalidAddress)
        } else {
            Ok(())
        }
    }

    fn release_scl(&self) {
        self.scl.pin.set_as_input(self.pull);
    }

    fn release_sda(&self) {
        self.sda.pin.set_as_input(self.pull);
    }

    fn drive_scl_low(&self) {
        self.scl.pin.set_low();
        self.scl.pin.set_as_output(Speed::High);
    }

    fn drive_sda_low(&self) {
        self.sda.pin.set_low();
        self.sda.pin.set_as_output(Speed::High);
    }

    fn delay_half(&self) {
        delay_loops(self.half_loops);
    }

    /// Release SCL and wait until it actually reads high (clock stretching).
    fn scl_high(&self) -> Result<(), Error> {
        self.release_scl();
        for _ in 0..100_000 {
            if self.scl.is_high() {
                return Ok(());
            }
        }
        Err(Error::Bus)
    }

    fn start(&mut self) -> Result<(), Error> {
        // Idle high, then SDA falling while SCL is high.
        self.release_sda();
        self.scl_high()?;
        self.delay_half();
        self.drive_sda_low();
        self.delay_half();
        self.drive_scl_low();
        self.delay_half();
        Ok(())
    }

    fn stop(&mut self) {
        // SDA rising while SCL is high.
        self.drive_scl_low();
        self.drive_sda_low();
        self.delay_half();
        let _ = self.scl_high();
        self.delay_half();
        self.release_sda();
        self.delay_half();
    }

    /// Send one byte, return whether the slave acknowledged it.
    fn write_byte(&mut self, byte: u8) -> Result<bool, Error> {
        for bit in (0..8).rev() {
            if byte & (1 << bit) != 0 {
                self.release_sda();
            } else {
                self.drive_sda_low();
            }
            self.delay_half();
            self.scl_high()?;
            self.delay_half();
            self.drive_scl_low();
            self.delay_half();
        }

        // Ninth clock: release SDA and sample the slave's ACK.
        self.release_sda();
        self.delay_half();
        self.scl_high()?;
        self.delay_half();
        let ack = self.sda.is_low();
        self.drive_scl_low();
        self.delay_half();
        Ok(ack)
    }

    fn read_bytes(&mut self, bytes: &mut [u8]) -> Result<(), Error> {
        let len = bytes.len();
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read_byte(index + 1 != len)?;
        }
        Ok(())
    }

    /// Read one byte and send ACK (or NACK for the last byte).
    fn read_byte(&mut self, ack: bool) -> Result<u8, Error> {
        let mut byte = 0u8;

        self.release_sda();
        for _ in 0..8 {
            self.delay_half();
            self.scl_high()?;
            self.delay_half();
            byte = (byte << 1) | u8::from(self.sda.is_high());
            self.drive_scl_low();
            self.delay_half();
        }

        // Ninth clock: drive ACK low, or leave SDA released for NACK.
        if ack {
            self.drive_sda_low();
        } else {
            self.release_sda();
        }
        self.delay_half();
        self.scl_high()?;
        self.delay_half();
        self.drive_scl_low();
        self.release_sda();
        self.delay_half();

        Ok(byte)
    }

    /// Clock SCL until a stuck-low SDA is released, then send STOP.
    fn recover(&mut self) {
        for _ in 0..9 {
            self.drive_scl_low();
            self.delay_half();
            let _ = self.scl_high();
            self.delay_half();
        }
        self.stop();
    }
}

impl ErrorType for SoftI2c<'_> {
    type Error = Error;
}

impl I2cTrait for SoftI2c<'_> {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        self.check_address(address)?;
        self.start()?;

        let mut first = true;
        for operation in operations.iter_mut() {
            if !first {
                // Repeated start between operations.
                self.start()?;
            }
            first = false;

            match operation {
                Operation::Write(bytes) => {
                    if !self.write_byte(address << 1)? {
                        self.stop();
                        return Err(Error::Nack);
                    }
                    for &byte in bytes.iter() {
                        if !self.write_byte(byte)? {
                            self.stop();
                            return Err(Error::Nack);
                        }
                    }
                }
                Operation::Read(bytes) => {
                    if !self.write_byte((address << 1) | 1)? {
                        self.stop();
                        return Err(Error::Nack);
                    }
                    self.read_bytes(bytes)?;
                }
            }
        }

        self.stop();
        Ok(())
    }
}

/// Number of delay-loop iterations in one SCL half period.
///
/// The loop executes from flash, so measuring it is the only reliable way to
/// know its rate. Calibration needs the Embassy time driver, which
/// `hal::init()` starts; without it, fall back to the nominal two cycles per
/// iteration (the bus will be slower than requested).
fn half_period_loops(frequency: u32) -> u32 {
    #[cfg(feature = "embassy")]
    {
        const CAL_LOOPS: u32 = 10_000;

        let start = embassy_time::Instant::now();
        delay_loops(CAL_LOOPS);
        let elapsed_us = start.elapsed().as_micros().max(1);

        // iterations = CAL_LOOPS * (1e6 / (2 * frequency)) / elapsed_us,
        // rounded up so SCL never exceeds the requested frequency.
        let numerator = CAL_LOOPS as u64 * 1_000_000;
        let denominator = 2 * frequency as u64 * elapsed_us;
        ((numerator + denominator - 1) / denominator).max(1) as u32
    }

    #[cfg(not(feature = "embassy"))]
    {
        let hclk = crate::rcc::clocks().hclk.0;
        (hclk / (4 * frequency)).max(1)
    }
}

/// Busy-wait for `iterations` two-instruction loops.
#[inline(always)]
fn delay_loops(iterations: u32) {
    let mut count = iterations.max(1);
    unsafe {
        asm!(
            "2:",
            "addi {count}, {count}, -1",
            "bne {count}, zero, 2b",
            count = inout(reg) count => _,
            options(nomem, nostack),
        );
    }
}
