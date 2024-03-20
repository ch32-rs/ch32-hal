//! I2C interface.
//!
//! NOTE:

use core::future::Future;

use crate::dma::NoDma;
// use crate::dma::NoDma;
use crate::gpio::sealed::{AFType, Pin};
use crate::gpio::{Pull, Speed};
use crate::interrupt::Interrupt;
use crate::time::Hertz;
use crate::{interrupt, into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// I2C error.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration lost
    Arbitration,
    /// ACK not received (either to the address or to a data byte)
    Nack,
    /// Timeout
    Timeout,
    /// CRC error
    Crc,
    /// Overrun error
    Overrun,
    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Duty {
    Duty2_1 = 0,
    Duty16_9 = 1,
}

/// I2C config
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Enable internal pullup on SDA.
    ///
    /// Using external pullup resistors is recommended for I2C. If you do
    /// have external pullups you should not enable this.
    pub sda_pullup: bool,
    /// Enable internal pullup on SCL.
    ///
    /// Using external pullup resistors is recommended for I2C. If you do
    /// have external pullups you should not enable this.
    pub scl_pullup: bool,
    /// Timeout.
    #[cfg(feature = "time")]
    pub timeout: embassy_time::Duration,
    pub duty: Duty,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sda_pullup: false,
            scl_pullup: false,
            #[cfg(feature = "time")]
            timeout: embassy_time::Duration::from_millis(1000),
            duty: Duty::Duty2_1,
        }
    }
}

#[derive(Copy, Clone)]
struct Timeout {
    #[cfg(feature = "time")]
    deadline: Instant,
}

#[allow(dead_code)]
impl Timeout {
    #[inline]
    fn check(self) -> Result<(), Error> {
        #[cfg(feature = "time")]
        if Instant::now() > self.deadline {
            return Err(Error::Timeout);
        }

        Ok(())
    }

    #[inline]
    fn with<R>(self, fut: impl Future<Output = Result<R, Error>>) -> impl Future<Output = Result<R, Error>> {
        #[cfg(feature = "time")]
        {
            use futures::FutureExt;

            embassy_futures::select::select(embassy_time::Timer::at(self.deadline), fut).map(|r| match r {
                embassy_futures::select::Either::First(_) => Err(Error::Timeout),
                embassy_futures::select::Either::Second(r) => r,
            })
        }

        #[cfg(not(feature = "time"))]
        fut
    }
}

/// I2C driver.
pub struct I2c<'d, T: Instance, TXDMA = NoDma, RXDMA = NoDma> {
    _peri: PeripheralRef<'d, T>,
    #[allow(dead_code)]
    tx_dma: PeripheralRef<'d, TXDMA>,
    #[allow(dead_code)]
    rx_dma: PeripheralRef<'d, RXDMA>,
    #[cfg(feature = "time")]
    timeout: Duration,
}

impl<'d, T: Instance, TXDMA, RXDMA> I2c<'d, T, TXDMA, RXDMA> {
    /// Create a new I2C driver.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T, REMAP>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = TXDMA> + 'd,
        rx_dma: impl Peripheral<P = RXDMA> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda, tx_dma, rx_dma);

        T::enable_and_reset();

        T::set_remap(REMAP);

        // CH32V2, CH32V3
        scl.set_as_af_output(AFType::OutputOpenDrain, Speed::High);
        sda.set_as_af_output(AFType::OutputOpenDrain, Speed::High);

        let mut this = Self {
            _peri: peri,
            tx_dma,
            rx_dma,
            #[cfg(feature = "time")]
            timeout: config.timeout,
        };

        this.init(freq, config);

        this
    }

    fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "time")]
            deadline: Instant::now() + self.timeout,
        }
    }

    // init as master mode
    fn init(&mut self, freq: Hertz, config: Config) {
        let regs = T::regs();

        // soft reset
        //regs.ctlr1().modify(|w| w.set_swrst(true));
        //regs.ctlr1().modify(|w| w.set_swrst(false));

        regs.ctlr1().modify(|w| w.set_pe(false)); // disale i2c

        let freq_in = crate::rcc::clocks().pclk1.0;
        let freq_range = freq_in / 1_000_000;

        assert!(freq_range >= 2 && freq_range <= 36); // MHz

        regs.ctlr2().modify(|w| w.set_freq(freq_range as u8)); // set i2c clock in

        let i2c_clk = freq.0;
        if i2c_clk <= 100_000 {
            let tmp = freq_in / (i2c_clk * 2);
            let tmp = u32::max(tmp, 0x04);

            regs.rtr().write(|w| w.set_trise((freq_range + 1) as u8));
            regs.ckcfgr().write(|w| {
                // w.set_f_s(false);
                w.set_ccr(tmp as _);
            });
        } else {
            // high speed, use duty cycle
            let tmp = if config.duty == Duty::Duty2_1 {
                freq_in / (i2c_clk * 3)
            } else {
                freq_in / (i2c_clk * 25)
            };
            let tmp = u32::max(tmp, 0x01);

            regs.rtr().write(|w| w.set_trise((freq_range * 300 / 1000 + 1) as u8));
            regs.ckcfgr().write(|w| {
                w.set_f_s(true);
                w.set_duty(config.duty as u8 != 0);
                w.set_ccr(tmp as u16);
            });
        }

        regs.ctlr1().modify(|w| w.set_pe(true));
    }

    fn check_and_clear_error_flags(&self) -> Result<crate::pac::i2c::regs::Star1, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let star1 = T::regs().star1().read();

        if star1.timeout() {
            T::regs().star1().modify(|w| w.set_timeout(false));
            return Err(Error::Timeout);
        }

        if star1.pecerr() {
            T::regs().star1().modify(|w| w.set_pecerr(false));
            return Err(Error::Crc);
        }

        if star1.ovr() {
            T::regs().star1().modify(|w| w.set_ovr(false));
            return Err(Error::Overrun);
        }

        if star1.af() {
            T::regs().star1().modify(|w| w.set_af(false));
            return Err(Error::Nack);
        }

        if star1.arlo() {
            T::regs().star1().modify(|w| w.set_arlo(false));
            return Err(Error::Arbitration);
        }

        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if star1.berr() {
            T::regs().star1().modify(|w| w.set_berr(false));
        }

        Ok(star1)
    }
    /// STAR1 and STAR2 have a complex read-clear rule. So we need to read STAR1 first.
    fn write_bytes(&mut self, addr: u8, bytes: &[u8], timeout: Timeout) -> Result<(), Error> {
        // Send a START condition
        let regs = T::regs();

        regs.ctlr1().modify(|w| w.set_start(true));

        // Wait until START condition was generated
        while !self.check_and_clear_error_flags()?.sb() {
            timeout.check()?;
        }

        // Also wait until signalled we're master and everything is waiting for us
        while {
            self.check_and_clear_error_flags()?;

            let sr2 = regs.star2().read();
            !sr2.msl() && !sr2.busy()
        } {
            timeout.check()?;
        }

        // Set up current address, we're trying to talk to
        regs.datar().write(|w| w.set_datar(addr << 1));

        // Wait until address was sent
        // Wait for the address to be acknowledged
        // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
        while !self.check_and_clear_error_flags()?.addr() {
            timeout.check()?;
        }

        // Clear condition by reading SR2
        let _ = regs.star2().read();

        // Send bytes
        for c in bytes {
            self.send_byte(*c, timeout)?;
        }
        // Fallthrough is success
        Ok(())
    }

    fn send_byte(&self, byte: u8, timeout: Timeout) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            !self.check_and_clear_error_flags()?.tx_e()
        } {
            timeout.check()?;
        }

        // Push out a byte of data
        T::regs().datar().write(|w| w.set_datar(byte));

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            !self.check_and_clear_error_flags()?.btf()
        } {
            timeout.check()?;
        }

        Ok(())
    }

    fn recv_byte(&self, timeout: Timeout) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            self.check_and_clear_error_flags()?;

            !T::regs().star1().read().rx_ne()
        } {
            timeout.check()?;
        }

        let value = T::regs().datar().read().datar();
        Ok(value)
    }

    fn blocking_read_timeout(&mut self, addr: u8, buffer: &mut [u8], timeout: Timeout) -> Result<(), Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Send a START condition and set ACK bit
            T::regs().ctlr1().modify(|w| {
                w.set_start(true);
                w.set_ack(true);
            });

            // Wait until START condition was generated
            while !self.check_and_clear_error_flags()?.sb() {
                timeout.check()?;
            }

            // Also wait until signalled we're master and everything is waiting for us
            while {
                let sr2 = T::regs().star2().read();
                !sr2.msl() && !sr2.busy()
            } {
                timeout.check()?;
            }

            // Set up current address, we're trying to talk to
            T::regs().datar().write(|w| w.set_datar((addr << 1) + 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            while !self.check_and_clear_error_flags()?.addr() {
                timeout.check()?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();

            // Receive bytes into buffer
            for c in buffer {
                *c = self.recv_byte(timeout)?;
            }

            // Prepare to send NACK then STOP after next byte
            T::regs().ctlr1().modify(|w| {
                w.set_ack(false);
                w.set_stop(true);
            });

            // Receive last byte
            *last = self.recv_byte(timeout)?;

            // Wait for the STOP to be sent.
            while T::regs().ctlr1().read().stop() {
                timeout.check()?;
            }

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::Overrun)
        }
    }

    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, read, self.timeout())
    }

    fn blocking_write_timeout(&mut self, addr: u8, write: &[u8], timeout: Timeout) -> Result<(), Error> {
        self.write_bytes(addr, write, timeout)?;
        // Send a STOP condition
        T::regs().ctlr1().modify(|w| w.set_stop(true));
        // Wait for STOP condition to transmit.
        while T::regs().ctlr1().read().stop() {
            timeout.check()?;
        }

        // Fallthrough is success
        Ok(())
    }

    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        let timeout = self.timeout();

        self.blocking_write_timeout(addr, write, timeout)
    }

    fn blocking_write_read_timeout(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        timeout: Timeout,
    ) -> Result<(), Error> {
        self.write_bytes(addr, write, timeout)?;
        self.blocking_read_timeout(addr, read, timeout)?;

        Ok(())
    }

    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        let timeout = self.timeout();

        self.blocking_write_read_timeout(addr, write, read, timeout)
    }
}

impl<'d, T: Instance, TXDMA, RXDMA> Drop for I2c<'d, T, TXDMA, RXDMA> {
    fn drop(&mut self) {
        T::regs().ctlr1().modify(|w| w.set_pe(false));
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> pac::i2c::I2c;
    }
}

/// I2C peripheral instance
pub trait Instance:
    sealed::Instance + crate::peripheral::RccPeripheral + crate::peripheral::RemapPeripheral + 'static
{
    // /// Event interrupt for this instance
    // type EventInterrupt: interrupt::typelevel::Interrupt;
    //  /// Error interrupt for this instance
    // type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

impl sealed::Instance for peripherals::I2C1 {
    fn regs() -> pac::i2c::I2c {
        pac::I2C1
    }
}
impl Instance for peripherals::I2C1 {}

impl sealed::Instance for peripherals::I2C2 {
    fn regs() -> pac::i2c::I2c {
        pac::I2C2
    }
}
impl crate::peripheral::sealed::RemapPeripheral for peripherals::I2C2 {
    fn set_remap(_remap: u8) {
        // nop
    }
}
impl crate::peripheral::RemapPeripheral for peripherals::I2C2 {}
impl Instance for peripherals::I2C2 {}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
// dma_trait!(RxDma, Instance);
// dma_trait!(TxDma, Instance);

pin_trait_impl!(crate::i2c::SclPin, I2C1, PB6, 0);
pin_trait_impl!(crate::i2c::SdaPin, I2C1, PB7, 0);

pin_trait_impl!(crate::i2c::SclPin, I2C1, PB8, 1);
pin_trait_impl!(crate::i2c::SdaPin, I2C1, PB9, 1);

// Note: There is no remapping functionality on I2C2
pin_trait_impl!(crate::i2c::SclPin, I2C2, PB10, 0);
pin_trait_impl!(crate::i2c::SdaPin, I2C2, PB11, 0);
