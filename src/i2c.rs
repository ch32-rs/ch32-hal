//! I2C interface.
//!
//! NOTE:

use fugit::HertzU32 as Hertz;

use crate::dma::NoDma;
use crate::gpio::sealed::Pin;
use crate::gpio::Pull;
use crate::interrupt::Interrupt;
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
    pub fn new<const REMAP: bool>(
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

        scl.set_as_af_output(); // auto opendrain
        sda.set_as_af_output(); // auto opendrain

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

    /*fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "time")]
            deadline: Instant::now() + self.timeout,
        }
    }*/

    fn init(&mut self, freq: Hertz, config: Config) {
        let regs = T::regs();

        regs.ctlr1().modify(|_, w| w.swrst().set_bit());
        regs.ctlr1().modify(|_, w| w.swrst().clear_bit());

        let sysclk = crate::rcc::clocks().hclk.to_Hz();
        let sysclk_mhz: u32 = crate::rcc::clocks().hclk.to_MHz();
        let i2c_clk = freq.to_Hz();

        crate::println!("sysclk: {}MHz, i2c_clk: {}Hz", sysclk_mhz, i2c_clk);

        regs.ctlr2().modify(|_, w| w.freq().variant(sysclk_mhz as u8));

        regs.ctlr1().modify(|_, w| w.pe().clear_bit());

        if freq.to_Hz() <= 100_000 {
            let tmp = (sysclk / (i2c_clk * 2)) & 0x0FFF;
            let tmp = u32::max(tmp, 0x04);

            regs.ckcfgr().write(|w| w.ccr().variant(tmp as _));
        } else {
            // high speed, use duty cycle
            let tmp = if config.duty == Duty::Duty2_1 {
                (sysclk / (i2c_clk * 3)) & 0x0FFF
            } else {
                (sysclk / (i2c_clk * 25)) & 0x0FFF
            };
            let tmp = u32::max(tmp, 0x01);

            regs.ckcfgr().write(|w| {
                w.f_s()
                    .set_bit()
                    .duty()
                    .variant(config.duty as u8 != 0)
                    .ccr()
                    .variant(tmp as u16)
            });
        }

        regs.ctlr1().modify(|_, w| w.pe().set_bit());

        // ack=0, master mode
        regs.ctlr1().modify(|_, w| w.ack().clear_bit());
    }

    fn check_and_clear_error_flags(&self) -> Result<crate::pac::i2c1::star1::R, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let star1 = T::regs().star1().read();

        if star1.pecerr().bit() {
            T::regs().star1().modify(|_, w| w.pecerr().clear_bit());
            return Err(Error::Crc);
        }

        if star1.ovr().bit() {
            T::regs().star1().modify(|_, w| w.ovr().clear_bit());
            return Err(Error::Overrun);
        }

        if star1.af().bit() {
            T::regs().star1().modify(|_, w| w.af().clear_bit());
            return Err(Error::Nack);
        }

        if star1.arlo().bit() {
            T::regs().star1().modify(|_, w| w.arlo().clear_bit());
            return Err(Error::Arbitration);
        }

        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if star1.berr().bit() {
            T::regs().star1().modify(|_, w| w.berr().clear_bit());
        }

        Ok(star1)
    }
    /// STAR1 and STAR2 have a complex read-clear rule. So we need to read STAR1 first.
    fn write_bytes(
        &mut self,
        addr: u8,
        bytes: &[u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        // Send a START condition
        let rb = T::regs();

        rb.ctlr1().modify(|_, w| w.start().set_bit());

        // Wait until START condition was generated
        while !self.check_and_clear_error_flags()?.sb().bit() {
            check_timeout()?;
        }

        // Also wait until signalled we're master and everything is waiting for us
        while {
            self.check_and_clear_error_flags()?;

            let sr2 = rb.star2().read();
            !sr2.msl().bit() && !sr2.busy().bit()
        } {
            check_timeout()?;
        }

        // Set up current address, we're trying to talk to
        rb.datar().write(|w| w.datar().variant(addr << 1));

        // Wait until address was sent
        // Wait for the address to be acknowledged
        // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
        while !self.check_and_clear_error_flags()?.addr().bit() {
            check_timeout()?;
        }

        // Clear condition by reading SR2
        let _ = rb.star2().read();

        // Send bytes
        for c in bytes {
            self.send_byte(*c, &check_timeout)?;
        }
        // Fallthrough is success
        Ok(())
    }

    fn send_byte(&self, byte: u8, check_timeout: impl Fn() -> Result<(), Error>) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            !self.check_and_clear_error_flags()?.tx_e().bit()
        } {
            check_timeout()?;
        }

        // Push out a byte of data
        T::regs().datar().write(|w| w.datar().variant(byte));

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            !self.check_and_clear_error_flags()?.btf().bit()
        } {
            check_timeout()?;
        }

        Ok(())
    }

    fn recv_byte(&self, check_timeout: impl Fn() -> Result<(), Error>) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            self.check_and_clear_error_flags()?;

            !T::regs().star1().read().rx_ne().bit()
        } {
            check_timeout()?;
        }

        let value = T::regs().datar().read().datar().bits();
        Ok(value)
    }

    pub fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Send a START condition and set ACK bit
            T::regs().ctlr1().modify(|_, w| w.start().set_bit().ack().set_bit());

            // Wait until START condition was generated
            while !self.check_and_clear_error_flags()?.sb().bit() {
                check_timeout()?;
            }

            // Also wait until signalled we're master and everything is waiting for us
            while {
                let sr2 = T::regs().star2().read();
                !sr2.msl().bit() && !sr2.busy().bit()
            } {
                check_timeout()?;
            }

            // Set up current address, we're trying to talk to
            T::regs().datar().write(|w| w.datar().variant((addr << 1) + 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            while !self.check_and_clear_error_flags()?.addr().bit() {
                check_timeout()?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();

            // Receive bytes into buffer
            for c in buffer {
                *c = self.recv_byte(&check_timeout)?;
            }

            // Prepare to send NACK then STOP after next byte
            T::regs().ctlr1().modify(|_, w| w.ack().clear_bit().stop().set_bit());

            // Receive last byte
            *last = self.recv_byte(&check_timeout)?;

            // Wait for the STOP to be sent.
            while T::regs().ctlr1().read().stop().bit() {
                check_timeout()?;
            }

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::Overrun)
        }
    }

    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, read, || Ok(()))
    }

    pub fn blocking_write_timeout(
        &mut self,
        addr: u8,
        write: &[u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.write_bytes(addr, write, &check_timeout)?;
        // Send a STOP condition
        T::regs().ctlr1().modify(|_, w| w.stop().set_bit());
        // Wait for STOP condition to transmit.
        while T::regs().ctlr1().read().stop().bit() {
            check_timeout()?;
        }

        // Fallthrough is success
        Ok(())
    }

    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        self.blocking_write_timeout(addr, write, || Ok(()))
    }

    pub fn blocking_write_read_timeout(
        &mut self,
        addr: u8,
        write: &[u8],
        read: &mut [u8],
        check_timeout: impl Fn() -> Result<(), Error>,
    ) -> Result<(), Error> {
        self.write_bytes(addr, write, &check_timeout)?;
        self.blocking_read_timeout(addr, read, &check_timeout)?;

        Ok(())
    }

    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        self.blocking_write_read_timeout(addr, write, read, || Ok(()))
    }
}

impl<'d, T: Instance, TXDMA, RXDMA> Drop for I2c<'d, T, TXDMA, RXDMA> {
    fn drop(&mut self) {
        T::regs().ctlr1().modify(|_, w| w.pe().clear_bit());
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Instance {
        fn regs() -> &'static pac::i2c1::RegisterBlock;

        fn set_remap(remap: bool);

        fn enable_and_reset();
    }
}

/// I2C peripheral instance
pub trait Instance: sealed::Instance + 'static {
    // /// Event interrupt for this instance
    // type EventInterrupt: interrupt::typelevel::Interrupt;
    //  /// Error interrupt for this instance
    // type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

impl sealed::Instance for peripherals::I2C1 {
    fn regs() -> &'static pac::i2c1::RegisterBlock {
        unsafe { &*pac::I2C1::PTR }
    }

    fn set_remap(remap: bool) {
        let afio = unsafe { &*pac::AFIO::PTR };

        afio.pcfr().modify(|_, w| w.i2c1rm().variant(remap));
    }

    fn enable_and_reset() {
        let rcc = unsafe { &*pac::RCC::PTR };

        rcc.apb1pcenr().modify(|_, w| w.i2c1en().set_bit());
        rcc.apb1prstr().modify(|_, w| w.i2c1rst().set_bit());
        rcc.apb1prstr().modify(|_, w| w.i2c1rst().clear_bit());
    }
}
impl Instance for peripherals::I2C1 {}

impl sealed::Instance for peripherals::I2C2 {
    fn regs() -> &'static pac::i2c1::RegisterBlock {
        unsafe { &*pac::I2C2::PTR }
    }

    fn set_remap(_remap: bool) {
        // Noop, no remap
    }

    fn enable_and_reset() {
        let rcc = unsafe { &*pac::RCC::PTR };

        rcc.apb1pcenr().modify(|_, w| w.i2c2en().set_bit());
        rcc.apb1prstr().modify(|_, w| w.i2c2rst().set_bit());
        rcc.apb1prstr().modify(|_, w| w.i2c2rst().clear_bit());
    }
}
impl Instance for peripherals::I2C2 {}

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: bool>: crate::gpio::Pin {}
    };
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
// dma_trait!(RxDma, Instance);
// dma_trait!(TxDma, Instance);

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

pin_trait_impl!(crate::i2c::SclPin, I2C1, PB6, false);
pin_trait_impl!(crate::i2c::SdaPin, I2C1, PB7, false);

pin_trait_impl!(crate::i2c::SclPin, I2C1, PB8, true);
pin_trait_impl!(crate::i2c::SdaPin, I2C1, PB9, true);

// Note: There is no remapping functionality on I2C2
pin_trait_impl!(crate::i2c::SclPin, I2C2, PB10, false);
pin_trait_impl!(crate::i2c::SdaPin, I2C2, PB11, false);
