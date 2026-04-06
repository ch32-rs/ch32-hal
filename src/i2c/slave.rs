//! I2C slave driver.
//!
//! Provides async I2C slave support for CH32 microcontrollers.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use crate::gpio::{AFType, Speed};
use crate::internal::drop::OnDrop;
use crate::{interrupt, Peri};

use super::{ErrorInterruptHandler, EventInterruptHandler, Instance, SclPin, SdaPin};

/// Slave command direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlaveCommandKind {
    /// Master is writing to us.
    Write,
    /// Master is reading from us.
    Read,
}

/// A command received from the I2C master.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveCommand {
    /// Direction of the transfer.
    pub kind: SlaveCommandKind,
}

/// Result of a slave transmit (respond_to_read).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SendStatus {
    /// All bytes sent, master NACKed the last one (normal termination).
    Done,
    /// Master stopped reading early. Contains number of unsent bytes.
    LeftoverBytes(usize),
}

/// I2C slave configuration.
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveConfig {
    /// 7-bit slave address (0x01..=0x7F).
    pub addr: u8,
    /// Respond to general call address (0x00).
    pub general_call: bool,
}

impl Default for SlaveConfig {
    fn default() -> Self {
        Self {
            addr: 0x55,
            general_call: false,
        }
    }
}

/// Async I2C slave driver.
pub struct I2cSlave<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> I2cSlave<'d, T> {
    /// Create a new async I2C slave.
    pub fn new<const REMAP: u8>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, impl SclPin<T, REMAP>>,
        sda: Peri<'d, impl SdaPin<T, REMAP>>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        config: SlaveConfig,
    ) -> Self {
        use crate::interrupt::typelevel::Interrupt;

        T::enable_and_reset();
        T::set_remap(REMAP);

        #[cfg(not(gpio_x0))]
        {
            scl.set_as_af_output(AFType::OutputOpenDrain, Speed::High);
            sda.set_as_af_output(AFType::OutputOpenDrain, Speed::High);
        }
        #[cfg(gpio_x0)]
        {
            // On CH32X0, I2C pins are automatically set to open-drain by hardware
            scl.set_as_af_output(AFType::OutputPushPull, Speed::High);
            sda.set_as_af_output(AFType::OutputPushPull, Speed::High);
        }

        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        let mut this = Self {
            _phantom: PhantomData,
        };

        this.init(config);
        this
    }

    fn init(&mut self, config: SlaveConfig) {
        let regs = T::regs();

        regs.ctlr1().modify(|w| w.set_pe(false));

        // Soft reset
        regs.ctlr1().modify(|w| w.set_swrst(true));
        regs.ctlr1().modify(|w| w.set_swrst(false));

        // Set peripheral clock frequency
        let freq_in = T::frequency().0;
        let freq_range = freq_in / 1_000_000;
        regs.ctlr2().modify(|w| w.set_freq(freq_range as u8));

        // Configure clock for 100kHz (needed even in slave mode for timing)
        let ccr = freq_in / (100_000 * 2);
        let ccr = u32::max(ccr, 0x04);
        regs.ckcfgr().write(|w| w.set_ccr(ccr as u16));

        // Set slave address (7-bit in bits [7:1])
        regs.oaddr1().write(|w| {
            w.set_add7_1(config.addr);
            w.set_must1(true); // bit 14 must be 1
            w.set_addmode(false); // 7-bit mode
        });

        // General call
        regs.ctlr1().modify(|w| {
            w.set_engc(config.general_call);
            w.set_nostretch(false); // enable clock stretching
        });

        // Enable ACK and peripheral
        regs.ctlr1().modify(|w| {
            w.set_ack(true);
            w.set_pe(true);
        });
    }

    fn enable_interrupts() {
        T::regs().ctlr2().modify(|w| {
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_itbufen(true);
        });
    }

    fn check_slave_errors(star1: &crate::pac::i2c::regs::Star1) -> Result<(), super::Error> {
        // The errata indicates that BERR may be incorrectly detected. It recommends ignoring and
        // clearing the BERR bit instead.
        if star1.berr() {
            T::regs().star1().modify(|w| w.set_berr(false));
        }
        if star1.arlo() {
            T::regs().star1().modify(|w| w.set_arlo(false));
            return Err(super::Error::Arbitration);
        }
        if star1.ovr() {
            T::regs().star1().modify(|w| w.set_ovr(false));
            return Err(super::Error::Overrun);
        }
        Ok(())
    }

    /// Wait for a command from the I2C master.
    ///
    /// Returns the command direction when an address match occurs.
    pub async fn listen(&mut self) -> Result<SlaveCommand, super::Error> {
        // Ensure ACK is enabled for address matching
        T::regs().ctlr1().modify(|w| w.set_ack(true));

        poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let star1 = T::regs().star1().read();

            if let Err(e) = Self::check_slave_errors(&star1) {
                return Poll::Ready(Err(e));
            }

            if star1.addr() {
                // Address matched! Read STAR2 to clear ADDR flag and determine direction.
                let star2 = T::regs().star2().read();
                let kind = if star2.tra() {
                    SlaveCommandKind::Read
                } else {
                    SlaveCommandKind::Write
                };
                Poll::Ready(Ok(SlaveCommand { kind }))
            } else if star1.stopf() {
                // Clear stale STOPF left over from a previous transaction.
                T::regs().ctlr1().modify(|w| w.set_pe(true));
                Self::enable_interrupts();
                Poll::Pending
            } else if star1.tx_e() {
                // TXE without ADDR means we're already in slave-transmit mode
                // (ADDR was cleared by a previous STAR2 read). Check direction via STAR2.
                let star2 = T::regs().star2().read();
                if star2.tra() && star2.busy() {
                    Poll::Ready(Ok(SlaveCommand { kind: SlaveCommandKind::Read }))
                } else {
                    Self::enable_interrupts();
                    Poll::Pending
                }
            } else {
                Self::enable_interrupts();
                Poll::Pending
            }
        })
        .await
    }

    /// Receive data from master (respond to a Write command).
    ///
    /// Returns the number of bytes received. If the master sends more bytes
    /// than `buffer` can hold, excess bytes are discarded.
    pub async fn respond_to_write(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, super::Error> {
        let mut offset = 0usize;

        let on_drop = OnDrop::new(|| {
            T::regs().ctlr2().modify(|w| {
                w.set_itevten(false);
                w.set_iterren(false);
                w.set_itbufen(false);
            });
        });

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let star1 = T::regs().star1().read();

            if let Err(e) = Self::check_slave_errors(&star1) {
                return Poll::Ready(Err(e));
            }

            // Drain available bytes
            if star1.rx_ne() {
                let byte = T::regs().datar().read().datar();
                if offset < buffer.len() {
                    buffer[offset] = byte;
                }
                offset += 1;
            }

            while T::regs().star1().read().rx_ne() {
                let byte = T::regs().datar().read().datar();
                if offset < buffer.len() {
                    buffer[offset] = byte;
                }
                offset += 1;
            }

            // Re-read after draining — STOPF or ADDR may have been set concurrently.
            let star1 = T::regs().star1().read();

            if star1.stopf() {
                T::regs().ctlr1().modify(|w| w.set_pe(true));
                let received = offset.min(buffer.len());
                return Poll::Ready(Ok(received));
            }

            if star1.addr() {
                // Repeated START — return bytes received so far.
                let received = offset.min(buffer.len());
                return Poll::Ready(Ok(received));
            }

            Self::enable_interrupts();
            Poll::Pending
        })
        .await;

        drop(on_drop);
        result
    }

    /// Send data to master (respond to a Read command).
    ///
    /// Sends bytes from `data` until the master NACKs or sends STOP.
    /// If the master requests more bytes than `data` contains, 0x00 padding is sent.
    pub async fn respond_to_read(
        &mut self,
        data: &[u8],
    ) -> Result<SendStatus, super::Error> {
        let mut offset = 0usize;

        let on_drop = OnDrop::new(|| {
            T::regs().ctlr2().modify(|w| {
                w.set_itevten(false);
                w.set_iterren(false);
                w.set_itbufen(false);
            });
        });

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let star1 = T::regs().star1().read();

            // AF = master sent NACK, normal end of read
            if star1.af() {
                T::regs().star1().modify(|w| w.set_af(false));
                let remaining = data.len().saturating_sub(offset);
                return if remaining == 0 {
                    Poll::Ready(Ok(SendStatus::Done))
                } else {
                    Poll::Ready(Ok(SendStatus::LeftoverBytes(remaining)))
                };
            }

            if let Err(e) = Self::check_slave_errors(&star1) {
                return Poll::Ready(Err(e));
            }

            if star1.stopf() {
                T::regs().ctlr1().modify(|w| w.set_pe(true));
                let remaining = data.len().saturating_sub(offset);
                return if remaining == 0 {
                    Poll::Ready(Ok(SendStatus::Done))
                } else {
                    Poll::Ready(Ok(SendStatus::LeftoverBytes(remaining)))
                };
            }

            // Send bytes when TX buffer is empty
            if star1.tx_e() {
                let byte = if offset < data.len() {
                    data[offset]
                } else {
                    0x00 // padding
                };
                T::regs().datar().write(|w| w.set_datar(byte));
                offset += 1;
            }

            Self::enable_interrupts();
            Poll::Pending
        })
        .await;

        drop(on_drop);
        result
    }
}

impl<'d, T: Instance> Drop for I2cSlave<'d, T> {
    fn drop(&mut self) {
        T::regs().ctlr1().modify(|w| w.set_pe(false));
    }
}
