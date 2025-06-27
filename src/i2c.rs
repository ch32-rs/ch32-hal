//! Inter-Integrated-Circuit (I2C)

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_futures::select::{select, Either};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::i2c::Operation;

use crate::dma::ChannelAndRequest;
use crate::gpio::{AFType, Speed};
use crate::internal::drop::OnDrop;
use crate::mode::{Async, Blocking, Mode};
// use crate::interrupt::Interrupt;
use crate::time::Hertz;
use crate::{interrupt, into_ref, peripherals, Peripheral, Timeout};

/// Event interrupt handler.
pub struct EventInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::EventInterrupt> for EventInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

/// Error interrupt handler.
pub struct ErrorInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

pub unsafe fn on_interrupt<T: Instance>() {
    let regs = T::regs();
    // i2c v2 only woke the task on transfer complete interrupts. v1 uses interrupts for a bunch of
    // other stuff, so we wake the task on every interrupt.
    T::state().waker.wake();
    critical_section::with(|_| {
        // Clear event interrupt flag.
        regs.ctlr2().modify(|w| {
            w.set_itevten(false);
            w.set_iterren(false);
        });
    });
}

/// I2C error.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error. (BERR)
    Bus,
    /// Arbitration lost (ARLO)
    Arbitration,
    /// ACK not received (either to the address or to a data byte) (AF)
    Nack,
    /// Timeout
    Timeout,
    /// CRC error
    Crc,
    /// Overrun error (OVR)
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
    /// Timeout.
    #[cfg(feature = "embassy")]
    pub timeout: embassy_time::Duration,
    pub duty: Duty,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(feature = "embassy")]
            timeout: embassy_time::Duration::from_millis(1000),
            duty: Duty::Duty2_1,
        }
    }
}

/// I2C driver.
pub struct I2c<'d, T: Instance, M: Mode> {
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    #[cfg(feature = "embassy")]
    timeout: embassy_time::Duration,
    _phantom: PhantomData<(&'d mut T, M)>,
}

impl<'d, T: Instance> I2c<'d, T, Async> {
    /// Create a new I2C driver.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T, REMAP>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T, REMAP>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, scl, sda, new_dma!(tx_dma), new_dma!(rx_dma), freq, config)
    }
}

impl<'d, T: Instance> I2c<'d, T, Blocking> {
    /// Create a new blocking I2C driver.
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T, REMAP>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T, REMAP>> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, scl, sda, None, None, freq, config)
    }
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    /// Create a new I2C driver.
    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T, REMAP>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T, REMAP>> + 'd,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        use crate::interrupt::typelevel::Interrupt;

        into_ref!(scl, sda);

        T::enable_and_reset();

        T::set_remap(REMAP);

        scl.set_as_af_output(AFType::OutputOpenDrain, Speed::High);
        sda.set_as_af_output(AFType::OutputOpenDrain, Speed::High);

        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        let mut this = Self {
            tx_dma,
            rx_dma,
            #[cfg(feature = "embassy")]
            timeout: config.timeout,
            _phantom: PhantomData,
        };

        this.init(freq, config);

        this
    }

    fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "embassy")]
            deadline: embassy_time::Instant::now() + self.timeout,
        }
    }
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M> {
    // init as master mode
    fn init(&mut self, freq: Hertz, config: Config) {
        let regs = T::regs();

        regs.ctlr1().modify(|w| w.set_pe(false)); // disale i2c

        // soft reset
        regs.ctlr1().modify(|w| w.set_swrst(true));
        regs.ctlr1().modify(|w| w.set_swrst(false));

        let freq_in = T::frequency().0;
        let freq_range = freq_in / 1_000_000;

        // assert!(freq_range >= 2 && freq_range <= 36); // MHz

        regs.ctlr2().modify(|w| w.set_freq(freq_range as u8)); // set i2c clock in

        let i2c_clk = freq.0;
        if i2c_clk <= 100_000 {
            let tmp = freq_in / (i2c_clk * 2);
            let tmp = u32::max(tmp, 0x04);

            #[cfg(i2c_v3)]
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

            #[cfg(i2c_v3)]
            regs.rtr().write(|w| w.set_trise((freq_range * 300 / 1000 + 1) as u8));
            regs.ckcfgr().write(|w| {
                w.set_f_s(true);
                w.set_duty(config.duty as u8 != 0);
                w.set_ccr(tmp as u16);
            });
        }

        regs.ctlr1().modify(|w| w.set_pe(true));
    }

    fn check_and_clear_error_flags() -> Result<crate::pac::i2c::regs::Star1, Error> {
        // Note that flags should only be cleared once they have been registered. If flags are
        // cleared otherwise, there may be an inherent race condition and flags may be missed.
        let star1 = T::regs().star1().read();

        #[cfg(i2c_v3)]
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

    fn write_bytes(&mut self, addr: u8, bytes: &[u8], timeout: Timeout, frame: FrameOptions) -> Result<(), Error> {
        if frame.send_start() {
            // Send a START condition

            T::regs().ctlr1().modify(|reg| {
                reg.set_start(true);
            });

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar(addr << 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();
        }

        // Send bytes
        for c in bytes {
            self.send_byte(*c, timeout)?;
        }

        if frame.send_stop() {
            // Send a STOP condition
            T::regs().ctlr1().modify(|reg| reg.set_stop(true));
        }

        // Fallthrough is success
        Ok(())
    }

    fn send_byte(&self, byte: u8, timeout: Timeout) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            !Self::check_and_clear_error_flags()?.tx_e()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        // Push out a byte of data
        T::regs()
            .datar()
            .write(|reg: &mut ch32_metapac::i2c::regs::Datar| reg.set_datar(byte));

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            !Self::check_and_clear_error_flags()?.btf()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        Ok(())
    }

    fn recv_byte(&self, timeout: Timeout) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            Self::check_and_clear_error_flags()?;

            !T::regs().star1().read().rx_ne()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        let value = T::regs().datar().read().datar();
        Ok(value)
    }

    fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        timeout: Timeout,
        frame: FrameOptions,
    ) -> Result<(), Error> {
        let Some((last, buffer)) = buffer.split_last_mut() else {
            return Err(Error::Overrun);
        };

        if frame.send_start() {
            // Send a START condition and set ACK bit
            T::regs().ctlr1().modify(|reg| {
                reg.set_start(true);
                reg.set_ack(true);
            });

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar((addr << 1) + 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();
        }

        // Receive bytes into buffer
        for c in buffer {
            *c = self.recv_byte(timeout)?;
        }

        // Prepare to send NACK then STOP after next byte
        T::regs().ctlr1().modify(|reg| {
            if frame.send_nack() {
                reg.set_ack(false);
            }
            if frame.send_stop() {
                reg.set_stop(true);
            }
        });

        // Receive last byte
        *last = self.recv_byte(timeout)?;

        // Fallthrough is success
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, read, self.timeout(), FrameOptions::FirstAndLastFrame)
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        self.write_bytes(addr, write, self.timeout(), FrameOptions::FirstAndLastFrame)?;

        // Fallthrough is success
        Ok(())
    }

    /// Blocking write, restart, read.
    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }

        let timeout = self.timeout();

        self.write_bytes(addr, write, timeout, FrameOptions::FirstFrame)?;
        self.blocking_read_timeout(addr, read, timeout, FrameOptions::FirstAndLastFrame)?;

        Ok(())
    }

    /// Blocking transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
    pub fn blocking_transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        let timeout = self.timeout();

        for (op, frame) in operation_frames(operations)? {
            match op {
                Operation::Read(read) => self.blocking_read_timeout(addr, read, timeout, frame)?,
                Operation::Write(write) => self.write_bytes(addr, write, timeout, frame)?,
            }
        }

        Ok(())
    }

    // Async

    #[inline] // pretty sure this should always be inlined
    fn enable_interrupts() -> () {
        T::regs().ctlr2().modify(|w| {
            w.set_iterren(true);
            w.set_itevten(true);
        });
    }
}

// ======== Async

impl<'d, T: Instance> I2c<'d, T, Async> {
    async fn write_frame(&mut self, address: u8, write: &[u8], frame: FrameOptions) -> Result<(), Error> {
        T::regs().ctlr2().modify(|w| {
            // Note: Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is used for
            // reception.
            w.set_itbufen(false);
            // DMA mode can be enabled for transmission by setting the DMAEN bit in the I2C_CR2
            // register.
            w.set_dmaen(true);
            // Sending NACK is not necessary (nor possible) for write transfer.
            w.set_last(false);
        });

        // Sentinel to disable transfer when an error occurs or future is canceled.
        // TODO: Generate STOP condition on cancel?
        let on_drop = OnDrop::new(|| {
            T::regs().ctlr2().modify(|w| {
                w.set_dmaen(false);
                w.set_iterren(false);
                w.set_itevten(false);
            })
        });

        let state = T::state();

        if frame.send_start() {
            // Send a START condition
            T::regs().ctlr1().modify(|reg| {
                reg.set_start(true);
            });

            // Wait until START condition was generated
            poll_fn(|cx| {
                state.waker.register(cx.waker());

                match Self::check_and_clear_error_flags() {
                    Err(e) => Poll::Ready(Err(e)),
                    Ok(sr1) => {
                        if sr1.sb() {
                            Poll::Ready(Ok(()))
                        } else {
                            // When pending, (re-)enable interrupts to wake us up.
                            Self::enable_interrupts();
                            Poll::Pending
                        }
                    }
                }
            })
            .await?;

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar(address << 1));

            // Wait for the address to be acknowledged
            poll_fn(|cx| {
                state.waker.register(cx.waker());

                match Self::check_and_clear_error_flags() {
                    Err(e) => Poll::Ready(Err(e)),
                    Ok(sr1) => {
                        if sr1.addr() {
                            Poll::Ready(Ok(()))
                        } else {
                            // When pending, (re-)enable interrupts to wake us up.
                            Self::enable_interrupts();
                            Poll::Pending
                        }
                    }
                }
            })
            .await?;

            // Clear condition by reading SR2
            T::regs().star2().read();
        }

        let dma_transfer = unsafe {
            // Set the I2C_DR register address in the DMA_SxPAR register. The data will be moved to
            // this address from the memory after each TxE event.
            let dst = T::regs().datar().as_ptr() as *mut u8;

            self.tx_dma.as_mut().unwrap().write(write, dst, Default::default())
        };

        // Wait for bytes to be sent, or an error to occur.
        let poll_error = poll_fn(|cx| {
            state.waker.register(cx.waker());

            match Self::check_and_clear_error_flags() {
                Err(e) => Poll::Ready(Err::<(), Error>(e)),
                Ok(_) => {
                    // When pending, (re-)enable interrupts to wake us up.
                    Self::enable_interrupts();
                    Poll::Pending
                }
            }
        });

        // Wait for either the DMA transfer to successfully finish, or an I2C error to occur.
        match select(dma_transfer, poll_error).await {
            Either::Second(Err(e)) => Err(e),
            _ => Ok(()),
        }?;

        T::regs().ctlr2().modify(|w| {
            w.set_dmaen(false);
        });

        if frame.send_stop() {
            // The I2C transfer itself will take longer than the DMA transfer, so wait for that to finish too.

            // 18.3.8 “Master transmitter: In the interrupt routine after the EOT interrupt, disable DMA
            // requests then wait for a BTF event before programming the Stop condition.”
            poll_fn(|cx| {
                state.waker.register(cx.waker());

                match Self::check_and_clear_error_flags() {
                    Err(e) => Poll::Ready(Err(e)),
                    Ok(sr1) => {
                        if sr1.btf() {
                            Poll::Ready(Ok(()))
                        } else {
                            // When pending, (re-)enable interrupts to wake us up.
                            Self::enable_interrupts();
                            Poll::Pending
                        }
                    }
                }
            })
            .await?;

            T::regs().ctlr1().modify(|w| {
                w.set_stop(true);
            });
        }

        drop(on_drop);

        // Fallthrough is success
        Ok(())
    }

    /// Write.
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.write_frame(address, write, FrameOptions::FirstAndLastFrame)
            .await?;

        Ok(())
    }

    /// Read.
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.read_frame(address, buffer, FrameOptions::FirstAndLastFrame)
            .await?;

        Ok(())
    }

    async fn read_frame(&mut self, address: u8, buffer: &mut [u8], frame: FrameOptions) -> Result<(), Error> {
        if buffer.is_empty() {
            return Err(Error::Overrun);
        }

        // Some branches below depend on whether the buffer contains only a single byte.
        let single_byte = buffer.len() == 1;

        T::regs().ctlr2().modify(|w| {
            // Note: Do not enable the ITBUFEN bit in the I2C_CR2 register if DMA is used for
            // reception.
            w.set_itbufen(false);
            // DMA mode can be enabled for transmission by setting the DMAEN bit in the I2C_CR2
            // register.
            w.set_dmaen(true);
            // If, in the I2C_CR2 register, the LAST bit is set, I2C automatically sends a NACK
            // after the next byte following EOT_1. The user can generate a Stop condition in
            // the DMA Transfer Complete interrupt routine if enabled.
            w.set_last(frame.send_nack() && !single_byte);
        });

        // Sentinel to disable transfer when an error occurs or future is canceled.
        // TODO: Generate STOP condition on cancel?
        let on_drop = OnDrop::new(|| {
            T::regs().ctlr2().modify(|w| {
                w.set_dmaen(false);
                w.set_iterren(false);
                w.set_itevten(false);
            })
        });

        let state = T::state();

        if frame.send_start() {
            // Send a START condition and set ACK bit
            T::regs().ctlr1().modify(|reg| {
                reg.set_start(true);
                reg.set_ack(true);
            });

            // Wait until START condition was generated
            poll_fn(|cx| {
                state.waker.register(cx.waker());

                match Self::check_and_clear_error_flags() {
                    Err(e) => Poll::Ready(Err(e)),
                    Ok(sr1) => {
                        if sr1.sb() {
                            Poll::Ready(Ok(()))
                        } else {
                            // When pending, (re-)enable interrupts to wake us up.
                            Self::enable_interrupts();
                            Poll::Pending
                        }
                    }
                }
            })
            .await?;

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar((address << 1) + 1));

            // Wait for the address to be acknowledged
            poll_fn(|cx| {
                state.waker.register(cx.waker());

                match Self::check_and_clear_error_flags() {
                    Err(e) => Poll::Ready(Err(e)),
                    Ok(sr1) => {
                        if sr1.addr() {
                            Poll::Ready(Ok(()))
                        } else {
                            // When pending, (re-)enable interrupts to wake us up.
                            Self::enable_interrupts();
                            Poll::Pending
                        }
                    }
                }
            })
            .await?;

            // 18.3.8: When a single byte must be received: the NACK must be programmed during EV6
            // event, i.e. program ACK=0 when ADDR=1, before clearing ADDR flag.
            if frame.send_nack() && single_byte {
                T::regs().ctlr1().modify(|w| {
                    w.set_ack(false);
                });
            }

            // Clear condition by reading SR2
            T::regs().star2().read();
        } else {
            // Before starting reception of single byte (but without START condition, i.e. in case
            // of continued frame), program NACK to emit at end of this byte.
            if frame.send_nack() && single_byte {
                T::regs().ctlr1().modify(|w| {
                    w.set_ack(false);
                });
            }
        }

        // 18.3.8: When a single byte must be received: [snip] Then the user can program the STOP
        // condition either after clearing ADDR flag, or in the DMA Transfer Complete interrupt
        // routine.
        if frame.send_stop() && single_byte {
            T::regs().ctlr1().modify(|w| {
                w.set_stop(true);
            });
        }

        let dma_transfer = unsafe {
            // Set the I2C_DR register address in the DMA_SxPAR register. The data will be moved
            // from this address from the memory after each RxE event.
            let src = T::regs().datar().as_ptr() as *mut u8;

            self.rx_dma.as_mut().unwrap().read(src, buffer, Default::default())
        };

        // Wait for bytes to be received, or an error to occur.
        let poll_error = poll_fn(|cx| {
            state.waker.register(cx.waker());

            match Self::check_and_clear_error_flags() {
                Err(e) => Poll::Ready(Err::<(), Error>(e)),
                _ => {
                    // When pending, (re-)enable interrupts to wake us up.
                    Self::enable_interrupts();
                    Poll::Pending
                }
            }
        });

        match select(dma_transfer, poll_error).await {
            Either::Second(Err(e)) => Err(e),
            _ => Ok(()),
        }?;

        T::regs().ctlr2().modify(|w| {
            w.set_dmaen(false);
        });

        if frame.send_stop() && !single_byte {
            T::regs().ctlr1().modify(|w| {
                w.set_stop(true);
            });
        }

        drop(on_drop);

        // Fallthrough is success
        Ok(())
    }

    /// Write, restart, read.
    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }

        self.write_frame(address, write, FrameOptions::FirstFrame).await?;
        self.read_frame(address, read, FrameOptions::FirstAndLastFrame).await
    }

    /// Transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
    pub async fn transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        for (op, frame) in operation_frames(operations)? {
            match op {
                Operation::Read(read) => self.read_frame(addr, read, frame).await?,
                Operation::Write(write) => self.write_frame(addr, write, frame).await?,
            }
        }

        Ok(())
    }
}

// ======== Common

impl<'d, T: Instance, M: Mode> Drop for I2c<'d, T, M> {
    fn drop(&mut self) {
        T::regs().ctlr1().modify(|w| w.set_pe(false));
    }
}

struct State {
    #[allow(unused)]
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral + crate::peripheral::RemapPeripheral {
    fn regs() -> crate::pac::i2c::I2c;
    fn state() -> &'static State;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Event interrupt for this instance
    type EventInterrupt: interrupt::typelevel::Interrupt;
    /// Error interrupt for this instance
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (i2c, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::i2c::I2c {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type EventInterrupt = crate::_generated::peripheral_interrupts::$inst::EV;
            type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::ER;
        }
    };
);

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
// pin_trait!(SmbaPin, Instance);
dma_trait!(RxDma, Instance);
dma_trait!(TxDma, Instance);

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::Timeout => embedded_hal::i2c::ErrorKind::Other,
            Self::Crc => embedded_hal::i2c::ErrorKind::Other,
            Self::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            Self::ZeroLengthTransfer => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::I2c for I2c<'d, T, M> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let _ = address;
        let _ = operations;
        todo!()
    }
}

impl<'d, T: Instance> embedded_hal_async::i2c::I2c for I2c<'d, T, Async> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        let _ = address;
        let _ = operations;
        todo!()
    }
}

// eh02 compatible

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::Read for I2c<'d, T, M> {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, buffer)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::Write for I2c<'d, T, M> {
    type Error = Error;

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal_02::blocking::i2c::WriteRead for I2c<'d, T, M> {
    type Error = Error;

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }
}

/// Frame type in I2C transaction.
///
/// This tells each method what kind of framing to use, to generate a (repeated) start condition (ST
/// or SR), and/or a stop condition (SP). For read operations, this also controls whether to send an
/// ACK or NACK after the last byte received.
///
/// For write operations, the following options are identical because they differ only in the (N)ACK
/// treatment relevant for read operations:
///
/// - `FirstFrame` and `FirstAndNextFrame`
/// - `NextFrame` and `LastFrameNoStop`
///
/// Abbreviations used below:
///
/// - `ST` = start condition
/// - `SR` = repeated start condition
/// - `SP` = stop condition
/// - `ACK`/`NACK` = last byte in read operation
#[derive(Copy, Clone)]
#[allow(dead_code)]
enum FrameOptions {
    /// `[ST/SR]+[NACK]+[SP]` First frame (of this type) in transaction and also last frame overall.
    FirstAndLastFrame,
    /// `[ST/SR]+[NACK]` First frame of this type in transaction, last frame in a read operation but
    /// not the last frame overall.
    FirstFrame,
    /// `[ST/SR]+[ACK]` First frame of this type in transaction, neither last frame overall nor last
    /// frame in a read operation.
    FirstAndNextFrame,
    /// `[ACK]` Middle frame in a read operation (neither first nor last).
    NextFrame,
    /// `[NACK]+[SP]` Last frame overall in this transaction but not the first frame.
    LastFrame,
    /// `[NACK]` Last frame in a read operation but not last frame overall in this transaction.
    LastFrameNoStop,
}

#[allow(dead_code)]
impl FrameOptions {
    /// Sends start or repeated start condition before transfer.
    fn send_start(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::FirstAndNextFrame => true,
            Self::NextFrame | Self::LastFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends stop condition after transfer.
    fn send_stop(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::LastFrame => true,
            Self::FirstFrame | Self::FirstAndNextFrame | Self::NextFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends NACK after last byte received, indicating end of read operation.
    fn send_nack(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::LastFrame | Self::LastFrameNoStop => true,
            Self::FirstAndNextFrame | Self::NextFrame => false,
        }
    }
}

/// Iterates over operations in transaction.
///
/// Returns necessary frame options for each operation to uphold the [transaction contract] and have
/// the right start/stop/(N)ACK conditions on the wire.
///
/// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
#[allow(dead_code)]
fn operation_frames<'a, 'b: 'a>(
    operations: &'a mut [embedded_hal::i2c::Operation<'b>],
) -> Result<impl IntoIterator<Item = (&'a mut embedded_hal::i2c::Operation<'b>, FrameOptions)>, Error> {
    use core::iter;

    use embedded_hal::i2c::Operation::{Read, Write};

    // Check empty read buffer before starting transaction. Otherwise, we would risk halting with an
    // error in the middle of the transaction.
    //
    // In principle, we could allow empty read frames within consecutive read operations, as long as
    // at least one byte remains in the final (merged) read operation, but that makes the logic more
    // complicated and error-prone.
    if operations.iter().any(|op| match op {
        Read(read) => read.is_empty(),
        Write(_) => false,
    }) {
        return Err(Error::Overrun);
    }

    let mut operations = operations.iter_mut().peekable();

    let mut next_first_frame = true;

    Ok(iter::from_fn(move || {
        let Some(op) = operations.next() else {
            return None;
        };

        // Is `op` first frame of its type?
        let first_frame = next_first_frame;
        let next_op = operations.peek();

        // Get appropriate frame options as combination of the following properties:
        //
        // - For each first operation of its type, generate a (repeated) start condition.
        // - For the last operation overall in the entire transaction, generate a stop condition.
        // - For read operations, check the next operation: if it is also a read operation, we merge
        //   these and send ACK for all bytes in the current operation; send NACK only for the final
        //   read operation's last byte (before write or end of entire transaction) to indicate last
        //   byte read and release the bus for transmission of the bus master's next byte (or stop).
        //
        // We check the third property unconditionally, i.e. even for write opeartions. This is okay
        // because the resulting frame options are identical for write operations.
        let frame = match (first_frame, next_op) {
            (true, None) => FrameOptions::FirstAndLastFrame,
            (true, Some(Read(_))) => FrameOptions::FirstAndNextFrame,
            (true, Some(Write(_))) => FrameOptions::FirstFrame,
            //
            (false, None) => FrameOptions::LastFrame,
            (false, Some(Read(_))) => FrameOptions::NextFrame,
            (false, Some(Write(_))) => FrameOptions::LastFrameNoStop,
        };

        // Pre-calculate if `next_op` is the first operation of its type. We do this here and not at
        // the beginning of the loop because we hand out `op` as iterator value and cannot access it
        // anymore in the next iteration.
        next_first_frame = match (&op, next_op) {
            (_, None) => false,
            (Read(_), Some(Write(_))) | (Write(_), Some(Read(_))) => true,
            (Read(_), Some(Read(_))) | (Write(_), Some(Write(_))) => false,
        };

        Some((op, frame))
    }))
}
