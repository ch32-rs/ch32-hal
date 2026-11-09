//! I2C slave driver: blocking, interrupt driven and DMA data phases.
//!
//! A transaction is handled in two steps: `listen()` waits for the master to
//! address this slave and reports the direction, then `respond_to_write()` or
//! `respond_to_read()` runs the data phase. The data phase has to start right
//! after `listen()` returns; see `respond_to_write_irq()` for why.
//!
//! Based on [PR #199](https://github.com/ch32-rs/ch32-hal/pull/199) (slave API
//! with blocking, interrupt and DMA transfers) and
//! [PR #164](https://github.com/ch32-rs/ch32-hal/pull/164) (slave driver and
//! example); both were reworked here rather than merged as they stand.

use super::*;

impl<'d, T: Instance, M: Mode> I2c<'d, T, M, Master> {
    /// Switch this driver to slave mode.
    ///
    /// The bus timing configured by the constructor is kept; only the own
    /// address and the slave control bits are reprogrammed.
    pub fn into_slave(mut self, config: SlaveConfig) -> Result<I2c<'d, T, M, Slave>, Error> {
        config.validate()?;
        Self::init_slave(config);

        Ok(I2c {
            tx_dma: self.tx_dma.take(),
            rx_dma: self.rx_dma.take(),
            #[cfg(feature = "embassy")]
            timeout: self.timeout,
            pins: self.pins,
            _phantom: PhantomData,
        })
    }
}

impl<'d, T: Instance> I2c<'d, T, Blocking, Slave> {
    /// Create a new blocking I2C slave.
    pub fn new_slave_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        I2c::<'d, T, Blocking, Master>::new_inner(peri, scl, sda, None, None, freq, Config::default())
            .into_slave(config)
    }

    /// Block until the master addresses this slave.
    pub fn blocking_listen(&mut self) -> Result<SlaveCommand, Error> {
        let timeout = self.timeout();

        // A stale STOPF from the previous transaction would end the next
        // response immediately, so flush it before arming ACK.
        Self::clear_slave_stale_flags();
        T::regs().ctlr2().modify(|w| w.set_itbufen(false));
        T::regs().ctlr1().modify(|w| w.set_ack(true));

        loop {
            let star1 = T::regs().star1().read();

            if star1.addr() {
                // Reading STAR1 and then STAR2 clears ADDR and releases SCL.
                let star2 = T::regs().star2().read();
                let kind = if star2.gencall() {
                    SlaveCommandKind::GeneralCall
                } else if star2.tra() {
                    SlaveCommandKind::Read
                } else {
                    SlaveCommandKind::Write
                };
                return Ok(SlaveCommand {
                    kind,
                    address: Self::decode_slave_address(star2),
                });
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }

    /// Receive bytes from a write transaction.
    ///
    /// Returns the number of bytes stored in `buf`. Bytes beyond `buf.len()`
    /// are acknowledged and discarded, so the bus is never held low waiting
    /// for space.
    pub fn blocking_respond_to_write(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let timeout = self.timeout();
        let mut stored = 0usize;

        loop {
            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let byte = T::regs().datar().read().datar();
                if stored < buf.len() {
                    buf[stored] = byte;
                    stored += 1;
                }
                continue;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Ok(stored);
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Ok(stored);
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }

    /// Transmit bytes for a read transaction.
    ///
    /// Data beyond `data` is padded with `0x00` until the master ends the
    /// transaction, matching the reference manual's slave transmitter flow.
    pub fn blocking_respond_to_read(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let timeout = self.timeout();
        let mut sent = 0usize;

        loop {
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.stopf() {
                Self::clear_stopf();
                // If the transmit register is still full, the last byte was
                // never clocked out.
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.tx_e() {
                let byte = data.get(sent).copied().unwrap_or(0x00);
                T::regs().datar().write(|w| w.set_datar(byte));
                sent += 1;
                continue;
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }
}

impl<'d, T: Instance> I2c<'d, T, Async, Slave> {
    /// Create a new interrupt-driven I2C slave.
    pub fn new_slave<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        use crate::interrupt::typelevel::Interrupt;

        let slave = I2c::<'d, T, Async, Master>::new_inner(peri, scl, sda, None, None, freq, Config::default())
            .into_slave(config)?;

        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        Ok(slave)
    }

    /// Create a new I2C slave that uses DMA for the data phase.
    pub fn new_slave_with_dma<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        use crate::interrupt::typelevel::Interrupt;

        let slave = I2c::<'d, T, Async, Master>::new_inner(
            peri,
            scl,
            sda,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            freq,
            Config::default(),
        )
        .into_slave(config)?;

        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        Ok(slave)
    }

    /// Wait until the master addresses this slave.
    pub async fn listen(&mut self) -> Result<SlaveCommand, Error> {
        Self::clear_slave_stale_flags();
        T::regs().ctlr2().modify(|w| w.set_itbufen(false));
        T::regs().ctlr1().modify(|w| w.set_ack(true));

        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.addr() {
                let star2 = T::regs().star2().read();
                let kind = if star2.gencall() {
                    SlaveCommandKind::GeneralCall
                } else if star2.tra() {
                    SlaveCommandKind::Read
                } else {
                    SlaveCommandKind::Write
                };
                return Poll::Ready(Ok(SlaveCommand {
                    kind,
                    address: Self::decode_slave_address(star2),
                }));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(false);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Receive bytes from a write transaction.
    pub async fn respond_to_write(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if self.rx_dma.is_some() {
            self.respond_to_write_dma(buf).await
        } else {
            self.respond_to_write_irq(buf).await
        }
    }

    async fn respond_to_write_irq(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);
        let mut stored = 0usize;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            // Spin for the first byte instead of sleeping on the RXNE
            // interrupt. The address match of a repeated START switches the
            // data register back to transmit, so a received byte that has not
            // been read yet is gone by then (measured: `STAR1` reads
            // `ADDR|TXE` with `RXNE` clear). That leaves the byte only about
            // one byte time of headroom, which the interrupt-to-task wake on
            // this core does not reliably meet: a write-then-restart-read
            // transaction lost its register pointer every time. The loop exits
            // as soon as anything happens on the bus, so it only spins while
            // the byte is already on its way.
            for _ in 0..20_000 {
                let star1 = T::regs().star1().read();
                if star1.rx_ne() || star1.stopf() || star1.addr() {
                    break;
                }
            }

            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let byte = T::regs().datar().read().datar();
                if stored < buf.len() {
                    buf[stored] = byte;
                    stored += 1;
                }
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(stored));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(stored));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Transmit bytes for a read transaction.
    pub async fn respond_to_read(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        if self.tx_dma.is_some() {
            self.respond_to_read_dma(data).await
        } else {
            self.respond_to_read_irq(data).await
        }
    }

    async fn respond_to_read_irq(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);
        let mut sent = 0usize;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.stopf() {
                Self::clear_stopf();
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.tx_e() {
                let byte = data.get(sent).copied().unwrap_or(0x00);
                T::regs().datar().write(|w| w.set_datar(byte));
                sent += 1;
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    async fn respond_to_write_dma(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let total = buf.len();
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        // The DMA transfer counter is 16 bits wide; the master path rejects
        // longer buffers the same way.
        if total > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }

        if total == 0 {
            T::regs().ctlr2().modify(|w| w.set_dmaen(false));
            on_drop.defuse();
            self.drain_excess_bytes().await?;
            return Ok(0);
        }

        let mut transfer = unsafe {
            let src = T::regs().datar().as_ptr().cast();
            self.rx_dma.as_mut().unwrap().read(src, buf, Default::default())
        };

        T::regs().ctlr2().modify(|w| {
            w.set_itbufen(false);
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_dmaen(true);
        });

        let mut remaining = total;
        let mut buffer_full = false;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            // Polling the DMA future registers its waker and reports when the
            // buffer is full.
            let dma_ready = Pin::new(&mut transfer).poll(cx).is_ready();
            let star1 = T::regs().star1().read();

            // A byte can arrive in the same cycle as STOPF; let the DMA take
            // it before ending the transaction.
            if star1.stopf() && !star1.rx_ne() {
                Self::clear_stopf();
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Ok(()));
            }

            if let Err(error) = Self::slave_handle_errors(&star1) {
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Err(error));
            }

            if dma_ready || !transfer.is_running() {
                remaining = transfer.get_remaining_transfers() as usize;
                buffer_full = remaining == 0;
                return Poll::Ready(Ok(()));
            }

            Self::enable_slave_interrupts(false);
            Poll::Pending
        })
        .await;

        // Stop the DMA before `buf` can be touched again.
        drop(transfer);
        Self::disable_slave_transfers();

        let received = total.saturating_sub(remaining);
        if let Err(error) = result {
            // `on_drop` is still armed and releases the bus on return.
            return Err(error);
        }
        on_drop.defuse();

        if buffer_full {
            // The master is still sending; acknowledge and discard until it
            // ends the transaction.
            self.drain_excess_bytes().await?;
            Ok(total)
        } else {
            Ok(received)
        }
    }

    async fn respond_to_read_dma(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let total = data.len();
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        // The DMA transfer counter is 16 bits wide; the master path rejects
        // longer buffers the same way.
        if total > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }

        if total == 0 {
            T::regs().ctlr2().modify(|w| w.set_dmaen(false));
            on_drop.defuse();
            self.send_padding_bytes().await?;
            return Ok(SendStatus::Done);
        }

        let mut transfer = unsafe {
            let dst = T::regs().datar().as_ptr().cast();
            self.tx_dma.as_mut().unwrap().write(data, dst, Default::default())
        };

        T::regs().ctlr2().modify(|w| {
            w.set_itbufen(false);
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_dmaen(true);
        });

        let mut sent = 0usize;
        let mut dma_done = false;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let dma_ready = Pin::new(&mut transfer).poll(cx).is_ready();
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if star1.stopf() {
                Self::clear_stopf();
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if let Err(error) = Self::slave_handle_errors(&star1) {
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Err(error));
            }

            if dma_ready || !transfer.is_running() {
                dma_done = true;
                return Poll::Ready(Ok(()));
            }

            Self::enable_slave_interrupts(false);
            Poll::Pending
        })
        .await;

        drop(transfer);
        Self::disable_slave_transfers();

        if let Err(error) = result {
            // `on_drop` is still armed and releases the bus on return.
            return Err(error);
        }
        on_drop.defuse();

        if dma_done {
            // The DMA only filled DR; the master may still be clocking bytes.
            self.send_padding_bytes().await?;
            Ok(SendStatus::Done)
        } else {
            Ok(Self::send_status(total, sent))
        }
    }

    /// Drain bytes after the receive buffer is full.
    async fn drain_excess_bytes(&mut self) -> Result<(), Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let _ = T::regs().datar().read().datar();
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(()));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Send `0x00` padding after the transmit buffer is exhausted.
    async fn send_padding_bytes(&mut self) -> Result<(), Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Poll::Ready(Ok(()));
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(()));
            }

            if star1.tx_e() {
                T::regs().datar().write(|w| w.set_datar(0x00));
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }
}
