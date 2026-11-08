//! USART - Universal Synchronous Asynchronous Receiver Transmitter

/*
Full-duplex or half-duplex synchronous or asynchronous communication
NRZ data format
Fractional baud rate generator
Programmable data length
Configurable stop bits
Supports LIN, IrDA encoder, smart card
Supports DMA
Multiple interrupt sources
 */

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};
#[cfg(usart_v4x7)]
use core::sync::atomic::AtomicUsize;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use futures::future::{select, Either};

use crate::dma::ChannelAndRequest;
use crate::gpio::{AfType, AnyPin, OutputType, Pull, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

mod ringbuffered;
pub use ringbuffered::RingBufferedUartRx;

/// Interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        let r = T::regs();
        let s = T::state();

        let (sr, cr1, cr3) = (r.statr().read(), r.ctlr1().read(), r.ctlr3().read());

        let has_errors = (sr.pe() && cr1.peie()) || ((sr.fe() || sr.ne() || sr.ore()) && cr3.eie());
        if has_errors {
            // Latch the receive error bits: this `STATR` read plus the DMA's
            // `DATAR` read can clear them before the reader gets to poll, and
            // the reader would then wait forever for a transfer that the
            // disabled DMA request can no longer complete.
            critical_section::with(|_| {
                // PE/FE/NE/ORE are `STATR` bits 0..=3.
                let error_bits = sr.0 & 0xf;
                let latched = s.rx_errors.load(Ordering::Relaxed);
                s.rx_errors.store(latched | error_bits, Ordering::Release);
            });

            // clear all interrupts and DMA Rx Request
            r.ctlr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // disable parity interrupt
                w.set_peie(false);
                // disable idle line interrupt
                w.set_idleie(false);
            });
            r.ctlr3().modify(|w| {
                // disable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(false);
                // disable DMA Rx Request
                w.set_dmar(false);
            });
        } else if cr1.idleie() && sr.idle() {
            // IDLE detected: no more data will come. Keep DMAR set: a
            // one-shot read stops the DMA itself once it sees this flag, and
            // the ring-buffered receiver must keep receiving across idle
            // gaps, because an idle bus is not the end of the stream.
            r.ctlr1().modify(|w| {
                // disable idle line detection
                w.set_idleie(false);
            });
        } else if cr1.tcie() && sr.tc() {
            // Transmission complete detected
            r.ctlr1().modify(|w| {
                // disable Transmission complete interrupt
                w.set_tcie(false);
            });
        } else if cr1.rxneie() {
            // We cannot check the RXNE flag as it is auto-cleared by the DMA controller

            // It is up to the listener to determine if this in fact was a RX event and disable the RXNE detection
        } else {
            return;
        }

        compiler_fence(Ordering::SeqCst);
        s.rx_waker.wake();
        s.tx_waker.wake();
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    DataBits8 = 0,
    DataBits9 = 1,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0,
    #[doc = "0.5 stop bits"]
    STOP0P5 = 0b01,
    #[doc = "2 stop bits"]
    STOP2 = 0b10,
    #[doc = "1.5 stop bits"]
    STOP1P5 = 0b11,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,

    /// If true: on a read-like method, if there is a latent error pending,
    /// the read will abort and the error will be reported and cleared
    ///
    /// If false: the error is ignored and cleared
    pub detect_previous_overrun: bool,

    half_duplex: bool,
}
impl Default for Config {
    /// 115200 8N1
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,

            detect_previous_overrun: false,

            half_duplex: false,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    // FE
    Framing,
    /// Noise error
    // NE
    Noise,
    /// RX buffer overrun
    // ORE
    Overrun,
    /// Parity check error
    // PE
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    BaudrateTooLow,
    BaudrateTooHigh,
}

enum ReadCompletionEvent {
    // DMA Read transfer completed first
    DmaCompleted,
    // Idle line detected first
    Idle(usize),
}

// ========
// ## Tx Part

/// Tx-only UART Driver.
pub struct UartTx<'d, T: Instance, M: Mode> {
    _phantom: PhantomData<(T, M)>,
    tx: Option<Peri<'d, AnyPin>>,
    cts: Option<Peri<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
}

impl<'d, T: Instance, M: Mode> UartTx<'d, T, M> {
    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(config)
    }

    fn new_inner(
        _peri: Peri<'d, T>,
        tx: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        enable_peripheral::<T>();
        let cleanup = OnDrop::new(|| {
            tx.as_ref().map(|pin| pin.set_as_disconnected());
            cts.as_ref().map(|pin| pin.set_as_disconnected());
            disable_peripheral::<T>();
        });

        let rb = T::regs();
        rb.ctlr3().modify(|w| w.set_ctse(cts.is_some()));
        configure(&rb, &config, T::frequency(), true, false)?;

        #[cfg(usart_v4x7)]
        if tx_dma.is_some() {
            T::Interrupt::unpend();
            unsafe { T::Interrupt::enable() };
        }

        // create state once!
        let _s = T::state();

        cleanup.defuse();
        Ok(Self {
            _phantom: PhantomData,
            tx,
            cts,
            tx_dma,
        })
    }

    /// Perform a blocking UART write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            while !rb.statr().read().tc() {} // wait tx complete
            rb.datar().write(|w| w.set_dr(c as u16));
        }
        Ok(())
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let rb = T::regs();
        if rb.ctlr1().read().te() {
            while !rb.statr().read().tc() {} // wait for last bit on wire.
        }
        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, Async> {
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    #[cfg_attr(usart_v4x7, doc = "The interrupt binding is required for asynchronous flush completion.")]
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        #[cfg(usart_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)), None, new_dma!(tx_dma), config)
    }

    /// Create a new tx-only UART with a clear-to-send pin
    #[cfg_attr(usart_v4x7, doc = "The interrupt binding is required for asynchronous flush completion.")]
    pub fn new_with_cts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        cts: Peri<'d, if_remap!(impl CtsPin<T, REMAP>)>,
        #[cfg(usart_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(cts, AfType::input(Pull::None)),
            new_dma!(tx_dma),
            config,
        )
    }

    /// Initiate an asynchronous UART write
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        if buffer.is_empty() {
            return Ok(());
        }
        if buffer.len() > 0xffff {
            return Err(Error::BufferTooLong);
        }
        let registers = T::regs();
        let cleanup = OnDrop::new(move || registers.ctlr3().modify(|register| register.set_dmat(false)));
        registers.statr().write(|register| {
            register.0 = u32::from(u16::MAX);
            register.set_tc(false);
        });
        let ch = self.tx_dma.as_mut().unwrap();
        T::regs().ctlr3().modify(|reg| {
            reg.set_dmat(true);
        });
        // If we don't assign future to a variable, the data register pointer
        // is held across an await and makes the future non-Send.
        let transfer = unsafe { ch.write(buffer, T::regs().datar().as_ptr() as _, Default::default()) };
        transfer.await;
        drop(cleanup);
        Ok(())
    }

    /// Wait until transmission complete
    pub async fn flush(&mut self) -> Result<(), Error> {
        let r = T::regs();
        let cleanup = OnDrop::new(move || r.ctlr1().modify(|register| register.set_tcie(false)));
        if r.ctlr1().read().te() && !r.statr().read().tc() {
            r.ctlr1().modify(|w| {
                // enable Transmission Complete interrupt
                w.set_tcie(true);
            });

            compiler_fence(Ordering::SeqCst);

            // future which completes when Transmission complete is detected
            let abort = poll_fn(move |cx| {
                let s = T::state();
                s.tx_waker.register(cx.waker());

                let sr = r.statr().read();
                if sr.tc() {
                    // Transmission complete detected
                    return Poll::Ready(());
                }

                Poll::Pending
            });

            abort.await;
        }

        drop(cleanup);

        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, Blocking> {
    /// Create a new blocking tx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)), None, None, config)
    }

    /// Create a new blocking tx-only UART with a clear-to-send pin
    pub fn new_blocking_with_cts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        cts: Peri<'d, if_remap!(impl CtsPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)), new_pin!(cts, AfType::input(Pull::None)), None, config)
    }
}

// ========
// ## Rx Part

/// Rx-only UART Driver.
pub struct UartRx<'d, T: Instance, M: Mode> {
    _phantom: PhantomData<(T, M)>,
    rx: Option<Peri<'d, AnyPin>>,
    rts: Option<Peri<'d, AnyPin>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    detect_previous_overrun: bool,
    buffered_sr: pac::usart::regs::Statr,
}

impl<'d, T: Instance, M: Mode> UartRx<'d, T, M> {
    fn new_inner(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        enable_peripheral::<T>();
        let cleanup = OnDrop::new(|| {
            rx.as_ref().map(|pin| pin.set_as_disconnected());
            rts.as_ref().map(|pin| pin.set_as_disconnected());
            disable_peripheral::<T>();
        });

        let r = T::regs();
        r.ctlr3().write(|w| {
            w.set_rtse(rts.is_some());
        });
        configure(&r, &config, T::frequency(), false, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // create state once!
        let _s = T::state();

        cleanup.defuse();
        Ok(Self {
            _phantom: PhantomData,
            rx,
            rts,
            rx_dma,
            detect_previous_overrun: config.detect_previous_overrun,
            buffered_sr: pac::usart::regs::Statr(0),
        })
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(config)?;
        self.detect_previous_overrun = config.detect_previous_overrun;
        Ok(())
    }

    // The same as embassy-stm32's usart_v1
    // checks rxne
    fn check_rx_flags(&mut self) -> Result<bool, Error> {
        let r = T::regs();
        loop {
            // Handle all buffered error flags.
            if self.buffered_sr.pe() {
                self.buffered_sr.set_pe(false);
                return Err(Error::Parity);
            } else if self.buffered_sr.fe() {
                self.buffered_sr.set_fe(false);
                return Err(Error::Framing);
            } else if self.buffered_sr.ne() {
                self.buffered_sr.set_ne(false);
                return Err(Error::Noise);
            } else if self.buffered_sr.ore() {
                self.buffered_sr.set_ore(false);
                return Err(Error::Overrun);
            } else if self.buffered_sr.rxne() {
                self.buffered_sr.set_rxne(false);
                return Ok(true);
            } else {
                // No error flags from previous iterations were set: Check the actual status register
                let sr = r.statr().read();
                if !sr.rxne() {
                    return Ok(false);
                }

                // Buffer the status register and let the loop handle the error flags.
                self.buffered_sr = sr;
            }
        }
    }

    /// Read a single u8 if there is one available, otherwise return WouldBlock
    #[allow(unused)]
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let r = T::regs();
        if self.check_rx_flags()? {
            Ok(r.datar().read().dr() as u8)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let r = T::regs();
        for b in buffer {
            while !self.check_rx_flags()? {}
            *b = r.datar().read().dr() as u8
        }
        Ok(())
    }
}

impl<'d, T: Instance> UartRx<'d, T, Async> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(rx, AfType::input(Pull::None)), None, new_dma!(rx_dma), config)
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_with_rts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        rts: Peri<'d, if_remap!(impl RtsPin<T, REMAP>)>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::None)),
            new_pin!(rts, AfType::output(OutputType::PushPull, Speed::High)),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Initiate an asynchronous UART read
    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.inner_read(buffer, false).await?;

        Ok(())
    }

    /// Initiate an asynchronous read with idle line detection enabled
    pub async fn read_until_idle(&mut self, buffer: &mut [u8]) -> Result<usize, Error> {
        self.inner_read(buffer, true).await
    }

    async fn inner_read_run(
        &mut self,
        buffer: &mut [u8],
        enable_idle_line_detection: bool,
    ) -> Result<ReadCompletionEvent, Error> {
        let r = T::regs();

        // make sure USART state is restored to neutral state when this future is dropped
        let on_drop = OnDrop::new(move || {
            // defmt::trace!("Clear all USART interrupts and DMA Read Request");
            // clear all interrupts and DMA Rx Request
            r.ctlr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // disable parity interrupt
                w.set_peie(false);
                // disable idle line interrupt
                w.set_idleie(false);
            });
            r.ctlr3().modify(|w| {
                // disable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(false);
                // disable DMA Rx Request
                w.set_dmar(false);
            });
        });

        let ch = self.rx_dma.as_mut().unwrap();

        let buffer_len = buffer.len();

        // Start USART DMA
        // will not do anything yet because DMAR is not yet set
        // future which will complete when DMA Read request completes
        let transfer = unsafe { ch.read(T::regs().datar().as_ptr() as _, buffer, Default::default()) };

        // clear ORE flag just before enabling DMA Rx Request: can be mandatory for the second transfer
        let status = r.statr().read();
        if self.detect_previous_overrun {
            if let Some(error) = receive_error(status) {
                let _ = r.datar().read();
                return Err(error);
            }
        }
        if !self.detect_previous_overrun || enable_idle_line_detection {
            // This read also clears the error and idle interrupt flags on v1.
            let _ = r.datar().read().dr();
            // clear_interrupt_flags(r, sr);
        }

        // Start from a clean slate: errors latched before this transfer belong
        // to an earlier read (or to a ring-buffered reception).
        critical_section::with(|_| T::state().rx_errors.store(0, Ordering::Release));

        r.ctlr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // enable parity interrupt if not ParityNone
            w.set_peie(w.pce());
            w.set_idleie(enable_idle_line_detection);
        });

        r.ctlr3().modify(|w| {
            // enable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(true);
            // enable DMA Rx Request
            w.set_dmar(true);
        });

        compiler_fence(Ordering::SeqCst);

        // In case of errors already pending when reception started, interrupts may have already been raised
        // and lead to reception abortion (Overrun error for instance). In such a case, all interrupts
        // have been disabled in interrupt handler and DMA Rx Request has been disabled.

        compiler_fence(Ordering::SeqCst);

        // future which completes when idle line or error is detected
        let mut first_poll = true;
        let abort = poll_fn(move |cx| {
            let s = T::state();

            s.rx_waker.register(cx.waker());

            compiler_fence(Ordering::SeqCst);

            // Take the error the interrupt handler latched before checking
            // `STATR`: the handler's own status read and the DMA's data read
            // may already have cleared the flag from the register.
            let latched = critical_section::with(|_| {
                let latched = s.rx_errors.load(Ordering::Acquire);
                s.rx_errors.store(0, Ordering::Release);
                latched
            });
            if let Some(error) = receive_error(pac::usart::regs::Statr(latched)) {
                r.ctlr3().modify(|register| register.set_dmar(false));
                let _ = r.datar().read();
                return Poll::Ready(Err(error));
            }

            // The first poll runs before any byte can have arrived. Reading
            // `STATR` here would arm the RM §17.10.1 "read STATR, then read
            // DATAR" clear sequence, so the DMA's own `DATAR` read would
            // clear a FE/NE/ORE raised by the first frame before the
            // interrupt handler gets to read it; the handler would then find
            // nothing and the read would hang. The interrupt handler is the
            // only observer of those bits, so only its latched copy is safe
            // to consume here.
            if first_poll {
                first_poll = false;
                return Poll::Pending;
            }

            let sr = r.statr().read();

            compiler_fence(Ordering::SeqCst);

            if let Some(error) = receive_error(sr) {
                r.ctlr3().modify(|register| register.set_dmar(false));
                let _ = r.datar().read();
                // all Rx interrupts and Rx DMA Request have already been cleared in interrupt handler

                return Poll::Ready(Err(error));
            }

            if enable_idle_line_detection && sr.idle() {
                r.ctlr3().modify(|register| register.set_dmar(false));
                let _ = r.datar().read();
                // Idle line detected
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        });

        // wait for the first of DMA request or idle line detected to completes
        // select consumes its arguments
        // when transfer is dropped, it will stop the DMA request
        let r = match select(abort, transfer).await {
            // DMA transfer completed first
            Either::Right(((), _)) => Ok(ReadCompletionEvent::DmaCompleted),

            // Idle line detected first
            Either::Left((Ok(()), mut transfer)) => {
                transfer.request_stop();
                Ok(ReadCompletionEvent::Idle(buffer_len - transfer.get_remaining_transfers() as usize))
            }

            // error occurred
            Either::Left((Err(e), _)) => Err(e),
        };

        drop(on_drop);

        r
    }

    async fn inner_read(&mut self, buffer: &mut [u8], enable_idle_line_detection: bool) -> Result<usize, Error> {
        if buffer.is_empty() {
            return Ok(0);
        } else if buffer.len() > 0xFFFF {
            return Err(Error::BufferTooLong);
        }

        let buffer_len = buffer.len();

        // wait for DMA to complete or IDLE line detection if requested
        let res = self.inner_read_run(buffer, enable_idle_line_detection).await;

        match res {
            Ok(ReadCompletionEvent::DmaCompleted) => Ok(buffer_len),
            Ok(ReadCompletionEvent::Idle(n)) => Ok(n),
            Err(e) => Err(e),
        }
    }
}

impl<'d, T: Instance> UartRx<'d, T, Blocking> {
    /// Create a new rx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Rx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(rx, AfType::input(Pull::None)), None, None, config)
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_blocking_with_rts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        rts: Peri<'d, if_remap!(impl RtsPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(peri, new_pin!(rx, AfType::input(Pull::None)), new_pin!(rts, AfType::output(OutputType::PushPull, Speed::High)), None, config)
    }
}

// ========
// ## Common part, Drop impl
impl<'d, T: Instance, M: Mode> Drop for UartTx<'d, T, M> {
    fn drop(&mut self) {
        #[cfg(usart_v4x7)]
        {
            T::regs().ctlr1().modify(|register| {
                register.set_te(false);
                register.set_tcie(false);
            });
            T::regs().ctlr3().modify(|register| {
                register.set_dmat(false);
                register.set_ctse(false);
            });
        }
        self.tx.as_ref().map(|x| x.set_as_disconnected());
        self.cts.as_ref().map(|x| x.set_as_disconnected());
        disable_peripheral::<T>();
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartRx<'d, T, M> {
    fn drop(&mut self) {
        #[cfg(usart_v4x7)]
        {
            T::regs().ctlr1().modify(|register| {
                register.set_re(false);
                register.set_rxneie(false);
                register.set_peie(false);
                register.set_idleie(false);
            });
            T::regs().ctlr3().modify(|register| {
                register.set_dmar(false);
                register.set_eie(false);
                register.set_rtse(false);
            });
        }
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
        disable_peripheral::<T>();
    }
}

// ========
// ## Bidirectional UART

/// Bidirectional UART Driver, which acts as a combination of [`UartTx`] and [`UartRx`].
///
/// ### Notes on [`embedded_io::Read`]
///
/// `embedded_io::Read` requires guarantees that the base [`UartRx`] cannot provide.
///
/// See [`UartRx`] for more details, and see [`BufferedUart`] and [`RingBufferedUartRx`]
/// as alternatives that do provide the necessary guarantees for `embedded_io::Read`.
pub struct Uart<'d, T: Instance, M: Mode> {
    tx: UartTx<'d, T, M>,
    rx: UartRx<'d, T, M>,
}

impl<'d, T: Instance, M: Mode> Uart<'d, T, M> {
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.tx.set_config(config)?;
        self.rx.set_config(config)
    }

    fn new_inner(
        _peri: Peri<'d, T>,
        rx: Option<Peri<'d, AnyPin>>,
        tx: Option<Peri<'d, AnyPin>>,
        rts: Option<Peri<'d, AnyPin>>,
        cts: Option<Peri<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // UartRx and UartTx have one refcount each.
        enable_peripheral::<T>();
        enable_peripheral::<T>();
        let cleanup = OnDrop::new(|| {
            rx.as_ref().map(|pin| pin.set_as_disconnected());
            tx.as_ref().map(|pin| pin.set_as_disconnected());
            rts.as_ref().map(|pin| pin.set_as_disconnected());
            cts.as_ref().map(|pin| pin.set_as_disconnected());
            disable_peripheral::<T>();
            disable_peripheral::<T>();
        });

        let r = T::regs();

        r.ctlr3().write(|w| {
            w.set_rtse(rts.is_some());
            w.set_ctse(cts.is_some());
        });
        configure(&r, &config, T::frequency(), true, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // create state once!
        let _s = T::state();

        cleanup.defuse();
        Ok(Self {
            tx: UartTx {
                _phantom: PhantomData,
                tx,
                cts,
                tx_dma,
            },
            rx: UartRx {
                _phantom: PhantomData,
                rx,
                rts,
                rx_dma,
                detect_previous_overrun: config.detect_previous_overrun,
                buffered_sr: pac::usart::regs::Statr(0),
            },
        })
    }

    /// Perform a blocking write
    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        self.tx.blocking_write(buffer)
    }

    /// Block until transmission complete
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        self.tx.blocking_flush()
    }

    /// Read a single `u8` or return `WouldBlock`
    #[allow(unused)]
    pub(crate) fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    /// Perform a blocking read into `buffer`
    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }

    /// Split the Uart into a transmitter and receiver, which is
    /// particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UartTx<'d, T, M>, UartRx<'d, T, M>) {
        (self.tx, self.rx)
    }
}

impl<'d, T: Instance> Uart<'d, T, Async> {
    /// Create a new bidirectional UARTUart
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::None)),
            new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_with_rtscts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rts: Peri<'d, if_remap!(impl RtsPin<T, REMAP>)>,
        cts: Peri<'d, if_remap!(impl CtsPin<T, REMAP>)>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::None)),
            new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(rts, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(cts, AfType::input(Pull::None)),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Half-duplex
    ///
    /// Note: Half duplex requires TX pin to have a pull-up resistor
    pub fn new_half_duplex<#[cfg(not(afio_h4))] const REMAP: u8>(
        _peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        // gpio_x0 doesn't expose an open-drain AF variant; fall back to push-pull
        // and rely on the user to provide an external pull-up.
        #[cfg(not(gpio_x0))]
        let tx_af = AfType::output(OutputType::OpenDrain, Speed::High);
        #[cfg(gpio_x0)]
        let tx_af = AfType::output(OutputType::PushPull, Speed::High);
        config.half_duplex = true;

        Self::new_inner(
            _peri,
            None,
            new_pin!(tx, tx_af),
            None,
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config
        )
    }
}

impl<'d, T: Instance> Uart<'d, T, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::None)),
            new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_blocking_with_rtscts<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        rx: Peri<'d, if_remap!(impl RxPin<T, REMAP>)>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        rts: Peri<'d, if_remap!(impl RtsPin<T, REMAP>)>,
        cts: Peri<'d, if_remap!(impl CtsPin<T, REMAP>)>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(rx, AfType::input(Pull::None)),
            new_pin!(tx, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(rts, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(cts, AfType::input(Pull::None)),
            None,
            None,
            config,
        )
    }

    pub fn new_blocking_half_duplex<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        tx: Peri<'d, if_remap!(impl TxPin<T, REMAP>)>,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        apply_remap!();
        #[cfg(not(gpio_x0))]
        let tx_af = AfType::output(OutputType::OpenDrain, Speed::High);
        #[cfg(gpio_x0)]
        let tx_af = AfType::output(OutputType::PushPull, Speed::High);
        config.half_duplex = true;

        Self::new_inner(peri, None, new_pin!(tx, tx_af), None, None, None, None, config)
    }
}

fn reconfigure<T: Instance>(config: &Config) -> Result<(), ConfigError> {
    baudrate_divisor(T::frequency().0, config.baudrate)?;
    let enabled = T::Interrupt::is_enabled();
    T::Interrupt::disable();
    let r = T::regs();

    let cr = r.ctlr1().read();
    configure(&r, config, T::frequency(), cr.te(), cr.re())?;

    T::Interrupt::unpend();
    if enabled {
        unsafe { T::Interrupt::enable() };
    }

    Ok(())
}

fn configure(
    rb: &pac::usart::Usart,
    config: &Config,
    pclk_freq: Hertz,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), ConfigError> {
    let divisor = baudrate_divisor(pclk_freq.0, config.baudrate)?;
    if !enable_rx && !enable_tx {
        panic!("USART: At least one of RX or TX should be enabled");
    }

    rb.ctlr1().modify(|register| register.set_ue(false));
    rb.ctlr2().modify(|w| w.set_stop(config.stop_bits as u8));

    rb.ctlr1().modify(|w| {
        w.set_m(config.data_bits as u8 != 0);
        w.set_pce(config.parity != Parity::ParityNone);
        w.set_ps(config.parity == Parity::ParityOdd); // 1 for odd parity, 0 for even parity
        w.set_te(enable_tx);
        w.set_re(enable_rx);
    });

    rb.ctlr3().modify(|w| w.set_hdsel(config.half_duplex));

    rb.brr().write(|w| w.0 = divisor);

    // enable uart
    rb.ctlr1().modify(|w| w.set_ue(true));

    Ok(())
}

fn baudrate_divisor(clock_hz: u32, baudrate: u32) -> Result<u32, ConfigError> {
    if baudrate == 0 {
        return Err(ConfigError::BaudrateTooLow);
    }
    // BRR holds USARTDIV in 4.16 fixed point, and the hardware requires
    // USARTDIV >= 1 (BRR >= 0x10), which the documented maximum baud rate
    // equals: 3 Mbps at 48 MHz, 4.5 at 72, 9 at 144 and 12.5 at 200.
    let divisor = (u64::from(clock_hz) + u64::from(baudrate) / 2) / u64::from(baudrate);
    if divisor < 0x10 {
        return Err(ConfigError::BaudrateTooHigh);
    }
    if divisor > 0xffff {
        return Err(ConfigError::BaudrateTooLow);
    }
    Ok(divisor as u32)
}

fn receive_error(status: pac::usart::regs::Statr) -> Option<Error> {
    if status.pe() {
        Some(Error::Parity)
    } else if status.fe() {
        Some(Error::Framing)
    } else if status.ne() {
        Some(Error::Noise)
    } else if status.ore() {
        Some(Error::Overrun)
    } else {
        None
    }
}

fn enable_peripheral<T: Instance>() {
    #[cfg(usart_v4x7)]
    critical_section::with(|cs| {
        let count = T::state().users.load(Ordering::Relaxed);
        if count == 0 {
            T::enable_and_reset_with_cs(cs);
        }
        T::state().users.store(count + 1, Ordering::Relaxed);
    });
    #[cfg(not(usart_v4x7))]
    T::enable_and_reset();
}

fn disable_peripheral<T: Instance>() {
    #[cfg(usart_v4x7)]
    critical_section::with(|cs| {
        let count = T::state().users.load(Ordering::Relaxed);
        assert!(count != 0);
        T::state().users.store(count - 1, Ordering::Relaxed);
        if count == 1 {
            T::regs().ctlr1().write(|register| register.set_ue(false));
            T::regs().ctlr3().write(|register| register.set_dmar(false));
            T::Interrupt::disable();
            T::Interrupt::unpend();
            T::disable_with_cs(cs);
        }
    });
    #[cfg(not(usart_v4x7))]
    T::disable();
}

impl<'d, T: Instance> core::fmt::Write for UartTx<'d, T, Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}
impl<'d, T: Instance> core::fmt::Write for Uart<'d, T, Blocking> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.blocking_write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

// Peripheral traits
struct State {
    #[cfg(usart_v4x7)]
    users: AtomicUsize,
    #[allow(unused)]
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
    /// PE/FE/NE/ORE bits the interrupt handler observed, latched for the
    /// reader because clearing them from the register races with the DMA.
    rx_errors: AtomicU32,
}

impl State {
    const fn new() -> Self {
        Self {
            #[cfg(usart_v4x7)]
            users: AtomicUsize::new(0),
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
            rx_errors: AtomicU32::new(0),
        }
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::usart::Usart;
    fn state() -> &'static State;

    // fn buffered_state() -> &'static buffered::State;
}

#[allow(private_bounds)]
pub trait Instance: embassy_hal_internal::PeripheralType + SealedInstance + crate::peripheral::RemapBound + 'static + Send {
    /// Interrupt for this instance.
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (usart, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::usart::Usart {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
    };
);

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);
pin_trait!(CtsPin, Instance);
pin_trait!(RtsPin, Instance);
pin_trait!(CkPin, Instance);

dma_trait!(TxDma, Instance);
dma_trait!(RxDma, Instance);

