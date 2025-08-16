//! USART - Universal Synchronous Asynchronous Receiver Transmitter

/*
Full-duplex or half-duplex synchronous or asynchronous communication
NRZ data format
Fractional baud rate generator, up to 3Mbps
Programmable data length
Configurable stop bits
Supports LIN, IrDA encoder, smart card
Supports DMA
Multiple interrupt sources
 */

use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use futures::future::{select, Either};

use crate::dma::ChannelAndRequest;
use crate::gpio::{AFType, AnyPin, Pull, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, into_ref, pac, peripherals, Peripheral, PeripheralRef};

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
            // IDLE detected: no more data will come
            r.ctlr1().modify(|w| {
                // disable idle line detection
                w.set_idleie(false);
            });
        } else if cr1.rxneie() {
            // We cannot check the RXNE flag as it is auto-cleared by the DMA controller

            // It is up to the listener to determine if this in fact was a RX event and disable the RXNE detection
        } else {
            return;
        }

        compiler_fence(Ordering::SeqCst);
        s.rx_waker.wake();
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
    tx: Option<PeripheralRef<'d, AnyPin>>,
    cts: Option<PeripheralRef<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
}

impl<'d, T: Instance, M: Mode> UartTx<'d, T, M> {
    fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(config)
    }

    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        tx: Option<PeripheralRef<'d, AnyPin>>,
        cts: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        let rb = T::regs();
        rb.ctlr3().modify(|w| w.set_ctse(cts.is_some()));
        configure(&rb, &config, T::frequency(), true, false)?;

        // create state once!
        let _s = T::state();

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

        while !rb.statr().read().txe() {} // wait tx ends
        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, Async> {
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, tx, tx_dma);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(tx.map_into()), None, new_dma!(tx_dma), config)
    }

    /// Create a new tx-only UART with a clear-to-send pin
    pub fn new_with_cts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, tx, cts, tx_dma);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        cts.set_as_input(Pull::None);
        T::set_remap(REMAP);

        Self::new_inner(
            peri,
            Some(tx.map_into()),
            Some(cts.map_into()),
            new_dma!(tx_dma),
            config,
        )
    }

    /// Initiate an asynchronous UART write
    pub async fn write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let ch = self.tx_dma.as_mut().unwrap();
        T::regs().ctlr3().modify(|reg| {
            reg.set_dmat(true);
        });
        // If we don't assign future to a variable, the data register pointer
        // is held across an await and makes the future non-Send.
        let transfer = unsafe { ch.write(buffer, T::regs().datar().as_ptr() as _, Default::default()) };
        transfer.await;
        Ok(())
    }
}

impl<'d, T: Instance> UartTx<'d, T, Blocking> {
    /// Create a new blocking tx-only UART with no hardware flow control.
    ///
    /// Useful if you only want Uart Tx. It saves 1 pin and consumes a little less power.
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, tx);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(tx.map_into()), None, None, config)
    }

    /// Create a new blocking tx-only UART with a clear-to-send pin
    pub fn new_blocking_with_cts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, tx, cts);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        cts.set_as_input(Pull::None);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(tx.map_into()), Some(cts.map_into()), None, config)
    }
}

// ========
// ## Rx Part

/// Rx-only UART Driver.
pub struct UartRx<'d, T: Instance, M: Mode> {
    _phantom: PhantomData<(T, M)>,
    rx: Option<PeripheralRef<'d, AnyPin>>,
    rts: Option<PeripheralRef<'d, AnyPin>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    detect_previous_overrun: bool,
    buffered_sr: ch32_metapac::usart::regs::Statr,
}

impl<'d, T: Instance, M: Mode> UartRx<'d, T, M> {
    fn new_inner(
        _peri: impl Peripheral<P = T> + 'd,
        rx: Option<PeripheralRef<'d, AnyPin>>,
        rts: Option<PeripheralRef<'d, AnyPin>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        let r = T::regs();
        r.ctlr3().write(|w| {
            w.set_rtse(rts.is_some());
        });
        configure(&r, &config, T::frequency(), false, true)?;

        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        // create state once!
        let _s = T::state();

        Ok(Self {
            _phantom: PhantomData,
            rx,
            rts,
            rx_dma,
            detect_previous_overrun: config.detect_previous_overrun,
            buffered_sr: ch32_metapac::usart::regs::Statr(0),
        })
    }

    /// Reconfigure the driver
    pub fn set_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        reconfigure::<T>(config)
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
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, rx_dma);

        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(rx.map_into()), None, new_dma!(rx_dma), config)
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_with_rts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T, REMAP>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, rts, rx_dma);

        rx.set_as_input(Pull::None);
        rts.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        Self::new_inner(
            peri,
            Some(rx.map_into()),
            Some(rts.map_into()),
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
        if !self.detect_previous_overrun {
            let _sr = r.statr().read();
            // This read also clears the error and idle interrupt flags on v1.
            let _ = r.datar().read().dr();
            // clear_interrupt_flags(r, sr);
        }

        r.ctlr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // enable parity interrupt if not ParityNone
            w.set_peie(w.pce());
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

        let cr3 = r.ctlr3().read();

        if !cr3.dmar() {
            // something went wrong
            // because the only way to get this flag cleared is to have an interrupt

            // DMA will be stopped when transfer is dropped

            let sr = r.statr().read();
            // This read also clears the error and idle interrupt flags on v1.
            let _ = r.datar().read().dr();
            //  clear_interrupt_flags(r, sr);

            if sr.pe() {
                return Err(Error::Parity);
            }
            if sr.fe() {
                return Err(Error::Framing);
            }
            if sr.ne() {
                return Err(Error::Noise);
            }
            if sr.ore() {
                return Err(Error::Overrun);
            }

            unreachable!();
        }

        if enable_idle_line_detection {
            // clear idle flag
            let _sr = r.statr().read();
            // This read also clears the error and idle interrupt flags on v1.
            let _ = r.datar().read().dr();
            // clear_interrupt_flags(r, sr);

            // enable idle interrupt
            r.ctlr1().modify(|w| {
                w.set_idleie(true);
            });
        }

        compiler_fence(Ordering::SeqCst);

        // future which completes when idle line or error is detected
        let abort = poll_fn(move |cx| {
            let s = T::state();

            s.rx_waker.register(cx.waker());

            let sr = r.statr().read();

            // This read also clears the error and idle interrupt flags on v1.
            let _ = r.datar().read().dr();
            // clear_interrupt_flags(r, sr);

            if enable_idle_line_detection {
                // enable idle interrupt
                r.ctlr1().modify(|w| {
                    w.set_idleie(true);
                });
            }

            compiler_fence(Ordering::SeqCst);

            let has_errors = sr.pe() || sr.fe() || sr.ne() || sr.ore();

            if has_errors {
                // all Rx interrupts and Rx DMA Request have already been cleared in interrupt handler

                if sr.pe() {
                    return Poll::Ready(Err(Error::Parity));
                }
                if sr.fe() {
                    return Poll::Ready(Err(Error::Framing));
                }
                if sr.ne() {
                    return Poll::Ready(Err(Error::Noise));
                }
                if sr.ore() {
                    return Poll::Ready(Err(Error::Overrun));
                }
            }

            if enable_idle_line_detection && sr.idle() {
                // Idle line detected
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        });

        // wait for the first of DMA request or idle line detected to completes
        // select consumes its arguments
        // when transfer is dropped, it will stop the DMA request
        let r = match select(transfer, abort).await {
            // DMA transfer completed first
            Either::Left(((), _)) => Ok(ReadCompletionEvent::DmaCompleted),

            // Idle line detected first
            Either::Right((Ok(()), transfer)) => Ok(ReadCompletionEvent::Idle(
                buffer_len - transfer.get_remaining_transfers() as usize,
            )),

            // error occurred
            Either::Right((Err(e), _)) => Err(e),
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
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx);

        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(rx.map_into()), None, None, config)
    }

    /// Create a new rx-only UART with a request-to-send pin
    pub fn new_blocking_with_rts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, rts);

        rx.set_as_input(Pull::None);
        rts.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        Self::new_inner(peri, Some(rx.map_into()), Some(rts.map_into()), None, config)
    }
}

// ========
// ## Common part, Drop impl
impl<'d, T: Instance, M: Mode> Drop for UartTx<'d, T, M> {
    fn drop(&mut self) {
        self.tx.as_ref().map(|x| x.set_as_disconnected());
        self.cts.as_ref().map(|x| x.set_as_disconnected());
        T::disable();
    }
}

impl<'d, T: Instance, M: Mode> Drop for UartRx<'d, T, M> {
    fn drop(&mut self) {
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
        T::disable();
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
        _peri: impl Peripheral<P = T> + 'd,
        rx: Option<PeripheralRef<'d, AnyPin>>,
        tx: Option<PeripheralRef<'d, AnyPin>>,
        rts: Option<PeripheralRef<'d, AnyPin>>,
        cts: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        // UartRx and UartTx have one refcount each.
        T::enable_and_reset();
        T::enable_and_reset(); // CH32: rfcnt is not used yet

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
                buffered_sr: ch32_metapac::usart::regs::Statr(0),
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
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, tx, tx_dma, rx_dma);

        rx.set_as_input(Pull::None);
        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner(
            peri,
            Some(rx.map_into()),
            Some(tx.map_into()),
            None,
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_with_rtscts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T, REMAP>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, tx, rts, cts, tx_dma, rx_dma);

        rx.set_as_input(Pull::None);
        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        rts.set_as_af_output(AFType::OutputPushPull, Speed::High);
        cts.set_as_input(Pull::None);

        Self::new_inner(
            peri,
            Some(rx.map_into()),
            Some(tx.map_into()),
            Some(rts.map_into()),
            Some(cts.map_into()),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Half-duplex
    ///
    /// Note: Half duplex requires TX pin to have a pull-up resistor
    pub fn new_half_duplex<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        config.half_duplex = true;

        Self::new_inner(_peri, None, Some(tx.map_into()), None, None, None, None, config)
    }
}

impl<'d, T: Instance> Uart<'d, T, Blocking> {
    /// Create a new blocking bidirectional UART.
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, tx);

        rx.set_as_input(Pull::None);
        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        Self::new_inner(
            peri,
            Some(rx.map_into()),
            Some(tx.map_into()),
            None,
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new bidirectional UART with request-to-send and clear-to-send pins
    pub fn new_blocking_with_rtscts<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rts: impl Peripheral<P = impl RtsPin<T, REMAP>> + 'd,
        cts: impl Peripheral<P = impl CtsPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, rx, tx, rts, cts);

        rx.set_as_input(Pull::None);
        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        rts.set_as_af_output(AFType::OutputPushPull, Speed::High);
        cts.set_as_input(Pull::None);
        T::set_remap(REMAP);

        Self::new_inner(
            peri,
            Some(rx.map_into()),
            Some(tx.map_into()),
            Some(rts.map_into()),
            Some(cts.map_into()),
            None,
            None,
            config,
        )
    }

    pub fn new_blocking_half_duplex<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        mut config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(peri, tx);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        config.half_duplex = true;

        Self::new_inner(peri, None, Some(tx.map_into()), None, None, None, None, config)
    }
}

fn reconfigure<T: Instance>(config: &Config) -> Result<(), ConfigError> {
    T::Interrupt::disable();
    let r = T::regs();

    let cr = r.ctlr1().read();
    configure(&r, config, T::frequency(), cr.re(), cr.te())?;

    T::Interrupt::unpend();
    unsafe { T::Interrupt::enable() };

    Ok(())
}

fn configure(
    rb: &pac::usart::Usart,
    config: &Config,
    pclk_freq: Hertz,
    enable_tx: bool,
    enable_rx: bool,
) -> Result<(), ConfigError> {
    if !enable_rx && !enable_tx {
        panic!("USART: At least one of RX or TX should be enabled");
    }

    rb.ctlr2().modify(|w| w.set_stop(config.stop_bits as u8));

    rb.ctlr1().modify(|w| {
        w.set_m(config.data_bits as u8 != 0);
        w.set_pce(config.parity != Parity::ParityNone);
        w.set_ps(config.parity == Parity::ParityOdd); // 1 for odd parity, 0 for even parity
        w.set_te(enable_tx);
        w.set_re(enable_rx);
    });

    if config.half_duplex {
        rb.ctlr3().modify(|w| w.set_hdsel(true));
    }

    // HCLK/(16*USARTDIV)
    // USARTDIV = DIV_M+(DIV_F/16)  via USART_BRR

    let clock_in = pclk_freq.0;
    let div_m = 25 * clock_in / (4 * config.baudrate);
    let mut tmpreg = (div_m / 100) << 4;

    let div_f = div_m - 100 * (tmpreg >> 4);
    tmpreg |= ((div_f * 16 + 50) / 100) & 0x0F;

    rb.brr().write(|w| w.0 = tmpreg);

    // enable uart
    rb.ctlr1().modify(|w| w.set_ue(true));

    Ok(())
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
    rx_waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral + crate::peripheral::RemapPeripheral {
    fn regs() -> crate::pac::usart::Usart;
    fn state() -> &'static State;

    // fn buffered_state() -> &'static buffered::State;
}

#[allow(private_bounds)]
pub trait Instance: Peripheral<P = Self> + SealedInstance + 'static + Send {
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
