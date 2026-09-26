//! `embedded-io` adapters for ch32-hal's USART, shared by the sensor examples.
//!
//! The `edrv-*` UART drivers are generic over [`embedded_io::Read`] /
//! [`embedded_io::Write`] (and the async equivalents) so that they do not depend
//! on any particular HAL. ch32-hal deliberately does **not** implement those
//! traits for its USART, and this example project is not allowed to change the
//! HAL, so the glue lives here instead.
//!
//! Two transports are provided, mirroring the two driver flavours:
//!
//! * [`UartIo`] - async, over a DMA-backed [`UartRx`]/[`UartTx`] pair. Used by
//!   the `embassy` examples.
//! * [`BlockingUartIo`] - blocking, over a plain polled UART. Used by the
//!   `*_blocking` examples, which do not need an executor at all.
//!
//! Include it from a binary with:
//!
//! ```ignore
//! #[path = "../uart_io.rs"]
//! mod uart_io;
//! ```
//!
//! # Why the adapters flush in `write`
//!
//! `embedded_io::Write::write` is only required to hand the bytes to the
//! peripheral, not to wait for them to reach the wire. On ch32-hal the async
//! `UartTx::write` stops once the DMA has taken ownership, so a command followed
//! immediately by a read can race the sensor: the sensor may not have seen the
//! complete command when we start listening for its answer. Flushing inside
//! `write` (rather than leaving it to the caller) makes every command-then-read
//! exchange safe without the drivers having to know about it.

#![allow(dead_code)]

use ch32_hal as hal;
use hal::mode::{Async, Blocking};
use hal::peripherals::USART1;
use hal::usart::{self, UartRx, UartTx};

/// Wrapper that turns ch32-hal's USART error into an `embedded_io::Error`.
///
/// `embedded_io::Error` requires `core::error::Error`, which in turn requires
/// `Debug` + `Display`.
#[derive(Debug)]
pub struct UartError(usart::Error);

impl core::fmt::Display for UartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "usart error: {:?}", self.0)
    }
}

impl core::error::Error for UartError {}

impl embedded_io::Error for UartError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

/// Async byte-stream transport over a DMA-backed USART1, read-only.
///
/// The PMSx003 only ever transmits, so its examples do not claim a TX pin.
pub struct UartRxIo {
    rx: UartRx<'static, USART1, Async>,
}

impl UartRxIo {
    /// Take ownership of the RX half produced by `Uart::split`, or of a
    /// receive-only [`UartRx`].
    pub fn new(rx: UartRx<'static, USART1, Async>) -> Self {
        Self { rx }
    }
}

impl embedded_io_async::ErrorType for UartRxIo {
    type Error = UartError;
}

impl embedded_io_async::Read for UartRxIo {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        read_async(&mut self.rx, buf).await
    }
}

/// Async byte-stream transport over a DMA-backed USART1.
pub struct UartIo {
    rx: UartRx<'static, USART1, Async>,
    tx: UartTx<'static, USART1, Async>,
}

impl UartIo {
    /// Pair the two halves produced by `Uart::split`.
    pub fn new(rx: UartRx<'static, USART1, Async>, tx: UartTx<'static, USART1, Async>) -> Self {
        Self { rx, tx }
    }
}

impl embedded_io_async::ErrorType for UartIo {
    type Error = UartError;
}

impl embedded_io_async::Read for UartIo {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        read_async(&mut self.rx, buf).await
    }
}

/// Shared body of the async `Read` impls.
///
/// `read_until_idle` stops as soon as the line goes quiet, which is what makes
/// it usable as a `Read`: it returns the bytes of one burst instead of waiting
/// for the caller's buffer to fill. It can also report a zero-length burst
/// (idle detected with nothing received), and `read_exact` treats `Ok(0)` as
/// end-of-stream, so keep waiting instead of handing that back.
async fn read_async(
    rx: &mut UartRx<'static, USART1, Async>,
    buf: &mut [u8],
) -> Result<usize, UartError> {
    if buf.is_empty() {
        return Ok(0);
    }

    loop {
        let read = rx.read_until_idle(buf).await.map_err(UartError)?;
        if read > 0 {
            return Ok(read);
        }
    }
}

impl embedded_io_async::Write for UartIo {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).await.map_err(UartError)?;
        // `write` only waits for the DMA to pick the bytes up; the sensor needs
        // them on the wire before it will answer. See the module docs.
        self.tx.blocking_flush().map_err(UartError)?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // Every `write` above already flushed.
        Ok(())
    }
}

/// Blocking byte-stream transport over a plain polled USART1, read-only.
pub struct BlockingUartRxIo {
    rx: UartRx<'static, USART1, Blocking>,
}

impl BlockingUartRxIo {
    /// Take ownership of the RX half of a blocking USART1.
    pub fn new(rx: UartRx<'static, USART1, Blocking>) -> Self {
        Self { rx }
    }
}

impl embedded_io::ErrorType for BlockingUartRxIo {
    type Error = UartError;
}

impl embedded_io::Read for BlockingUartRxIo {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        read_blocking(&mut self.rx, buf)
    }
}

/// Blocking byte-stream transport over a plain polled USART1.
pub struct BlockingUartIo {
    rx: UartRx<'static, USART1, Blocking>,
    tx: UartTx<'static, USART1, Blocking>,
}

impl BlockingUartIo {
    /// Pair the two halves produced by `Uart::split`.
    pub fn new(rx: UartRx<'static, USART1, Blocking>, tx: UartTx<'static, USART1, Blocking>) -> Self {
        Self { rx, tx }
    }
}

impl embedded_io::ErrorType for BlockingUartIo {
    type Error = UartError;
}

impl embedded_io::Read for BlockingUartIo {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        read_blocking(&mut self.rx, buf)
    }
}

/// Shared body of the blocking `Read` impls.
///
/// Polled reads always fill the whole buffer, so there is no zero-length result
/// to filter out here.
fn read_blocking(
    rx: &mut UartRx<'static, USART1, Blocking>,
    buf: &mut [u8],
) -> Result<usize, UartError> {
    if buf.is_empty() {
        return Ok(0);
    }

    rx.blocking_read(buf).map_err(UartError)?;
    Ok(buf.len())
}

impl embedded_io::Write for BlockingUartIo {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.blocking_write(buf).map_err(UartError)?;
        // The polled write already waits for the shift register, but keeping the
        // flush explicit means both transports behave identically.
        self.tx.blocking_flush().map_err(UartError)?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
