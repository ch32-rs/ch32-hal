use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use futures::future::{select, Either};

use super::{Error, Instance, UartRx};
use crate::dma::{ReadableRingBuffer, TransferOptions};
use crate::gpio::{AnyPin, SealedPin};
use crate::mode::Async;
use crate::Peri;

/// Rx-only UART driver that receives continuously into a DMA ring buffer.
///
/// Unlike [`UartRx::read`], no data is lost between reads, and dropping a `read` future
/// doesn't stop reception.
pub struct RingBufferedUartRx<'d, T: Instance> {
    _phantom: PhantomData<T>,
    rx: Option<Peri<'d, AnyPin>>,
    rts: Option<Peri<'d, AnyPin>>,
    ring_buf: ReadableRingBuffer<'d, u8>,
}

impl<'d, T: Instance> UartRx<'d, T, Async> {
    /// Turn the `UartRx` into a buffered UART which can continuously receive in the background
    /// without the possibility of losing bytes. The `dma_buf` is a buffer registered to the
    /// DMA controller, and must be large enough to prevent overflows.
    pub fn into_ring_buffered(mut self, dma_buf: &'d mut [u8]) -> RingBufferedUartRx<'d, T> {
        assert!(!dma_buf.is_empty() && dma_buf.len() <= 0xFFFF);

        let opts = TransferOptions {
            half_transfer_ir: true,
            ..Default::default()
        };

        let rx_dma = self.rx_dma.take().unwrap();
        let ring_buf = unsafe {
            ReadableRingBuffer::new(
                rx_dma.channel,
                rx_dma.request,
                T::regs().datar().as_ptr() as _,
                dma_buf,
                opts,
            )
        };
        let rx = self.rx.take();
        let rts = self.rts.take();

        // Dropping `self` would disable the peripheral.
        mem::forget(self);

        RingBufferedUartRx {
            _phantom: PhantomData,
            rx,
            rts,
            ring_buf,
        }
    }
}

impl<'d, T: Instance> RingBufferedUartRx<'d, T> {
    /// Start receiving in the background into the previously passed buffer.
    ///
    /// This is done automatically by `read()` if required.
    pub fn start_uart(&mut self) {
        // Clear the buffer so that it is ready to receive data
        compiler_fence(Ordering::SeqCst);
        self.ring_buf.reset();
        self.ring_buf.start();

        let r = T::regs();
        // enable interrupts
        r.ctlr1().modify(|w| {
            // disable RXNE interrupt
            w.set_rxneie(false);
            // enable parity interrupt if not ParityNone
            w.set_peie(w.pce());
            // enable idle line interrupt
            w.set_idleie(true);
        });
        r.ctlr3().modify(|w| {
            // enable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.set_eie(true);
            // enable DMA Rx Request
            w.set_dmar(true);
        });
    }

    /// Stop DMA backed UART receiver
    fn stop_uart(&mut self) {
        // paused, not stopped: the next read restarts it after an error
        self.ring_buf.request_pause();

        let r = T::regs();
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

        compiler_fence(Ordering::SeqCst);
    }

    /// Read bytes that are readily available in the ring buffer.
    /// If no bytes are currently available in the buffer the call waits until some
    /// bytes are available (at least one byte and at most half the buffer size), or the line
    /// goes idle.
    ///
    /// Background receive is started if `start_uart()` has not been previously called.
    ///
    /// Receive in the background is terminated if an error is returned.
    /// It must then manually be started again by calling `start_uart()` or by re-calling `read()`.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let r = T::regs();

        // Start DMA and Uart if it was not already started,
        // otherwise check for errors in status register.
        let sr = clear_idle_flag::<T>();
        if !r.ctlr3().read().dmar() {
            self.start_uart();
        } else {
            check_for_errors(sr)?;
        }

        loop {
            match self.ring_buf.read(buf) {
                Ok((0, _)) => {}
                Ok((len, _)) => {
                    return Ok(len);
                }
                Err(_) => {
                    self.stop_uart();
                    return Err(Error::Overrun);
                }
            }

            match self.wait_for_data_or_idle().await {
                Ok(_) => {}
                Err(err) => {
                    self.stop_uart();
                    return Err(err);
                }
            }
        }
    }

    /// Wait for uart idle or dma half-full or full
    async fn wait_for_data_or_idle(&mut self) -> Result<(), Error> {
        compiler_fence(Ordering::SeqCst);

        // The interrupt handler disables the idle interrupt once it fires.
        critical_section::with(|_| T::regs().ctlr1().modify(|w| w.set_idleie(true)));

        // Future which completes when idle line is detected
        let s = T::state();
        let uart = poll_fn(|cx| {
            s.rx_waker.register(cx.waker());

            compiler_fence(Ordering::SeqCst);

            // Critical section is needed so that IDLE isn't set after
            // our read but before we clear it.
            let sr = critical_section::with(|_| clear_idle_flag::<T>());

            check_for_errors(sr)?;

            if sr.idle() {
                // Idle line is detected
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        });

        let mut dma_init = false;
        // Future which completes when there is dma is half full or full
        let dma = poll_fn(|cx| {
            self.ring_buf.set_waker(cx.waker());

            let status = match dma_init {
                false => Poll::Pending,
                true => Poll::Ready(()),
            };

            dma_init = true;
            status
        });

        match select(core::pin::pin!(uart), core::pin::pin!(dma)).await {
            Either::Left((result, _)) => result,
            Either::Right(((), _)) => Ok(()),
        }
    }
}

impl<T: Instance> Drop for RingBufferedUartRx<'_, T> {
    fn drop(&mut self) {
        self.stop_uart();
        self.ring_buf.request_stop();
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
        T::disable();
    }
}

/// Return the status register, and clear the IDLE flag (and the error flags): a status
/// register read followed by a data register read.
fn clear_idle_flag<T: Instance>() -> ch32_metapac::usart::regs::Statr {
    let r = T::regs();
    let sr = r.statr().read();
    let _ = r.datar().read();
    sr
}

fn check_for_errors(s: ch32_metapac::usart::regs::Statr) -> Result<(), Error> {
    if s.pe() {
        Err(Error::Parity)
    } else if s.fe() {
        Err(Error::Framing)
    } else if s.ne() {
        Err(Error::Noise)
    } else if s.ore() {
        Err(Error::Overrun)
    } else {
        Ok(())
    }
}
