use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use futures::future::{select, Either};

use super::{receive_error, Error, Instance, UartRx, IDLE_LATCHED};
use crate::dma::{ReadableRingBuffer, TransferOptions};
use crate::gpio::{AnyPin, SealedPin};
use crate::mode::Async;
use crate::Peri;

/// Rx-only UART driver that receives continuously into a DMA ring buffer.
///
/// Unlike [`UartRx::read`], no data is lost between reads, and dropping a `read` future
/// doesn't stop reception.
///
/// The DMA channel's interrupt counts the laps of the buffer and the USART's interrupt reports
/// receive errors: both must be able to preempt the code that reads (see
/// [`ReadableRingBuffer`](crate::dma::ReadableRingBuffer)).
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
        // An error latched for an earlier one-shot read belongs to it, not to this reception.
        critical_section::with(|_| T::state().rx_errors.store(0, Ordering::Release));

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
    /// Start receiving in the background into the previously passed buffer, from an empty
    /// buffer (a reception already running starts again).
    ///
    /// This is done automatically by `read()` if required.
    pub fn start_uart(&mut self) {
        // Reception off at the USART; the DMA channel is left running, as the handler's error
        // path leaves it. A channel that could not serve the USART's last request at once (the
        // DMA busy with other channels) keeps that request, and serves it once it is enabled and
        // the DMA is free, whatever DMAR, RXNE or the channel's set-up have become (measured on a
        // CH32V203): the old byte it then copies must go with the dropped data. Pausing the
        // channel would only carry that copy into the new reception. Left running, the channel
        // serves it as soon as the DMA is free, before the start position below is taken unless
        // other channels hold the DMA until then.
        self.stop_uart();
        compiler_fence(Ordering::SeqCst);
        if !self.ring_buf.is_running() {
            self.ring_buf.start();
        }

        let r = T::regs();
        critical_section::with(|_| {
            // Empty from the DMA's position: with DMAR off, only a request the channel was still
            // holding can move it.
            self.ring_buf.clear();
            T::state().ring_idle.store(true, Ordering::Relaxed);
            r.ctlr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // enable parity interrupt if not ParityNone
                w.set_peie(w.pce());
                // armed for as long as reception runs (see the handler)
                w.set_idleie(true);
            });
            // Apart from a request the channel was still holding, nothing reads DATAR while the
            // DMA request is off. Clear the flags an earlier reception left with the RM §17.10.1
            // sequence, a STATR read followed at once by a DATAR read: left started, the sequence
            // would be ended by the DMA's read of the next byte, clearing that byte's errors. A
            // byte waiting in DATAR goes with the dropped data, as does one that completes between
            // the two reads (unless the channel still holds a request for it, see above). The
            // request goes on right after: a byte completing from then on waits a few
            // instructions for the DMA, not the time the steps above take (on a CH32X035 at
            // 48 MHz, long enough for the bytes behind it to overrun at 2 Mbaud). The handler
            // cannot run before the request is on, so a parity error cannot stop this reception
            // before it starts.
            let _ = r.statr().read();
            let _ = r.datar().read();
            r.ctlr3().modify(|w| {
                // enable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(true);
                // enable DMA Rx Request
                w.set_dmar(true);
            });
        });
    }

    /// Stop DMA backed UART receiver
    fn stop_uart(&mut self) {
        // The DMA channel keeps running (see `start_uart`); without DMAR it receives nothing new.
        let r = T::regs();
        // The handler writes the same registers. Reception is over, so an error it latched is
        // dropped: the caller reports its own error, or starts again.
        critical_section::with(|_| {
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
            T::state().ring_idle.store(false, Ordering::Relaxed);
            T::state().rx_errors.store(0, Ordering::Release);
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
    /// Receive in the background is terminated if an error is returned, and the bytes received
    /// but not returned yet are dropped, including any this call had already copied (an error is
    /// not tied to a position in the buffer, and the bad byte may be among them).
    /// It must then manually be started again by calling `start_uart()` or by re-calling `read()`.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        // An error the handler saw since the last read stopped the DMA request: report it before
        // starting again.
        let (error, _, receiving) = take_rx_events::<T>();
        if let Some(error) = error {
            self.stop_uart();
            return Err(error);
        }
        if !receiving {
            self.start_uart();
        }

        let mut idle_woke = false;
        loop {
            // Forget the idle lines seen so far before looking at the buffer: a burst that ends
            // after this look wakes the wait below, and one that ended before it is in the buffer.
            // The idle line interrupt is armed again if the handler had to disarm it, in the same
            // critical section: an error latched after it has disarmed the interrupt again.
            let error = critical_section::with(|_| {
                let (error, _, _) = take_rx_events::<T>();
                if error.is_none() {
                    T::regs().ctlr1().modify(|w| w.set_idleie(true));
                }
                error
            });
            if let Some(error) = error {
                self.stop_uart();
                return Err(error);
            }
            // Likewise for the DMA's half and full transfer interrupts: registered before the
            // look, one that comes after it wakes this task, which then looks again.
            poll_fn(|cx| {
                self.ring_buf.set_waker(cx.waker());
                Poll::Ready(())
            })
            .await;
            match self.ring_buf.read(buf) {
                Ok((0, _)) => {
                    // Woken by an idle line, yet nothing new: a byte the DMA has yet to take (busy
                    // with other channels) keeps IDLE set, and arming the interrupt again brings
                    // the handler straight back. Let the other tasks run, then look again: a half
                    // or full transfer interrupt meanwhile wakes this task while it yields, not the
                    // wait below.
                    if idle_woke {
                        idle_woke = false;
                        let mut yielded = false;
                        poll_fn(|cx| {
                            if yielded {
                                return Poll::Ready(());
                            }
                            yielded = true;
                            cx.waker().wake_by_ref();
                            Poll::Pending
                        })
                        .await;
                        continue;
                    }
                }
                Ok((len, _)) => {
                    // A bad byte can reach the buffer after the check above: the DMA reads it as
                    // soon as it is complete, while its error interrupt is still to run. The
                    // handler preempts this task, so by now it has latched the error, and the
                    // copy is dropped with the rest of the reception.
                    let (error, _, _) = take_rx_events::<T>();
                    if let Some(error) = error {
                        self.stop_uart();
                        return Err(error);
                    }
                    return Ok(len);
                }
                Err(_) => {
                    self.stop_uart();
                    return Err(Error::Overrun);
                }
            }

            match self.wait_for_data_or_idle().await {
                Ok(idle) => idle_woke = idle,
                Err(err) => {
                    self.stop_uart();
                    return Err(err);
                }
            }
        }
    }

    /// Wait for uart idle (`Ok(true)`) or dma half-full or full (`Ok(false)`)
    async fn wait_for_data_or_idle(&mut self) -> Result<bool, Error> {
        compiler_fence(Ordering::SeqCst);

        // Future which completes when idle line is detected (the handler latches it)
        let s = T::state();
        let uart = poll_fn(|cx| {
            s.rx_waker.register(cx.waker());

            compiler_fence(Ordering::SeqCst);

            let (error, idle, _) = take_rx_events::<T>();
            if let Some(error) = error {
                return Poll::Ready(Err(error));
            }
            if idle {
                return Poll::Ready(Ok(true));
            }
            Poll::Pending
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
            Either::Right(((), _)) => Ok(false),
        }
    }
}

impl<T: Instance> Drop for RingBufferedUartRx<'_, T> {
    fn drop(&mut self) {
        self.stop_uart();
        self.ring_buf.request_stop();
        self.rx.as_ref().map(|x| x.set_as_disconnected());
        self.rts.as_ref().map(|x| x.set_as_disconnected());
        // This drop owns the RX half's refcount taken by `UartRx::new_inner`;
        // `mem::forget` in `into_ring_buffered` transferred it here.
        super::disable_peripheral::<T>();
    }
}

/// What the interrupt handler saw: the receive error it latched, if any; whether it latched an
/// idle line; and whether the DMA request is still on (its error path clears it). One critical
/// section, so that all three come from the same side of the handler.
///
/// While receiving, the reader never reads STATR: a STATR read starts the RM §17.10.1 "read
/// STATR, then read DATAR" clear sequence, and the DMA's next DATAR read would end it and clear a
/// PE, FE, NE or ORE before the handler sees it, as `UartRx::read_until_idle` already avoids.
fn take_rx_events<T: Instance>() -> (Option<Error>, bool, bool) {
    critical_section::with(|_| {
        let s = T::state();
        let r = T::regs();
        let latched = s.rx_errors.load(Ordering::Acquire);
        s.rx_errors.store(0, Ordering::Release);
        let error = receive_error(ch32_metapac::usart::regs::Statr(latched));
        (error, latched & IDLE_LATCHED != 0, r.ctlr3().read().dmar())
    })
}
