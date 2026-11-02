use core::future::{poll_fn, Future};
use core::pin::Pin;
use core::task::{Context, Poll};

use super::transfer::{hardware_error, DmaFrame};
use super::{Async, Error, I3c, Instance, OnDrop};
use crate::dma::Transfer;

const DMA_REQUESTS: u32 = (1 << 8) | (1 << 12);
const ERRORS: u32 = (1 << 10) | (1 << 11);
const FRAME_COMPLETE: u32 = 1 << 9;
const CONTROL_READY: u32 = 1 << 2;

fn check_error(events: u32, status: u32) -> Result<(), Error> {
    if events & (1 << 11) != 0 {
        Err(hardware_error(status))
    } else if events & (1 << 10) != 0 {
        Err(Error::ShortRead)
    } else {
        Ok(())
    }
}

fn poll_transfer(transfer: &mut Option<Transfer<'_>>, context: &mut Context<'_>) -> Result<bool, Error> {
    let Some(transfer) = transfer else {
        return Ok(true);
    };
    let complete = Pin::new(&mut *transfer).poll(context).is_ready();
    if transfer.has_error() || (complete && transfer.get_remaining_transfers() != 0) {
        return Err(Error::Dma);
    }
    Ok(complete)
}

impl<T: Instance> I3c<'_, T, Async> {
    pub(super) async fn run_dma(&mut self, frame: DmaFrame<'_>) -> Result<(), Error> {
        self.begin(frame.address, frame.legacy)?;
        let cleanup = OnDrop::new(Self::quiesce);
        let regs = T::regs();
        let timeout = self.config.timeout;
        let mut tx = frame.tx.map(|buffer| {
            let dma = self.tx_dma.as_mut().unwrap();
            unsafe {
                Transfer::new_write_checked(
                    dma.channel.reborrow(),
                    dma.request,
                    buffer,
                    regs.tdr().as_ptr().cast::<u8>(),
                    Default::default(),
                )
            }
        });
        let mut rx = frame.rx.map(|buffer| {
            let dma = self.rx_dma.as_mut().unwrap();
            unsafe {
                Transfer::new_read_checked(
                    dma.channel.reborrow(),
                    dma.request,
                    regs.rdr().as_ptr().cast::<u8>(),
                    buffer,
                    Default::default(),
                )
            }
        });
        let requests = OnDrop::new(|| regs.cfgr().modify(|reg| reg.0 &= !DMA_REQUESTS));
        let enabled = (u32::from(tx.is_some()) << 12) | (u32::from(rx.is_some()) << 8);
        regs.cfgr().modify(|reg| reg.0 = (reg.0 & !DMA_REQUESTS) | enabled);
        let mut queued = 0;
        let mut frame_done = false;
        let transaction = poll_fn(|context| {
            T::state().waker.register(context.waker());
            regs.intenr().write(|reg| reg.0 = 0);
            for _ in 0..3 {
                let events = regs.evr().read().0;
                if let Err(error) = check_error(events, regs.stater().read().0) {
                    return Poll::Ready(Err(error));
                }
                let tx_done = match poll_transfer(&mut tx, context) {
                    Ok(done) => done,
                    Err(error) => return Poll::Ready(Err(error)),
                };
                let rx_done = match poll_transfer(&mut rx, context) {
                    Ok(done) => done,
                    Err(error) => return Poll::Ready(Err(error)),
                };
                if queued < frame.count && events & CONTROL_READY != 0 {
                    regs.ctlr().write(|reg| reg.0 = frame.controls[queued]);
                    queued += 1;
                    continue;
                }
                if events & FRAME_COMPLETE != 0 {
                    frame_done = true;
                    if queued != frame.count {
                        return Poll::Ready(Err(Error::ShortRead));
                    }
                    if tx_done && rx_done {
                        let events = regs.evr().read().0;
                        if let Err(error) = check_error(events, regs.stater().read().0) {
                            return Poll::Ready(Err(error));
                        }
                        if tx.as_ref().is_some_and(Transfer::has_error) || rx.as_ref().is_some_and(Transfer::has_error)
                        {
                            return Poll::Ready(Err(Error::Dma));
                        }
                        if events & (1 << 5) != 0 {
                            return Poll::Ready(Err(Error::Fifo));
                        }
                        return Poll::Ready(Ok(()));
                    }
                }
                break;
            }
            regs.intenr().write(|reg| {
                reg.0 = ERRORS
                    | if frame_done { 0 } else { FRAME_COMPLETE }
                    | if queued < frame.count { CONTROL_READY } else { 0 };
            });
            Poll::Pending
        });
        let result = embassy_time::with_timeout(timeout, transaction)
            .await
            .map_err(|_| Error::Timeout)
            .and_then(core::convert::identity);
        drop(requests);
        drop(rx);
        drop(tx);
        if result.is_ok() {
            Self::finish();
            self.needs_recovery = false;
            cleanup.defuse();
        }
        result
    }
}
