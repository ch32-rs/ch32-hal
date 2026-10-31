use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use core::task::Poll;

use ch32_metapac::usbhs::vals::{EpRxResponse, EpTog, EpTxResponse};
use embassy_usb_driver::EndpointError;

use super::endpoint::Endpoint;
use super::{setup_pending, take_setup, EpState, Instance, IN_STATE, OUT_STATE};
use crate::usb::InOut;

pub struct ControlPipe<'d, T: Instance, const SIZE: usize> {
    _phantom: PhantomData<&'d mut T>,
    ep0: Endpoint<'d, T, InOut, SIZE>,
}

impl<'d, T: Instance, const SIZE: usize> ControlPipe<'d, T, SIZE> {
    pub(crate) fn new(ep0: Endpoint<'d, T, InOut, SIZE>) -> Self {
        Self {
            _phantom: PhantomData,
            ep0,
        }
    }
}

/// Wait for the interrupt to complete an EP0 stage; a new setup packet aborts it.
async fn complete(s: &EpState) -> Result<usize, EndpointError> {
    poll_fn(|ctx| {
        s.waker.register(ctx.waker());
        if s.ready.load(Ordering::Acquire) {
            s.ready.store(false, Ordering::Relaxed);
            Poll::Ready(Ok(s.len.load(Ordering::Relaxed) as usize))
        } else if setup_pending() {
            Poll::Ready(Err(EndpointError::Disabled))
        } else {
            Poll::Pending
        }
    })
    .await
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::ControlPipe for ControlPipe<'d, T, SIZE> {
    fn max_packet_size(&self) -> usize {
        use embassy_usb_driver::Endpoint;

        self.ep0.info().max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        // the interrupt copied it and set EP0 to NAK both ways
        poll_fn(|ctx| {
            OUT_STATE[0].waker.register(ctx.waker());
            match take_setup() {
                Some(p) => Poll::Ready(p),
                None => Poll::Pending,
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        let d = T::dregs();

        if buf.len() > self.ep0.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if setup_pending() {
            return Err(EndpointError::Disabled);
        }

        OUT_STATE[0].ready.store(false, Ordering::Relaxed);
        critical_section::with(|_| {
            // the chunks embassy-usb asks for are the host's packets: the length if the
            // interrupt only sees the host move on
            OUT_STATE[0].len.store(buf.len() as u16, Ordering::Relaxed);
            OUT_STATE[0].armed.store(true, Ordering::Relaxed);
            if first {
                d.ep_rx_ctrl(0).write(|v| {
                    v.set_mask_uep_r_tog(EpTog::DATA1);
                    v.set_mask_uep_r_res(EpRxResponse::ACK);
                })
            } else {
                // the interrupt flipped the toggle after the last packet
                d.ep_rx_ctrl(0).modify(|v| v.set_mask_uep_r_res(EpRxResponse::ACK))
            }
        });

        let len = complete(&OUT_STATE[0]).await?;
        if len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        self.ep0.data.buffer.read_volatile(&mut buf[..len]);
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let d = T::dregs();

        if data.len() > self.ep0.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        if setup_pending() {
            return Err(EndpointError::Disabled);
        }

        self.ep0.data.buffer.write_volatile(data);
        IN_STATE[0].ready.store(false, Ordering::Relaxed);
        critical_section::with(|_| {
            if last {
                // the status stage (a zero-length OUT, DATA1) right away: the host gets through
                // even if no entry sees the data packet's completion (its OUT stands for it)
                OUT_STATE[0].ready.store(false, Ordering::Relaxed);
                OUT_STATE[0].len.store(0, Ordering::Relaxed);
                d.ep_rx_ctrl(0).write(|w| {
                    w.set_mask_uep_r_tog(EpTog::DATA1);
                    w.set_mask_uep_r_res(EpRxResponse::ACK);
                });
            }
            IN_STATE[0].armed.store(true, Ordering::Relaxed);
            d.ep_t_len(0).write(|v| v.set_len(data.len() as u16));
            if first {
                d.ep_tx_ctrl(0).write(|v| {
                    v.set_mask_uep_t_res(EpTxResponse::ACK);
                    v.set_mask_uep_t_tog(EpTog::DATA1);
                })
            } else {
                d.ep_tx_ctrl(0).modify(|v| v.set_mask_uep_t_res(EpTxResponse::ACK));
            }
        });

        complete(&IN_STATE[0]).await?;

        if last {
            if complete(&OUT_STATE[0]).await? != 0 {
                error!("Expected 0 len OUT stage, found non-zero len, aborting");
                return Err(EndpointError::BufferOverflow);
            }
        }
        Ok(())
    }

    async fn accept(&mut self) {
        let d = T::dregs();

        if setup_pending() {
            return;
        }
        IN_STATE[0].ready.store(false, Ordering::Relaxed);
        critical_section::with(|_| {
            d.ep_t_len(0).write(|w| w.set_len(0));
            d.ep_tx_ctrl(0).modify(|w| {
                // status stage starts with DATA1
                w.set_mask_uep_t_tog(EpTog::DATA1);
                w.set_mask_uep_t_res(EpTxResponse::ACK);
            });
        });

        // a new setup packet instead of the status stage: nothing left to do
        let _ = complete(&IN_STATE[0]).await;
    }

    async fn reject(&mut self) {
        let d = T::dregs();

        critical_section::with(|_| {
            OUT_STATE[0].armed.store(false, Ordering::Relaxed);
            IN_STATE[0].armed.store(false, Ordering::Relaxed);
            d.ep_tx_ctrl(0).modify(|v| v.set_mask_uep_t_res(EpTxResponse::STALL));
            d.ep_rx_ctrl(0).modify(|v| v.set_mask_uep_r_res(EpRxResponse::STALL));
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        let r = T::regs();
        trace!("accepting address: {}", addr);
        self.accept().await;
        r.dev_ad().write(|w| w.set_addr(addr));
    }
}
