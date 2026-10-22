use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::Ordering;
use core::task::Poll;

use ch32_metapac::usbhs::vals::{EpRxResponse, EpTxResponse};
use embassy_usb_driver::{Direction, EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType};

use super::{iso_in_enqueue, state, EpState, Instance};
use crate::usb::{Dir, EndpointData, In, Out};

pub struct Endpoint<'d, T: Instance, D: Dir, const SIZE: usize> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    pub(crate) data: EndpointData<'d, SIZE>,
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> Endpoint<'d, T, D, SIZE> {
    pub(crate) fn new(info: EndpointInfo, data: EndpointData<'d, SIZE>) -> Self {
        let index = info.addr.index();
        T::dregs().ep_max_len(index).write(|v| v.set_len(info.max_packet_size));
        if info.ep_type != EndpointType::Control {
            match info.addr.direction() {
                Direction::Out => {
                    T::dregs().ep_rx_dma(index - 1).write_value(data.buffer.addr() as u32);
                }
                Direction::In => {
                    T::dregs().ep_tx_dma(index - 1).write_value(data.buffer.addr() as u32);
                }
            }
        }
        Self {
            _phantom: PhantomData,
            info,
            data,
        }
    }

    fn state(&self) -> &'static EpState {
        state(self.info.addr)
    }

    async fn wait_enabled_internal(&mut self) {
        poll_fn(|ctx| {
            self.state().waker.register(ctx.waker());
            if self.is_enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    fn is_enabled(&self) -> bool {
        match self.info.addr.direction() {
            Direction::Out => T::dregs().ep_config().read().r_en(self.info.addr.index() - 1),
            Direction::In => T::dregs().ep_config().read().t_en(self.info.addr.index() - 1),
        }
    }
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> embassy_usb_driver::Endpoint for Endpoint<'d, T, D, SIZE> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        self.wait_enabled_internal().await
    }
}

impl<'d, T: Instance, const SIZE: usize> EndpointOut for Endpoint<'d, T, Out, SIZE> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        if !self.is_enabled() {
            return Err(EndpointError::Disabled);
        }
        // The buffer must be at least as large as the endpoint's max_packet_size
        // to avoid potential buffer overflow when receiving data
        if buf.len() < self.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let s = self.state();
        let index = self.info.addr.index();

        if s.iso.load(Ordering::Relaxed) {
            // drain the ring the interrupt fills
            return poll_fn(|ctx| {
                s.waker.register(ctx.waker());
                if !self.is_enabled() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                match super::iso_out_take(index, |p| {
                    let len = p.len().min(buf.len());
                    buf[..len].copy_from_slice(&p[..len]);
                    len
                }) {
                    Some(len) => Poll::Ready(Ok(len)),
                    None => Poll::Pending,
                }
            })
            .await;
        }

        // arm: ACK until the interrupt has one packet, then NAK until the next read
        critical_section::with(|_| {
            if !s.ready.load(Ordering::Acquire) {
                s.armed.store(true, Ordering::Relaxed);
                T::dregs()
                    .ep_rx_ctrl(index)
                    .modify(|v| v.set_mask_uep_r_res(EpRxResponse::ACK));
            }
        });

        poll_fn(|ctx| {
            s.waker.register(ctx.waker());
            if s.ready.load(Ordering::Acquire) {
                s.ready.store(false, Ordering::Relaxed);
                let len = s.len.load(Ordering::Relaxed) as usize;
                if len > buf.len() {
                    return Poll::Ready(Err(EndpointError::BufferOverflow));
                }
                self.data.buffer.read_volatile(&mut buf[..len]);
                Poll::Ready(Ok(len))
            } else if !self.is_enabled() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl<'d, T: Instance, const SIZE: usize> EndpointIn for Endpoint<'d, T, In, SIZE> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if !self.is_enabled() {
            return Err(EndpointError::Disabled);
        }
        if buf.len() > self.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        let s = self.state();
        let index = self.info.addr.index();

        if s.iso.load(Ordering::Relaxed) {
            // queue it; wait while the ring is full
            return poll_fn(|ctx| {
                s.waker.register(ctx.waker());
                if !self.is_enabled() {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if iso_in_enqueue(index, buf) {
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Pending
                }
            })
            .await;
        }

        s.ready.store(false, Ordering::Relaxed);
        self.data.buffer.write_volatile(buf);
        let d = T::dregs();
        critical_section::with(|_| {
            d.ep_t_len(index).write(|v| v.set_len(buf.len() as u16));
            d.ep_tx_ctrl(index).modify(|v| v.set_mask_uep_t_res(EpTxResponse::ACK));
        });

        poll_fn(|ctx| {
            s.waker.register(ctx.waker());
            if s.ready.load(Ordering::Acquire) {
                s.ready.store(false, Ordering::Relaxed);
                Poll::Ready(Ok(()))
            } else if !self.is_enabled() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await
    }
}
