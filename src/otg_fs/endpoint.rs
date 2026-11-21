use core::marker::PhantomData;
use core::sync::atomic::{fence, AtomicBool, Ordering};
use core::task::Poll;

use embassy_usb_driver::{Direction, EndpointError, EndpointInfo, Unsupported};
use futures::future::poll_fn;

use super::regs::{self, EpRxResponse, EpTxResponse};
use super::{
    bus_rst_pending, ep_buf_addr, ep_is_bidir, setup_pending, take_setup, Instance, EP0_WAKER, EP_MAX_PACKET_SIZE,
    IN_STATE, OUT_STATE,
};
use crate::usb::{Dir, EndpointDataBuffer, In, Out};

static OUT_LOGICALLY_ENABLED: [AtomicBool; super::MAX_NR_EP] = [const { AtomicBool::new(false) }; super::MAX_NR_EP];

pub(crate) fn set_out_logically_enabled(ep: usize, enabled: bool) {
    OUT_LOGICALLY_ENABLED[ep].store(enabled, Ordering::Release);
}

pub(crate) fn is_out_logically_enabled(ep: usize) -> bool {
    OUT_LOGICALLY_ENABLED[ep].load(Ordering::Acquire)
}

#[inline]
unsafe fn dma_read_volatile(base: usize, buf: &mut [u8]) {
    assert!(base != 0, "USBFS endpoint DMA buffer not initialized");
    fence(Ordering::Acquire);
    let src = base as *const u8;
    for (i, dst) in buf.iter_mut().enumerate() {
        *dst = core::ptr::read_volatile(src.add(i));
    }
}

#[inline]
unsafe fn dma_write_volatile(base: usize, buf: &[u8]) {
    assert!(base != 0, "USBFS endpoint DMA buffer not initialized");
    let dst = base as *mut u8;
    for (i, &byte) in buf.iter().enumerate() {
        core::ptr::write_volatile(dst.add(i), byte);
    }
    fence(Ordering::Release);
}

struct TxDropGuard<T: Instance> {
    ep: usize,
    completed: bool,
    _phantom: PhantomData<T>,
}

impl<T: Instance> Drop for TxDropGuard<T> {
    fn drop(&mut self) {
        if !self.completed {
            critical_section::with(|_| {
                regs::ep_tx_ctrl_set_res(T::regs(), self.ep, EpTxResponse::NAK);
                IN_STATE[self.ep].ready.store(false, Ordering::Relaxed);
            });
        }
    }
}

struct RxDropGuard<T: Instance> {
    ep: usize,
    completed: bool,
    _phantom: PhantomData<T>,
}

impl<T: Instance> Drop for RxDropGuard<T> {
    fn drop(&mut self) {
        if !self.completed {
            critical_section::with(|_| {
                regs::ep_rx_ctrl_set_res(T::regs(), self.ep, EpRxResponse::NAK);
                OUT_STATE[self.ep].ready.store(false, Ordering::Relaxed);
            });
        }
    }
}

/// USB endpoint.
pub struct Endpoint<'d, T, D, const SIZE: usize> {
    _phantom: PhantomData<&'d mut (T, D, EndpointDataBuffer<SIZE>)>,
    info: EndpointInfo,
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> Endpoint<'d, T, D, SIZE> {
    pub(crate) fn new(info: EndpointInfo) -> Self {
        Self {
            _phantom: PhantomData,
            info,
        }
    }

    async fn wait_enabled_internal(&mut self) {
        let ep = self.info.addr.index();
        let waker = match self.info.addr.direction() {
            Direction::In => &IN_STATE[ep].waker,
            Direction::Out => &OUT_STATE[ep].waker,
        };
        poll_fn(|ctx| {
            waker.register(ctx.waker());
            if self.is_enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        trace!("endpoint {} enabled", ep);
    }

    fn is_enabled(&self) -> bool {
        let ep = self.info.addr.index();
        match self.info.addr.direction() {
            Direction::In => regs::ep_is_enabled(T::regs(), ep, Direction::In),
            Direction::Out => {
                if ep_is_bidir(ep) {
                    OUT_LOGICALLY_ENABLED[ep].load(Ordering::Acquire)
                } else {
                    regs::ep_is_enabled(T::regs(), ep, Direction::Out)
                }
            }
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

    fn set_stall(&mut self, _stalled: bool) -> Result<(), Unsupported> {
        Err(Unsupported)
    }

    fn is_stalled(&self) -> Result<bool, Unsupported> {
        Err(Unsupported)
    }
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In, SIZE> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let ep = self.info.addr.index();
        trace!("endpoint {} IN", ep);
        if !self.is_enabled() {
            error!("write to disabled ep {}", ep);
            return Err(EndpointError::Disabled);
        }
        if buf.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }

        let regs = T::regs();
        let base = ep_buf_addr(ep);
        if base == 0 {
            return Err(EndpointError::Disabled);
        }

        // Hardware layout (`BUF_MOD = 0`): bidirectional endpoint numbers keep
        // `RX_EN = 1` whenever `TX_EN = 1`, so `IN` always lives at `base + 64`
        // and `OUT` at `base + 0`. Unidirectional `IN` endpoints have `RX_EN = 0`
        // and live at `base + 0`.
        let tx_offset = if ep_is_bidir(ep) {
            usize::from(EP_MAX_PACKET_SIZE)
        } else {
            0
        };
        unsafe { dma_write_volatile(base + tx_offset, buf) };

        let mut guard = TxDropGuard::<T> {
            ep,
            completed: false,
            _phantom: PhantomData,
        };
        critical_section::with(|_| {
            IN_STATE[ep].ready.store(false, Ordering::Relaxed);
            regs::set_ep_t_len(regs, ep, buf.len() as u16);
            regs::ep_tx_ctrl_set_res(regs, ep, EpTxResponse::ACK);
        });

        let res = poll_fn(|ctx| {
            IN_STATE[ep].waker.register(ctx.waker());

            if IN_STATE[ep].ready.load(Ordering::Acquire) {
                IN_STATE[ep].ready.store(false, Ordering::Relaxed);
                Poll::Ready(Ok(()))
            } else if !self.is_enabled() || bus_rst_pending() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await;

        if res.is_ok() {
            guard.completed = true;
        }
        res
    }
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out, SIZE> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let ep = self.info.addr.index();
        trace!("endpoint {} OUT", ep);
        if !self.is_enabled() {
            error!("OUT disabled EP");
            return Err(EndpointError::Disabled);
        }
        if buf.len() < usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }

        let regs = T::regs();
        let rx_addr = ep_buf_addr(ep);
        if rx_addr == 0 {
            return Err(EndpointError::Disabled);
        }

        let mut guard = RxDropGuard::<T> {
            ep,
            completed: false,
            _phantom: PhantomData,
        };
        critical_section::with(|_| {
            OUT_STATE[ep].ready.store(false, Ordering::Relaxed);
            regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::ACK);
        });

        let res = poll_fn(|ctx| {
            OUT_STATE[ep].waker.register(ctx.waker());

            if OUT_STATE[ep].ready.load(Ordering::Acquire) {
                OUT_STATE[ep].ready.store(false, Ordering::Relaxed);
                let len = OUT_STATE[ep].len.load(Ordering::Relaxed) as usize;
                if len > usize::from(EP_MAX_PACKET_SIZE) || len > buf.len() {
                    Poll::Ready(Err(EndpointError::BufferOverflow))
                } else {
                    unsafe { dma_read_volatile(rx_addr, &mut buf[..len]) };
                    Poll::Ready(Ok(len))
                }
            } else if !self.is_enabled() || bus_rst_pending() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else {
                Poll::Pending
            }
        })
        .await;

        if res.is_ok() {
            guard.completed = true;
        }
        res
    }
}

pub struct ControlPipe<'d, T, const SIZE: usize> {
    max_packet_size: u16,
    _phantom: PhantomData<&'d mut (T, EndpointDataBuffer<SIZE>)>,
}

impl<'d, T: Instance, const SIZE: usize> ControlPipe<'d, T, SIZE> {
    pub(crate) fn new(max_packet_size: u16) -> Self {
        Self {
            max_packet_size,
            _phantom: PhantomData,
        }
    }

    async fn wait_ep0_in(&self) -> Result<(), EndpointError> {
        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());
            if setup_pending() || bus_rst_pending() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if IN_STATE[0].ready.load(Ordering::Acquire) {
                IN_STATE[0].ready.store(false, Ordering::Relaxed);
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    async fn wait_ep0_out(&self) -> Result<usize, EndpointError> {
        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());
            if setup_pending() || bus_rst_pending() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if OUT_STATE[0].ready.load(Ordering::Acquire) {
                OUT_STATE[0].ready.store(false, Ordering::Relaxed);
                Poll::Ready(Ok(OUT_STATE[0].len.load(Ordering::Relaxed) as usize))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Runs the zero-length IN status stage; returns `false` if aborted by a new SETUP or bus reset.
    async fn status_in(&self) -> bool {
        if setup_pending() || bus_rst_pending() {
            return false;
        }
        let regs = T::regs();
        let mut guard = TxDropGuard::<T> {
            ep: 0,
            completed: false,
            _phantom: PhantomData,
        };
        critical_section::with(|_| {
            IN_STATE[0].ready.store(false, Ordering::Relaxed);
            regs::set_ep_t_len(regs, 0, 0);
            regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
        });
        let ok = self.wait_ep0_in().await.is_ok();
        if ok {
            guard.completed = true;
            regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
        }
        ok
    }
}

impl<'d, T, const SIZE: usize> embassy_usb_driver::ControlPipe for ControlPipe<'d, T, SIZE>
where
    T: Instance,
{
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());
            match take_setup() {
                Some(data) => Poll::Ready(data),
                None => Poll::Pending,
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        let regs = T::regs();

        if buf.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        if setup_pending() || bus_rst_pending() {
            return Err(EndpointError::Disabled);
        }

        let mut guard = RxDropGuard::<T> {
            ep: 0,
            completed: false,
            _phantom: PhantomData,
        };
        critical_section::with(|_| {
            OUT_STATE[0].ready.store(false, Ordering::Relaxed);
            if first {
                regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
            } else {
                regs::ep_rx_ctrl_flip_tog(regs, 0, EpRxResponse::ACK);
            }
        });

        let len = self.wait_ep0_out().await?;
        guard.completed = true;
        if len > usize::from(EP_MAX_PACKET_SIZE) || len > buf.len() {
            return Err(EndpointError::BufferOverflow);
        }
        unsafe { dma_read_volatile(ep_buf_addr(0), &mut buf[..len]) };
        Ok(len)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let regs = T::regs();

        if data.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        if setup_pending() || bus_rst_pending() {
            return Err(EndpointError::Disabled);
        }

        unsafe { dma_write_volatile(ep_buf_addr(0), data) };

        let mut tx_guard = TxDropGuard::<T> {
            ep: 0,
            completed: false,
            _phantom: PhantomData,
        };
        critical_section::with(|_| {
            IN_STATE[0].ready.store(false, Ordering::Relaxed);
            regs::set_ep_t_len(regs, 0, data.len() as u16);
            if first {
                regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
            } else {
                regs::ep_tx_ctrl_flip_tog(regs, 0, EpTxResponse::ACK);
            }
        });

        self.wait_ep0_in().await?;
        tx_guard.completed = true;

        if last {
            if setup_pending() || bus_rst_pending() {
                return Err(EndpointError::Disabled);
            }
            let mut rx_guard = RxDropGuard::<T> {
                ep: 0,
                completed: false,
                _phantom: PhantomData,
            };
            critical_section::with(|_| {
                OUT_STATE[0].ready.store(false, Ordering::Relaxed);
                regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
            });

            let len = self.wait_ep0_out().await?;
            rx_guard.completed = true;
            if len != 0 {
                error!("Expected 0 len OUT stage, found non-zero len, aborting");
                return Err(EndpointError::BufferOverflow);
            }
        }

        Ok(())
    }

    async fn accept(&mut self) {
        let _ = self.status_in().await;
    }

    async fn reject(&mut self) {
        let regs = T::regs();
        critical_section::with(|_| {
            IN_STATE[0].reset();
            OUT_STATE[0].reset();
            regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::STALL);
            regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::STALL);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        if self.status_in().await {
            regs::set_device_address(T::regs(), addr);
        }
    }
}
