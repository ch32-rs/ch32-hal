use core::marker::PhantomData;
use core::sync::atomic::{compiler_fence, Ordering};
use core::task::Poll;

use embassy_usb_driver::{Direction, EndpointError, EndpointInfo, Unsupported};
use futures::future::poll_fn;

use super::regs::{self, EpRxResponse, EpTxResponse, UsbToken};
use super::{ep_buf_addr, Instance, EP0_WAKER, EP_IN_WAKERS, EP_MAX_PACKET_SIZE, EP_OUT_WAKERS};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointDataBuffer, In, Out};

#[inline]
unsafe fn dma_read_volatile(base: usize, buf: &mut [u8]) {
    debug_assert!(base != 0);
    let src = base as *const u8;
    for (i, dst) in buf.iter_mut().enumerate() {
        *dst = core::ptr::read_volatile(src.add(i));
    }
}

#[inline]
unsafe fn dma_write_volatile(base: usize, buf: &[u8]) {
    debug_assert!(base != 0);
    let dst = base as *mut u8;
    for (i, &byte) in buf.iter().enumerate() {
        core::ptr::write_volatile(dst.add(i), byte);
    }
    compiler_fence(Ordering::SeqCst);
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
        let waker = match self.info.addr.direction() {
            Direction::In => &EP_IN_WAKERS[self.info.addr.index()],
            Direction::Out => &EP_OUT_WAKERS[self.info.addr.index()],
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
        trace!("endpoint {} enabled", self.info.addr.index());
    }

    fn is_enabled(&self) -> bool {
        regs::ep_is_enabled(T::regs(), self.info.addr.index(), self.info.addr.direction())
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
        trace!("endpoint {} IN", self.info.addr.index());
        if !self.is_enabled() {
            error!("write to disabled ep {}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }
        if buf.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        // Hardware layout (BUF_MOD = 0): when both RX_EN and TX_EN are active
        // on `ep`, OUT occupies `[UEPn_DMA .. +64)` and IN occupies
        // `[UEPn_DMA + 64 .. +128)`. When only TX_EN is active, IN occupies
        // `[UEPn_DMA .. +64)`.
        let tx_offset = if regs::ep_is_enabled(regs, ep, Direction::Out) {
            usize::from(EP_MAX_PACKET_SIZE)
        } else {
            0
        };
        unsafe { dma_write_volatile(ep_buf_addr(ep) + tx_offset, buf) };
        regs::set_ep_t_len(regs, ep, buf.len() as u16);
        regs::ep_tx_ctrl_set_res(regs, ep, EpTxResponse::ACK);

        // Wait for TX complete
        poll_fn(|ctx| {
            EP_IN_WAKERS[ep].register(ctx.waker());

            if !self.is_enabled() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep as usize == ep && status.token == UsbToken::IN {
                    regs::ep_tx_ctrl_flip_tog(regs, ep, EpTxResponse::NAK);
                    regs::clear_transfer(regs);
                    Poll::Ready(Ok(()))
                } else {
                    Poll::Pending
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() }
            ret
        })
        .await
    }
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out, SIZE> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        trace!("endpoint {} OUT", self.info.addr.index());
        if !self.is_enabled() {
            error!("OUT disabled EP");
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();
        let rx_addr = ep_buf_addr(ep);

        regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::ACK);

        // poll for packet
        poll_fn(|ctx| {
            EP_OUT_WAKERS[ep].register(ctx.waker());

            if !self.is_enabled() {
                return Poll::Ready(Err(EndpointError::Disabled));
            }

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep as usize == ep && status.token == UsbToken::OUT {
                    let ret = if status.tog_ok {
                        // upper bits are reserved (0)
                        let len = regs::rx_len(regs) as usize;
                        let res = if len > buf.len() {
                            Err(EndpointError::BufferOverflow)
                        } else {
                            unsafe { dma_read_volatile(rx_addr, &mut buf[..len]) };
                            Ok(len)
                        };
                        regs::ep_rx_ctrl_flip_tog(regs, ep, EpRxResponse::NAK);
                        Poll::Ready(res)
                    } else {
                        // A retransmitted packet leaves the endpoint armed.
                        Poll::Pending
                    };

                    regs::clear_transfer(regs);
                    ret
                } else {
                    Poll::Pending
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await
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
}

impl<'d, T, const SIZE: usize> embassy_usb_driver::ControlPipe for ControlPipe<'d, T, SIZE>
where
    T: Instance,
{
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(move |ctx| {
            EP0_WAKER.register(ctx.waker());
            let poll_result = {
                let regs = T::regs();
                if regs::int_fg(regs).transfer {
                    let int_status = regs::int_st(regs);

                    // assume the setup packet is always for endpoint 0.
                    // the hardware doesn't seem to set mask_uis_endp to 0
                    // when the setup packet arrives.
                    match int_status.token {
                        UsbToken::SETUP => {
                            // SETUP packet token
                            regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
                            regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::NAK);

                            let mut data = [0u8; 8];
                            unsafe { dma_read_volatile(ep_buf_addr(0), &mut data) };

                            // Clear Flag
                            regs::clear_transfer(regs);
                            Poll::Ready(data)
                        }
                        _ => Poll::Pending,
                    }
                } else {
                    Poll::Pending
                }
            };
            unsafe { T::Interrupt::enable() };
            poll_result
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        let regs = T::regs();

        if buf.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }

        if first {
            regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
        } else {
            regs::ep_rx_ctrl_flip_tog(regs, 0, EpRxResponse::ACK);
        }

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep == 0 && status.token == UsbToken::OUT {
                    let len = regs::rx_len(regs) as usize;
                    let res = if len > buf.len() {
                        Err(EndpointError::BufferOverflow)
                    } else {
                        unsafe { dma_read_volatile(ep_buf_addr(0), &mut buf[..len]) };
                        Ok(len)
                    };
                    regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                    regs::clear_transfer(regs);
                    Poll::Ready(res)
                } else {
                    Poll::Pending
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await?;

        Ok(bytes_read)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let regs = T::regs();

        if data.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }

        unsafe { dma_write_volatile(ep_buf_addr(0), data) };
        regs::set_ep_t_len(regs, 0, data.len() as u16);

        if first {
            regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
        } else {
            regs::ep_tx_ctrl_flip_tog(regs, 0, EpTxResponse::ACK);
        }

        // Poll for last packet to finsh transfer
        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());
            let poll_result = {
                let interrupt_flags = regs::int_fg(regs);
                if interrupt_flags.transfer {
                    let status = regs::int_st(regs);
                    if status.ep == 0 {
                        let res = match status.token {
                            UsbToken::IN => {
                                regs::ep_tx_ctrl_set_res(regs, 0, EpTxResponse::NAK);
                                Poll::Ready(Ok(()))
                            }
                            token => {
                                error!("unexpected USB Token {}, aborting", token.to_bits());
                                Poll::Ready(Err(EndpointError::Disabled))
                            }
                        };

                        regs::clear_transfer(regs);
                        res
                    } else {
                        Poll::Pending
                    }
                } else {
                    Poll::Pending
                }
            };
            unsafe { T::Interrupt::enable() };
            poll_result
        })
        .await?;

        if last {
            // Set RX to true to expect the STATUS (OUT) packet
            regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);

            // Expect the empty OUT token for status
            poll_fn(|ctx| {
                EP0_WAKER.register(ctx.waker());

                let poll_res = if regs::int_fg(regs).transfer {
                    let status = regs::int_st(regs);
                    if status.ep == 0 {
                        let res = match status.token {
                            UsbToken::OUT => {
                                if regs::rx_len(regs) != 0 {
                                    error!("Expected 0 len OUT stage, found non-zero len, aborting");
                                    Poll::Ready(Err(EndpointError::BufferOverflow))
                                } else {
                                    // Set the EP back to NAK so that we are "not ready to recieve"
                                    regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                                    Poll::Ready(Ok(()))
                                }
                            }
                            token => {
                                error!("Got {} instead of OUT token", token.to_bits());
                                Poll::Ready(Err(EndpointError::Disabled))
                            }
                        };
                        regs::clear_transfer(regs);

                        res
                    } else {
                        Poll::Pending
                    }
                } else {
                    Poll::Pending
                };

                unsafe { T::Interrupt::enable() };

                poll_res
            })
            .await?;
        }

        Ok(())
    }

    async fn accept(&mut self) {
        let regs = T::regs();

        // Rx should already be NAK
        regs::set_ep_t_len(regs, 0, 0);
        regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);

        poll_fn(|ctx| {
            EP0_WAKER.register(ctx.waker());

            let res = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep == 0 {
                    match status.token {
                        UsbToken::IN => {
                            // Maybe stall? and unstall when we actually set addr
                            regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
                            regs::clear_transfer(regs);
                        }
                        token => panic!("Token = {}", token.to_bits()),
                    }
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            res
        })
        .await;
    }

    async fn reject(&mut self) {
        let regs = T::regs();
        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::STALL);
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::STALL);
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        regs::set_device_address(T::regs(), addr);
    }
}
