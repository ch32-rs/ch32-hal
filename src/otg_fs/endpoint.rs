use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo};
use futures::future::poll_fn;

use super::regs::{self, EpRxResponse, EpTxResponse, UsbToken};
use super::{Instance, EP_MAX_PACKET_SIZE, EP_WAKERS};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{Dir, EndpointData, In, Out};

/// USB endpoint.
pub struct Endpoint<'d, T, D, const SIZE: usize> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    data: EndpointData<'d, SIZE>,
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> Endpoint<'d, T, D, SIZE> {
    pub(crate) fn new(info: EndpointInfo, data: EndpointData<'d, SIZE>) -> Self {
        regs::set_ep_dma(T::regs(), info.addr.index() as usize, data.buffer.addr() as u32);
        Self {
            _phantom: PhantomData,
            info,
            data,
        }
    }

    async fn wait_enabled_internal(&mut self) {
        poll_fn(|ctx| {
            EP_WAKERS[self.info.addr.index() as usize].register(ctx.waker());
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
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In, SIZE> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        trace!("endpoint {} IN", self.info.addr.index());
        if !self.is_enabled() {
            error!("write to disabled ep {}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        // Write buffer, txLen, and ACK
        self.data.buffer.write_volatile(buf);
        regs::set_ep_t_len(regs, ep, buf.len() as u16);
        regs::ep_tx_ctrl_set_res(regs, ep, EpTxResponse::ACK);

        // Wait for TX complete
        let tx_result = poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep as usize == ep {
                    let token = status.token;
                    let ret = match token {
                        UsbToken::IN => {
                            regs::ep_tx_ctrl_flip_tog(regs, ep, EpTxResponse::NAK);
                            Poll::Ready(Ok(()))
                        }
                        token => {
                            error!("Unexpected USB Token {}", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };
                    regs::clear_transfer(regs);
                    ret
                } else {
                    Poll::Pending
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() }
            ret
        })
        .await;

        tx_result
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

        // Tx Ctrl should be NAK

        regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::ACK);

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep as usize == ep {
                    let ret = match status.token {
                        UsbToken::OUT if status.tog_ok => {
                            // upper bits are reserved (0)
                            let len = regs::rx_len(regs) as usize;

                            self.data.buffer.read_volatile(&mut buf[..len]);
                            regs::ep_rx_ctrl_flip_tog(regs, ep, EpRxResponse::NAK);
                            Poll::Ready(Ok(len))
                        }
                        // A retransmitted packet leaves the endpoint armed.
                        UsbToken::OUT => Poll::Pending,
                        token => {
                            error!("Unexpected USB Token {}", token.to_bits());
                            regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::NAK);
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
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
        .await?;

        Ok(bytes_read)
    }
}

pub struct ControlPipe<'d, T, const SIZE: usize> {
    ep0_buf: EndpointData<'d, SIZE>,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance, const SIZE: usize> ControlPipe<'d, T, SIZE> {
    pub(crate) fn new(ep0_buf: EndpointData<'d, SIZE>) -> Self {
        Self {
            ep0_buf,
            _phantom: PhantomData,
        }
    }
}

impl<'d, T, const SIZE: usize> embassy_usb_driver::ControlPipe for ControlPipe<'d, T, SIZE>
where
    T: Instance,
{
    fn max_packet_size(&self) -> usize {
        usize::from(EP_MAX_PACKET_SIZE)
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(move |ctx| {
            super::EP_WAKERS[0].register(ctx.waker());
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
                            self.ep0_buf.buffer.read_volatile(&mut data[..8]);

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

        if buf.len() > self.ep0_buf.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        if first {
            regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
        } else {
            regs::ep_rx_ctrl_flip_tog(regs, 0, EpRxResponse::ACK);
        }

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let ret = if regs::int_fg(regs).transfer {
                let status = regs::int_st(regs);
                if status.ep == 0 {
                    let res = match status.token {
                        UsbToken::OUT => {
                            let len = regs::rx_len(regs) as usize;
                            self.ep0_buf.buffer.read_volatile(&mut buf[..len]);
                            regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                            Poll::Ready(Ok(len))
                        }
                        // X0 names bit 0 of the token field `SOF`, not `RSVD`.
                        _ => unreachable!(),
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
            ret
        })
        .await?;

        Ok(bytes_read)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let regs = T::regs();

        if data.len() > self.ep0_buf.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        self.ep0_buf.buffer.write_volatile(data);
        regs::set_ep_t_len(regs, 0, data.len() as u16);

        if first {
            regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
        } else {
            regs::ep_tx_ctrl_flip_tog(regs, 0, EpTxResponse::ACK);
        }

        // Poll for last packet to finsh transfer
        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());
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
                super::EP_WAKERS[0].register(ctx.waker());

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
            super::EP_WAKERS[0].register(ctx.waker());

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
