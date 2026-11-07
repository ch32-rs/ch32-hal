use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo};
use futures::future::poll_fn;

use super::{endpoint_is_enabled, set_ep_dma, set_ep_tx_len, Instance, EP_MAX_PACKET_SIZE, EP_WAKERS};
use crate::interrupt::typelevel::Interrupt;
use crate::pac::usb::vals::{EpRxResponse, EpTxResponse, UsbToken};
use crate::usb::{Dir, EndpointData, In, Out};

/// USB endpoint.
pub struct Endpoint<'d, T, D, const SIZE: usize> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    data: EndpointData<'d, SIZE>,
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> Endpoint<'d, T, D, SIZE> {
    pub(crate) fn new(info: EndpointInfo, data: EndpointData<'d, SIZE>) -> Self {
        set_ep_dma(T::regs(), info.addr.index() as usize, data.buffer.addr());
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
        endpoint_is_enabled(T::regs(), self.info.addr.index(), self.info.addr.direction())
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

        self.data.buffer.write_volatile(buf);
        set_ep_tx_len(regs, ep, buf.len());
        regs.uep_ctrl(ep).modify(|v| {
            v.set_t_tog(!v.t_tog());
            v.set_mask_t_res(EpTxResponse::ACK);
        });

        poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let token = status.mask_token();
                    let ret = match token {
                        UsbToken::IN => {
                            regs.uep_ctrl(ep).modify(|v| {
                                v.set_mask_t_res(EpTxResponse::NAK);
                            });
                            Poll::Ready(Ok(()))
                        }
                        token => {
                            error!("Unexpected USB Token {}", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };
                    regs.int_fg().write(|v| v.set_transfer(true));
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

        // L1FS names the RX toggle MASK_R_TOG (otg_fs calls it R_TOG).
        regs.uep_ctrl(ep).modify(|v| {
            v.set_mask_r_tog(!v.mask_r_tog());
            v.set_mask_r_res(EpRxResponse::ACK);
        });

        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let ret = match status.mask_token() {
                        UsbToken::OUT => {
                            let len = regs.rx_len().read().0 as usize;
                            self.data.buffer.read_volatile(&mut buf[..len]);
                            Poll::Ready(Ok(len))
                        }
                        token => {
                            error!("Unexpected USB Token {}", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };

                    regs.uep_ctrl(ep).modify(|v| {
                        v.set_mask_r_res(EpRxResponse::NAK);
                    });
                    regs.int_fg().write(|v| v.set_transfer(true));
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
                if regs.int_fg().read().transfer() {
                    let int_status = regs.int_st().read();

                    // Assume SETUP is always for EP0 — hardware may not set
                    // mask_uis_endp to 0 when the SETUP packet arrives.
                    match int_status.mask_token() {
                        UsbToken::SETUP => {
                            regs.uep_ctrl(0).write(|w| {
                                w.set_mask_t_res(EpTxResponse::NAK);
                                w.set_mask_r_res(EpRxResponse::NAK);
                            });

                            let mut data = [0u8; 8];
                            self.ep0_buf.buffer.read_volatile(&mut data[..8]);

                            regs.int_fg().write(|v| v.set_transfer(true));
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
            regs.uep_ctrl(0).modify(|v| {
                v.set_mask_r_tog(true);
                v.set_mask_r_res(EpRxResponse::ACK);
            });
        } else {
            regs.uep_ctrl(0).modify(|v| {
                v.set_mask_r_tog(!v.mask_r_tog());
                v.set_mask_r_res(EpRxResponse::ACK);
            });
        }

        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() == 0 {
                    let res = match status.mask_token() {
                        UsbToken::OUT => {
                            let len = regs.rx_len().read().0 as usize;
                            self.ep0_buf.buffer.read_volatile(&mut buf[..len]);
                            regs.uep_ctrl(0).modify(|v| {
                                v.set_mask_r_res(EpRxResponse::NAK);
                            });
                            Poll::Ready(Ok(len))
                        }
                        UsbToken::RSVD | UsbToken::IN | UsbToken::SETUP => unreachable!(),
                    };

                    regs.int_fg().write(|v| v.set_transfer(true));
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
        set_ep_tx_len(regs, 0, data.len());

        if first {
            regs.uep_ctrl(0).modify(|v| {
                v.set_mask_t_res(EpTxResponse::ACK);
                v.set_t_tog(true);
            });
        } else {
            regs.uep_ctrl(0).modify(|v| {
                v.set_mask_t_res(EpTxResponse::ACK);
                v.set_t_tog(!v.t_tog());
            });
        }

        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());
            let poll_result = {
                let interrupt_flags = regs.int_fg().read();
                if interrupt_flags.transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() == 0 {
                        let res = match status.mask_token() {
                            UsbToken::IN => {
                                regs.uep_ctrl(0).modify(|v| {
                                    v.set_mask_t_res(EpTxResponse::NAK);
                                });
                                Poll::Ready(Ok(()))
                            }
                            token => {
                                error!("unexpected USB Token {}, aborting", token.to_bits());
                                Poll::Ready(Err(EndpointError::Disabled))
                            }
                        };

                        regs.int_fg().write(|v| v.set_transfer(true));
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
            regs.uep_ctrl(0).modify(|v| {
                v.set_mask_r_res(EpRxResponse::ACK);
                v.set_mask_r_tog(true);
            });

            poll_fn(|ctx| {
                super::EP_WAKERS[0].register(ctx.waker());

                let poll_res = if regs.int_fg().read().transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() == 0 {
                        let res = match status.mask_token() {
                            UsbToken::OUT => {
                                if regs.rx_len().read().0 != 0 {
                                    error!("Expected 0 len OUT stage, found non-zero len, aborting");
                                    Poll::Ready(Err(EndpointError::BufferOverflow))
                                } else {
                                    regs.uep_ctrl(0).modify(|v| {
                                        v.set_mask_r_res(EpRxResponse::NAK);
                                    });
                                    Poll::Ready(Ok(()))
                                }
                            }
                            _ => {
                                error!("Got {} instead of OUT token", status.mask_token().to_bits());
                                Poll::Ready(Err(EndpointError::Disabled))
                            }
                        };
                        regs.int_fg().write(|v| v.set_transfer(true));

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

        set_ep_tx_len(regs, 0, 0);
        regs.uep_ctrl(0).modify(|v| {
            v.set_t_tog(true);
            v.set_mask_t_res(EpTxResponse::ACK);
        });

        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let res = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() == 0 {
                    match status.mask_token() {
                        UsbToken::IN => {
                            regs.uep_ctrl(0).modify(|v| {
                                v.set_mask_t_res(EpTxResponse::NAK);
                            });
                            regs.int_fg().write(|v| v.set_transfer(true));
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
        regs.uep_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::STALL);
            v.set_mask_t_res(EpTxResponse::STALL);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        T::regs().dev_ad().write(|v| {
            v.set_mask_usb_addr(addr);
        });
    }
}
