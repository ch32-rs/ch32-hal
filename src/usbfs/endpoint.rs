use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{Direction, EndpointError, EndpointInfo};
use futures::future::poll_fn;

use super::vals::{EpRxResponse, EpTxResponse, UsbToken};

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
        // Write DMA address - full 32-bit address
        T::regs()
            .uep_dma(info.addr.index() as usize)
            .write_value(data.buffer.addr() as u32);

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
        trace!("USBFS EP{} enabled", self.info.addr.index());
    }

    fn is_enabled(&self) -> bool {
        let regs = T::regs();
        let ep = self.info.addr.index();
        let dir = self.info.addr.direction();

        match (ep, dir) {
            (4, Direction::In) => regs.uep4_1_mod().read().tx_en(0),
            (4, Direction::Out) => regs.uep4_1_mod().read().rx_en(0),
            (1, Direction::In) => regs.uep4_1_mod().read().tx_en(1),
            (1, Direction::Out) => regs.uep4_1_mod().read().rx_en(1),

            (2, Direction::In) => regs.uep2_3_mod().read().tx_en(0),
            (2, Direction::Out) => regs.uep2_3_mod().read().rx_en(0),
            (3, Direction::In) => regs.uep2_3_mod().read().tx_en(1),
            (3, Direction::Out) => regs.uep2_3_mod().read().rx_en(1),

            (5, Direction::In) => regs.uep5_6_mod().read().tx_en(0),
            (5, Direction::Out) => regs.uep5_6_mod().read().rx_en(0),
            (6, Direction::In) => regs.uep5_6_mod().read().tx_en(1),
            (6, Direction::Out) => regs.uep5_6_mod().read().rx_en(1),

            (7, Direction::In) => regs.uep7_mod().read().tx_en(),
            (7, Direction::Out) => regs.uep7_mod().read().rx_en(),

            _ => panic!("Unsupported EP"),
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

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In, SIZE> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        trace!("USBFS EP{} IN write {} bytes", self.info.addr.index(), buf.len());

        if !self.is_enabled() {
            error!("USBFS write to disabled EP{}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        // Write buffer and set length
        self.data.buffer.write_volatile(buf);
        regs.uep_t_len(ep).write_value(buf.len() as u8);

        // Toggle and ACK
        regs.uep_tx_ctrl(ep).modify(|v| {
            v.set_t_tog(!v.t_tog());
            v.set_mask_t_res(EpTxResponse::ACK.to_bits());
        });

        // Wait for TX complete
        poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let token = UsbToken::from_bits(status.mask_token());
                    let ret = match token {
                        UsbToken::IN => {
                            regs.uep_tx_ctrl(ep).modify(|v| {
                                v.set_mask_t_res(EpTxResponse::NAK.to_bits());
                            });
                            Poll::Ready(Ok(()))
                        }
                        _ => {
                            error!("USBFS unexpected token {}", token.to_bits());
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
        trace!("USBFS EP{} OUT read", self.info.addr.index());

        if !self.is_enabled() {
            error!("USBFS read from disabled EP{}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        // Toggle and ACK
        regs.uep_rx_ctrl(ep).modify(|v| {
            v.set_r_tog(!v.r_tog());
            v.set_mask_r_res(EpRxResponse::ACK.to_bits());
        });

        // Wait for RX
        poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let token = UsbToken::from_bits(status.mask_token());
                    let ret = match token {
                        UsbToken::OUT => {
                            let len = regs.rx_len().read().0 as usize;
                            self.data.buffer.read_volatile(&mut buf[..len]);
                            Poll::Ready(Ok(len))
                        }
                        _ => {
                            error!("USBFS unexpected token {}", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };

                    regs.uep_rx_ctrl(ep).modify(|v| {
                        v.set_mask_r_res(EpRxResponse::NAK.to_bits());
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
        .await
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

                    let token = UsbToken::from_bits(int_status.mask_token());
                    match token {
                        UsbToken::SETUP => {
                            regs.uep_tx_ctrl(0).write(|w| w.set_mask_t_res(EpTxResponse::NAK.to_bits()));
                            regs.uep_rx_ctrl(0).write(|w| w.set_mask_r_res(EpRxResponse::NAK.to_bits()));

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
            regs.uep_rx_ctrl(0).write(|v| {
                v.set_r_tog(true);
                v.set_mask_r_res(EpRxResponse::ACK.to_bits());
            })
        } else {
            regs.uep_rx_ctrl(0).modify(|v| {
                v.set_r_tog(!v.r_tog());
                v.set_mask_r_res(EpRxResponse::ACK.to_bits());
            });
        }

        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() == 0 {
                    let token = UsbToken::from_bits(status.mask_token());
                    let res = match token {
                        UsbToken::OUT => {
                            let len = regs.rx_len().read().0 as usize;
                            self.ep0_buf.buffer.read_volatile(&mut buf[..len]);
                            regs.uep_rx_ctrl(0).modify(|v| {
                                v.set_mask_r_res(EpRxResponse::NAK.to_bits());
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
        .await
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let regs = T::regs();

        if data.len() > self.ep0_buf.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        self.ep0_buf.buffer.write_volatile(data);
        regs.uep_t_len(0).write_value(data.len() as u8);

        if first {
            regs.uep_tx_ctrl(0).write(|v| {
                v.set_mask_t_res(EpTxResponse::ACK.to_bits());
                v.set_t_tog(true);
            });
        } else {
            regs.uep_tx_ctrl(0).modify(|v| {
                v.set_mask_t_res(EpTxResponse::ACK.to_bits());
                v.set_t_tog(!v.t_tog());
            });
        }

        // Wait for IN transfer complete
        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());
            let poll_result = {
                let interrupt_flags = regs.int_fg().read();
                if interrupt_flags.transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() == 0 {
                        let token = UsbToken::from_bits(status.mask_token());
                        let res = match token {
                            UsbToken::IN => {
                                regs.uep_tx_ctrl(0).modify(|v| {
                                    v.set_mask_t_res(EpTxResponse::NAK.to_bits());
                                });
                                Poll::Ready(Ok(()))
                            }
                            _ => {
                                error!("USBFS unexpected token {}", token.to_bits());
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
            // Expect STATUS (OUT) packet
            regs.uep_rx_ctrl(0).write(|v| {
                v.set_mask_r_res(EpRxResponse::ACK.to_bits());
                v.set_r_tog(true);
            });

            poll_fn(|ctx| {
                super::EP_WAKERS[0].register(ctx.waker());

                let poll_res = if regs.int_fg().read().transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() == 0 {
                        let token = UsbToken::from_bits(status.mask_token());
                        let res = match token {
                            UsbToken::OUT => {
                                if regs.rx_len().read().0 != 0 {
                                    error!("USBFS expected 0-len STATUS, got non-zero");
                                    Poll::Ready(Err(EndpointError::BufferOverflow))
                                } else {
                                    regs.uep_rx_ctrl(0).modify(|v| {
                                        v.set_mask_r_res(EpRxResponse::NAK.to_bits());
                                    });
                                    Poll::Ready(Ok(()))
                                }
                            }
                            _ => {
                                error!("USBFS expected OUT token for STATUS");
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

        regs.uep_t_len(0).write_value(0);
        regs.uep_tx_ctrl(0).write(|v| {
            v.set_t_tog(true);
            v.set_mask_t_res(EpTxResponse::ACK.to_bits());
        });

        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let res = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() == 0 {
                    let token = UsbToken::from_bits(status.mask_token());
                    match token {
                        UsbToken::IN => {
                            regs.uep_tx_ctrl(0).write(|v| {
                                v.set_mask_t_res(EpTxResponse::NAK.to_bits());
                            });
                            regs.int_fg().write(|v| v.set_transfer(true));
                        }
                        _ => panic!("USBFS unexpected token {}", token.to_bits()),
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
        regs.uep_rx_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::STALL.to_bits());
        });
        regs.uep_tx_ctrl(0).write(|v| {
            v.set_mask_t_res(EpTxResponse::STALL.to_bits());
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        self.accept().await;
        T::regs().dev_ad().write(|v| {
            v.set_mask_usb_addr(addr);
        });
    }
}

