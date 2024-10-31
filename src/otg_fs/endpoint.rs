use core::marker::PhantomData;
use core::task::Poll;

use ch32_metapac::otg::vals::{EpRxResponse, EpTxResponse, UsbToken};
use embassy_usb_driver::{Direction, EndpointAllocError, EndpointError, EndpointInfo};
use futures::future::poll_fn;

use crate::usb::{Dir, EndpointData, In, Out};
use super::{Instance, EP_MAX_PACKET_SIZE, EP_WAKERS};
use crate::interrupt::typelevel::Interrupt;

/// USB endpoint.
pub struct Endpoint<'d, T, D> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    data: EndpointData<'d>,
}

impl<'d, T: Instance, D: Dir> Endpoint<'d, T, D> {
    pub(crate) fn new(info: EndpointInfo, data: EndpointData<'d>) -> Self {
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
        trace!("endpoint {} enabled", self.info.addr.index());
    }

    fn is_enabled(&self) -> bool {
        let regs = T::regs();
        match (self.info.addr.index(), self.info.addr.direction()) {
            (4, Direction::In) => regs.uep4_1_mod().read().uep4_tx_en(),
            (4, Direction::Out) => regs.uep4_1_mod().read().uep4_rx_en(),
            (1, Direction::In) => regs.uep4_1_mod().read().uep1_tx_en(),
            (1, Direction::Out) => regs.uep4_1_mod().read().uep1_rx_en(),

            (2, Direction::In) => regs.uep2_3_mod().read().uep2_tx_en(),
            (2, Direction::Out) => regs.uep2_3_mod().read().uep2_rx_en(),
            (3, Direction::In) => regs.uep2_3_mod().read().uep3_tx_en(),
            (3, Direction::Out) => regs.uep2_3_mod().read().uep3_rx_en(),

            (5, Direction::In) => regs.uep5_6_mod().read().uep5_tx_en(),
            (5, Direction::Out) => regs.uep5_6_mod().read().uep5_rx_en(),
            (6, Direction::In) => regs.uep5_6_mod().read().uep6_tx_en(),
            (6, Direction::Out) => regs.uep5_6_mod().read().uep6_rx_en(),

            (7, Direction::In) => regs.uep7_mod().read().uep7_tx_en(),
            (7, Direction::Out) => regs.uep7_mod().read().uep7_rx_en(),

            _ => panic!("Unsupported EP"),
        }
    }
}

impl<'d, T: Instance, D: Dir> embassy_usb_driver::Endpoint for Endpoint<'d, T, D> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        self.wait_enabled_internal().await
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In> {
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
        regs.uep_t_len(ep).write_value(buf.len() as u8);
        regs.uep_tx_ctrl(ep).modify(|v| {
            v.set_t_tog(!v.t_tog());
            v.set_mask_t_res(EpTxResponse::ACK);
        });

        // Wait for TX complete
        let tx_result = poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let token = status.mask_token();
                    let ret = match token {
                        UsbToken::IN => {
                            regs.uep_tx_ctrl(ep).modify(|v| {
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
        .await;

        tx_result
    }
}

impl<'d, T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        trace!("endpoint {} OUT", self.info.addr);
        if !self.is_enabled() {
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        // Tx Ctrl should be NAK

        regs.uep_rx_ctrl(ep).modify(|v| {
            v.set_r_tog(!v.r_tog());
            v.set_mask_r_res(EpRxResponse::ACK);
        });

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[ep].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() as usize == ep {
                    let ret = match status.mask_token() {
                        UsbToken::OUT => {
                            // upper bits are reserved (0)
                            let len = regs.rx_len().read().0 as usize;

                            // Assertion exists because looks like embassy-usb expects no partial reads.
                            // https://github.com/embassy-rs/embassy/blob/6e0b08291b63a0da8eba9284869d1d046bc5dabb/embassy-usb/src/lib.rs#L408
                            debug_assert_eq!(len, buf.len());
                            if len == buf.len() {
                                self.data.buffer.read_volatile(&mut buf[..len]);
                                Poll::Ready(Ok(len))
                            } else {
                                Poll::Ready(Err(EndpointError::BufferOverflow))
                            }
                        }
                        token => {
                            error!("Unexpected USB Token {}", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };

                    regs.uep_rx_ctrl(ep).modify(|v| {
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

pub struct ControlPipe<'d, T> {
    ep0_buf: EndpointData<'d>,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    pub(crate) fn new(ep0_buf: EndpointData<'d>) -> Self {
        Self {
            ep0_buf,
            _phantom: PhantomData,
        }
    }
}

impl<'d, T> embassy_usb_driver::ControlPipe for ControlPipe<'d, T>
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

                    // assume the setup packet is always for endpoint 0.
                    // the hardware doesn't seem to set mask_uis_endp to 0
                    // when the setup packet arrives.
                    match int_status.mask_token() {
                        UsbToken::SETUP => {
                            // SETUP packet token
                            regs.uep_tx_ctrl(0).write(|w| w.set_mask_t_res(EpTxResponse::NAK));
                            regs.uep_rx_ctrl(0).write(|w| w.set_mask_r_res(EpRxResponse::NAK));

                            let mut data = [0u8; 8];
                            self.ep0_buf.buffer.read_volatile(&mut data[..8]);

                            regs.int_fg().write(|v| v.set_transfer(true));
                            // Clear Flag
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

    async fn data_out(&mut self, buf: &mut [u8], first: bool, last: bool) -> Result<usize, EndpointError> {
        assert!(first && last, "TODO support chunking");

        let regs = T::regs();

        if buf.len() > self.ep0_buf.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        regs.uep_rx_ctrl(0).modify(|v| {
            v.set_r_tog(!v.r_tog());
            v.set_mask_r_res(EpRxResponse::ACK);
        });

        // poll for packet
        let bytes_read = poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());

            let ret = if regs.int_fg().read().transfer() {
                let status = regs.int_st().read();
                if status.mask_uis_endp() == 0 {
                    let ret = match status.mask_token() {
                        UsbToken::RSVD | UsbToken::IN => unreachable!(),
                        UsbToken::SETUP => Poll::Ready(Err(EndpointError::Disabled)),
                        UsbToken::OUT => {
                            let len = regs.rx_len().read().0 as usize;
                            // https://github.com/embassy-rs/embassy/blob/6e0b08291b63a0da8eba9284869d1d046bc5dabb/embassy-usb/src/lib.rs#L408
                            // Embassy expects the whole buffer to be filled
                            if len == buf.len() {
                                self.ep0_buf.buffer.read_volatile(&mut buf[..len]);
                                Poll::Ready(Ok(len))
                            } else {
                                Poll::Ready(Err(EndpointError::BufferOverflow))
                            }
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
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await?;

        regs.uep_rx_ctrl(0).modify(|v| {
            v.set_mask_r_res(EpRxResponse::NAK);
        });

        Ok(bytes_read)
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let regs = T::regs();

        if data.len() > self.ep0_buf.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        self.ep0_buf.buffer.write_volatile(data);
        // TODO: manual is wrong here, t_len(3) should be a u16
        regs.uep_t_len(0).write_value(data.len() as u8);

        if first {
            regs.uep_tx_ctrl(0).write(|v| {
                v.set_mask_t_res(EpTxResponse::ACK);
                v.set_t_tog(true);
            });
        } else {
            regs.uep_tx_ctrl(0).modify(|v| {
                v.set_mask_t_res(EpTxResponse::ACK);
                v.set_t_tog(!v.t_tog());
            });
        }

        // Poll for last packet to finsh transfer
        poll_fn(|ctx| {
            super::EP_WAKERS[0].register(ctx.waker());
            let poll_result = {
                let interrupt_flags = regs.int_fg().read();
                if interrupt_flags.transfer() {
                    let status = regs.int_st().read();
                    if status.mask_uis_endp() == 0 {
                        let res = match status.mask_token() {
                            UsbToken::IN => {
                                regs.uep_tx_ctrl(0).modify(|v| {
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
            regs.uep_rx_ctrl(0).modify(|v| {
                // Set RX to true to expect the STATUS (OUT) packet
                v.set_mask_r_res(EpRxResponse::ACK);
            });
            // Expect the empty OUT token for status
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
                                    // Set the EP back to NAK so that we are "not ready to recieve"
                                    regs.uep_rx_ctrl(0).write(|v| {
                                        v.set_r_tog(true);
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

        // Rx should already be NAK
        regs.uep_t_len(0).write_value(0);
        regs.uep_tx_ctrl(0).write(|v| {
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
                            // Maybe stall? and unstall when we actually set addr
                            regs.uep_tx_ctrl(0).write(|v| {
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
        regs.uep_rx_ctrl(0).write(|v| {
            v.set_mask_r_res(EpRxResponse::STALL);
        });
        regs.uep_tx_ctrl(0).write(|v| {
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
