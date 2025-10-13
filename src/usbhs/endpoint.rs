use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use ch32_metapac::usbhs::vals::{EpRxResponse, EpTog, EpTxResponse, UsbToken};
use embassy_usb_driver::{Direction, EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType};

use super::{EndpointData, Instance, EP_WAKERS};
use crate::usb::{Dir, In, Out};

pub struct Endpoint<'d, T: Instance, D: Dir> {
    _phantom: PhantomData<(&'d mut T, D)>,
    info: EndpointInfo,
    pub(crate) data: EndpointData<'d>,
}

impl<'d, T: Instance, D: Dir> Endpoint<'d, T, D> {
    pub(crate) fn new(info: EndpointInfo, data: EndpointData<'d>) -> Self {
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
    }

    fn is_enabled(&self) -> bool {
        match self.info.addr.direction() {
            Direction::Out => T::dregs().ep_config().read().r_en(self.info.addr.index() - 1),
            Direction::In => T::dregs().ep_config().read().t_en(self.info.addr.index() - 1),
        }
    }

    pub(crate) async fn data_out(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let r = T::regs();
        let d = T::dregs();
        let index = self.info.addr.index();

        poll_fn(|ctx| {
            super::EP_WAKERS[index].register(ctx.waker());

            let transfer = d.int_fg().read().transfer();
            let status = r.int_st().read();
            if transfer && status.endp() == index as u8 {
                let res = match status.token() {
                    UsbToken::OUT => {
                        let len = r.rx_len().read() as usize;
                        if len <= buf.len() {
                            self.data.buffer.read_volatile(&mut buf[..len]);
                            Poll::Ready(Ok(len))
                        } else {
                            Poll::Ready(Err(EndpointError::BufferOverflow))
                        }
                    }
                    token => {
                        error!("unexpected USB Token {}, aborting", token.to_bits());
                        Poll::Ready(Err(EndpointError::Disabled))
                    }
                };

                d.ep_rx_ctrl(index).modify(|v| {
                    v.set_mask_uep_r_res(EpRxResponse::NAK);
                    v.set_mask_uep_r_tog(if let EpTog::DATA0 = v.mask_uep_r_tog() {
                        EpTog::DATA1
                    } else {
                        EpTog::DATA0
                    });
                });

                d.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| d.int_en().modify(|w| w.set_transfer(true)));
                res
            } else {
                Poll::Pending
            }
        })
        .await
    }

    pub(crate) fn data_in_write_buffer(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let d = T::dregs();
        let index = self.info.addr.index();

        if data.len() > self.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        self.data.buffer.write_volatile(data);

        d.ep_t_len(index).write(|v| v.set_len(data.len() as u16));
        Ok(())
    }

    pub(crate) async fn data_in_transfer(&mut self) -> Result<(), EndpointError> {
        let d = T::dregs();
        let r = T::regs();
        let index = self.info.addr.index();

        poll_fn(|ctx| {
            EP_WAKERS[index].register(ctx.waker());

            let transfer = d.int_fg().read().transfer();
            let status = r.int_st().read();
            if transfer && status.endp() == index as u8 {
                let res = match status.token() {
                    UsbToken::IN => Poll::Ready(Ok(())),
                    token => {
                        error!("unexpected USB Token {}, aborting", token.to_bits());
                        Poll::Ready(Err(EndpointError::Disabled))
                    }
                };
                d.ep_tx_ctrl(index).modify(|v| {
                    v.set_mask_uep_t_res(EpTxResponse::NAK);
                    v.set_mask_uep_t_tog(if let EpTog::DATA0 = v.mask_uep_t_tog() {
                        EpTog::DATA1
                    } else {
                        EpTog::DATA0
                    });
                });

                d.int_fg().write(|w| w.set_transfer(true));
                critical_section::with(|_| d.int_en().modify(|w| w.set_transfer(true)));
                res
            } else {
                Poll::Pending
            }
        })
        .await
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

impl<'d, T: Instance> EndpointOut for Endpoint<'d, T, Out> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        if !self.is_enabled() {
            error!("read from disabled ep {}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }

        let d = T::dregs();
        let index = self.info.addr.index();

        if buf.len() > self.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        d.ep_rx_ctrl(index).modify(|v| {
            v.set_mask_uep_r_res(EpRxResponse::ACK);
        });

        self.data_out(buf).await
    }
}

impl<'d, T: Instance> EndpointIn for Endpoint<'d, T, In> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        if !self.is_enabled() {
            error!("write to disabled ep {}", self.info.addr.index());
            return Err(EndpointError::Disabled);
        }

        let d = T::dregs();
        let index = self.info.addr.index();

        self.data_in_write_buffer(buf)?;

        d.ep_tx_ctrl(index).modify(|v| {
            v.set_mask_uep_t_res(EpTxResponse::ACK);
        });

        self.data_in_transfer().await
    }
}
