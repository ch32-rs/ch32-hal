use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use ch32_metapac::usbhs::vals::{EpRxResponse, EpTog, EpTxResponse, UsbToken};
use embassy_usb_driver::EndpointError;

use super::endpoint::Endpoint;
use super::{Instance, EP_WAKERS};
use crate::usb::InOut;

pub struct ControlPipe<'d, T: Instance> {
    _phantom: PhantomData<&'d mut T>,
    ep0: Endpoint<'d, T, InOut>,
}

impl<'d, T: Instance> ControlPipe<'d, T> {
    pub(crate) fn new(ep0: Endpoint<'d, T, InOut>) -> Self {
        Self {
            _phantom: PhantomData,
            ep0,
        }
    }
}

impl<'d, T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'d, T> {
    fn max_packet_size(&self) -> usize {
        use embassy_usb_driver::Endpoint;

        self.ep0.info().max_packet_size as usize
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|ctx| {
            EP_WAKERS[0].register(ctx.waker());
            let r = T::regs();
            let d = T::dregs();

            let flag = r.int_fg().read();

            if flag.setup_act() {
                d.ep_rx_ctrl(0).write(|w| w.set_mask_uep_r_res(EpRxResponse::NAK));
                d.ep_tx_ctrl(0).write(|w| w.set_mask_uep_t_res(EpTxResponse::NAK));

                let mut data: [u8; 8] = [0; 8];
                self.ep0.data.buffer.read_volatile(&mut data[..]);
                r.int_fg().write(|w| {
                    w.set_setup_act(true);
                    w.set_transfer(true);
                });
                critical_section::with(|_| {
                    r.int_en().modify(|w| {
                        w.set_setup_act(true);
                        w.set_transfer(true);
                    });
                });
                Poll::Ready(data)
            } else {
                Poll::Pending
            }
        })
        .await
    }

    async fn data_out(&mut self, buf: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        let d = T::dregs();

        if buf.len() > self.ep0.data.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }

        if first {
            d.ep_rx_ctrl(0).write(|v| {
                v.set_mask_uep_r_tog(EpTog::DATA1);
                v.set_mask_uep_r_res(EpRxResponse::ACK);
            })
        } else {
            d.ep_rx_ctrl(0).modify(|v| {
                v.set_mask_uep_r_res(EpRxResponse::ACK);
            })
        }

        self.ep0.data_out(buf).await
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        let d = T::dregs();
        let r = T::regs();

        self.ep0.data_in_write_buffer(data)?;

        if first {
            d.ep_tx_ctrl(0).write(|v| {
                v.set_mask_uep_t_res(EpTxResponse::ACK);
                v.set_mask_uep_t_tog(EpTog::DATA1);
            })
        } else {
            d.ep_tx_ctrl(0).modify(|v| {
                v.set_mask_uep_t_res(EpTxResponse::ACK);
            });
        }

        self.ep0.data_in_transfer().await?;

        if last {
            // status stage = recieve, DATA1, ack
            d.ep_rx_ctrl(0).write(|w| {
                w.set_mask_uep_r_tog(EpTog::DATA1);
                w.set_mask_uep_r_res(EpRxResponse::ACK);
            });
            poll_fn(|ctx| {
                EP_WAKERS[0].register(ctx.waker());

                let transfer = d.int_fg().read().transfer();
                let status = d.int_st().read();

                if transfer && status.endp() == 0 {
                    let res = match status.token() {
                        UsbToken::OUT => {
                            if r.rx_len().read() != 0 {
                                error!("Expected 0 len OUT stage, found non-zero len, aborting");
                                Poll::Ready(Err(EndpointError::BufferOverflow))
                            } else {
                                // Set the EP back to NAK so that we are "not ready to recieve"
                                Poll::Ready(Ok(()))
                            }
                        }
                        token => {
                            error!("Got {} instead of OUT token", token.to_bits());
                            Poll::Ready(Err(EndpointError::Disabled))
                        }
                    };

                    d.ep_rx_ctrl(0).write(|v| {
                        v.set_mask_uep_r_res(EpRxResponse::NAK);
                    });
                    d.int_fg().write(|w| w.set_transfer(true));
                    critical_section::with(|_| r.int_en().modify(|v| v.set_transfer(true)));
                    res
                } else {
                    Poll::Pending
                }
            })
            .await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        let d = T::dregs();

        d.ep_t_len(0).write(|w| w.set_len(0));
        d.ep_tx_ctrl(0).modify(|w| {
            // status stage starts with DATA1
            w.set_mask_uep_t_tog(EpTog::DATA1);
            w.set_mask_uep_t_res(EpTxResponse::ACK);
        });

        unwrap!(self.ep0.data_in_transfer().await);
    }

    async fn reject(&mut self) {
        let d = T::dregs();

        d.ep_tx_ctrl(0).modify(|v| {
            v.set_mask_uep_t_res(EpTxResponse::STALL);
        });
        d.ep_rx_ctrl(0).modify(|v| {
            v.set_mask_uep_r_res(EpRxResponse::STALL);
        });
    }

    async fn accept_set_address(&mut self, addr: u8) {
        let r = T::regs();
        trace!("accepting address: {}", addr);
        self.accept().await;
        r.dev_ad().write(|w| w.set_addr(addr));
    }
}
