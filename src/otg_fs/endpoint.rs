use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointDisabledError, EndpointError, EndpointInfo};

use super::regs::{self, EpRxResponse, EpTxResponse, IntSt, Regs, UsbToken};
use super::{is_endpoint_stalled, set_endpoint_stalled, Instance, EP_WAKERS};
use crate::interrupt::typelevel::Interrupt;
use crate::usb::{packets, Dir, EndpointData, In, Out};

/// USB endpoint.
pub struct Endpoint<'d, T, D, const SIZE: usize> {
    _phantom: PhantomData<&'d (T, D)>,
    info: EndpointInfo,
    data: EndpointData<'d, SIZE>,
}

/// The completed transaction of endpoint `ep` waiting in `INT_ST`, if any.
/// A SETUP packet belongs to the control pipe, whatever endpoint `INT_ST`
/// names.
fn pending_transaction(regs: Regs, ep: usize) -> Option<IntSt> {
    if !regs::int_fg(regs).transfer {
        return None;
    }
    let status = regs::int_st(regs);
    (!matches!(status.token, UsbToken::SETUP) && usize::from(status.ep) == ep).then_some(status)
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

    fn enabled(&self) -> bool {
        regs::ep_is_enabled(T::regs(), self.info.addr.index(), self.info.addr.direction())
    }
}

impl<'d, T: Instance, D: Dir, const SIZE: usize> embassy_usb_driver::Endpoint for Endpoint<'d, T, D, SIZE> {
    fn info(&self) -> EndpointInfo {
        self.info
    }

    async fn wait_enabled(&mut self) {
        poll_fn(|ctx| {
            EP_WAKERS[self.info.addr.index() as usize].register(ctx.waker());
            if self.enabled() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
        trace!("endpoint {} enabled", self.info.addr.index());
    }

    fn is_enabled(&self) -> bool {
        self.enabled()
    }

    async fn set_stalled(&mut self, stalled: bool) {
        set_endpoint_stalled::<T>(self.info.addr, stalled);
    }

    fn is_stalled(&self) -> bool {
        is_endpoint_stalled(self.info.addr)
    }
}

impl<'d, T: Instance, const SIZE: usize> Endpoint<'d, T, In, SIZE> {
    async fn write_packet(&mut self, buf: &[u8]) -> Result<(), EndpointDisabledError> {
        trace!("endpoint {} IN", self.info.addr.index());
        if !self.enabled() {
            error!("write to disabled ep {}", self.info.addr.index());
            return Err(EndpointDisabledError);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        self.data.buffer.write_volatile(buf);
        regs::set_ep_t_len(regs, ep, buf.len() as u16);

        poll_fn(|ctx| {
            EP_WAKERS[ep].register(ctx.waker());

            let ret = if !self.enabled() {
                Poll::Ready(Err(EndpointDisabledError))
            } else if let Some(status) = pending_transaction(regs, ep) {
                let ret = match status.token {
                    UsbToken::IN => {
                        regs::ep_tx_ctrl_flip_tog(regs, ep, EpTxResponse::NAK);
                        Poll::Ready(Ok(()))
                    }
                    token => {
                        error!("Unexpected USB Token {}", token.to_bits());
                        Poll::Ready(Err(EndpointDisabledError))
                    }
                };
                regs::clear_transfer(regs);
                ret
            } else {
                // Armed on every poll: clearing a stall NAKs the endpoint.
                if !is_endpoint_stalled(self.info.addr) {
                    regs::ep_tx_ctrl_set_res(regs, ep, EpTxResponse::ACK);
                }
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() }
            ret
        })
        .await
    }
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointIn for Endpoint<'d, T, In, SIZE> {
    async fn write_transfer(&mut self, buf: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        let max_packet_size = usize::from(self.info.max_packet_size);
        crate::usb::write_transfer(buf, needs_zlp, max_packet_size, async |packet| {
            self.write_packet(packet).await
        })
        .await
    }
}

impl<'d, T: Instance, const SIZE: usize> Endpoint<'d, T, Out, SIZE> {
    async fn read_packet(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        trace!("endpoint {} OUT", self.info.addr.index());
        if !self.enabled() {
            error!("OUT disabled EP");
            return Err(EndpointError::Disabled);
        }

        let ep = self.info.addr.index();
        let regs = T::regs();

        poll_fn(|ctx| {
            EP_WAKERS[ep].register(ctx.waker());

            let ret = if !self.enabled() {
                Poll::Ready(Err(EndpointError::Disabled))
            } else if let Some(status) = pending_transaction(regs, ep) {
                let ret = match status.token {
                    UsbToken::OUT if status.tog_ok => {
                        // upper bits are reserved (0)
                        let len = regs::rx_len(regs) as usize;
                        let ret = if len > buf.len() {
                            Err(EndpointError::BufferOverflow)
                        } else {
                            self.data.buffer.read_volatile(&mut buf[..len]);
                            Ok(len)
                        };
                        regs::ep_rx_ctrl_flip_tog(regs, ep, EpRxResponse::NAK);
                        Poll::Ready(ret)
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
                // Armed on every poll: clearing a stall NAKs the endpoint.
                if !is_endpoint_stalled(self.info.addr) {
                    regs::ep_rx_ctrl_set_res(regs, ep, EpRxResponse::ACK);
                }
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await
    }
}

impl<'d, T: Instance, const SIZE: usize> embassy_usb_driver::EndpointOut for Endpoint<'d, T, Out, SIZE> {
    async fn read_transfer(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let max_packet_size = usize::from(self.info.max_packet_size);
        crate::usb::read_transfer(buf, max_packet_size, async |packet| self.read_packet(packet).await).await
    }
}

/// EP0, driven by `Bus`. Bus events and SETUP packets stay pending in the
/// interrupt flags until `Bus::poll` takes them, so a control transfer they
/// abort only has to stop waiting.
pub(super) struct ControlPipe<'d, T, const SIZE: usize> {
    ep0: EndpointData<'d, SIZE>,
    _phantom: PhantomData<T>,
}

impl<'d, T: Instance, const SIZE: usize> ControlPipe<'d, T, SIZE> {
    pub(super) fn new(ep0: EndpointData<'d, SIZE>) -> Self {
        Self {
            ep0,
            _phantom: PhantomData,
        }
    }

    pub(super) fn max_packet_size(&self) -> usize {
        usize::from(self.ep0.max_packet_size)
    }

    /// Takes the pending SETUP packet, which starts a control transfer.
    pub(super) fn take_setup(&mut self, regs: Regs) -> Option<[u8; 8]> {
        // The hardware doesn't seem to set the endpoint of a SETUP token to 0.
        if !regs::int_fg(regs).transfer || !matches!(regs::int_st(regs).token, UsbToken::SETUP) {
            return None;
        }
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::NAK);

        let mut setup = [0u8; 8];
        self.ep0.buffer.read_volatile(&mut setup);
        regs::clear_transfer(regs);
        Some(setup)
    }

    /// Waits for the next EP0 transaction, which `handle` completes. Fails,
    /// leaving the event for `Bus::poll`, once a bus event or a new SETUP
    /// packet aborts the control transfer.
    async fn transaction<R>(
        &mut self,
        mut handle: impl FnMut(&mut EndpointData<'d, SIZE>, Regs, UsbToken) -> Poll<R>,
    ) -> Result<R, EndpointDisabledError> {
        poll_fn(|ctx| {
            EP_WAKERS[0].register(ctx.waker());
            let regs = T::regs();
            let flags = regs::int_fg(regs);
            let ret = if flags.bus_rst || flags.suspend {
                Poll::Ready(Err(EndpointDisabledError))
            } else if flags.transfer {
                let status = regs::int_st(regs);
                match status.token {
                    UsbToken::SETUP => Poll::Ready(Err(EndpointDisabledError)),
                    token if status.ep == 0 => {
                        let ret = handle(&mut self.ep0, regs, token);
                        regs::clear_transfer(regs);
                        ret.map(Ok)
                    }
                    _ => Poll::Pending,
                }
            } else {
                Poll::Pending
            };
            unsafe { T::Interrupt::enable() };
            ret
        })
        .await
    }

    /// Receives the OUT data stage, one packet at a time.
    pub(super) async fn data_out(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let regs = T::regs();
        let max_packet_size = self.max_packet_size();
        let mut length = 0;
        loop {
            // The data stage starts with DATA1.
            if length == 0 {
                regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
            } else {
                regs::ep_rx_ctrl_flip_tog(regs, 0, EpRxResponse::ACK);
            }
            let remaining = &mut buf[length..];
            let received = self
                .transaction(|ep0, regs, token| match token {
                    UsbToken::OUT => {
                        let len = usize::from(regs::rx_len(regs));
                        regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                        Poll::Ready(if len > remaining.len() {
                            Err(EndpointError::BufferOverflow)
                        } else {
                            ep0.buffer.read_volatile(&mut remaining[..len]);
                            Ok(len)
                        })
                    }
                    _ => Poll::Pending,
                })
                .await??;
            length += received;
            if received < max_packet_size || length == buf.len() {
                return Ok(length);
            }
        }
    }

    /// Sends the IN data stage, one packet at a time, then waits for the
    /// status stage.
    pub(super) async fn data_in(&mut self, data: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        let regs = T::regs();
        for (packet, first, _) in packets(data, needs_zlp, self.max_packet_size()) {
            self.ep0.buffer.write_volatile(packet);
            regs::set_ep_t_len(regs, 0, packet.len() as u16);
            // The data stage starts with DATA1.
            if first {
                regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
            } else {
                regs::ep_tx_ctrl_flip_tog(regs, 0, EpTxResponse::ACK);
            }
            self.transaction(|_, regs, token| match token {
                UsbToken::IN => {
                    regs::ep_tx_ctrl_set_res(regs, 0, EpTxResponse::NAK);
                    Poll::Ready(())
                }
                _ => Poll::Pending,
            })
            .await?;
        }

        // The status stage is a zero-length DATA1 OUT packet.
        regs::ep_rx_ctrl_write(regs, 0, true, EpRxResponse::ACK);
        self.transaction(|_, regs, token| match token {
            UsbToken::OUT => {
                // Set the EP back to NAK so that we are "not ready to receive"
                regs::ep_rx_ctrl_set_res(regs, 0, EpRxResponse::NAK);
                Poll::Ready(())
            }
            _ => Poll::Pending,
        })
        .await
    }

    /// Sends the zero-length DATA1 IN packet of the status stage.
    async fn status_in(&mut self) -> Result<(), EndpointDisabledError> {
        let regs = T::regs();
        // Rx should already be NAK
        regs::set_ep_t_len(regs, 0, 0);
        regs::ep_tx_ctrl_write(regs, 0, true, EpTxResponse::ACK);
        self.transaction(|_, regs, token| match token {
            UsbToken::IN => {
                regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::NAK);
                Poll::Ready(())
            }
            _ => Poll::Pending,
        })
        .await
    }

    pub(super) async fn accept(&mut self) {
        let _ = self.status_in().await;
    }

    pub(super) fn reject(&mut self) {
        let regs = T::regs();
        regs::ep_rx_ctrl_write(regs, 0, false, EpRxResponse::STALL);
        regs::ep_tx_ctrl_write(regs, 0, false, EpTxResponse::STALL);
    }

    pub(super) async fn accept_set_address(&mut self, addr: u8) {
        if self.status_in().await.is_ok() {
            regs::set_device_address(T::regs(), addr);
        }
    }
}
