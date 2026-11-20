use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointDisabledError, EndpointError};

use super::device::{slot, state, Inner, Phase};
use super::{Instance, EP0_SIZE};
use crate::usb::packets;

/// EP0 in SETUP flow mode: the controller answers the host with NRDY until
/// the next stage is armed, then sends ERDY.
pub(super) struct ControlPipe<T: Instance> {
    /// `Inner::control_generation` of the SETUP being served.
    generation: u32,
    _instance: PhantomData<T>,
}

impl<T: Instance> ControlPipe<T> {
    pub(super) fn new() -> Self {
        Self {
            generation: 0,
            _instance: PhantomData,
        }
    }

    /// Takes the pending SETUP packet, which starts a control transfer.
    pub(super) fn take_setup(&mut self, inner: &mut Inner) -> Option<[u8; 8]> {
        let setup = inner.setup.take()?;
        self.generation = inner.control_generation;
        Some(setup)
    }

    /// Runs one stage: `start` arms it and the interrupt handler completes the
    /// EP0 slot. An OUT stage copies its data to `out` and returns the length.
    /// Fails once the transfer is aborted.
    async fn stage(
        &mut self,
        incoming: bool,
        out: &mut [u8],
        mut start: impl FnMut(&mut Inner),
    ) -> Result<usize, EndpointError> {
        let slot = slot(0, incoming);
        let mut started = false;
        poll_fn(|context| {
            state::<T>().endpoint_wakers[0].register(context.waker());
            state::<T>().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                if inner.control_aborted(self.generation) {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if !started {
                    started = true;
                    inner.endpoints[slot].phase = Phase::Busy;
                    start(&mut inner);
                    return Poll::Pending;
                }
                if inner.endpoints[slot].phase != Phase::Done {
                    return Poll::Pending;
                }
                inner.endpoints[slot].phase = Phase::Idle;
                if incoming {
                    return Poll::Ready(Ok(0));
                }
                let length = inner.endpoints[slot].transfer.progress;
                if length > out.len() {
                    return Poll::Ready(Err(EndpointError::BufferOverflow));
                }
                unsafe { inner.ep0.read(&mut out[..length]) };
                Poll::Ready(Ok(length))
            })
        })
        .await
    }

    /// Completes the status stage with an IN acknowledgement.
    async fn status_in(&mut self, address: Option<u8>) {
        let _ = self
            .stage(true, &mut [], |inner| {
                inner.address = address;
                inner.in_last = true;
                inner.regs.ep0_tx_ready(0, 0);
                inner.regs.ep0_rx_ready();
            })
            .await;
    }

    /// Receives the OUT data stage, one packet per stage.
    pub(super) async fn data_out(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        let mut length = 0;
        loop {
            let received = self
                .stage(false, &mut buffer[length..], |inner| inner.regs.ep0_rx_ready())
                .await?;
            length += received;
            if received < EP0_SIZE || length == buffer.len() {
                return Ok(length);
            }
        }
    }

    /// Sends the IN data stage, one packet per stage. The last packet's stage
    /// ends with the status stage.
    pub(super) async fn data_in(&mut self, data: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        for (packet, first, last) in packets(data, needs_zlp, EP0_SIZE) {
            self.stage(true, &mut [], |inner| {
                unsafe { inner.ep0.write(packet) };
                let seq = if first { 0 } else { (inner.tx_seq + 1) & 0x1f };
                inner.in_last = last;
                inner.regs.ep0_tx_ready(seq, packet.len() as u16);
            })
            .await
            .map_err(|_| EndpointDisabledError)?;
        }
        Ok(())
    }

    pub(super) async fn accept(&mut self) {
        self.status_in(None).await;
    }

    pub(super) fn reject(&mut self) {
        state::<T>().inner.lock(|inner| {
            let inner = inner.borrow();
            if !inner.control_aborted(self.generation) {
                inner.regs.ep0_stall();
            }
        });
    }

    pub(super) async fn accept_set_address(&mut self, address: u8) {
        if address <= 127 {
            self.status_in(Some(address)).await;
        }
    }
}
