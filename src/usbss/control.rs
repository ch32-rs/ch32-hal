use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::EndpointError;

use super::device::{slot, state, Inner, Phase, EP0_SIZE};
use super::Instance;

/// EP0 in SETUP flow mode: the controller answers the host with NRDY until
/// the next stage is armed, then sends ERDY.
pub struct ControlPipe<'d, T: Instance> {
    /// `Inner::control_generation` of the SETUP being served.
    generation: u32,
    _lifetime: PhantomData<&'d mut T>,
}

impl<T: Instance> ControlPipe<'_, T> {
    pub(super) fn new() -> Self {
        Self {
            generation: 0,
            _lifetime: PhantomData,
        }
    }

    /// Runs one stage of the control transfer: `start` hands it to the
    /// controller, then the interrupt handler completes the EP0 slot. An OUT
    /// stage copies the received data to `out` and returns its length.
    ///
    /// Fails once a newer SETUP or a bus reset ends the control transfer.
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
                if !inner.powered || inner.control_generation != self.generation {
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
}

impl<T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'_, T> {
    fn max_packet_size(&self) -> usize {
        EP0_SIZE
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|context| {
            state::<T>().endpoint_wakers[0].register(context.waker());
            state::<T>().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                match inner.setup.take() {
                    Some(setup) => {
                        self.generation = inner.control_generation;
                        Poll::Ready(setup)
                    }
                    None => Poll::Pending,
                }
            })
        })
        .await
    }

    async fn data_out(&mut self, buffer: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        self.stage(false, buffer, |inner| inner.regs.ep0_rx_ready()).await
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > EP0_SIZE {
            return Err(EndpointError::BufferOverflow);
        }
        // The last packet's stage ends with the status stage.
        self.stage(true, &mut [], |inner| {
            unsafe { inner.ep0.write(data) };
            let seq = if first { 0 } else { (inner.tx_seq + 1) & 0x1f };
            inner.in_last = last;
            inner.regs.ep0_tx_ready(seq, data.len() as u16);
        })
        .await
        .map(drop)
    }

    async fn accept(&mut self) {
        self.status_in(None).await;
    }

    async fn reject(&mut self) {
        state::<T>().inner.lock(|inner| {
            let inner = inner.borrow();
            if inner.powered && inner.control_generation == self.generation {
                inner.regs.ep0_stall();
            }
        });
    }

    async fn accept_set_address(&mut self, address: u8) {
        if address <= 127 {
            self.status_in(Some(address)).await;
        }
    }
}
