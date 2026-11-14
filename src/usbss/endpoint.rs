use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo, Unsupported};

use super::device::{dma_capable, slot, state, Bus, Phase, Transfer};
use super::Instance;
use crate::internal::drop::OnDrop;

/// How long a cancelled IN transfer keeps its endpoint unused. A chain armed
/// while the host still reads the burst that was cut short starts with a
/// packet of wrong data; a 16-packet burst takes 35 us.
const IN_ABORT_SETTLE_US: u32 = 100;

/// A data endpoint.
///
/// Transfers DMA straight to or from the caller's buffer. Dropping a transfer
/// future aborts the transfer and loses the data in flight; for an IN transfer
/// the drop blocks for 100 us.
pub struct Endpoint<'d, T: Instance, const IN: bool> {
    info: EndpointInfo,
    _lifetime: PhantomData<&'d mut T>,
}

impl<'d, T: Instance, const IN: bool> Endpoint<'d, T, IN> {
    pub(super) fn new(info: EndpointInfo) -> Self {
        Self {
            info,
            _lifetime: PhantomData,
        }
    }

    fn max_packet_size(&self) -> usize {
        usize::from(self.info.max_packet_size)
    }

    /// Runs one transfer and returns the bytes moved. `address` must stay
    /// valid until the future completes or is dropped.
    async fn run(&mut self, address: usize, length: usize, zlp: bool) -> Result<usize, EndpointError> {
        let slot = slot(self.info.addr.index(), IN);
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let powered = inner.powered;
            let endpoint = &mut inner.endpoints[slot];
            if !powered || !endpoint.enabled || endpoint.stalled {
                return Err(EndpointError::Disabled);
            }
            endpoint.transfer = Transfer {
                address,
                length,
                progress: 0,
                zlp,
            };
            inner.start(slot);
            Ok(())
        })?;
        let abort = OnDrop::new(move || {
            let aborted = state::<T>().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                let regs = inner.regs;
                let endpoint = &mut inner.endpoints[slot];
                let busy = endpoint.phase == Phase::Busy;
                if busy {
                    regs.abort(slot / 2, IN);
                }
                endpoint.phase = Phase::Idle;
                busy
            });
            if aborted && IN {
                crate::internal::delay::block_for_us(IN_ABORT_SETTLE_US);
            }
        });
        let result = poll_fn(|context| {
            state::<T>().endpoint_wakers[slot].register(context.waker());
            state::<T>().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                let endpoint = &mut inner.endpoints[slot];
                match endpoint.phase {
                    Phase::Busy => Poll::Pending,
                    Phase::Done => {
                        endpoint.phase = Phase::Idle;
                        Poll::Ready(Ok(endpoint.transfer.progress))
                    }
                    // A reset, disable or stall dropped the transfer.
                    Phase::Idle => Poll::Ready(Err(EndpointError::Disabled)),
                }
            })
        })
        .await;
        abort.defuse();
        result
    }
}

impl<T: Instance, const IN: bool> embassy_usb_driver::Endpoint for Endpoint<'_, T, IN> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let slot = slot(self.info.addr.index(), IN);
        poll_fn(|context| {
            state::<T>().endpoint_wakers[slot].register(context.waker());
            state::<T>().inner.lock(|inner| {
                let inner = inner.borrow();
                if inner.powered && inner.endpoints[slot].enabled {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }

    fn set_stall(&mut self, stalled: bool) -> Result<(), Unsupported> {
        Bus::<T>::set_endpoint_stalled(self.info.addr, stalled);
        Ok(())
    }

    fn is_stalled(&self) -> Result<bool, Unsupported> {
        Ok(Bus::<T>::is_endpoint_stalled(self.info.addr))
    }
}

impl<T: Instance> Endpoint<'_, T, false> {
    /// Receives until a short packet or a full `buffer`, or just one packet
    /// if `single`.
    async fn receive(&mut self, buffer: &mut [u8], single: bool) -> Result<usize, EndpointError> {
        let mps = self.max_packet_size();
        let length = if single { mps } else { buffer.len() };
        // The controller writes whole packets.
        if length > buffer.len() || length % mps != 0 || !dma_capable(&buffer[..length]) {
            return Err(EndpointError::BufferOverflow);
        }
        self.run(buffer.as_ptr() as usize, length, false).await
    }
}

impl<T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'_, T, false> {
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        self.receive(buffer, true).await
    }

    async fn read_transfer(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        self.receive(buffer, false).await
    }
}

impl<T: Instance> Endpoint<'_, T, true> {
    /// Sends `buffer`, then a ZLP if `zlp`.
    async fn send(&mut self, buffer: &[u8], zlp: bool) -> Result<(), EndpointError> {
        if buffer.is_empty() && !zlp {
            return Ok(());
        }
        if !buffer.is_empty() && !dma_capable(buffer) {
            return Err(EndpointError::BufferOverflow);
        }
        self.run(buffer.as_ptr() as usize, buffer.len(), zlp).await?;
        Ok(())
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write(&mut self, buffer: &[u8]) -> Result<(), EndpointError> {
        if buffer.len() > self.max_packet_size() {
            return Err(EndpointError::BufferOverflow);
        }
        self.send(buffer, buffer.is_empty()).await
    }

    async fn write_transfer(&mut self, buffer: &[u8], needs_zlp: bool) -> Result<(), EndpointError> {
        self.send(buffer, needs_zlp && buffer.len() % self.max_packet_size() == 0)
            .await
    }
}
