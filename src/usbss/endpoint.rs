use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointDisabledError, EndpointError, EndpointInfo};

use super::device::{slot, state, Bus, Phase, Transfer};
use super::{dma_capable, Instance};
use crate::internal::drop::OnDrop;

/// How long a cancelled IN transfer keeps its endpoint unused. A chain armed
/// while the host still reads the burst that was cut short starts with a
/// packet of wrong data; a 16-packet burst takes 35 us.
const IN_ABORT_SETTLE_US: u32 = 100;

/// A data endpoint.
///
/// Transfers DMA straight to or from the caller's buffer, which has to lie in
/// the controller's DMA range `0x200A_0000..0x2018_0000`: an OUT transfer fails
/// with `BufferOverflow` otherwise, and an IN transfer panics. Dropping a
/// transfer future aborts the transfer and loses the data in flight; for an IN
/// transfer the drop blocks for 100 us.
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
    async fn run(&mut self, address: usize, length: usize, zlp: bool) -> Result<usize, EndpointDisabledError> {
        let slot = slot(self.info.addr.index(), IN);
        state::<T>().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let powered = inner.powered;
            let endpoint = &mut inner.endpoints[slot];
            if !powered || !endpoint.enabled || endpoint.stalled {
                return Err(EndpointDisabledError);
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
                    Phase::Idle => Poll::Ready(Err(EndpointDisabledError)),
                }
            })
        })
        .await;
        abort.defuse();
        result
    }
}

impl<T: Instance, const IN: bool> embassy_usb_driver::Endpoint for Endpoint<'_, T, IN> {
    fn info(&self) -> EndpointInfo {
        self.info
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

    fn is_enabled(&self) -> bool {
        Bus::<T>::is_endpoint_enabled(self.info.addr)
    }

    async fn set_stalled(&mut self, stalled: bool) {
        Bus::<T>::set_endpoint_stalled(self.info.addr, stalled);
    }

    fn is_stalled(&self) -> bool {
        Bus::<T>::is_endpoint_stalled(self.info.addr)
    }
}

impl<T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'_, T, false> {
    async fn read_transfer(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        // The controller writes whole packets.
        if buffer.len() % self.max_packet_size() != 0 || !dma_capable(buffer) {
            return Err(EndpointError::BufferOverflow);
        }
        Ok(self.run(buffer.as_ptr() as usize, buffer.len(), false).await?)
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write_transfer(&mut self, buffer: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        // An empty transfer is a single ZLP.
        let zlp = buffer.is_empty() || (needs_zlp && buffer.len() % self.max_packet_size() == 0);
        assert!(
            buffer.is_empty() || dma_capable(buffer),
            "USBSS IN transfers need a buffer in the DMA range"
        );
        self.run(buffer.as_ptr() as usize, buffer.len(), zlp).await?;
        Ok(())
    }
}
