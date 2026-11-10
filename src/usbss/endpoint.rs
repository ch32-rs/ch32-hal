use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo, Unsupported};

use super::device::{dma_capable, slot, state, Buffer, Bus, Phase, Transfer};
use super::Instance;
use crate::internal::drop::OnDrop;

/// How long a cancelled IN transfer keeps its endpoint unused. A chain armed
/// while the host still reads the burst that was cut short starts with a
/// packet of wrong data; a 16-packet burst takes 35 us.
const IN_ABORT_SETTLE_US: u32 = 100;

/// A data endpoint.
///
/// Transfers DMA straight to or from the caller's buffer when it is 4-byte
/// aligned and inside the controller's DMA range; otherwise they are copied
/// through the endpoint memory one burst at a time. Dropping a transfer future
/// aborts the transfer and loses the data in flight; for an IN transfer the
/// drop blocks for 100 us.
pub struct Endpoint<'d, T: Instance, const IN: bool> {
    info: EndpointInfo,
    /// This endpoint's piece of the endpoint memory.
    buffer: Buffer,
    _lifetime: PhantomData<&'d mut T>,
}

impl<'d, T: Instance, const IN: bool> Endpoint<'d, T, IN> {
    pub(super) fn new(info: EndpointInfo, buffer: Buffer) -> Self {
        Self {
            info,
            buffer,
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
        // Aborts the transfer if the future is dropped before it completes.
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
        let mut total = 0;
        loop {
            let rest = &mut buffer[total..];
            // Only whole packets go straight to `buffer`; a shorter tail could
            // be overrun.
            let direct = dma_capable(rest) && rest.len() >= mps;
            let (address, room) = if direct {
                (rest.as_ptr() as usize, rest.len() / mps * mps)
            } else {
                (
                    self.buffer.address,
                    rest.len().next_multiple_of(mps).clamp(mps, self.buffer.size),
                )
            };
            let room = if single { mps } else { room };
            let received = self.run(address, room, false).await?;
            if !direct {
                if received > rest.len() {
                    return Err(EndpointError::BufferOverflow);
                }
                unsafe { self.buffer.read(&mut rest[..received]) };
            }
            total += received;
            if single || received < room || total == buffer.len() {
                return Ok(total);
            }
        }
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
        if dma_capable(buffer) {
            self.run(buffer.as_ptr() as usize, buffer.len(), zlp).await?;
            return Ok(());
        }
        for chunk in buffer.chunks(self.buffer.size) {
            unsafe { self.buffer.write(chunk) };
            self.run(self.buffer.address, chunk.len(), false).await?;
        }
        if zlp {
            self.run(self.buffer.address, 0, false).await?;
        }
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
