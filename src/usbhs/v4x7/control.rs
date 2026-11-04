use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::EndpointError;

use super::endpoint::{read_packet, write_packet};
use super::{Instance, Registers, ACK, PRESERVE_FLAGS, STALL};

pub struct ControlPipe<'d, T: Instance> {
    max_packet_size: u16,
    generation: u32,
    _lifetime: PhantomData<&'d mut T>,
}

impl<T: Instance> ControlPipe<'_, T> {
    pub(super) fn new(max_packet_size: u16) -> Self {
        Self {
            max_packet_size,
            generation: 0,
            _lifetime: PhantomData,
        }
    }

    fn prepare(&self, incoming: bool, first: bool) -> Result<(), EndpointError> {
        T::state().inner.lock(|inner| {
            let inner = inner.borrow();
            let endpoint = &inner.endpoints[incoming as usize];
            if !inner.powered || !endpoint.enabled || endpoint.generation != self.generation {
                return Err(EndpointError::Disabled);
            }
            let regs = Registers(T::REGS);
            if regs.control(0, false).read() & 0x88 == 0x88 {
                return Err(EndpointError::Disabled);
            }
            if first {
                regs.control(0, incoming)
                    .modify(|value| *value = (*value & 3) | 4 | PRESERVE_FLAGS);
            }
            if !incoming {
                regs.control(0, false)
                    .modify(|value| *value = (*value & 4) | ACK | PRESERVE_FLAGS);
            }
            Ok(())
        })
    }

    async fn status_in(&mut self) -> Result<(), EndpointError> {
        self.prepare(true, true)?;
        write_packet::<T>(0, &[], Some(self.generation)).await
    }
}

impl<T: Instance> embassy_usb_driver::ControlPipe for ControlPipe<'_, T> {
    fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    async fn setup(&mut self) -> [u8; 8] {
        poll_fn(|context| {
            T::state().endpoint_wakers[0].register(context.waker());
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                if let Some(setup) = inner.setup.take() {
                    self.generation = inner.endpoints[0].generation;
                    return Poll::Ready(setup);
                }
                if inner.powered {
                    Registers(T::REGS)
                        .control(0, false)
                        .modify(|value| *value = (*value & 4) | ACK | PRESERVE_FLAGS);
                }
                Poll::Pending
            })
        })
        .await
    }

    async fn data_out(&mut self, buffer: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        self.prepare(false, first)?;
        read_packet::<T>(0, buffer, Some(self.generation)).await
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        self.prepare(true, first)?;
        if last {
            self.prepare(false, true)?;
        }
        write_packet::<T>(0, data, Some(self.generation)).await?;
        if last {
            read_packet::<T>(0, &mut [], Some(self.generation)).await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        let _ = self.status_in().await;
    }

    async fn reject(&mut self) {
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if inner.powered
                && inner.endpoints[0].generation == self.generation
                && Registers(T::REGS).control(0, false).read() & 0x88 != 0x88
            {
                for incoming in [false, true] {
                    inner.endpoints[incoming as usize].stalled = true;
                    Registers(T::REGS).control(0, incoming).write_value(STALL);
                }
            }
        });
    }

    async fn accept_set_address(&mut self, address: u8) {
        if address > 127 || self.status_in().await.is_err() {
            return;
        }
        T::state().inner.lock(|inner| {
            let inner = inner.borrow();
            if inner.powered
                && inner.endpoints[0].generation == self.generation
                && Registers(T::REGS).control(0, false).read() & 0x88 != 0x88
            {
                T::REGS.usb_dev_ad().write(|reg| reg.set_mask_usb_addr(address));
            }
        });
    }
}
