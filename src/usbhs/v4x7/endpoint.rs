use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointDisabledError, EndpointError, EndpointInfo};

use super::{
    control_aborted, is_endpoint_stalled, set_endpoint_stalled, slot, Instance, Registers, ACK, NAK, PRESERVE_FLAGS,
};

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
}

impl<T: Instance, const IN: bool> embassy_usb_driver::Endpoint for Endpoint<'_, T, IN> {
    fn info(&self) -> EndpointInfo {
        self.info
    }

    async fn wait_enabled(&mut self) {
        let index = slot(self.info.addr.index(), IN);
        poll_fn(|context| {
            T::state().endpoint_wakers[index].register(context.waker());
            T::state().inner.lock(|inner| {
                let inner = inner.borrow();
                if inner.powered && inner.endpoints[index].enabled {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }

    fn is_enabled(&self) -> bool {
        let index = slot(self.info.addr.index(), IN);
        T::state().inner.lock(|inner| {
            let inner = inner.borrow();
            inner.powered && inner.endpoints[index].enabled
        })
    }

    async fn set_stalled(&mut self, stalled: bool) {
        set_endpoint_stalled::<T>(self.info.addr, stalled);
    }

    fn is_stalled(&self) -> bool {
        is_endpoint_stalled::<T>(self.info.addr)
    }
}

impl<T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'_, T, false> {
    async fn read_transfer(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        crate::usb::read_transfer(buffer, usize::from(self.info.max_packet_size), async |packet| {
            read_packet::<T>(index, packet, None).await
        })
        .await
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write_transfer(&mut self, buffer: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        let index = self.info.addr.index();
        crate::usb::write_transfer(
            buffer,
            needs_zlp,
            usize::from(self.info.max_packet_size),
            async |packet| write_packet::<T>(index, packet, None).await,
        )
        .await
    }
}

pub(super) async fn read_packet<T: Instance>(
    index: usize,
    buffer: &mut [u8],
    expected_generation: Option<u32>,
) -> Result<usize, EndpointError> {
    let mut generation = expected_generation;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, false)].register(context.waker());
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let endpoint = &inner.endpoints[slot(index, false)];
            let expected = *generation.get_or_insert(endpoint.generation);
            if !endpoint.enabled || endpoint.stalled || expected != endpoint.generation {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if index == 0 && control_aborted::<T>(&inner, expected) {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let regs = Registers(T::REGS);
            let endpoint = &mut inner.endpoints[slot(index, false)];
            if endpoint.overflow {
                endpoint.overflow = false;
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }
            if !endpoint.rx_ready {
                regs.control(index, false)
                    .modify(|value| *value = (*value & 0x0c) | ACK | PRESERVE_FLAGS);
                return Poll::Pending;
            }
            let config = endpoint.config.unwrap();
            let length = usize::from(endpoint.rx_length);
            endpoint.rx_ready = false;
            let result = if length > buffer.len() || length > usize::from(config.buffer.size) {
                Err(EndpointError::BufferOverflow)
            } else {
                unsafe { config.buffer.read(&mut buffer[..length]) };
                Ok(length)
            };
            regs.control(index, false).modify(|value| {
                *value = (*value & 0x0c) | if index == 0 { NAK } else { ACK } | PRESERVE_FLAGS;
            });
            Poll::Ready(result)
        })
    })
    .await
}

/// Sends one packet of at most the endpoint's maximum packet size.
pub(super) async fn write_packet<T: Instance>(
    index: usize,
    buffer: &[u8],
    expected_generation: Option<u32>,
) -> Result<(), EndpointDisabledError> {
    let mut generation = expected_generation;
    let mut submitted = false;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, true)].register(context.waker());
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return Poll::Ready(Err(EndpointDisabledError));
            }
            let endpoint = &inner.endpoints[slot(index, true)];
            let expected = *generation.get_or_insert(endpoint.generation);
            if !endpoint.enabled || endpoint.stalled || expected != endpoint.generation {
                return Poll::Ready(Err(EndpointDisabledError));
            }
            if index == 0 && control_aborted::<T>(&inner, expected) {
                return Poll::Ready(Err(EndpointDisabledError));
            }
            let regs = Registers(T::REGS);
            let endpoint = &mut inner.endpoints[slot(index, true)];
            if !submitted {
                if endpoint.tx_busy {
                    return Poll::Pending;
                }
                let config = endpoint.config.unwrap();
                debug_assert!(buffer.len() <= usize::from(config.info.max_packet_size));
                unsafe { config.buffer.write(buffer) };
                endpoint.tx_done = false;
                endpoint.tx_busy = true;
                regs.tx_length(index).write_value(buffer.len() as u16);
                regs.control(index, true)
                    .modify(|value| *value = (*value & 0x0c) | ACK | PRESERVE_FLAGS);
                submitted = true;
            }
            if endpoint.tx_done {
                endpoint.tx_done = false;
                Poll::Ready(Ok(()))
            } else {
                Poll::Pending
            }
        })
    })
    .await
}
