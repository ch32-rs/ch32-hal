use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo};

use super::{slot, Instance, Registers, ACK, NAK, PRESERVE_FLAGS};

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
    fn info(&self) -> &EndpointInfo {
        &self.info
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
}

impl<T: Instance> embassy_usb_driver::EndpointOut for Endpoint<'_, T, false> {
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        read_packet::<T>(self.info.addr.index(), buffer, None).await
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write(&mut self, buffer: &[u8]) -> Result<(), EndpointError> {
        if buffer.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        write_packet::<T>(self.info.addr.index(), buffer, None).await
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
            let endpoint = &mut inner.endpoints[slot(index, false)];
            let expected = *generation.get_or_insert(endpoint.generation);
            if !endpoint.enabled || endpoint.stalled || expected != endpoint.generation {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let regs = Registers(T::REGS);
            if index == 0 && regs.control(0, false).read() & 0x88 == 0x88 {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
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

pub(super) async fn write_packet<T: Instance>(
    index: usize,
    buffer: &[u8],
    expected_generation: Option<u32>,
) -> Result<(), EndpointError> {
    let mut generation = expected_generation;
    let mut submitted = false;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, true)].register(context.waker());
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !inner.powered {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let endpoint = &mut inner.endpoints[slot(index, true)];
            let expected = *generation.get_or_insert(endpoint.generation);
            if !endpoint.enabled || endpoint.stalled || expected != endpoint.generation {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            let regs = Registers(T::REGS);
            if index == 0 && regs.control(0, false).read() & 0x88 == 0x88 {
                return Poll::Ready(Err(EndpointError::Disabled));
            }
            if endpoint.overflow {
                endpoint.overflow = false;
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }
            if !submitted {
                if endpoint.tx_busy {
                    return Poll::Pending;
                }
                let config = endpoint.config.unwrap();
                if buffer.len() > usize::from(config.info.max_packet_size) {
                    return Poll::Ready(Err(EndpointError::BufferOverflow));
                }
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
