use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::{EndpointError, EndpointInfo, EndpointType};

use super::{iso, regs, slot, EndpointState, Inner, Instance, Res};

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
        if self.info.ep_type == EndpointType::Isochronous {
            return iso::read::<T>(self.info.addr.index(), buffer).await;
        }
        read_packet::<T>(self.info.addr.index(), buffer, &mut None).await
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write(&mut self, buffer: &[u8]) -> Result<(), EndpointError> {
        if buffer.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        if self.info.ep_type == EndpointType::Isochronous {
            return iso::write::<T>(self.info.addr.index(), buffer).await;
        }
        write_packet::<T>(self.info.addr.index(), buffer, &mut None).await
    }
}

fn usable<'a, T: Instance>(
    inner: &'a mut Inner,
    index: usize,
    incoming: bool,
    generation: &mut Option<u32>,
) -> Result<&'a mut EndpointState, EndpointError> {
    if !inner.powered {
        return Err(EndpointError::Disabled);
    }
    let endpoint = &mut inner.endpoints[slot(index, incoming)];
    let expected = *generation.get_or_insert(endpoint.generation);
    if !endpoint.enabled || endpoint.stalled || expected != endpoint.generation {
        return Err(EndpointError::Disabled);
    }
    if index == 0 && regs::setup_pending(T::REGS) {
        return Err(EndpointError::Disabled);
    }
    if endpoint.overflow {
        endpoint.overflow = false;
        return Err(EndpointError::BufferOverflow);
    }
    Ok(endpoint)
}

fn with<T: Instance, R>(
    index: usize,
    incoming: bool,
    generation: &mut Option<u32>,
    f: impl FnOnce(&mut EndpointState) -> Poll<Result<R, EndpointError>>,
) -> Poll<Result<R, EndpointError>> {
    T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        match usable::<T>(&mut inner, index, incoming, generation) {
            Ok(endpoint) => f(endpoint),
            Err(error) => Poll::Ready(Err(error)),
        }
    })
}

pub(super) async fn read_packet<T: Instance>(
    index: usize,
    buffer: &mut [u8],
    generation: &mut Option<u32>,
) -> Result<usize, EndpointError> {
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, false)].register(context.waker());
        with::<T, _>(index, false, generation, |endpoint| {
            let config = endpoint.config.unwrap();
            if endpoint.queued() == 0 {
                endpoint.rx_wanted = buffer.len() as u16;
                if index == 0 {
                    regs::set_res(T::REGS, 0, false, Res::Ack);
                } else {
                    endpoint.arm_out(T::REGS, index);
                }
                return Poll::Pending;
            }
            let length = usize::from(endpoint.length);
            let result = if length > buffer.len() || length > usize::from(config.buffer.size) {
                Err(EndpointError::BufferOverflow)
            } else {
                unsafe { config.buffer.read(&mut buffer[..length]) };
                Ok(length)
            };
            endpoint.tail = endpoint.tail.wrapping_add(1);
            if index == 0 {
                regs::set_res(T::REGS, 0, false, Res::Nak);
            } else {
                endpoint.arm_out(T::REGS, index);
            }
            Poll::Ready(result)
        })
    })
    .await
}

pub(super) async fn write_packet<T: Instance>(
    index: usize,
    buffer: &[u8],
    generation: &mut Option<u32>,
) -> Result<(), EndpointError> {
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, true)].register(context.waker());
        with::<T, _>(index, true, generation, |endpoint| {
            let config = endpoint.config.unwrap();
            if endpoint.queued() != 0 {
                return Poll::Pending;
            }
            if buffer.len() > usize::from(config.buffer.size) {
                return Poll::Ready(Err(EndpointError::BufferOverflow));
            }
            unsafe { config.buffer.write(buffer) };
            endpoint.head = endpoint.head.wrapping_add(1);
            regs::arm_in(T::REGS, index, buffer.len() as u16);
            Poll::Ready(Ok(()))
        })
    })
    .await?;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, true)].register(context.waker());
        with::<T, _>(index, true, generation, |endpoint| {
            if endpoint.queued() != 0 {
                Poll::Pending
            } else {
                Poll::Ready(Ok(()))
            }
        })
    })
    .await
}
