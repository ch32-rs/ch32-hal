use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_usb_driver::EndpointError;

use super::endpoint::{usable, wait};
use super::{regs, Instance, Res};

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

    fn prepare(&self, incoming: bool, first: bool, wanted: usize) -> Result<(), EndpointError> {
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let endpoint = usable::<T>(&mut inner, 0, incoming, &mut Some(self.generation))?;
            if !incoming {
                let address = endpoint.config.unwrap().buffer.address;
                endpoint.start(address, wanted, self.max_packet_size, false);
            }
            regs::ep0_prepare(T::REGS, incoming, first);
            Ok(())
        })
    }

    async fn send(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let endpoint = usable::<T>(&mut inner, 0, true, &mut Some(self.generation))?;
            let buffer = endpoint.config.unwrap().buffer;
            unsafe { buffer.write(data) };
            let next = endpoint.start(buffer.address, data.len(), self.max_packet_size, false);
            regs::arm(T::REGS, 0, true, next);
            Ok(())
        })?;
        wait::<T>(0, true, Some(self.generation)).await?;
        Ok(())
    }

    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        poll_fn(|context| {
            T::state().endpoint_wakers[0].register(context.waker());
            T::state().inner.lock(|inner| {
                let mut inner = inner.borrow_mut();
                match usable::<T>(&mut inner, 0, false, &mut Some(self.generation)) {
                    Ok(endpoint) if endpoint.busy => Poll::Pending,
                    Ok(endpoint) => {
                        let length = endpoint.done;
                        unsafe { endpoint.config.unwrap().buffer.read(&mut buffer[..length]) };
                        Poll::Ready(Ok(length))
                    }
                    Err(error) => Poll::Ready(Err(error)),
                }
            })
        })
        .await
    }

    async fn status_in(&mut self) -> Result<(), EndpointError> {
        self.prepare(true, true, 0)?;
        self.send(&[]).await
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
                    regs::await_setup(T::REGS);
                }
                Poll::Pending
            })
        })
        .await
    }

    async fn data_out(&mut self, buffer: &mut [u8], first: bool, _last: bool) -> Result<usize, EndpointError> {
        self.prepare(false, first, buffer.len())?;
        self.receive(buffer).await
    }

    async fn data_in(&mut self, data: &[u8], first: bool, last: bool) -> Result<(), EndpointError> {
        if data.len() > usize::from(self.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        self.prepare(true, first, 0)?;
        if last {
            self.prepare(false, true, 0)?;
        }
        self.send(data).await?;
        if last {
            self.receive(&mut []).await?;
        }
        Ok(())
    }

    async fn accept(&mut self) {
        let _ = self.status_in().await;
    }

    async fn reject(&mut self) {
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if inner.powered && inner.endpoints[0].generation == self.generation && !regs::setup_pending(T::REGS) {
                for incoming in [false, true] {
                    inner.endpoints[incoming as usize].stalled = true;
                    regs::restart(T::REGS, 0, incoming, Res::Stall);
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
            if inner.powered && inner.endpoints[0].generation == self.generation && !regs::setup_pending(T::REGS) {
                regs::set_address(T::REGS, address);
            }
        });
    }
}
