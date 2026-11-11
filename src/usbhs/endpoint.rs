use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
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

    fn isochronous(&self) -> bool {
        self.info.ep_type == EndpointType::Isochronous
    }

    async fn transfer(&self, address: usize, size: usize, zlp: bool) -> Result<usize, EndpointError> {
        let index = self.info.addr.index();
        let packet = self.info.max_packet_size;
        if size != 0 {
            assert!(
                address % 4 == 0 && (size <= usize::from(packet) || packet % 4 == 0),
                "USBHS transfers need 4-byte aligned buffers"
            );
            #[cfg(target_arch = "riscv32")]
            assert!(
                regs::memory_valid(address, size),
                "USBHS transfers need buffers in DMA-reachable RAM"
            );
        }
        let limit = if regs::BURST && index != 0 {
            u16::MAX / packet * packet
        } else {
            packet
        };
        let mut generation = None;
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let endpoint = usable::<T>(&mut inner, index, IN, &mut generation)?;
            let next = endpoint.start(address, size, limit, zlp);
            regs::arm(T::REGS, index, IN, next);
            Ok(())
        })?;
        let abort = OnDrop::new(move || abort::<T>(index, IN, generation));
        let done = wait::<T>(index, IN, generation).await?;
        abort.defuse();
        Ok(done)
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
        if self.isochronous() {
            return iso::read::<T>(self.info.addr.index(), buffer).await;
        }
        let packet = usize::from(self.info.max_packet_size);
        if buffer.len() < packet {
            return Err(EndpointError::BufferOverflow);
        }
        self.transfer(buffer.as_mut_ptr() as usize, packet, false).await
    }

    async fn read_transfer(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        let packet = usize::from(self.info.max_packet_size);
        if self.isochronous() {
            let mut n = 0;
            loop {
                let i = self.read(&mut buffer[n..]).await?;
                n += i;
                if i < packet {
                    return Ok(n);
                }
            }
        }
        let size = buffer.len() / packet * packet;
        if size == 0 {
            return Err(EndpointError::BufferOverflow);
        }
        self.transfer(buffer.as_mut_ptr() as usize, size, false).await
    }
}

impl<T: Instance> embassy_usb_driver::EndpointIn for Endpoint<'_, T, true> {
    async fn write(&mut self, buffer: &[u8]) -> Result<(), EndpointError> {
        if buffer.len() > usize::from(self.info.max_packet_size) {
            return Err(EndpointError::BufferOverflow);
        }
        if self.isochronous() {
            return iso::write::<T>(self.info.addr.index(), buffer).await;
        }
        self.transfer(buffer.as_ptr() as usize, buffer.len(), false).await?;
        Ok(())
    }

    async fn write_transfer(&mut self, buffer: &[u8], needs_zlp: bool) -> Result<(), EndpointError> {
        let packet = usize::from(self.info.max_packet_size);
        if self.isochronous() {
            for chunk in buffer.chunks(packet) {
                self.write(chunk).await?;
            }
            if needs_zlp && buffer.len() % packet == 0 {
                self.write(&[]).await?;
            }
            return Ok(());
        }
        if buffer.is_empty() && !needs_zlp {
            return Ok(());
        }
        let zlp = needs_zlp && !buffer.is_empty() && buffer.len() % packet == 0;
        self.transfer(buffer.as_ptr() as usize, buffer.len(), zlp).await?;
        Ok(())
    }
}

pub(super) fn usable<'a, T: Instance>(
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

pub(super) async fn wait<T: Instance>(
    index: usize,
    incoming: bool,
    generation: Option<u32>,
) -> Result<usize, EndpointError> {
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, incoming)].register(context.waker());
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let endpoint = usable::<T>(&mut inner, index, incoming, &mut { generation })?;
            if endpoint.busy {
                Poll::Pending
            } else {
                Poll::Ready(Ok(endpoint.done))
            }
        })
    })
    .await
}

fn abort<T: Instance>(index: usize, incoming: bool, generation: Option<u32>) {
    let powered = T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        let powered = inner.powered;
        let endpoint = &mut inner.endpoints[slot(index, incoming)];
        if powered && endpoint.busy && Some(endpoint.generation) == generation {
            endpoint.busy = false;
            regs::set_res(T::REGS, index, incoming, Res::Nak);
        }
        powered
    });
    if powered {
        regs::wait_sie(T::REGS);
    }
}
