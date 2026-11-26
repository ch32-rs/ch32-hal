use core::marker::PhantomData;

use embassy_usb_driver::{EndpointDisabledError, EndpointError};

use super::endpoint::{read_packet, write_packet};
use super::{control_aborted, Inner, Instance, Registers, ACK, PRESERVE_FLAGS, STALL};
use crate::usb::packets;

/// EP0, driven by `Bus`.
pub(super) struct ControlPipe<T: Instance> {
    max_packet_size: u16,
    /// EP0 generation of the SETUP packet being served.
    generation: u32,
    _instance: PhantomData<T>,
}

impl<T: Instance> ControlPipe<T> {
    pub(super) fn new(max_packet_size: u16) -> Self {
        Self {
            max_packet_size,
            generation: 0,
            _instance: PhantomData,
        }
    }

    pub(super) fn max_packet_size(&self) -> usize {
        usize::from(self.max_packet_size)
    }

    /// Takes the pending SETUP packet, which starts a control transfer.
    /// Without one, keeps EP0 ready to receive it.
    pub(super) fn take_setup(&mut self, inner: &mut Inner) -> Option<[u8; 8]> {
        if let Some(setup) = inner.setup.take() {
            self.generation = inner.endpoints[0].generation;
            return Some(setup);
        }
        if inner.powered {
            Registers(T::REGS)
                .control(0, false)
                .modify(|value| *value = (*value & 4) | ACK | PRESERVE_FLAGS);
        }
        None
    }

    /// Arms an EP0 stage. The first packet of a stage is DATA1.
    fn prepare(&self, incoming: bool, first: bool) -> Result<(), EndpointDisabledError> {
        T::state().inner.lock(|inner| {
            if control_aborted::<T>(&inner.borrow(), self.generation) {
                return Err(EndpointDisabledError);
            }
            let regs = Registers(T::REGS);
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

    async fn status_in(&mut self) -> Result<(), EndpointDisabledError> {
        self.prepare(true, true)?;
        write_packet::<T>(0, &[], Some(self.generation)).await
    }

    /// Receives the OUT data stage, one packet at a time.
    pub(super) async fn data_out(&mut self, buffer: &mut [u8]) -> Result<usize, EndpointError> {
        let mut length = 0;
        loop {
            self.prepare(false, length == 0)?;
            let received = read_packet::<T>(0, &mut buffer[length..], Some(self.generation)).await?;
            length += received;
            if received < self.max_packet_size() || length == buffer.len() {
                return Ok(length);
            }
        }
    }

    /// Sends the IN data stage, one packet at a time, then waits for the
    /// status stage.
    pub(super) async fn data_in(&mut self, data: &[u8], needs_zlp: bool) -> Result<(), EndpointDisabledError> {
        for (packet, first, last) in packets(data, needs_zlp, self.max_packet_size()) {
            self.prepare(true, first)?;
            if last {
                self.prepare(false, true)?;
            }
            write_packet::<T>(0, packet, Some(self.generation)).await?;
            if last {
                read_packet::<T>(0, &mut [], Some(self.generation))
                    .await
                    .map_err(|_| EndpointDisabledError)?;
            }
        }
        Ok(())
    }

    pub(super) async fn accept(&mut self) {
        let _ = self.status_in().await;
    }

    pub(super) fn reject(&mut self) {
        T::state().inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            if !control_aborted::<T>(&inner, self.generation) {
                for incoming in [false, true] {
                    inner.endpoints[incoming as usize].stalled = true;
                    Registers(T::REGS).control(0, incoming).write_value(STALL);
                }
            }
        });
    }

    pub(super) async fn accept_set_address(&mut self, address: u8) {
        if address > 127 || self.status_in().await.is_err() {
            return;
        }
        T::state().inner.lock(|inner| {
            if !control_aborted::<T>(&inner.borrow(), self.generation) {
                T::REGS.usb_dev_ad().write(|reg| reg.set_mask_usb_addr(address));
            }
        });
    }
}
