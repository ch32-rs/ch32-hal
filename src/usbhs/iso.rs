use core::future::poll_fn;
use core::sync::atomic::{fence, Ordering};
use core::task::{Poll, Waker};

use embassy_usb_driver::{Direction, EndpointError, EndpointType};

use super::{regs, slot, EndpointConfig, EndpointState, Instance, ENDPOINTS};

/// Packet buffers per isochronous OUT endpoint (15 packets can wait for the task).
pub const ISO_OUT_SLOTS: usize = 16;
/// Packets queued per isochronous IN endpoint.
pub const ISO_IN_SLOTS: usize = 2;
// the ring indices are u8s that wrap and are masked: slot counts are powers of two up to 256
const _: () = assert!(256 % ISO_OUT_SLOTS == 0 && 256 % ISO_IN_SLOTS == 0);

pub(crate) const fn slots(incoming: bool) -> u8 {
    if incoming {
        ISO_IN_SLOTS as u8
    } else {
        ISO_OUT_SLOTS as u8
    }
}

pub(crate) const fn ring_size(incoming: bool, max_packet_size: u16) -> usize {
    slots(incoming) as usize * stride(max_packet_size)
}

const fn stride(max_packet_size: u16) -> usize {
    4 + (max_packet_size as usize).next_multiple_of(4)
}

pub(crate) fn microframe_index(frame: u16) -> u16 {
    (frame & 0x7ff) << 3 | frame >> 13
}

impl EndpointConfig {
    pub(crate) fn packet(&self, i: u8) -> usize {
        let n = slots(self.info.addr.is_in());
        self.address + usize::from(i & (n - 1)) * stride(self.info.max_packet_size)
    }

    pub(crate) fn length(&self, i: u8) -> u16 {
        unsafe { core::ptr::read_volatile((self.packet(i) - 4) as *const u32) as u16 }
    }

    pub(crate) fn set_length(&self, i: u8, length: u16) {
        unsafe { core::ptr::write_volatile((self.packet(i) - 4) as *mut u32, u32::from(length)) }
    }
}

impl EndpointState {
    pub(crate) fn queued(&self) -> u8 {
        self.head.wrapping_sub(self.tail)
    }
}

pub(crate) fn out_push(endpoint: &mut EndpointState, config: &EndpointConfig, length: u16) -> bool {
    if endpoint.queued() < slots(false) - 1 {
        config.set_length(endpoint.head, length);
        endpoint.head = endpoint.head.wrapping_add(1);
        true
    } else {
        endpoint.overruns = endpoint.overruns.wrapping_add(1);
        false
    }
}

pub(crate) fn in_next(endpoint: &mut EndpointState, config: &EndpointConfig) -> Option<(usize, u16)> {
    if endpoint.loaded {
        endpoint.tail = endpoint.tail.wrapping_add(1);
    }
    endpoint.loaded = endpoint.queued() > 0;
    endpoint
        .loaded
        .then(|| (config.packet(endpoint.tail), config.length(endpoint.tail)))
}

#[cfg(not(usbhs_v3))]
pub(crate) fn in_reload(endpoint: &mut EndpointState, config: &EndpointConfig) -> Option<(usize, u16)> {
    if endpoint.loaded || endpoint.queued() == 0 || !endpoint.enabled || endpoint.stalled {
        return None;
    }
    endpoint.loaded = true;
    Some((config.packet(endpoint.tail), config.length(endpoint.tail)))
}

fn valid(ep: usize) -> bool {
    ep != 0 && ep < ENDPOINTS
}

fn check(
    powered: bool,
    endpoint: &EndpointState,
    generation: &mut Option<u32>,
) -> Result<EndpointConfig, EndpointError> {
    let expected = *generation.get_or_insert(endpoint.generation);
    match endpoint.config {
        Some(config)
            if powered
                && endpoint.enabled
                && !endpoint.stalled
                && expected == endpoint.generation
                && config.info.ep_type == EndpointType::Isochronous =>
        {
            Ok(config)
        }
        _ => Err(EndpointError::Disabled),
    }
}

fn take<T: Instance, R>(
    index: usize,
    generation: &mut Option<u32>,
    f: impl FnOnce(&[u8]) -> R,
) -> Result<Option<R>, EndpointError> {
    let found = T::state().inner.lock(|inner| {
        let inner = inner.borrow();
        let endpoint = &inner.endpoints[slot(index, false)];
        let config = check(inner.powered, endpoint, generation)?;
        Ok((endpoint.queued() > 0).then(|| (config.packet(endpoint.tail), config.length(endpoint.tail))))
    })?;
    let Some((address, length)) = found else {
        return Ok(None);
    };
    fence(Ordering::Acquire);
    let result = f(unsafe { core::slice::from_raw_parts(address as *const u8, usize::from(length)) });
    T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        let endpoint = &mut inner.endpoints[slot(index, false)];
        if Some(endpoint.generation) == *generation {
            endpoint.tail = endpoint.tail.wrapping_add(1);
        }
    });
    Ok(Some(result))
}

fn fill<T: Instance>(
    index: usize,
    generation: &mut Option<u32>,
    f: impl FnOnce(&mut [u8]) -> Option<usize>,
) -> Result<bool, EndpointError> {
    let found = T::state().inner.lock(|inner| {
        let inner = inner.borrow();
        let endpoint = &inner.endpoints[slot(index, true)];
        let config = check(inner.powered, endpoint, generation)?;
        Ok((endpoint.queued() < slots(true)).then(|| (endpoint.head, config)))
    })?;
    let Some((head, config)) = found else {
        return Ok(false);
    };
    let size = usize::from(config.info.max_packet_size);
    let Some(length) = f(unsafe { core::slice::from_raw_parts_mut(config.packet(head) as *mut u8, size) }) else {
        return Ok(false);
    };
    fence(Ordering::Release);
    T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        let endpoint = &mut inner.endpoints[slot(index, true)];
        if Some(endpoint.generation) != *generation || endpoint.head != head {
            return Err(EndpointError::Disabled);
        }
        config.set_length(head, length.min(size) as u16);
        endpoint.head = head.wrapping_add(1);
        Ok(true)
    })
}

pub(super) async fn read<T: Instance>(index: usize, buffer: &mut [u8]) -> Result<usize, EndpointError> {
    let mut generation = None;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, false)].register(context.waker());
        take::<T, _>(index, &mut generation, |packet| {
            let target = buffer.get_mut(..packet.len()).ok_or(EndpointError::BufferOverflow)?;
            target.copy_from_slice(packet);
            Ok(packet.len())
        })?
        .map_or(Poll::Pending, Poll::Ready)
    })
    .await
}

pub(super) async fn write<T: Instance>(index: usize, data: &[u8]) -> Result<(), EndpointError> {
    let mut generation = None;
    poll_fn(|context| {
        T::state().endpoint_wakers[slot(index, true)].register(context.waker());
        let queued = fill::<T>(index, &mut generation, |packet| {
            packet[..data.len()].copy_from_slice(data);
            Some(data.len())
        })?;
        if queued {
            Poll::Ready(Ok(()))
        } else {
            Poll::Pending
        }
    })
    .await
}

/// Call `f` from the USB interrupt on every SOF (every 125 us microframe at
/// high speed). Keep it short: it delays every other USB completion.
pub fn set_sof_callback<T: Instance>(f: fn()) {
    T::state().inner.lock(|inner| {
        let mut inner = inner.borrow_mut();
        inner.sof = Some(f);
        if inner.powered {
            regs::enable_sof(T::REGS);
        }
    });
}

/// The current microframe number: 8 per frame, wrapping at 2^14. Read from the
/// frame number register, so it also counts microframes whose SOF interrupt was
/// missed.
pub fn microframe<T: Instance>() -> u16 {
    T::state().inner.lock(|inner| {
        if inner.borrow().powered {
            microframe_index(regs::frame(T::REGS))
        } else {
            0
        }
    })
}

/// Whether the host enabled endpoint `ep` in `direction` (for an isochronous
/// one: selected its interface's streaming alternate setting).
pub fn endpoint_enabled<T: Instance>(ep: usize, direction: Direction) -> bool {
    valid(ep)
        && T::state().inner.lock(|inner| {
            let inner = inner.borrow();
            inner.powered && inner.endpoints[slot(ep, direction == Direction::In)].enabled
        })
}

/// Take the oldest packet waiting on isochronous OUT endpoint `ep`, in place: `f` reads
/// it, then its slot goes back to the ring. `None`: no packet waiting (or not an
/// isochronous OUT endpoint). One reader per endpoint; callable from interrupts.
pub fn iso_out_take<T: Instance, R>(ep: usize, f: impl FnOnce(&[u8]) -> R) -> Option<R> {
    if !valid(ep) {
        return None;
    }
    take::<T, R>(ep, &mut None, f).ok().flatten()
}

/// Packets dropped because isochronous OUT endpoint `ep`'s ring was full (the
/// newest packet is dropped), since the driver was created.
pub fn iso_out_overruns<T: Instance>(ep: usize) -> u32 {
    if !valid(ep) {
        return 0;
    }
    T::state()
        .inner
        .lock(|inner| inner.borrow().endpoints[slot(ep, false)].overruns)
}

/// Wake `waker` when isochronous IN endpoint `ep` sends a packet (a slot frees
/// up), to refill it from a task. One waiter per endpoint, shared with the
/// endpoint's `write`.
pub fn iso_in_register_waker<T: Instance>(ep: usize, waker: &Waker) {
    if valid(ep) {
        T::state().endpoint_wakers[slot(ep, true)].register(waker);
    }
}

/// Packets queued on isochronous IN endpoint `ep` and not yet taken by the
/// host (including the one loaded in the endpoint).
pub fn iso_in_pending<T: Instance>(ep: usize) -> u8 {
    if !valid(ep) {
        return 0;
    }
    T::state()
        .inner
        .lock(|inner| inner.borrow().endpoints[slot(ep, true)].queued())
}

/// Queue one packet on isochronous IN endpoint `ep`; callable from interrupts.
/// `false`: not queued (ring full, endpoint not enabled or not isochronous,
/// packet too long).
pub fn iso_in_prime<T: Instance>(ep: usize, data: &[u8]) -> bool {
    valid(ep)
        && fill::<T>(ep, &mut None, |packet| {
            let target = packet.get_mut(..data.len())?;
            target.copy_from_slice(data);
            Some(data.len())
        })
        .unwrap_or(false)
}

/// Build one packet on isochronous IN endpoint `ep` in place and queue it: `f` fills the
/// next free slot (the endpoint's max packet size) and returns the packet's length.
/// `false`: not queued (ring full, endpoint not enabled or not isochronous). Callable
/// from interrupts.
pub fn iso_in_fill<T: Instance>(ep: usize, f: impl FnOnce(&mut [u8]) -> usize) -> bool {
    valid(ep) && fill::<T>(ep, &mut None, |packet| Some(f(packet))).unwrap_or(false)
}
