use embassy_usb_driver::{Direction, EndpointAllocError};

pub(crate) struct EndpointBufferAllocator<'d, const NR_EP: usize> {
    ep_buffer: &'d mut [EndpointDataBuffer; NR_EP],
    ep_next: usize,
}

impl<'d, const NR_EP: usize> EndpointBufferAllocator<'d, NR_EP> {
    pub fn new(ep_buffer: &'d mut [EndpointDataBuffer; NR_EP]) -> Self {
        Self { ep_buffer, ep_next: 0 }
    }

    pub fn alloc_endpoint(
        &mut self,
        max_packet_size: u16,
    ) -> Result<EndpointData<'d>, embassy_usb_driver::EndpointAllocError> {
        if self.ep_next >= NR_EP {
            error!("out of endpoint buffers");
            return Err(EndpointAllocError);
        }

        let ep_buf_idx = self.ep_next;
        self.ep_next += 1;

        Ok(EndpointData {
            max_packet_size,
            buffer: unsafe { core::mem::transmute(&self.ep_buffer[ep_buf_idx] as *const EndpointDataBuffer) },
        })
    }
}

pub(crate) struct EndpointData<'d> {
    pub max_packet_size: u16,
    pub buffer: &'d mut EndpointDataBuffer,
}

impl<'d> EndpointData<'d> {
    pub fn addr(&self) -> usize {
        self.buffer.addr()
    }
}

// todo generic
const EP_MAX_PACKET_SIZE: usize = 64;

#[repr(C, align(4))]
pub struct EndpointDataBuffer {
    data: [u8; EP_MAX_PACKET_SIZE as usize],
}

impl Default for EndpointDataBuffer {
    fn default() -> Self {
        unsafe {
            EndpointDataBuffer {
                data: core::mem::zeroed(),
            }
        }
    }
}

impl EndpointDataBuffer {
    pub(crate) fn read_volatile(&self, buf: &mut [u8]) {
        assert!(buf.len() <= self.data.len());
        let len = buf.len();

        for i in 0..len {
            buf[i] = unsafe { core::ptr::read_volatile(&self.data[i]) };
        }
    }

    pub(crate) fn write_volatile(&mut self, buf: &[u8]) {
        assert!(buf.len() <= self.data.len());
        let len = buf.len();

        for i in 0..len {
            unsafe { core::ptr::write_volatile(&mut self.data[i], buf[i]) };
        }
    }

    pub(crate) fn addr(&self) -> usize {
        self.data.as_ptr() as usize
    }
}

/// USB Direction Trait
pub trait Dir {
    /// Returns the direction value.
    fn dir() -> Direction;
}

/// Marker type for the "IN" direction.
pub struct In;
impl Dir for In {
    fn dir() -> Direction {
        Direction::In
    }
}

/// Marker type for the "OUT" direction.
pub struct Out;
impl Dir for Out {
    fn dir() -> Direction {
        Direction::Out
    }
}
