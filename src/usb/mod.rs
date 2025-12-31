use embassy_usb_driver::EndpointAllocError;

pub(crate) struct EndpointBufferAllocator<'d, const NR_EP: usize, const SIZE: usize> {
    ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP],
    ep_next: usize,
}

/// Compile-time validation for buffer sizes
const fn validate_buffer_size(size: usize) -> bool {
    matches!(size, 8 | 16 | 32 | 64 | 512)
}

impl<'d, const NR_EP: usize, const SIZE: usize> EndpointBufferAllocator<'d, NR_EP, SIZE> {
    pub fn new(ep_buffer: &'d mut [EndpointDataBuffer<SIZE>; NR_EP]) -> Self {
        const { assert!(validate_buffer_size(SIZE), "Invalid buffer size") };
        Self { ep_buffer, ep_next: 0 }
    }

    pub fn alloc_endpoint(
        &mut self,
        max_packet_size: u16,
    ) -> Result<EndpointData<'d, SIZE>, embassy_usb_driver::EndpointAllocError> {
        if self.ep_next >= NR_EP {
            error!("out of endpoint buffers");
            return Err(EndpointAllocError);
        }

        let ep_buf_idx = self.ep_next;
        self.ep_next += 1;

        assert!(
            max_packet_size as usize <= SIZE,
            "max_packet_size {} exceeds buffer size {}",
            max_packet_size,
            SIZE
        );

        Ok(EndpointData {
            max_packet_size,
            buffer: unsafe { core::mem::transmute(&self.ep_buffer[ep_buf_idx] as *const EndpointDataBuffer<SIZE>) },
        })
    }
}

pub struct EndpointData<'d, const SIZE: usize> {
    pub max_packet_size: u16,
    pub buffer: &'d mut EndpointDataBuffer<SIZE>,
}

impl<'d, const SIZE: usize> EndpointData<'d, SIZE> {
    pub fn addr(&self) -> usize {
        self.buffer.addr()
    }
}

#[repr(C, align(4))]
pub struct EndpointDataBuffer<const SIZE: usize> {
    data: [u8; SIZE],
}

impl<const SIZE: usize> Default for EndpointDataBuffer<SIZE> {
    fn default() -> Self {
        const { assert!(validate_buffer_size(SIZE), "Invalid buffer size") };
        unsafe {
            EndpointDataBuffer {
                data: core::mem::zeroed(),
            }
        }
    }
}

impl<const SIZE: usize> EndpointDataBuffer<SIZE> {
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

// Type aliases for common buffer sizes
pub type EndpointDataBuffer8 = EndpointDataBuffer<8>;
pub type EndpointDataBuffer16 = EndpointDataBuffer<16>;
pub type EndpointDataBuffer32 = EndpointDataBuffer<32>;
pub type EndpointDataBuffer64 = EndpointDataBuffer<64>;
pub type EndpointDataBuffer512 = EndpointDataBuffer<512>;

pub type EndpointData8<'d> = EndpointData<'d, 8>;
pub type EndpointData16<'d> = EndpointData<'d, 16>;
pub type EndpointData32<'d> = EndpointData<'d, 32>;
pub type EndpointData64<'d> = EndpointData<'d, 64>;
pub type EndpointData512<'d> = EndpointData<'d, 512>;

pub type EndpointBufferAllocator8<'d, const NR_EP: usize> = EndpointBufferAllocator<'d, NR_EP, 8>;
pub type EndpointBufferAllocator16<'d, const NR_EP: usize> = EndpointBufferAllocator<'d, NR_EP, 16>;
pub type EndpointBufferAllocator32<'d, const NR_EP: usize> = EndpointBufferAllocator<'d, NR_EP, 32>;
pub type EndpointBufferAllocator64<'d, const NR_EP: usize> = EndpointBufferAllocator<'d, NR_EP, 64>;
pub type EndpointBufferAllocator512<'d, const NR_EP: usize> = EndpointBufferAllocator<'d, NR_EP, 512>;

/// USB Direction Trait
pub trait Dir {}

/// Marker type for the "IN" direction.
pub struct In;
impl Dir for In {}

/// Marker type for the "OUT" direction.
pub struct Out;
impl Dir for Out {}

/// Marker type for the control endpoint
pub struct InOut;
impl Dir for InOut {}
