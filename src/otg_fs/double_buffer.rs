//! Software state of double-buffered bulk endpoints.
//!
//! The hardware has one DMA address and one transmit length per endpoint, so the
//! interrupt handler points the endpoint at the other buffer after each packet.

/// IN: up to two packets, sent in the order they were queued.
pub(super) struct TxQueue {
    addr: [u32; 2],
    len: [u16; 2],
    head: usize,
    queued: usize,
}

impl TxQueue {
    pub(super) const fn new(addr: [u32; 2]) -> Self {
        Self {
            addr,
            len: [0; 2],
            head: 0,
            queued: 0,
        }
    }

    pub(super) fn reset(&mut self) {
        self.head = 0;
        self.queued = 0;
    }

    /// The buffer to fill next, if one is free.
    pub(super) fn free_buffer(&self) -> Option<usize> {
        (self.queued < 2).then_some((self.head + self.queued) % 2)
    }

    /// Queues the free buffer, filled with `len` bytes. Returns whether the endpoint
    /// was idle, so the packet has to be started.
    pub(super) fn enqueue(&mut self, len: u16) -> bool {
        let buffer = (self.head + self.queued) % 2;
        self.len[buffer] = len;
        self.queued += 1;
        self.queued == 1
    }

    /// The packet being sent went out. Returns false if none was being sent.
    pub(super) fn complete(&mut self) -> bool {
        if self.queued == 0 {
            return false;
        }
        self.queued -= 1;
        self.head = (self.head + 1) % 2;
        true
    }

    /// The DMA address and length of the packet to send, if one is queued.
    pub(super) fn sending(&self) -> Option<(u32, u16)> {
        (self.queued != 0).then(|| (self.addr[self.head], self.len[self.head]))
    }
}

/// OUT: up to two received packets, read in the order they arrived.
pub(super) struct RxQueue {
    addr: [u32; 2],
    len: [u16; 2],
    head: usize,
    ready: usize,
    armed: bool,
}

impl RxQueue {
    pub(super) const fn new(addr: [u32; 2]) -> Self {
        Self {
            addr,
            len: [0; 2],
            head: 0,
            ready: 0,
            armed: false,
        }
    }

    /// Drops received packets and receives into buffer 0. Returns its DMA address.
    pub(super) fn reset(&mut self) -> u32 {
        self.head = 0;
        self.ready = 0;
        self.armed = true;
        self.addr[0]
    }

    /// Whether the endpoint is receiving into a buffer.
    pub(super) fn receiving(&self) -> bool {
        self.armed
    }

    /// A packet of `len` bytes arrived. Returns the DMA address to receive into next,
    /// or `None` when both buffers are full.
    pub(super) fn complete(&mut self, len: u16) -> Option<u32> {
        self.len[(self.head + self.ready) % 2] = len;
        self.ready += 1;
        self.arm()
    }

    /// The buffer index and length of the oldest received packet.
    pub(super) fn next_packet(&self) -> Option<(usize, u16)> {
        (self.ready != 0).then(|| (self.head, self.len[self.head]))
    }

    /// The oldest packet was read. Returns the DMA address to receive into, if the
    /// endpoint had stopped because both buffers were full.
    pub(super) fn consume(&mut self) -> Option<u32> {
        self.head = (self.head + 1) % 2;
        self.ready -= 1;
        if self.armed {
            None
        } else {
            self.arm()
        }
    }

    fn arm(&mut self) -> Option<u32> {
        self.armed = self.ready < 2;
        self.armed.then(|| self.addr[(self.head + self.ready) % 2])
    }
}
