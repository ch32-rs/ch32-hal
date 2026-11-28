#![cfg_attr(gpdma, allow(unused))]

use core::future::poll_fn;
use core::ops::Range;
use core::sync::atomic::{compiler_fence, fence, Ordering};
use core::task::{Poll, Waker};

use super::word::Word;

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OverrunError;

pub trait DmaCtrl {
    /// Get the NDTR register value, i.e. the space left in the underlying
    /// buffer until the dma writer wraps.
    fn get_remaining_transfers(&self) -> usize;

    /// Get the transfer completed counter.
    /// This counter is incremented by the dma controller when NDTR is reloaded,
    /// i.e. when the writing wraps.
    fn get_complete_count(&self) -> usize;

    /// Reset the transfer completed counter to 0 and return the value just prior to the reset.
    fn reset_complete_count(&mut self) -> usize;

    /// Set the waker for a running poll_fn
    fn set_waker(&mut self, waker: &Waker);

    /// Whether the transfer-complete flag is set, i.e. its interrupt has not counted that lap yet.
    fn complete_pending(&self) -> bool;

    /// Clear the transfer-complete flag, so that its interrupt does not count that lap.
    fn clear_complete(&mut self);
}

/// A position in the ring buffer, counted in laps (`complete_count`) and words into the current
/// lap (`pos`). The read side of this file follows embassy-stm32's ring buffer
/// (embassy-rs/embassy@62622d7d2d998574d02d385694729bc0ba3a3fcc, `embassy-stm32/src/dma/ringbuffer.rs`).
#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct DmaIndex {
    complete_count: usize,
    pos: usize,
}

impl DmaIndex {
    /// Reset the index to the start of the buffer (position 0, lap 0).
    fn reset(&mut self) {
        self.pos = 0;
        self.complete_count = 0;
    }

    /// Synchronise the index against the live DMA hardware state.
    fn dma_sync(&mut self, cap: usize, dma: &mut impl DmaCtrl) {
        // The ordering of the these lines matters.
        // FIRST reset complete count, THEN read remaining transfers.
        //
        // With this order when DMA wraps between these two reads,
        // `laps_completed` will be 0 while `pos` appears to go
        // backwards comparing to the previous value in `self.pos`.
        // This ensures that the approximated position never goes ahead of
        // the real DMA position.
        //
        // If the order was reversed and DMA wrapped between these two reads,
        // `laps_completed` will be 1 while `pos` has a value before wrap.
        // This is not acceptable since the approximated position will jump
        // ahead of the real DMA position.
        let laps_completed = dma.reset_complete_count();
        let pos = cap - dma.get_remaining_transfers();
        // It is also not acceptable for approximated position to go backward, so it
        // must be modified such that it is at least not behind previous position.
        // This also happens when the DMA wrapped but the transfer complete interrupt
        // has not run yet. Compare against the previous position including the laps
        // just counted because `self.pos` may still be extended from a previous wrap.
        //
        // It is possible for the following condition to be true in two cases:
        // - This function is called when DMA wraps and complete count is not yet recorded.
        //   So `laps_completed` is 0 and `pos < self.pos`. It is like `pos < prev_pos`
        //   which indicating going backward, therefore the condition detects it.
        // - The case above happens twice (or more) in a row.
        //   After the first call `self.pos` is already extended. During the second call
        //   DMA wraps again, but is not recorded in complete count, only the previous wrap is.
        //   So `laps_completed` is 1 and `cap + pos < self.pos`. It is like `cap + pos < cap + prev_pos`
        //   since `prev_pos` is already extended, so it is equivalent to `pos < prev_pos`
        //   which indicating going backward, therefore the condition detects it.
        //
        // `self.pos` is below `2 * cap`, so two laps or more never take this branch; testing that
        // first keeps `laps_completed * cap` from overflowing after a long time without reads.
        self.pos = if laps_completed < 2 && laps_completed * cap + pos < self.pos {
            // `self.complete_count` cannot be manually incremented here because in
            // next call to this function `laps_completed` will be 1, so it would
            // be applied twice. However `self.pos` can temporarily be extended
            // beyond normal range. The next call will set it back to normal value
            // and increment `self.complete_count` compensating `cap` added here.
            pos + cap
        } else {
            pos
        };
        // The interrupt's count wraps: fewer than 2^32 laps must end between two looks. The sum
        // saturates, which the overrun checks then catch before any multiplication.
        self.complete_count = self.complete_count.saturating_add(laps_completed);
    }

    /// Advance the index by `steps` words, incrementing `complete_count` for
    /// each full lap completed.
    fn advance(&mut self, cap: usize, steps: usize) {
        let next = self.pos + steps;
        // The reads advance by at most a lap from within a lap: no division then (a hardware
        // divide takes tens of cycles on these cores).
        if next < cap {
            self.pos = next;
        } else if next < 2 * cap {
            self.complete_count = self.complete_count.saturating_add(1);
            self.pos = next - cap;
        } else {
            self.complete_count = self.complete_count.saturating_add(next / cap);
            self.pos = next % cap;
        }
    }

    /// Subtract the smaller `complete_count` from both indices so that the
    /// absolute lap counts stay small. This prevents overflow in [`diff`] after
    /// many laps while leaving the relative difference unchanged.
    fn normalize(lhs: &mut DmaIndex, rhs: &mut DmaIndex) {
        let min_count = lhs.complete_count.min(rhs.complete_count);
        lhs.complete_count -= min_count;
        rhs.complete_count -= min_count;
    }

    /// Return how many words `self` is ahead of `rhs` in linear (unwrapped)
    /// space. A negative result means `self` is behind `rhs`, which indicates
    /// a driver bug or an out-of-band DMA reset.
    fn diff(&self, cap: usize, rhs: &DmaIndex) -> isize {
        self.total(cap) as isize - rhs.total(cap) as isize
    }

    fn total(&self, cap: usize) -> usize {
        self.complete_count * cap + self.pos
    }
}

/// A "read-only" ring-buffer to be used together with the DMA controller which
/// writes in a circular way, "uncontrolled" to the buffer.
///
/// The DMA's position is tracked as a lap count plus a position in the lap, so that the buffer
/// can tell how far the DMA is ahead of the reader, wrap included.
pub struct ReadableDmaRingBuffer<'a, W: Word> {
    pub(crate) dma_buf: &'a mut [W],
    write_index: DmaIndex,
    read_index: DmaIndex,
}

impl<'a, W: Word> ReadableDmaRingBuffer<'a, W> {
    /// Construct an empty buffer.
    pub fn new(dma_buf: &'a mut [W]) -> Self {
        Self {
            dma_buf,
            write_index: Default::default(),
            read_index: Default::default(),
        }
    }

    /// Reset the ring buffer to its initial state: empty, from the DMA's current position.
    ///
    /// A lap that ended before that position goes with the dropped data: if its interrupt has not
    /// run yet, its transfer-complete flag is cleared, or that interrupt would count it as a lap
    /// of the new data.
    pub fn clear(&mut self, dma: &mut impl DmaCtrl) {
        loop {
            // The interrupt cannot run in this critical section. A lap that ended before goes with
            // the dropped data: its flag is cleared first. A lap that ends while the position is
            // read sets the flag again, and may have ended on either side of the position: try
            // again (the next attempt drops that lap too, whether its interrupt counted it in
            // between or its flag is still set).
            let cap = self.cap();
            let write_index = &mut self.write_index;
            let settled = critical_section::with(|_| {
                dma.clear_complete();
                dma.reset_complete_count();
                write_index.reset();
                write_index.dma_sync(cap, dma);
                !dma.complete_pending()
            });
            if settled {
                break;
            }
        }
        // The writer may stay at `cap` (NDTR read as 0, the reload's lap still to be counted),
        // which its next sync needs; the reader starts at 0 of the next lap, the same place.
        self.read_index = self.write_index;
        self.read_index.advance(self.cap(), 0);
    }

    /// The capacity of the ringbuffer
    pub const fn cap(&self) -> usize {
        self.dma_buf.len()
    }

    /// Sync against the DMA hardware and return the number of readable elements available.
    ///
    /// A DMA position behind the reader (only possible if the channel was reset behind this
    /// buffer's back) is reported as an overrun too: either way the data cannot be trusted.
    fn sync_len(&mut self, dma: &mut impl DmaCtrl) -> Result<usize, OverrunError> {
        self.write_index.dma_sync(self.cap(), dma);
        DmaIndex::normalize(&mut self.write_index, &mut self.read_index);
        // More than a lap apart is an overrun (or a DMA behind the reader) whatever the
        // positions. Checking it first keeps `complete_count * cap` in `diff` from overflowing
        // when the buffer goes unread for a long time.
        if self.write_index.complete_count > 1 || self.read_index.complete_count > 1 {
            return Err(OverrunError);
        }

        let diff = self.write_index.diff(self.cap(), &self.read_index);

        if diff < 0 || diff > self.cap() as isize {
            Err(OverrunError)
        } else {
            Ok(diff as usize)
        }
    }

    /// Read elements from the ring buffer
    /// Return a tuple of the length read and the length remaining in the buffer
    /// If not all of the elements were read, then there will be some elements in the buffer remaining
    /// The length remaining is the capacity, ring_buf.len(), less the elements remaining after the read
    /// OverrunError is returned if the portion to be read was overwritten by the DMA controller.
    pub fn read(&mut self, dma: &mut impl DmaCtrl, buf: &mut [W]) -> Result<(usize, usize), OverrunError> {
        let (len, unread) = self.read_raw(dma, buf)?;
        Ok((len, self.cap() - unread))
    }

    /// Read an exact number of elements from the ringbuffer.
    ///
    /// Returns the remaining number of elements available for immediate reading.
    /// OverrunError is returned if the portion to be read was overwritten by the DMA controller.
    ///
    /// Async/Wake Behavior:
    /// The underlying DMA peripheral only can wake us when its buffer pointer has reached the halfway point,
    /// and when it wraps around. This means that when called with a buffer of length 'M', when this
    /// ring buffer was created with a buffer of size 'N':
    /// - If M equals N/2 or N/2 divides evenly into M, this function will return every N/2 elements read on the DMA source.
    /// - Otherwise, this function may need up to N/2 extra elements to arrive before returning.
    pub async fn read_exact(&mut self, dma: &mut impl DmaCtrl, buffer: &mut [W]) -> Result<usize, OverrunError> {
        let mut read_data = 0;
        let buffer_len = buffer.len();

        poll_fn(|cx| {
            dma.set_waker(cx.waker());

            match self.read_raw(dma, &mut buffer[read_data..]) {
                Ok((len, unread)) => {
                    read_data += len;
                    if read_data == buffer_len {
                        Poll::Ready(Ok(unread))
                    } else {
                        Poll::Pending
                    }
                }
                Err(e) => Poll::Ready(Err(e)),
            }
        })
        .await
    }

    /// Read elements: the length read and the number of elements still unread after it.
    fn read_raw(&mut self, dma: &mut impl DmaCtrl, buf: &mut [W]) -> Result<(usize, usize), OverrunError> {
        fence(Ordering::Acquire);

        let available = self.sync_len(dma)?;
        let readable = available.min(buf.len());
        if readable == 0 {
            return Ok((0, available));
        }

        let read_start = self.read_index;
        self.copy_from(read_start.pos, &mut buf[..readable]);

        // The DMA may have caught up with the read position and written into the
        // copied data meanwhile. All of it is intact if the DMA is still at most
        // one buffer length ahead of the first word copied. The copy has to
        // complete before the DMA position is read.
        fence(Ordering::SeqCst);
        self.write_index.dma_sync(self.cap(), dma);
        // A lap that ends between that sync's lap count and NDTR reads, with the DMA back at the
        // position of the previous sync, is invisible to it; its interrupt has counted it by now,
        // so sync again before trusting the copy.
        if dma.get_complete_count() != 0 {
            self.write_index.dma_sync(self.cap(), dma);
        }
        // More than a lap apart is an overrun before any multiplication (laps may have piled up
        // while the copy was preempted).
        if self.write_index.complete_count > read_start.complete_count + 1
            || self.write_index.diff(self.cap(), &read_start) > self.cap() as isize
        {
            return Err(OverrunError);
        }

        self.read_index = read_start;
        self.read_index.advance(self.cap(), readable);

        let remaining = self.write_index.diff(self.cap(), &self.read_index).max(0) as usize;
        Ok((readable, remaining))
    }

    /// Copy `buf.len()` words from the DMA buffer into `buf`, starting at buffer
    /// position `start` and wrapping at the end of the buffer.
    ///
    /// Copies in at most two contiguous runs, so there is no modulo per word.
    fn copy_from(&self, start: usize, buf: &mut [W]) {
        let cap = self.cap();
        assert!(start < cap && buf.len() <= cap);
        let src = self.dma_buf.as_ptr();
        let (head, tail) = buf.split_at_mut(buf.len().min(cap - start));
        for (i, w) in head.iter_mut().enumerate() {
            // SAFETY: `start + i < cap`. Volatile, because the DMA writes the buffer.
            *w = unsafe { core::ptr::read_volatile(src.add(start + i)) };
        }
        for (i, w) in tail.iter_mut().enumerate() {
            // SAFETY: `i < buf.len() - (cap - start) <= start < cap`.
            *w = unsafe { core::ptr::read_volatile(src.add(i)) };
        }
    }
}

pub struct WritableDmaRingBuffer<'a, W: Word> {
    pub(crate) dma_buf: &'a mut [W],
    end: usize,
}

impl<'a, W: Word> WritableDmaRingBuffer<'a, W> {
    pub fn new(dma_buf: &'a mut [W]) -> Self {
        Self { dma_buf, end: 0 }
    }

    /// Reset the ring buffer to its initial state
    pub fn clear(&mut self, dma: &mut impl DmaCtrl) {
        self.end = 0;
        dma.reset_complete_count();
    }

    /// The capacity of the ringbuffer
    pub const fn cap(&self) -> usize {
        self.dma_buf.len()
    }

    /// The current position of the ringbuffer
    fn pos(&self, dma: &mut impl DmaCtrl) -> usize {
        self.cap() - dma.get_remaining_transfers()
    }

    /// Write elements directly to the buffer. This must be done before the DMA is started
    /// or after the buffer has been cleared using `clear()`.
    pub fn write_immediate(&mut self, buffer: &[W]) -> Result<(usize, usize), OverrunError> {
        if self.end != 0 {
            return Err(OverrunError);
        }
        let written = self.copy_from(buffer, 0..self.cap());
        self.end = written % self.cap();
        Ok((written, self.cap() - written))
    }

    /// Write an exact number of elements to the ringbuffer.
    pub async fn write_exact(&mut self, dma: &mut impl DmaCtrl, buffer: &[W]) -> Result<usize, OverrunError> {
        let mut written_data = 0;
        let buffer_len = buffer.len();

        poll_fn(|cx| {
            dma.set_waker(cx.waker());

            compiler_fence(Ordering::SeqCst);

            match self.write(dma, &buffer[written_data..buffer_len]) {
                Ok((len, remaining)) => {
                    written_data += len;
                    if written_data == buffer_len {
                        Poll::Ready(Ok(remaining))
                    } else {
                        Poll::Pending
                    }
                }
                Err(e) => Poll::Ready(Err(e)),
            }
        })
        .await
    }

    /// Write elements from the ring buffer
    /// Return a tuple of the length written and the capacity remaining to be written in the buffer
    pub fn write(&mut self, dma: &mut impl DmaCtrl, buf: &[W]) -> Result<(usize, usize), OverrunError> {
        let start = self.pos(dma);
        if start > self.end {
            // The occupied portion in the ring buffer DOES wrap
            let len = self.copy_from(buf, self.end..start);

            compiler_fence(Ordering::SeqCst);

            // Confirm that the DMA is not inside data we could have written
            let (pos, complete_count) = critical_section::with(|_| (self.pos(dma), dma.get_complete_count()));
            if (pos >= self.end && pos < start) || (complete_count > 0 && pos >= start) || complete_count > 1 {
                Err(OverrunError)
            } else {
                self.end = (self.end + len) % self.cap();

                Ok((len, self.cap() - (start - self.end)))
            }
        } else if start == self.end && dma.get_complete_count() == 0 {
            Ok((0, 0))
        } else if start <= self.end && self.end + buf.len() < self.cap() {
            // The occupied portion in the ring buffer DOES NOT wrap
            // and copying elements into the buffer WILL NOT cause it to

            // Copy into the dma buffer
            let len = self.copy_from(buf, self.end..self.cap());

            compiler_fence(Ordering::SeqCst);

            // Confirm that the DMA is not inside data we could have written
            let pos = self.pos(dma);
            if pos > self.end || pos < start || dma.get_complete_count() > 1 {
                Err(OverrunError)
            } else {
                self.end = (self.end + len) % self.cap();

                Ok((len, self.cap() - (self.end - start)))
            }
        } else {
            // The occupied portion in the ring buffer DOES NOT wrap
            // and copying elements into the buffer WILL cause it to

            let tail = self.copy_from(buf, self.end..self.cap());
            let head = self.copy_from(&buf[tail..], 0..start);

            compiler_fence(Ordering::SeqCst);

            // Confirm that the DMA is not inside data we could have written
            let pos = self.pos(dma);
            if pos > self.end || pos < start || dma.reset_complete_count() > 1 {
                Err(OverrunError)
            } else {
                self.end = head;

                Ok((tail + head, self.cap() - (start - self.end)))
            }
        }
    }
    /// Copy into the dma buffer at `data_range` from `buf`
    fn copy_from(&mut self, buf: &[W], data_range: Range<usize>) -> usize {
        // Limit the number of elements that can be copied
        let length = usize::min(data_range.len(), buf.len());

        // Copy into dma buffer from read buffer
        // We need to do it like this instead of a simple copy_from_slice() because
        // reading from a part of memory that may be simultaneously written to is unsafe
        unsafe {
            let dma_buf = self.dma_buf.as_mut_ptr();

            for i in 0..length {
                core::ptr::write_volatile(dma_buf.offset((data_range.start + i) as isize), buf[i]);
            }
        }

        length
    }
}
