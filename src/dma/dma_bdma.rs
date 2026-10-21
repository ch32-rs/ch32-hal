use core::future::{poll_fn, Future};
use core::pin::Pin;
use core::sync::atomic::{compiler_fence, fence, AtomicUsize, Ordering};
use core::task::{Context, Poll, Waker};

use embassy_sync::waitqueue::AtomicWaker;

use super::ringbuffer::{DmaCtrl, OverrunError, ReadableDmaRingBuffer, WritableDmaRingBuffer};
use super::word::{Word, WordSize};
use super::{AnyChannel, Channel, Dir, Request, STATE};
use crate::interrupt::typelevel::Interrupt;
use crate::pac::InterruptNumber;
use crate::{interrupt, pac, Peri};

pub(crate) struct ChannelInfo {
    pub(crate) dma: DmaInfo,
    pub(crate) num: usize,
    /// PFIC interrupt this channel's completion is delivered on. `None` for a
    /// channel whose controller has no per-channel interrupt in the metadata.
    pub(crate) irq: Option<pac::Interrupt>,
}

#[derive(Clone, Copy)]
pub(crate) enum DmaInfo {
    Dma(pac::dma::Dma),
}

/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// Request priority level
    pub priority: Priority,
    /// Enable circular DMA
    ///
    /// Note:
    /// If you enable circular mode manually, you may want to build and `.await` the `Transfer` in a separate task.
    /// Since DMA in circular mode need manually stop, `.await` in current task would block the task forever.
    pub circular: bool,
    /// Enable half transfer interrupt
    pub half_transfer_ir: bool,
    /// Enable transfer complete interrupt
    pub complete_transfer_ir: bool,
    /// Increment the peripheral address after each transfer.
    ///
    /// Off by default: a peripheral's data register sits at a fixed address.
    /// Turn it on together with `memory_to_memory` for a memory-to-memory copy,
    /// where both ends are real addresses.
    pub peripheral_increment: bool,
    /// Memory-to-memory mode.
    ///
    /// The controller then reads from the peripheral address and writes to the
    /// memory address on its own, without waiting for a peripheral request.
    /// Off by default, which is what every peripheral driver needs.
    pub memory_to_memory: bool,
}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {
            priority: Priority::VeryHigh,
            circular: false,
            half_transfer_ir: false,
            complete_transfer_ir: true,
            peripheral_increment: false,
            memory_to_memory: false,
        }
    }
}

/// DMA request priority
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Priority {
    /// Low Priority
    Low,
    /// Medium Priority
    Medium,
    /// High Priority
    High,
    /// Very High Priority
    VeryHigh,
}

/// Register encoding of a [`Priority`].
///
/// V1-style PACs expose this as `pac::dma::vals::Pl`; the H4 block has the
/// same two bits but no value enum, so both sides go through this.
fn priority_bits(priority: Priority) -> u8 {
    match priority {
        Priority::Low => 0,
        Priority::Medium => 1,
        Priority::High => 2,
        Priority::VeryHigh => 3,
    }
}

/// Register encoding of a [`WordSize`]: 0 = byte, 1 = half-word, 2 = word.
fn size_bits(size: WordSize) -> u8 {
    match size {
        WordSize::OneByte => 0,
        WordSize::TwoBytes => 1,
        WordSize::FourBytes => 2,
    }
}

/// `DIR` bit: set when the peripheral is written, clear when it is read.
fn dir_bit(dir: Dir) -> bool {
    matches!(dir, Dir::MemoryToPeripheral)
}

/// DMAMUX slot for a channel. DMA1 CH1..CH8 occupy slots 0..8 and DMA2
/// CH1..CH8 occupy slots 8..16, so the slot is the controller base address
/// offset from DMA1 divided by the 0x400 stride, times 8, plus the channel.
#[cfg(ch32h4)]
fn mux_slot(info: &ChannelInfo) -> usize {
    let DmaInfo::Dma(r) = info.dma;
    (r.as_ptr() as usize - pac::DMA1.as_ptr() as usize) / 0x400 * 8 + info.num
}

/// Program one channel's address, count and configuration registers.
///
/// The two PACs describe the same layout, but the V1 block names the
/// registers `CR`/`NDTR`/`PAR`/`MAR` and types the fields as enums, while H4
/// names them `CFGR`/`CNTR`/`PADDR`/`MADDR` and types them as raw values.
#[cfg(not(ch32h4))]
fn write_channel(
    ch: pac::dma::Ch,
    peri_addr: *const u32,
    mem_addr: *mut u32,
    mem_len: usize,
    incr_mem: bool,
    data_size: WordSize,
    dir: Dir,
    options: &TransferOptions,
) {
    use pac::dma::vals::{Dir as DirVal, Pl, Size};

    ch.par().write_value(peri_addr as u32); // PADDR
    ch.mar().write_value(mem_addr as u32); // MADDR
    ch.ndtr().write(|w| w.set_ndt(mem_len as u16)); // CNTR
    ch.cr().write(|w| {
        w.set_psize(match size_bits(data_size) {
            0 => Size::BITS8,
            1 => Size::BITS16,
            _ => Size::BITS32,
        });
        w.set_msize(match size_bits(data_size) {
            0 => Size::BITS8,
            1 => Size::BITS16,
            _ => Size::BITS32,
        });
        w.set_pinc(options.peripheral_increment);
        w.set_mem2mem(options.memory_to_memory);
        w.set_minc(incr_mem);
        w.set_dir(if dir_bit(dir) {
            DirVal::FROMMEMORY
        } else {
            DirVal::FROMPERIPHERAL
        });
        w.set_teie(true); // error
        w.set_tcie(options.complete_transfer_ir); // tx complete
        w.set_htie(options.half_transfer_ir); // half
        w.set_circ(options.circular); // circular
        w.set_pl(match priority_bits(options.priority) {
            0 => Pl::LOW,
            1 => Pl::MEDIUM,
            2 => Pl::HIGH,
            _ => Pl::VERYHIGH,
        });
        w.set_en(false); // don't start yet
    });
}

#[cfg(ch32h4)]
fn write_channel(
    ch: pac::dma::Ch,
    peri_addr: *const u32,
    mem_addr: *mut u32,
    mem_len: usize,
    incr_mem: bool,
    data_size: WordSize,
    dir: Dir,
    options: &TransferOptions,
) {
    ch.paddr().write_value(peri_addr as u32);
    ch.maddr().write_value(mem_addr as u32);
    ch.cntr().write(|w| w.set_ndt(mem_len as u16));
    ch.cfgr().write(|w| {
        w.set_psize(size_bits(data_size));
        w.set_msize(size_bits(data_size));
        w.set_pinc(options.peripheral_increment);
        w.set_mem2mem(options.memory_to_memory);
        w.set_minc(incr_mem);
        w.set_dir(dir_bit(dir));
        w.set_teie(true);
        w.set_tcie(options.complete_transfer_ir);
        w.set_htie(options.half_transfer_ir);
        w.set_circ(options.circular);
        w.set_pl(priority_bits(options.priority));
        w.set_en(false);
    });
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: AtomicUsize,
}

impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: AtomicUsize::new(0),
    };
}

/// safety: must be called only once
///
/// `enable_irqs` is false on a core that does not own the DMA interrupts. On
/// CH32H4 an interrupt above 31 is shared and delivered to exactly one core:
/// enabling it assigns it to the core that did the enable (RM §4.7.5.45,
/// `PFIC_IALLOCR`). Enabling all of them here would take them from the boot
/// core. The channel IRQ is unmasked later, when this core starts a transfer
/// on a channel it owns.
pub(crate) unsafe fn init(
    cs: critical_section::CriticalSection,
    dma_priority: interrupt::Priority,
    enable_irqs: bool,
) {
    foreach_interrupt! {
        ($peri:ident, dma, $block:ident, $signal_name:ident, $irq:ident) => {
            crate::interrupt::typelevel::$irq::set_priority_with_cs(cs, dma_priority);
            if enable_irqs {
                crate::interrupt::typelevel::$irq::enable();
            }
        };
    }
    crate::_generated::init_dma();
}

impl AnyChannel {
    /// Safety: Must be called with a matching set of parameters for a valid dma channel
    pub(crate) unsafe fn on_irq(&self) {
        let info = self.info();
        let state = &STATE[self.id as usize];
        match self.info().dma {
            DmaInfo::Dma(r) => {
                // V1 calls the flag registers ISR/IFCR and the channel
                // configuration CR; H4 calls them INTFR/INTFCR and CFGR. The
                // bit positions are identical.
                #[cfg(not(ch32h4))]
                let (teif, htif, htie, tcif, tcie) = {
                    let isr = r.isr().read();
                    let cr = r.ch(info.num).cr().read();
                    (isr.teif(info.num), isr.htif(info.num), cr.htie(), isr.tcif(info.num), cr.tcie())
                };
                #[cfg(ch32h4)]
                let (teif, htif, htie, tcif, tcie) = {
                    let isr = r.intfr().read();
                    let cr = r.ch(info.num).cfgr().read();
                    (isr.teif(info.num), isr.htif(info.num), cr.htie(), isr.tcif(info.num), cr.tcie())
                };

                if teif {
                    panic!("DMA: error on DMA@{:08x} channel {}", r.as_ptr() as u32, info.num);
                }

                if htif && htie {
                    // Acknowledge half transfer complete interrupt
                    #[cfg(not(ch32h4))]
                    r.ifcr().write(|w| w.set_htif(info.num, true));
                    #[cfg(ch32h4)]
                    r.intfcr().write(|w| w.set_chtif(info.num, true));
                } else if tcif && tcie {
                    // Acknowledge transfer complete interrupt
                    #[cfg(not(ch32h4))]
                    r.ifcr().write(|w| w.set_tcif(info.num, true));
                    #[cfg(ch32h4)]
                    r.intfcr().write(|w| w.set_ctcif(info.num, true));
                    critical_section::with(|_| {
                        // #safty this is okay because critical section ensures
                        // no interruption
                        let cnt = state.complete_count.load(Ordering::Acquire);
                        state.complete_count.store(cnt + 1, Ordering::Release);
                    });
                } else {
                    return;
                }

                state.waker.wake();
            }
        }
    }

    unsafe fn configure(
        &self,
        request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        options: TransferOptions,
    ) {
        let info = self.info();

        // Unmask this channel's interrupt on the core that starts the transfer.
        // `init` only does this for the core that owns the DMA interrupts; on
        // CH32H4 enabling an interrupt above 31 assigns it to the enabling core
        // (RM §4.7.5.45), so the other core must enable exactly the channels it
        // uses and no others.
        if let Some(irq) = info.irq {
            unsafe { qingke::pfic::enable_interrupt(irq.number() as u8) };
        }

        assert!(mem_len > 0 && mem_len <= 0xFFFF);

        // On CH32H4 the request number selects the peripheral inside the
        // DMAMUX. The mux has one 7-bit slot per channel: DMA1 CH1..CH8 are
        // slots 0..8, DMA2 CH1..CH8 are slots 8..16, packed four per CFGR.
        // Fixed-wiring chips pass `()` and never reach this.
        #[cfg(ch32h4)]
        {
            let slot = mux_slot(info);
            pac::DMAMUX.cfgr(slot / 4).modify(|w| {
                w.set_channel_mux(slot % 4, request);
            });
        }
        #[cfg(not(ch32h4))]
        let _ = request;

        match self.info().dma {
            DmaInfo::Dma(r) => {
                let state: &ChannelState = &STATE[self.id as usize];
                let ch = r.ch(info.num);

                state.complete_count.store(0, Ordering::Release);
                self.clear_irqs();

                write_channel(
                    ch, peri_addr, mem_addr, mem_len, incr_mem, data_size, dir, &options,
                );
            }
        }
    }

    fn start(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Dma(r) => {
                let ch = r.ch(info.num);
                #[cfg(not(ch32h4))]
                ch.cr().modify(|w| w.set_en(true));
                #[cfg(ch32h4)]
                ch.cfgr().modify(|w| w.set_en(true));
            }
        }
    }

    fn clear_irqs(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Dma(r) => {
                #[cfg(not(ch32h4))]
                r.ifcr().write(|w| {
                    w.set_htif(info.num, true);
                    w.set_tcif(info.num, true);
                    w.set_teif(info.num, true);
                });
                #[cfg(ch32h4)]
                r.intfcr().write(|w| {
                    w.set_chtif(info.num, true);
                    w.set_ctcif(info.num, true);
                    w.set_cteif(info.num, true);
                });
            }
        }
    }

    fn request_stop(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Dma(r) => {
                // Disable the channel. Keep the IEs enabled so the irqs still fire.
                #[cfg(not(ch32h4))]
                r.ch(info.num).cr().write(|w| {
                    w.set_teie(true);
                    w.set_tcie(true);
                });
                #[cfg(ch32h4)]
                r.ch(info.num).cfgr().write(|w| {
                    w.set_teie(true);
                    w.set_tcie(true);
                });
            }
        }
    }

    fn is_running(&self) -> bool {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Dma(r) => {
                let state: &ChannelState = &STATE[self.id as usize];
                let ch = r.ch(info.num);
                #[cfg(not(ch32h4))]
                let (en, circular) = {
                    let cr = ch.cr().read();
                    (cr.en(), cr.circ())
                };
                #[cfg(ch32h4)]
                let (en, circular) = {
                    let cr = ch.cfgr().read();
                    (cr.en(), cr.circ())
                };
                let tcif = state.complete_count.load(Ordering::Acquire) != 0;
                en && (circular || !tcif)
            }
        }
    }

    pub fn get_remaining_transfers(&self) -> u16 {
        let info = self.info();
        match self.info().dma {
            #[cfg(not(ch32h4))]
            DmaInfo::Dma(r) => r.ch(info.num).ndtr().read().ndt(),
            #[cfg(ch32h4)]
            DmaInfo::Dma(r) => r.ch(info.num).cntr().read().ndt(),
        }
    }

    fn disable_circular_mode(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Dma(regs) => {
                #[cfg(not(ch32h4))]
                regs.ch(info.num).cr().modify(|w| w.set_circ(false));
                #[cfg(ch32h4)]
                regs.ch(info.num).cfgr().modify(|w| w.set_circ(false));
            }
        }
    }

    fn poll_stop(&self) -> Poll<()> {
        compiler_fence(Ordering::SeqCst);

        if !self.is_running() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: Peri<'a, AnyChannel>,
}

impl<'a> Transfer<'a> {
    /// Create a new read DMA transfer (peripheral to memory).
    pub unsafe fn new_read<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    /// Create a new read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn new_read_raw<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        peri_addr: *mut W,
        buf: *mut [W],
        options: TransferOptions,
    ) -> Self {
        let (ptr, len) = super::slice_ptr_parts_mut(buf);
        assert!(len > 0 && len <= 0xFFFF);

        Self::new_inner(
            channel.into(),
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            ptr as *mut u32,
            len,
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral).
    pub unsafe fn new_write<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        buf: &'a [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    /// Create a new write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn new_write_raw<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        buf: *const [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        let (ptr, len) = super::slice_ptr_parts(buf);
        assert!(len > 0 && len <= 0xFFFF);

        Self::new_inner(
            channel.into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            ptr as *mut u32,
            len,
            true,
            W::size(),
            options,
        )
    }

    /// Create a new write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn new_write_repeated<W: Word>(
        channel: Peri<'a, impl Channel>,
        request: Request,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        Self::new_inner(
            channel.into(),
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            false,
            W::size(),
            options,
        )
    }

    unsafe fn new_inner(
        channel: Peri<'a, AnyChannel>,
        _request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        options: TransferOptions,
    ) -> Self {
        channel.configure(
            _request, dir, peri_addr, mem_addr, mem_len, incr_mem, data_size, options,
        );
        channel.start();

        Self { channel }
    }

    /// Request the transfer to stop.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u16 {
        self.channel.get_remaining_transfers()
    }

    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state: &ChannelState = &STATE[self.channel.id as usize];

        state.waker.register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

// ==============================

struct DmaCtrlImpl<'a>(Peri<'a, AnyChannel>);

impl<'a> DmaCtrl for DmaCtrlImpl<'a> {
    fn get_remaining_transfers(&self) -> usize {
        self.0.get_remaining_transfers() as _
    }

    fn get_complete_count(&self) -> usize {
        STATE[self.0.id as usize].complete_count.load(Ordering::Acquire)
    }

    fn reset_complete_count(&mut self) -> usize {
        let state = &STATE[self.0.id as usize];
        critical_section::with(|_| {
            let old_val = state.complete_count.load(Ordering::Acquire);
            state.complete_count.store(0, Ordering::Release);
            old_val
        })
    }

    fn set_waker(&mut self, waker: &Waker) {
        STATE[self.0.id as usize].waker.register(waker);
    }
}

/// Ringbuffer for receiving data using DMA circular mode.
pub struct ReadableRingBuffer<'a, W: Word> {
    channel: Peri<'a, AnyChannel>,
    ringbuf: ReadableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> ReadableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: Peri<'a, impl Channel>,
        _request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        let channel: Peri<'a, AnyChannel> = channel.into();

        let buffer_ptr = buffer.as_mut_ptr();
        let len = buffer.len();
        let dir = Dir::PeripheralToMemory;
        let data_size = W::size();

        options.complete_transfer_ir = true;
        options.circular = true;

        channel.configure(
            _request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            true,
            data_size,
            options,
        );

        Self {
            channel,
            ringbuf: ReadableDmaRingBuffer::new(buffer),
        }
    }

    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start()
    }

    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf.clear(&mut DmaCtrlImpl(self.channel.reborrow()));
    }

    /// Read elements from the ring buffer
    /// Return a tuple of the length read and the length remaining in the buffer
    /// If not all of the elements were read, then there will be some elements in the buffer remaining
    /// The length remaining is the capacity, ring_buf.len(), less the elements remaining after the read
    /// OverrunError is returned if the portion to be read was overwritten by the DMA controller.
    pub fn read(&mut self, buf: &mut [W]) -> Result<(usize, usize), OverrunError> {
        self.ringbuf.read(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
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
    pub async fn read_exact(&mut self, buffer: &mut [W]) -> Result<usize, OverrunError> {
        self.ringbuf
            .read_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }

    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }

    /// Request DMA to stop.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Stop the DMA transfer and await until the buffer is full.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when the buffer is full.
    ///
    /// This is designed to be used with streaming input data such as the
    /// I2S/SAI or ADC.
    ///
    /// When using the UART, you probably want `request_stop()`.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        //wait until cr.susp reads as true
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}

impl<'a, W: Word> Drop for ReadableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

/// Ringbuffer for writing data using DMA circular mode.
pub struct WritableRingBuffer<'a, W: Word> {
    channel: Peri<'a, AnyChannel>,
    ringbuf: WritableDmaRingBuffer<'a, W>,
}

impl<'a, W: Word> WritableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: Peri<'a, impl Channel>,
        _request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        let channel: Peri<'a, AnyChannel> = channel.into();

        let len = buffer.len();
        let dir = Dir::MemoryToPeripheral;
        let data_size = W::size();
        let buffer_ptr = buffer.as_mut_ptr();

        options.complete_transfer_ir = true;
        options.circular = true;

        channel.configure(
            _request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            true,
            data_size,
            options,
        );

        Self {
            channel,
            ringbuf: WritableDmaRingBuffer::new(buffer),
        }
    }

    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start()
    }

    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf.clear(&mut DmaCtrlImpl(self.channel.reborrow()));
    }

    /// Write elements directly to the raw buffer.
    /// This can be used to fill the buffer before starting the DMA transfer.
    #[allow(dead_code)]
    pub fn write_immediate(&mut self, buf: &[W]) -> Result<(usize, usize), OverrunError> {
        self.ringbuf.write_immediate(buf)
    }

    /// Write elements from the ring buffer
    /// Return a tuple of the length written and the length remaining in the buffer
    pub fn write(&mut self, buf: &[W]) -> Result<(usize, usize), OverrunError> {
        self.ringbuf.write(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }

    /// Write an exact number of elements to the ringbuffer.
    pub async fn write_exact(&mut self, buffer: &[W]) -> Result<usize, OverrunError> {
        self.ringbuf
            .write_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }

    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }

    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }

    /// Request DMA to stop.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_stop(&mut self) {
        self.channel.request_stop()
    }

    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_stop`](Self::request_stop).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }

    /// Stop the DMA transfer and await until the buffer is empty.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when all available data has been written.
    ///
    /// This is designed to be used with streaming output data such as the
    /// I2S/SAI or DAC.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        //wait until cr.susp reads as true
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}

impl<'a, W: Word> Drop for WritableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}
