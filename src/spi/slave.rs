//! SPI slave streaming over circular DMA, in halves and in place.
//!
//! For a master that clocks continuous bursts (e.g. an FPGA streaming audio frames): RX and
//! TX each run a circular DMA over a buffer of two halves. When RX completes a half, that
//! half holds received words and the same half of the TX buffer has gone out (TX runs a
//! word or two ahead of RX), so the owner can refill it -- it goes out one pass later.
//!
//! Hardware NSS: each burst starts on a word boundary as long as the slave was enabled
//! while NSS was high. [`SpiSlaveRing::start`] waits for a pause between bursts to do that.
//! If the stream ever slips (a mis-aligned word), [`SpiSlaveRing::restart`] resets the
//! peripheral and starts again in a pause.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embedded_hal::spi::Mode;

use super::{BitOrder, CsPin, Instance, MisoPin, MosiPin, RxDma, SckPin, TxDma, Word};
use crate::dma::{CircularTransfer, Priority, Request, RingPositionReader};
use crate::gpio::{AfType, AnyPin, OutputType, Pull, SealedPin, Speed};
use crate::Peri;

/// Slave configuration. The clock comes from the master.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct SlaveConfig {
    pub mode: Mode,
    pub bit_order: BitOrder,
    /// DMA priority of both channels
    pub priority: Priority,
}

impl Default for SlaveConfig {
    fn default() -> Self {
        Self {
            mode: embedded_hal::spi::MODE_0,
            bit_order: BitOrder::MsbFirst,
            priority: Priority::VeryHigh,
        }
    }
}

/// Slave streaming errors. After any of them, [`SpiSlaveRing::restart`].
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlaveError {
    /// The SPI overran (a word arrived before the last one was taken). It stops receiving
    /// until reset.
    Overrun,
    /// A DMA transfer error stopped a channel.
    Dma,
    /// NSS stayed low: no pause between bursts to start in.
    NoPause,
    /// The RX position went backwards: the DMA interrupt was held off for more than a pass
    /// and missed counting it, so the half index can't be trusted. Run the DMA interrupt
    /// above code that can take that long.
    LostLap,
}

/// A completed half.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Half {
    /// 0 or 1: read [`SpiSlaveRing::rx_half`], refill [`SpiSlaveRing::tx_half_mut`]
    pub index: usize,
    /// halves completed since the last [`SpiSlaveRing::next_half`] that weren't returned (the
    /// owner was late; their RX data is lost and their TX half went out again unchanged)
    pub missed: u32,
}

/// Spins on NSS while waiting for a pause: generously more than one frame period at the
/// slowest rate a master is expected to use.
const NSS_SPIN: u32 = 100_000;

/// SPI slave with RX and TX on circular DMA (see the module docs).
pub struct SpiSlaveRing<'d, T: Instance, W: Word> {
    _peri: Peri<'d, T>,
    nss: Peri<'d, AnyPin>,
    _sck: Peri<'d, AnyPin>,
    _mosi: Peri<'d, AnyPin>,
    _miso: Peri<'d, AnyPin>,
    rx: CircularTransfer<'d>,
    tx: CircularTransfer<'d>,
    rx_request: Request,
    tx_request: Request,
    rx_buf: *mut [W],
    tx_buf: *mut [W],
    half_len: usize,
    delivered: u32,
    config: SlaveConfig,
    _phantom: PhantomData<&'d mut [W]>,
}

// SAFETY: the buffers are borrowed exclusively for 'd; the raw pointers only avoid aliasing
// the halves the DMA is using
unsafe impl<'d, T: Instance, W: Word + Send> Send for SpiSlaveRing<'d, T, W> {}

impl<'d, T: Instance, W: Word> SpiSlaveRing<'d, T, W> {
    /// `rx_buf` and `tx_buf` are the same length; each pass uses 2 x `half_len` words of
    /// them (all of them by default). Nothing runs until [`start`](Self::start).
    #[allow(clippy::too_many_arguments)]
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        nss: Peri<'d, if_remap!(impl CsPin<T, REMAP>)>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_buf: &'d mut [W],
        tx_buf: &'d mut [W],
        config: SlaveConfig,
    ) -> Self {
        assert!(rx_buf.len() == tx_buf.len() && rx_buf.len() >= 2 && rx_buf.len() <= 0xFFFF);
        apply_remap!();

        // master-driven inputs; NSS pulled high so an unpowered master reads as a pause
        set_as_af!(sck, AfType::input(Pull::None));
        set_as_af!(mosi, AfType::input(Pull::None));
        set_as_af!(nss, AfType::input(Pull::Up));
        set_as_af!(miso, AfType::output(OutputType::PushPull, Speed::High));

        let half_len = rx_buf.len() / 2;
        Self {
            _peri: peri,
            nss: nss.into(),
            _sck: sck.into(),
            _mosi: mosi.into(),
            _miso: miso.into(),
            rx_request: rx_dma.request(),
            tx_request: tx_dma.request(),
            rx: CircularTransfer::new(rx_dma),
            tx: CircularTransfer::new(tx_dma),
            rx_buf,
            tx_buf,
            half_len,
            delivered: 0,
            config,
            _phantom: PhantomData,
        }
    }

    /// Words per half; takes effect at the next [`start`](Self::start) /
    /// [`restart`](Self::restart).
    pub fn set_half_len(&mut self, words: usize) {
        assert!(words > 0 && 2 * words <= self.rx_buf.len());
        self.half_len = words;
    }

    /// Words per half in use.
    pub fn half_len(&self) -> usize {
        self.half_len
    }

    /// Set up SPI and both DMA channels and enable the slave in a pause between bursts
    /// (NSS high). TX starts with whatever is in the TX buffer.
    pub fn start(&mut self) -> Result<(), SlaveError> {
        let regs = T::REGS;
        T::enable_and_reset();
        regs.ctlr1().write(|w| {
            w.set_cpha(self.config.mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition);
            w.set_cpol(self.config.mode.polarity == embedded_hal::spi::Polarity::IdleHigh);
            w.set_mstr(false);
            w.set_lsbfirst(self.config.bit_order_lsb());
            w.set_ssm(false); // hardware NSS
            w.set_dff(W::size() == crate::dma::word::WordSize::TwoBytes);
            w.set_spe(false);
        });

        let n = 2 * self.half_len;
        let dr = regs.datar().as_ptr() as *mut W;
        // SAFETY: the buffers are ours for 'd; the DMA is stopped before they're reconfigured
        unsafe {
            let rx = core::ptr::slice_from_raw_parts_mut(self.rx_buf as *mut W, n);
            let tx = core::ptr::slice_from_raw_parts(self.tx_buf as *const W, n);
            self.rx
                .configure_read(self.rx_request, dr, rx, self.config.priority, true);
            self.tx
                .configure_write(self.tx_request, tx, dr, self.config.priority, false);
        }
        self.delivered = 0;

        // RM order: RX DMA request, channels, TX DMA request (preloads DR), then SPE
        regs.ctlr2().modify(|w| w.set_rxdmaen(true));
        self.rx.start();
        self.tx.start();
        regs.ctlr2().modify(|w| w.set_txdmaen(true));

        self.enable_in_pause()
    }

    /// Stop both channels and the slave.
    pub fn stop(&mut self) {
        let regs = T::REGS;
        regs.ctlr1().modify(|w| w.set_spe(false));
        self.rx.stop();
        self.tx.stop();
        regs.ctlr2().modify(|w| {
            w.set_rxdmaen(false);
            w.set_txdmaen(false);
        });
    }

    /// After a slip or an error: stop, reset the peripheral (clears the shift register,
    /// the data register and OVR) and start again in a pause.
    pub fn restart(&mut self) -> Result<(), SlaveError> {
        self.stop();
        self.start()
    }

    /// Wait for the next completed RX half.
    pub async fn next_half(&mut self) -> Result<Half, SlaveError> {
        poll_fn(|cx| {
            self.rx.register_waker(cx.waker());
            if let Some(e) = self.error() {
                return Poll::Ready(Err(e));
            }
            let pos = self.rx.position();
            let done = pos
                .laps
                .wrapping_mul(2)
                .wrapping_add((pos.index / self.half_len) as u32);
            let new = done.wrapping_sub(self.delivered);
            if new == 0 {
                return Poll::Pending;
            }
            if new > u32::MAX / 2 {
                return Poll::Ready(Err(SlaveError::LostLap));
            }
            self.delivered = done;
            Poll::Ready(Ok(Half {
                index: (done.wrapping_sub(1) % 2) as usize,
                missed: new - 1,
            }))
        })
        .await
    }

    fn error(&self) -> Option<SlaveError> {
        if self.rx.take_error() | self.tx.take_error() {
            return Some(SlaveError::Dma);
        }
        if T::REGS.statr().read().ovr() {
            return Some(SlaveError::Overrun);
        }
        None
    }

    /// Received words of half `index` (complete once [`next_half`](Self::next_half)
    /// returned it, until the DMA comes back to it one pass later).
    pub fn rx_half(&self, index: usize) -> &[W] {
        assert!(index < 2);
        // SAFETY: inside the buffer; the DMA is writing the other half
        unsafe { core::slice::from_raw_parts((self.rx_buf as *const W).add(index * self.half_len), self.half_len) }
    }

    /// TX half `index`, free to refill after [`next_half`](Self::next_half) returned it.
    pub fn tx_half_mut(&mut self, index: usize) -> &mut [W] {
        assert!(index < 2);
        // SAFETY: inside the buffer; the DMA is reading the other half
        unsafe { core::slice::from_raw_parts_mut((self.tx_buf as *mut W).add(index * self.half_len), self.half_len) }
    }

    /// [`rx_half`](Self::rx_half) and [`tx_half_mut`](Self::tx_half_mut) of the same half,
    /// together.
    pub fn half_mut(&mut self, index: usize) -> (&[W], &mut [W]) {
        assert!(index < 2);
        // SAFETY: as in `rx_half` and `tx_half_mut`; RX and TX are separate buffers
        unsafe {
            (
                core::slice::from_raw_parts((self.rx_buf as *const W).add(index * self.half_len), self.half_len),
                core::slice::from_raw_parts_mut((self.tx_buf as *mut W).add(index * self.half_len), self.half_len),
            )
        }
    }

    /// Reads the RX position from anywhere (e.g. a USB SOF interrupt measuring the
    /// master's clock). Valid until the next [`start`](Self::start) / [`restart`](Self::restart),
    /// which reset it.
    pub fn rx_position(&self) -> RingPositionReader {
        self.rx.reader()
    }

    fn nss_high(&self) -> bool {
        self.nss.block().indr().read().idr(self.nss._pin() as usize)
    }

    /// Set SPE right after NSS rises, so the slave starts at the next burst's first bit.
    /// A bus that stays idle (NSS high) counts as a pause.
    fn enable_in_pause(&mut self) -> Result<(), SlaveError> {
        let regs = T::REGS;
        let mut n = 0;
        while self.nss_high() {
            n += 1;
            if n == NSS_SPIN {
                regs.ctlr1().modify(|w| w.set_spe(true));
                return Ok(());
            }
        }
        // in a burst: catch its end with nothing in between (a burst is a few us)
        critical_section::with(|_| {
            let mut n = 0;
            while !self.nss_high() {
                n += 1;
                if n == NSS_SPIN {
                    return Err(SlaveError::NoPause);
                }
            }
            regs.ctlr1().modify(|w| w.set_spe(true));
            Ok(())
        })
    }
}

impl<'d, T: Instance, W: Word> Drop for SpiSlaveRing<'d, T, W> {
    fn drop(&mut self) {
        self.stop();
        T::disable();
    }
}

impl SlaveConfig {
    fn bit_order_lsb(&self) -> bool {
        match self.bit_order {
            #[cfg(not(spi_v0))]
            BitOrder::LsbFirst => true,
            BitOrder::MsbFirst => false,
        }
    }
}
