//! SPI, Serial Peripheral Interface
//!
//! Capabilities:
//!
//! - Supports full-duplex synchronous serial mode
//! - Supports single-wire half-duplex mode
//! - Supports master and slave modes, multiple slave modes
//! - Supports 8-bit or 16-bit data structures
//! - The highest clock frequency supports up to half of F_HCLK
//! - Data order supports MSB or LSB first (CH32V003 supports MSB first only)
//! - Supports hardware or software control of NSS pin
//! - Transmission and reception support hardware CRC check
//! - Transmission and reception buffers support DMA transfer
//! - Supports changing clock phase and polarity

use core::marker::PhantomData;
use core::ptr;

use embassy_futures::join::join;
use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};
use pac::spi::vals::BaudRate;
use pac::spi::Spi as Regs;

use crate::dma::{slice_ptr_parts, word, ChannelAndRequest};
use crate::gpio::AFType;
use crate::gpio::{AnyPin, Pull, Speed};
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::time::Hertz;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef};

/// SPI Error
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid framing.
    Framing,
    /// CRC error (only if hardware CRC checking is enabled).
    Crc,
    /// Mode fault
    ModeFault,
    /// Overrun.
    Overrun,
}

#[derive(Copy, Clone)]
pub enum BitOrder {
    // CH32V003 supports MSB first only
    #[cfg(not(spi_v0))]
    LsbFirst,
    MsbFirst,
}

#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    pub mode: Mode,
    pub bit_order: BitOrder,
    pub frequency: Hertz,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::hz(1_000_000),
        }
    }
}

impl Config {
    // CPHA
    fn raw_phase(&self) -> bool {
        match self.mode.phase {
            Phase::CaptureOnSecondTransition => true,
            Phase::CaptureOnFirstTransition => false,
        }
    }

    // CPOL
    fn raw_polarity(&self) -> bool {
        match self.mode.polarity {
            Polarity::IdleLow => false,
            Polarity::IdleHigh => true,
        }
    }

    fn lsb_first(&self) -> bool {
        match self.bit_order {
            #[cfg(not(spi_v0))]
            BitOrder::LsbFirst => true,
            BitOrder::MsbFirst => false,
        }
    }

    fn from_cfgr(cfgr: &pac::spi::regs::Ctlr1, bus_clk: Hertz) -> Self {
        let polarity = if cfgr.cpol() {
            Polarity::IdleHigh
        } else {
            Polarity::IdleLow
        };
        let phase = if cfgr.cpha() {
            Phase::CaptureOnSecondTransition
        } else {
            Phase::CaptureOnFirstTransition
        };

        let bit_order = match cfgr.lsbfirst() {
            #[cfg(spi_v0)]
            _ => BitOrder::MsbFirst,
            #[cfg(not(spi_v0))]
            true => BitOrder::LsbFirst,
            #[cfg(not(spi_v0))]
            false => BitOrder::MsbFirst,
        };

        let mode = Mode { polarity, phase };

        let spi_freq = match cfgr.br() {
            BaudRate::DIV_2 => bus_clk / 2_u32,
            BaudRate::DIV_4 => bus_clk / 4_u32,
            BaudRate::DIV_8 => bus_clk / 8_u32,
            BaudRate::DIV_16 => bus_clk / 16_u32,
            BaudRate::DIV_32 => bus_clk / 32_u32,
            BaudRate::DIV_64 => bus_clk / 64_u32,
            BaudRate::DIV_128 => bus_clk / 128_u32,
            BaudRate::DIV_256 => bus_clk / 256_u32,
        };

        Self {
            mode,
            bit_order,
            frequency: spi_freq,
        }
    }
}

/// SPI driver.
pub struct Spi<'d, T: Instance, M: PeriMode> {
    _peri: PeripheralRef<'d, T>,
    sck: Option<PeripheralRef<'d, AnyPin>>,
    mosi: Option<PeripheralRef<'d, AnyPin>>,
    miso: Option<PeripheralRef<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _phantom: PhantomData<M>,
    current_word_size: word_impl::Config,
}

impl<'d, T: Instance, M: PeriMode> Spi<'d, T, M> {
    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        sck: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Self {
        into_ref!(peri);

        let regs = T::REGS;

        let div = calculate_baud_rate(T::frequency().0, config.frequency.0);

        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        T::enable_and_reset();

        // high speed mode
        if config.frequency.0 >= 36_000_000 && div == BaudRate::DIV_2 && miso.is_some() {
            regs.hscr().write(|w| w.set_hsrxen(true));
        }

        regs.ctlr2().modify(|w| w.set_ssoe(false));
        regs.ctlr1().modify(|w| {
            w.set_cpol(cpol);
            w.set_cpha(cpha);
            w.set_mstr(true); // master
            w.set_br(div);
            w.set_spe(true);
            w.set_lsbfirst(config.lsb_first());
            w.set_ssi(true);
            w.set_ssm(true);
            w.set_crcen(false);
            w.set_bidimode(false); // undirectional
            w.set_rxonly(mosi.is_none());
            w.set_dff(false); // u8
        });

        Self {
            _peri: peri,
            sck,
            mosi,
            miso,
            tx_dma,
            rx_dma,
            current_word_size: <u8 as SealedWord>::CONFIG,
            _phantom: PhantomData,
        }
    }

    /// Reconfigure the SPI peripheral.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ()> {
        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        let lsbfirst = config.lsb_first();

        let br = calculate_baud_rate(T::frequency().0, config.frequency.0);

        T::REGS.ctlr1().modify(|w| {
            w.set_cpol(cpol);
            w.set_cpha(cpha);
            w.set_br(br);
            w.set_lsbfirst(lsbfirst);
        });

        Ok(())
    }

    /// Get current SPI configuration. Useful for get the current baudrate.
    pub fn get_current_config(&self) -> Config {
        let bus_freq = T::frequency();

        Config::from_cfgr(&T::REGS.ctlr1().read(), bus_freq)
    }

    fn set_word_size(&mut self, config: word_impl::Config) {
        if self.current_word_size == config {
            return;
        }
        T::REGS.ctlr1().modify(|w| {
            w.set_dff(config == <u16 as SealedWord>::CONFIG);
        });
        self.current_word_size = config;
    }

    // blocking functions

    /// Blocking write.
    pub fn blocking_write<W: Word>(&mut self, words: &[W]) -> Result<(), Error> {
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(T::REGS);
        self.set_word_size(W::CONFIG);
        for word in words.iter() {
            let _ = transfer_word(&T::REGS, *word)?;
        }
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(T::REGS);
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(&T::REGS, W::default())?;
        }
        Ok(())
    }

    /// Blocking in-place bidirectional transfer.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub fn blocking_transfer_in_place<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(T::REGS);
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(&T::REGS, *word)?;
        }
        Ok(())
    }

    /// Blocking bidirectional transfer.
    ///
    /// This transfers both buffers at the same time, so it is NOT equivalent to `write` followed by `read`.
    ///
    /// The transfer runs for `max(read.len(), write.len())` bytes. If `read` is shorter extra bytes are ignored.
    /// If `write` is shorter it is padded with zero bytes.
    pub fn blocking_transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(T::REGS);
        self.set_word_size(W::CONFIG);
        let len = read.len().max(write.len());
        for i in 0..len {
            let wb = write.get(i).copied().unwrap_or_default();
            let rb = transfer_word(&T::REGS, wb)?;
            if let Some(r) = read.get_mut(i) {
                *r = rb;
            }
        }
        Ok(())
    }
}

impl<'d, T: Instance> Spi<'d, T, Blocking> {
    /// Create a new SPI driver.
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_blocking_rxonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            None,
            Some(miso.map_into()),
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_blocking_txonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode, without SCK pin.
    ///
    /// This can be useful for bit-banging non-SPI protocols.
    pub fn new_blocking_txonly_nosck<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(mosi);

        T::set_remap(REMAP);

        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner(peri, None, Some(mosi.map_into()), None, None, None, config)
    }
}

impl<'d, T: Instance> Spi<'d, T, Async> {
    /// Create a new SPI driver.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            None,
            Some(miso.map_into()),
            None,
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            None,
            new_dma!(tx_dma),
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode, without SCK pin.
    ///
    /// This can be useful for bit-banging non-SPI protocols.
    pub fn new_txonly_nosck<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(mosi);

        T::set_remap(REMAP);

        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner(peri, None, Some(mosi.map_into()), None, new_dma!(tx_dma), None, config)
    }

    /// SPI write, using DMA.
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        T::REGS.ctlr1().modify(|w| {
            w.set_spe(false);
        });

        let tx_dst = T::REGS.datar().as_ptr();
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write(data, tx_dst as *mut _, Default::default())
        };

        T::REGS.ctlr2().modify(|w| w.set_txdmaen(true)); // set txdma en
        T::REGS.ctlr1().modify(|w| {
            w.set_spe(true);
        });

        tx_f.await;

        finish_dma(T::REGS);

        Ok(())
    }

    /// SPI read, using DMA.
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        T::REGS.ctlr1().modify(|w| {
            w.set_spe(false);
        });

        flush_rx_fifo(T::REGS);

        T::REGS.ctlr2().modify(|w| w.set_rxdmaen(true)); // set rxdma en

        let clock_byte_count = data.len();

        let rx_src = T::REGS.datar().as_ptr() as *mut _;
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read(rx_src, data, Default::default()) };

        let tx_dst = T::REGS.datar().as_ptr() as *mut _;
        let clock_byte = 0x00u8;
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_repeated(&clock_byte, clock_byte_count, tx_dst, Default::default())
        };

        T::REGS.ctlr2().modify(|w| w.set_txdmaen(true));

        T::REGS.ctlr1().modify(|w| {
            w.set_spe(true);
        });

        join(tx_f, rx_f).await;

        finish_dma(T::REGS);

        Ok(())
    }

    async fn transfer_inner<W: Word>(&mut self, read: *mut [W], write: *const [W]) -> Result<(), Error> {
        let (_, rx_len) = slice_ptr_parts(read);
        let (_, tx_len) = slice_ptr_parts(write);
        assert_eq!(rx_len, tx_len);
        if rx_len == 0 {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        T::REGS.ctlr1().modify(|w| {
            w.set_spe(false);
        });

        // SPIv3 clears rxfifo on SPE=0
        flush_rx_fifo(T::REGS);

        T::REGS.ctlr2().modify(|w| w.set_rxdmaen(true));

        let rx_src = T::REGS.datar().as_ptr() as *mut _;
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read_raw(rx_src, read, Default::default()) };

        let tx_dst = T::REGS.datar().as_ptr() as *mut _;
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_raw(write, tx_dst, Default::default())
        };

        T::REGS.ctlr2().modify(|w| w.set_txdmaen(true));
        T::REGS.ctlr1().modify(|w| {
            w.set_spe(true);
        });

        join(tx_f, rx_f).await;

        finish_dma(T::REGS);

        Ok(())
    }

    /// Bidirectional transfer, using DMA.
    ///
    /// This transfers both buffers at the same time, so it is NOT equivalent to `write` followed by `read`.
    ///
    /// The transfer runs for `max(read.len(), write.len())` bytes. If `read` is shorter extra bytes are ignored.
    /// If `write` is shorter it is padded with zero bytes.
    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        self.transfer_inner(read, write).await
    }

    /// In-place bidirectional transfer, using DMA.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub async fn transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        self.transfer_inner(data, data).await
    }
}

impl<'d, T: Instance, M: PeriMode> Drop for Spi<'d, T, M> {
    fn drop(&mut self) {
        use crate::gpio::SealedPin;

        self.sck.as_ref().map(|x| x.set_as_disconnected());
        self.mosi.as_ref().map(|x| x.set_as_disconnected());
        self.miso.as_ref().map(|x| x.set_as_disconnected());

        T::disable();
    }
}

// Get CTRL1.BR
#[inline]
fn calculate_baud_rate(hclk: u32, clk: u32) -> BaudRate {
    // only div2, div4 to div256 are valid
    let div = hclk / clk;

    match div {
        1..=2 => BaudRate::DIV_2,
        3..=4 => BaudRate::DIV_4,
        5..=8 => BaudRate::DIV_8,
        9..=16 => BaudRate::DIV_16,
        17..=32 => BaudRate::DIV_32,
        33..=64 => BaudRate::DIV_64,
        65..=128 => BaudRate::DIV_128,
        129..=256 => BaudRate::DIV_256,
        // fallback to the lowest baudrate
        0 => BaudRate::DIV_2,
        _ => BaudRate::DIV_256,
    }
}

fn check_error_flags(sr: &pac::spi::regs::Statr) -> Result<(), Error> {
    if sr.ovr() {
        return Err(Error::Overrun);
    }
    if sr.modf() {
        return Err(Error::ModeFault);
    }
    if sr.crcerr() {
        return Err(Error::Crc);
    }

    Ok(())
}

fn spin_until_tx_ready(regs: &pac::spi::Spi) -> Result<(), Error> {
    loop {
        let sr = regs.statr().read();

        check_error_flags(&sr)?;

        if sr.txe() {
            return Ok(());
        }
    }
}

fn spin_until_rx_ready(regs: &pac::spi::Spi) -> Result<(), Error> {
    loop {
        let sr = regs.statr().read();

        check_error_flags(&sr)?;

        if sr.rxne() {
            return Ok(());
        }
    }
}

fn flush_rx_fifo(regs: pac::spi::Spi) {
    while regs.statr().read().rxne() {
        let _ = regs.datar().read();
    }
}

fn finish_dma(regs: Regs) {
    while regs.statr().read().bsy() {}

    // Disable the spi peripheral
    regs.ctlr1().modify(|w| {
        w.set_spe(false);
    });

    // The peripheral automatically disables the DMA stream on completion without error,
    // but it does not clear the RXDMAEN/TXDMAEN flag in CR2.
    regs.ctlr2().modify(|reg| {
        reg.set_txdmaen(false);
        reg.set_rxdmaen(false);
    });
}

fn transfer_word<W: Word>(regs: &pac::spi::Spi, tx_word: W) -> Result<W, Error> {
    spin_until_tx_ready(regs)?;

    unsafe {
        ptr::write_volatile(regs.datar().as_ptr() as _, tx_word);
    }

    spin_until_rx_ready(regs)?;

    let rx_word = unsafe { ptr::read_volatile(regs.datar().as_ptr() as _) };
    Ok(rx_word)
}

impl<'d, T: Instance, M: PeriMode> embedded_hal::spi::ErrorType for Spi<'d, T, M> {
    type Error = Error;
}

impl<'d, T: Instance, W: Word, M: PeriMode> embedded_hal::spi::SpiBus<W> for Spi<'d, T, M> {
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.blocking_read(words)
    }

    fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.blocking_write(words)
    }

    fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.blocking_transfer(read, write)
    }

    fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.blocking_transfer_in_place(words)
    }
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match *self {
            Self::Framing => embedded_hal::spi::ErrorKind::FrameFormat,
            Self::Crc => embedded_hal::spi::ErrorKind::Other,
            Self::ModeFault => embedded_hal::spi::ErrorKind::ModeFault,
            Self::Overrun => embedded_hal::spi::ErrorKind::Overrun,
        }
    }
}

trait SealedWord {
    const CONFIG: word_impl::Config;
}

/// Word sizes usable for SPI.
#[allow(private_bounds)]
pub trait Word: word::Word + SealedWord {}

macro_rules! impl_word {
    ($T:ty, $config:expr) => {
        impl SealedWord for $T {
            const CONFIG: Config = $config;
        }
        impl Word for $T {}
    };
}

mod word_impl {
    use super::*;

    pub type Config = u8;

    impl_word!(u8, 0);
    impl_word!(u16, 1);
}

trait SealedInstance {
    const REGS: Regs;
}

/// SPI instance trait.
#[allow(private_bounds)]
pub trait Instance:
    Peripheral<P = Self> + crate::peripheral::RccPeripheral + crate::peripheral::RemapPeripheral + SealedInstance
{
}

foreach_peripheral!(
    (spi, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            const REGS: Regs = crate::pac::$inst;
        }

        impl Instance for peripherals::$inst {}
    };
);

pin_trait!(SckPin, Instance);
pin_trait!(MosiPin, Instance);
pin_trait!(MisoPin, Instance);
pin_trait!(CsPin, Instance);

// I2S pins
pin_trait!(MckPin, Instance);
pin_trait!(CkPin, Instance);

dma_trait!(RxDma, Instance);
dma_trait!(TxDma, Instance);
