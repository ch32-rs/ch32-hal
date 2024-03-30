//! SPI, Serial Peripheral Interface

/*
Supports full-duplex synchronous serial mode
Supports single-wire half-duplex mode
Supports master and slave modes, multiple slave modes
Supports 8-bit or 16-bit data structures
The highest clock frequency supports up to half of F_HCLK
Data order supports MSB or LSB first
Supports hardware or software control of NSS pin
Transmission and reception support hardware CRC check
Transmission and reception buffers support DMA transfer
Supports changing clock phase and polarity
*/

use core::ptr;

use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};
use pac::spi::vals::BaudRate;

use crate::dma::word;
use crate::gpio::sealed::AFType;
use crate::gpio::{AnyPin, Pull, Speed};
use crate::time::Hertz;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef};

#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Framing,
    Crc,
    ModeFault,
    Overrun,
}

#[derive(Copy, Clone)]
pub enum BitOrder {
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
            BitOrder::LsbFirst => true,
            BitOrder::MsbFirst => false,
        }
    }
}

/// SPI driver.
pub struct Spi<'d, T: Instance, Tx, Rx> {
    _peri: PeripheralRef<'d, T>,
    sck: Option<PeripheralRef<'d, AnyPin>>,
    mosi: Option<PeripheralRef<'d, AnyPin>>,
    miso: Option<PeripheralRef<'d, AnyPin>>,
    txdma: PeripheralRef<'d, Tx>,
    rxdma: PeripheralRef<'d, Rx>,
    current_word_size: word_impl::Config,
}

impl<'d, T: Instance, Tx, Rx> Spi<'d, T, Tx, Rx> {
    /// Create a new SPI driver.
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        txdma: impl Peripheral<P = Tx> + 'd,
        rxdma: impl Peripheral<P = Rx> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner::<REMAP>(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            txdma,
            rxdma,
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T, REMAP>> + 'd,
        txdma: impl Peripheral<P = Tx> + 'd, // TODO remove
        rxdma: impl Peripheral<P = Rx> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(sck, miso);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        miso.set_as_input(Pull::None);

        Self::new_inner::<REMAP>(
            peri,
            Some(sck.map_into()),
            None,
            Some(miso.map_into()),
            txdma,
            rxdma,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T, REMAP>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        txdma: impl Peripheral<P = Tx> + 'd,
        rxdma: impl Peripheral<P = Rx> + 'd, // TODO remove
        config: Config,
    ) -> Self {
        into_ref!(sck, mosi);

        T::set_remap(REMAP);

        sck.set_as_af_output(AFType::OutputPushPull, Speed::High);
        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner::<REMAP>(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            None,
            txdma,
            rxdma,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode, without SCK pin.
    ///
    /// This can be useful for bit-banging non-SPI protocols.
    pub fn new_txonly_nosck<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T, REMAP>> + 'd,
        txdma: impl Peripheral<P = Tx> + 'd,
        rxdma: impl Peripheral<P = Rx> + 'd, // TODO: remove
        config: Config,
    ) -> Self {
        into_ref!(mosi);

        T::set_remap(REMAP);

        mosi.set_as_af_output(AFType::OutputPushPull, Speed::High);

        Self::new_inner::<REMAP>(peri, None, Some(mosi.map_into()), None, txdma, rxdma, config)
    }

    fn new_inner<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        sck: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        txdma: impl Peripheral<P = Tx> + 'd,
        rxdma: impl Peripheral<P = Rx> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, txdma, rxdma);

        let div = calculate_clock_div(crate::rcc::clocks().hclk.0, config.frequency.0);

        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        T::enable_and_reset();

        let regs = T::regs();

        regs.ctlr2().modify(|w| w.set_ssoe(false));
        regs.ctlr1().modify(|w| {
            w.set_cpol(cpol);
            w.set_cpha(cpha);
            w.set_mstr(true);
            w.set_br(div);
            w.set_spe(true);
            w.set_lsbfirst(config.lsb_first());
            w.set_ssi(true);
            w.set_ssm(true);
            w.set_crcen(false);
            w.set_bidimode(false);
            w.set_rxonly(mosi.is_none());
            w.set_dff(false); // u8
        });
        Self {
            _peri: peri,
            sck,
            mosi,
            miso,
            txdma,
            rxdma,
            current_word_size: <u8 as sealed::Word>::CONFIG,
        }
    }

    fn set_word_size(&mut self, config: word_impl::Config) {
        if self.current_word_size != config {
            self.current_word_size = config;
            T::regs().ctlr1().modify(|w| {
                w.set_dff(config == <u16 as sealed::Word>::CONFIG);
            });
        }
    }

    /// Blocking write.
    pub fn blocking_write<W: Word>(&mut self, words: &[W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(&T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter() {
            let _ = transfer_word(&T::regs(), *word)?;
        }
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(&T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(&T::regs(), W::default())?;
        }
        Ok(())
    }

    /// Blocking in-place bidirectional transfer.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub fn blocking_transfer_in_place<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(&T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(&T::regs(), *word)?;
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
        T::regs().ctlr1().modify(|w| w.set_spe(true));
        flush_rx_fifo(&T::regs());
        self.set_word_size(W::CONFIG);
        let len = read.len().max(write.len());
        for i in 0..len {
            let wb = write.get(i).copied().unwrap_or_default();
            let rb = transfer_word(&T::regs(), wb)?;
            if let Some(r) = read.get_mut(i) {
                *r = rb;
            }
        }
        Ok(())
    }
}

// Get CTRL1.BR
#[inline]
fn calculate_clock_div(hclk: u32, clk: u32) -> BaudRate {
    // only div2, div4 to div256 are valid
    let div = hclk / clk;

    match div {
        0 => unreachable!(),
        1..=2 => BaudRate::DIV_2,
        3..=4 => BaudRate::DIV_4,
        5..=8 => BaudRate::DIV_8,
        9..=16 => BaudRate::DIV_16,
        17..=32 => BaudRate::DIV_32,
        33..=64 => BaudRate::DIV_64,
        65..=128 => BaudRate::DIV_128,
        129..=256 => BaudRate::DIV_256,
        _ => unreachable!(),
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

fn flush_rx_fifo(regs: &pac::spi::Spi) {
    while regs.statr().read().rxne() {
        let _ = regs.datar().read();
    }
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

impl<'d, T: Instance, Tx, Rx> embedded_hal::spi::ErrorType for Spi<'d, T, Tx, Rx> {
    type Error = Error;
}

impl<'d, T: Instance, W: Word, Tx, Rx> embedded_hal::spi::SpiBus<W> for Spi<'d, T, Tx, Rx> {
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

pub(crate) mod sealed {
    use super::*;

    pub trait Word {
        const CONFIG: word_impl::Config;
    }
}

/// Word sizes usable for SPI.
pub trait Word: word::Word + sealed::Word {}

macro_rules! impl_word {
    ($T:ty, $config:expr) => {
        impl sealed::Word for $T {
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
    fn regs() -> crate::pac::spi::Spi;
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
            fn regs() -> crate::pac::spi::Spi {
                crate::pac::$inst
            }
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
