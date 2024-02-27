//! SPI

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

Max clock: 24MHz(div2), to hclk/256
*/

use core::ptr;

use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};
use fugit::HertzU32 as Hertz;

use crate::dma::word;
use crate::gpio::{AnyPin, Pull};
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
            frequency: Hertz::from_raw(1_000_000),
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

        sck.set_as_af_output();
        //sck.set_speed(crate::gpio::Speed::VeryHigh);
        mosi.set_as_af_output();
        //mosi.set_speed(crate::gpio::Speed::VeryHigh);
        miso.set_as_input(Pull::None);
        // miso.set_speed(crate::gpio::Speed::VeryHigh);

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

        sck.set_as_af_output();
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

        sck.set_as_af_output();
        mosi.set_as_af_output();

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

        mosi.set_as_af_output();

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

        let div = calculate_clock_div(crate::rcc::clocks().hclk.to_Hz(), config.frequency.to_Hz());

        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        T::enable_and_reset();

        let regs = T::regs();

        regs.ctlr2().modify(|_, w| w.ssoe().clear_bit());
        regs.ctlr1().modify(|_, w| {
            w.cpol()
                .bit(cpol)
                .cpha()
                .bit(cpha)
                .mstr()
                .set_bit() // master
                .br()
                .variant(div)
                .spe()
                .set_bit() // enable
                .lsbfirst()
                .bit(config.lsb_first())
                .ssi()
                .set_bit() // software ss
                .ssm()
                .set_bit() // soft ss
                .crcen()
                .clear_bit() // crc disabled
                .bidimode()
                .clear_bit()
                .rxonly()
                .bit(mosi.is_none())
                .dff()
                .bit(false) // u8
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
            T::regs().ctlr1().modify(|_, w| {
                w.dff()
                    .bit(config == <u16 as sealed::Word>::CONFIG)
                    .bidimode()
                    .bit(config == <u16 as sealed::Word>::CONFIG)
            });
        }
    }

    /// Blocking write.
    pub fn blocking_write<W: Word>(&mut self, words: &[W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|_, w| w.spe().set_bit());
        flush_rx_fifo(T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter() {
            let _ = transfer_word(T::regs(), *word)?;
        }
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|_, w| w.spe().set_bit());
        flush_rx_fifo(T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(T::regs(), W::default())?;
        }
        Ok(())
    }

    /// Blocking in-place bidirectional transfer.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub fn blocking_transfer_in_place<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        T::regs().ctlr1().modify(|_, w| w.spe().bit(true));
        flush_rx_fifo(T::regs());
        self.set_word_size(W::CONFIG);
        for word in words.iter_mut() {
            *word = transfer_word(T::regs(), *word)?;
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
        T::regs().ctlr1().modify(|_, w| w.spe().bit(true));
        flush_rx_fifo(T::regs());
        self.set_word_size(W::CONFIG);
        let len = read.len().max(write.len());
        for i in 0..len {
            let wb = write.get(i).copied().unwrap_or_default();
            let rb = transfer_word(T::regs(), wb)?;
            if let Some(r) = read.get_mut(i) {
                *r = rb;
            }
        }
        Ok(())
    }
}

// Get CTRL1.BR
#[inline]
fn calculate_clock_div(hclk: u32, clk: u32) -> u8 {
    // only div2, div4 to div256 are valid
    let div = hclk / clk;
    match div {
        0 => unreachable!(),
        1..=2 => 0b000,
        3..=4 => 0b001,
        5..=8 => 0b010,
        9..=16 => 0b011,
        17..=32 => 0b100,
        33..=64 => 0b101,
        65..=128 => 0b110,
        129..=256 => 0b111,
        _ => unreachable!(),
    }
}

fn check_error_flags(sr: u32) -> Result<(), Error> {
    // TODO: actually check the flags
    Ok(())
}

fn spin_until_tx_ready(regs: &'static pac::spi1::RegisterBlock) -> Result<(), Error> {
    loop {
        let sr = regs.statr().read();

        check_error_flags(sr.bits())?;

        if sr.txe().bit_is_set() {
            return Ok(());
        }
    }
}

fn spin_until_rx_ready(regs: &'static pac::spi1::RegisterBlock) -> Result<(), Error> {
    loop {
        let sr = regs.statr().read();

        check_error_flags(sr.bits())?;

        if sr.rxne().bit_is_set() {
            return Ok(());
        }
    }
}

fn flush_rx_fifo(regs: &'static pac::spi1::RegisterBlock) {
    while regs.statr().read().rxne().bit_is_set() {
        let _ = regs.datar().read();
    }
}

fn transfer_word<W: Word>(regs: &'static pac::spi1::RegisterBlock, tx_word: W) -> Result<W, Error> {
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

    pub trait Instance {
        #[inline(always)]
        fn regs() -> &'static pac::spi1::RegisterBlock {
            unsafe { &*pac::SPI1::ptr() }
        }

        fn set_remap(remap: u8) {
            let afio = unsafe { &*pac::AFIO::ptr() };
            let remap = remap == 1;
            afio.pcfr().modify(|_, w| w.spi1rm().variant(remap));
        }

        fn enable_and_reset() {
            let rcc = unsafe { &*pac::RCC::ptr() };
            rcc.apb2pcenr().modify(|_, w| w.spi1en().set_bit());
            rcc.apb2prstr().modify(|_, w| w.spi1rst().set_bit());
            rcc.apb2prstr().modify(|_, w| w.spi1rst().clear_bit());
        }
    }

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

/// SPI instance trait.
pub trait Instance: Peripheral<P = Self> + sealed::Instance {}

impl sealed::Instance for peripherals::SPI1 {}
impl Instance for peripherals::SPI1 {}

impl sealed::Instance for peripherals::SPI2 {}
impl Instance for peripherals::SPI2 {}

impl sealed::Instance for peripherals::SPI3 {}
impl Instance for peripherals::SPI3 {}

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8>: crate::gpio::Pin {}
    };
}

pin_trait!(SckPin, Instance);
pin_trait!(MosiPin, Instance);
pin_trait!(MisoPin, Instance);
pin_trait!(CsPin, Instance);
// dma_trait!(RxDma, Instance);
// dma_trait!(TxDma, Instance);

macro_rules! pin_trait_impl {
    (crate::$mod:ident::$trait:ident, $instance:ident, $pin:ident, $remap:expr) => {
        impl crate::$mod::$trait<crate::peripherals::$instance, $remap> for crate::peripherals::$pin {}
    };
}

pin_trait_impl!(crate::spi::CsPin, SPI1, PA4, 0);
pin_trait_impl!(crate::spi::SckPin, SPI1, PA5, 0);
pin_trait_impl!(crate::spi::MisoPin, SPI1, PA6, 0);
pin_trait_impl!(crate::spi::MosiPin, SPI1, PA7, 0);

pin_trait_impl!(crate::spi::CsPin, SPI1, PA15, 1);
pin_trait_impl!(crate::spi::SckPin, SPI1, PB3, 1);
pin_trait_impl!(crate::spi::MisoPin, SPI1, PB4, 1);
pin_trait_impl!(crate::spi::MosiPin, SPI1, PB5, 1);

pin_trait_impl!(crate::spi::CsPin, SPI2, PB12, 0);
pin_trait_impl!(crate::spi::SckPin, SPI2, PB13, 0);
pin_trait_impl!(crate::spi::MisoPin, SPI2, PB14, 0);
pin_trait_impl!(crate::spi::MosiPin, SPI2, PB15, 0);

pin_trait_impl!(crate::spi::CsPin, SPI3, PA15, 0);
pin_trait_impl!(crate::spi::SckPin, SPI3, PB3, 0);
pin_trait_impl!(crate::spi::MisoPin, SPI3, PB4, 0);
pin_trait_impl!(crate::spi::MisoPin, SPI3, PB5, 0);

pin_trait_impl!(crate::spi::CsPin, SPI3, PA4, 1);
pin_trait_impl!(crate::spi::SckPin, SPI3, PC10, 1);
pin_trait_impl!(crate::spi::MisoPin, SPI3, PC11, 1);
pin_trait_impl!(crate::spi::MosiPin, SPI3, PC12, 1);
