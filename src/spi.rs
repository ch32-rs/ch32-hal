//! SPI, Serial Peripheral Interface
//!
//! Capabilities:
//!
//! Master-mode transfers with software NSS and caller-managed chip select.
//! Supports 8/16-bit words, all four clock modes, blocking and DMA transfers,
//! and MSB/LSB first (MSB first only on CH32V003). The maximum serial clock is
//! half the peripheral bus clock.

use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr;
use core::task::Poll;

use embassy_futures::join::join;
use embassy_futures::select::{select, Either};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::spi::{Mode, Phase, Polarity, MODE_0};
use pac::spi::vals::BaudRate;
use pac::spi::Spi as Regs;

use crate::dma::{slice_ptr_parts, word, ChannelAndRequest};
use crate::gpio::{AfType, AnyPin, OutputType, Pull, Speed};
use crate::internal::drop::OnDrop;
use crate::mode::{Async, Blocking, Mode as PeriMode};
use crate::time::Hertz;
use crate::{interrupt, pac, peripherals, Peri};

#[cfg(test)]
#[path = "spi/tests.rs"]
mod tests;

/// SPI error interrupt handler.
pub struct InterruptHandler<T: Instance>(PhantomData<T>);

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        if check_error_flags(&T::REGS.statr().read()).is_err() {
            T::REGS.ctlr2().modify(|w| {
                w.set_errie(false);
                w.set_txdmaen(false);
                w.set_rxdmaen(false);
            });
            T::state().waker.wake();
        }
    }
}

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
    /// Buffer exceeds the DMA transfer counter.
    BufferTooLong,
    /// The constructor did not provide the required pins or DMA channel.
    UnsupportedOperation,
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
    _peri: Peri<'d, T>,
    sck: Option<Peri<'d, AnyPin>>,
    mosi: Option<Peri<'d, AnyPin>>,
    miso: Option<Peri<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _phantom: PhantomData<M>,
    current_word_size: word_impl::Config,
}

impl<'d, T: Instance, M: PeriMode> Spi<'d, T, M> {
    fn new_inner(
        peri: Peri<'d, T>,
        sck: Option<Peri<'d, AnyPin>>,
        mosi: Option<Peri<'d, AnyPin>>,
        miso: Option<Peri<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Self {
        let regs = T::REGS;

        let div = calculate_baud_rate(T::frequency().0, config.frequency.0).expect("unrepresentable SPI frequency");

        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        T::enable_and_reset();

        configure_high_speed(regs, T::frequency().0, div, miso.is_some());

        regs.ctlr2().modify(|w| w.set_ssoe(false));
        regs.ctlr1().modify(|w| {
            w.set_cpol(cpol);
            w.set_cpha(cpha);
            w.set_mstr(true); // master
            w.set_br(div);
            w.set_spe(false);
            w.set_lsbfirst(config.lsb_first());
            w.set_ssi(true);
            w.set_ssm(true);
            w.set_crcen(false);
            w.set_bidimode(false); // undirectional
            w.set_rxonly(false);
            w.set_dff(false); // u8
        });
        #[cfg(spi_v4x7)]
        if tx_dma.is_some() || rx_dma.is_some() {
            use crate::interrupt::typelevel::Interrupt;
            T::Interrupt::unpend();
            unsafe { T::Interrupt::enable() };
        }

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

        let br = calculate_baud_rate(T::frequency().0, config.frequency.0).ok_or(())?;
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.blocking_flush().map_err(|_| ())?;
        T::REGS.ctlr1().modify(|w| w.set_spe(false));
        configure_high_speed(T::REGS, T::frequency().0, br, self.miso.is_some());

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

    fn prepare<W: Word>(&mut self, transmit_only: bool) -> Result<(), Error> {
        self.blocking_flush()?;
        T::REGS.ctlr1().modify(|w| w.set_spe(false));
        flush_rx_fifo(T::REGS);
        self.set_word_size(W::CONFIG);
        T::REGS.ctlr1().modify(|w| {
            w.set_mstr(true);
            w.set_ssm(true);
            w.set_ssi(true);
            w.set_bidimode(transmit_only);
            w.set_bidioe(transmit_only);
            w.set_rxonly(false);
        });
        Ok(())
    }

    /// Wait for the final bit to leave the peripheral.
    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        // A disabled peripheral has no transfer in flight. On CH32V4x7
        // STATR.TXE reads 0 while SPE=0 (measured on CH32V467), so polling it
        // before the first `prepare` enabled SPE would never complete.
        if !T::REGS.ctlr1().read().spe() {
            return Ok(());
        }
        loop {
            let status = T::REGS.statr().read();
            if let Err(error) = check_error_flags(&status) {
                cleanup(T::REGS);
                return Err(error);
            }
            if status.txe() && !status.bsy() {
                return Ok(());
            }
        }
    }

    /// Blocking write.
    pub fn blocking_write<W: Word>(&mut self, words: &[W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }
        if self.mosi.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(true)?;
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        for word in words {
            spin_until_tx_ready(&T::REGS)?;
            unsafe {
                ptr::write_volatile(T::REGS.datar().as_ptr().cast(), *word);
            }
        }
        self.blocking_flush()
    }

    /// Blocking read, generating exactly one frame of clocks per word.
    pub fn blocking_read<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }
        if self.miso.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(false)?;
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        for word in words {
            *word = transfer_word(&T::REGS, W::default())?;
        }
        self.blocking_flush()
    }

    /// Blocking transfer using the same buffer for transmit and receive.
    pub fn blocking_transfer_in_place<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.is_empty() {
            return Ok(());
        }
        if self.mosi.is_none() || self.miso.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(false)?;
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        for word in words {
            *word = transfer_word(&T::REGS, *word)?;
        }
        self.blocking_flush()
    }

    /// Blocking transfer of `max(read.len(), write.len())` words.
    ///
    /// Missing transmit words are zero; extra received words are discarded.
    pub fn blocking_transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        let length = read.len().max(write.len());
        if length == 0 {
            return Ok(());
        }
        if (!read.is_empty() && self.miso.is_none()) || (!write.is_empty() && self.mosi.is_none()) {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(false)?;
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        for index in 0..length {
            let sent = write.get(index).copied().unwrap_or_default();
            let received = transfer_word(&T::REGS, sent)?;
            if let Some(destination) = read.get_mut(index) {
                *destination = received;
            }
        }
        self.blocking_flush()
    }
}

impl<'d, T: Instance> Spi<'d, T, Blocking> {
    /// Create a new SPI driver.
    pub fn new_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(miso, AfType::input(Pull::None)),
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_blocking_rxonly<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            new_pin!(miso, AfType::input(Pull::None)),
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_blocking_txonly<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode, without SCK pin.
    ///
    /// This can be useful for bit-banging non-SPI protocols.
    pub fn new_blocking_txonly_nosck<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            None,
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            None,
            None,
            config,
        )
    }
}

impl<'d, T: Instance> Spi<'d, T, Async> {
    /// Create a new SPI driver.
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        #[cfg(spi_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(miso, AfType::input(Pull::None)),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in RX-only mode (only MISO pin, no MOSI).
    pub fn new_rxonly<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        #[cfg(spi_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            new_pin!(miso, AfType::input(Pull::None)),
            None,
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a receive-only driver with DMA-generated dummy transmit words.
    pub fn new_rxonly_with_tx_dma<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        miso: Peri<'d, if_remap!(impl MisoPin<T, REMAP>)>,
        #[cfg(spi_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();
        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            new_pin!(miso, AfType::input(Pull::None)),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode (only MOSI pin, no MISO).
    pub fn new_txonly<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        sck: Peri<'d, if_remap!(impl SckPin<T, REMAP>)>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        #[cfg(spi_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            new_pin!(sck, AfType::output(OutputType::PushPull, Speed::High)),
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            new_dma!(tx_dma),
            None,
            config,
        )
    }

    /// Create a new SPI driver, in TX-only mode, without SCK pin.
    ///
    /// This can be useful for bit-banging non-SPI protocols.
    pub fn new_txonly_nosck<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        mosi: Peri<'d, if_remap!(impl MosiPin<T, REMAP>)>,
        #[cfg(spi_v4x7)] _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>>,
        tx_dma: Peri<'d, impl TxDma<T>>,
        config: Config,
    ) -> Self {
        assert!(
            calculate_baud_rate(T::frequency().0, config.frequency.0).is_some(),
            "unrepresentable SPI frequency"
        );
        apply_remap!();

        Self::new_inner(
            peri,
            None,
            new_pin!(mosi, AfType::output(OutputType::PushPull, Speed::High)),
            None,
            new_dma!(tx_dma),
            None,
            config,
        )
    }

    async fn wait_error() -> Error {
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            if let Err(error) = check_error_flags(&T::REGS.statr().read()) {
                return Poll::Ready(error);
            }
            #[cfg(spi_v4x7)]
            T::REGS.ctlr2().modify(|w| w.set_errie(true));
            #[cfg(not(spi_v4x7))]
            cx.waker().wake_by_ref();
            Poll::Pending
        })
        .await
    }

    /// Wait until the transmit buffer and shift register are empty.
    pub async fn flush(&mut self) -> Result<(), Error> {
        // Same boundary as `blocking_flush`: while SPE=0 the CH32V4x7 TXE
        // flag is not meaningful, and no transfer can be in flight.
        if !T::REGS.ctlr1().read().spe() {
            return Ok(());
        }
        poll_fn(|cx| {
            let status = T::REGS.statr().read();
            if let Err(error) = check_error_flags(&status) {
                cleanup(T::REGS);
                return Poll::Ready(Err(error));
            }
            if status.txe() && !status.bsy() {
                Poll::Ready(Ok(()))
            } else {
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        })
        .await
    }

    /// Write using DMA; received data is ignored in transmit-only hardware mode.
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        if data.len() > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        if self.mosi.is_none() || self.tx_dma.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(true)?;
        let transfer = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write(data, T::REGS.datar().as_ptr().cast(), Default::default())
        };
        T::REGS.ctlr2().modify(|w| w.set_txdmaen(true));
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        if let Either::First(error) = select(Self::wait_error(), transfer).await {
            return Err(error);
        }
        self.flush().await
    }

    /// Read using DMA and exactly one frame of clocks per word.
    ///
    /// RX-only constructors generate dummy transmit words in software. Supply
    /// both DMA channels with `new_rxonly_with_tx_dma` for DMA clock generation.
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        if data.len() > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        if self.miso.is_none() || self.rx_dma.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(false)?;
        let count = data.len();
        let dummy = W::default();
        let receive = unsafe {
            self.rx_dma
                .as_mut()
                .unwrap()
                .read(T::REGS.datar().as_ptr().cast(), data, Default::default())
        };
        if let Some(transmit) = self.tx_dma.as_mut() {
            let transmit =
                unsafe { transmit.write_repeated(&dummy, count, T::REGS.datar().as_ptr().cast(), Default::default()) };
            T::REGS.ctlr2().modify(|w| {
                w.set_rxdmaen(true);
                w.set_txdmaen(true);
            });
            T::REGS.ctlr1().modify(|w| w.set_spe(true));
            if let Either::First(error) = select(Self::wait_error(), join(transmit, receive)).await {
                return Err(error);
            }
        } else {
            let mut sent = 0;
            let transmit = poll_fn(|cx| {
                if T::REGS.statr().read().txe() {
                    unsafe {
                        ptr::write_volatile(T::REGS.datar().as_ptr().cast(), dummy);
                    }
                    sent += 1;
                }
                if sent == count {
                    Poll::Ready(())
                } else {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                }
            });
            T::REGS.ctlr2().modify(|w| w.set_rxdmaen(true));
            T::REGS.ctlr1().modify(|w| w.set_spe(true));
            if let Either::First(error) = select(Self::wait_error(), join(transmit, receive)).await {
                return Err(error);
            }
        }
        self.flush().await
    }

    async fn transfer_inner<W: Word>(&mut self, read: *mut [W], write: *const [W]) -> Result<(), Error> {
        let (_, length) = slice_ptr_parts(read);
        let (_, sent) = slice_ptr_parts(write);
        assert_eq!(length, sent);
        if length == 0 {
            return Ok(());
        }
        if length > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        if self.tx_dma.is_none() || self.rx_dma.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        let _cleanup = OnDrop::new(|| cleanup(T::REGS));
        self.prepare::<W>(false)?;
        let receive = unsafe {
            self.rx_dma
                .as_mut()
                .unwrap()
                .read_raw(T::REGS.datar().as_ptr().cast(), read, Default::default())
        };
        let transmit = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_raw(write, T::REGS.datar().as_ptr().cast(), Default::default())
        };
        T::REGS.ctlr2().modify(|w| {
            w.set_rxdmaen(true);
            w.set_txdmaen(true);
        });
        T::REGS.ctlr1().modify(|w| w.set_spe(true));
        if let Either::First(error) = select(Self::wait_error(), join(transmit, receive)).await {
            return Err(error);
        }
        self.flush().await
    }

    /// Transfer `max(read.len(), write.len())` words using DMA.
    ///
    /// Missing transmit words are zero; extra received words are discarded.
    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        if read.len().max(write.len()) > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        if (!read.is_empty() && self.miso.is_none()) || (!write.is_empty() && self.mosi.is_none()) {
            return Err(Error::UnsupportedOperation);
        }
        let common = read.len().min(write.len());
        self.transfer_inner(&mut read[..common], &write[..common]).await?;
        if read.len() > common {
            self.read(&mut read[common..]).await?;
        }
        if write.len() > common {
            self.write(&write[common..]).await?;
        }
        Ok(())
    }

    /// Transfer using the same buffer for transmit and receive.
    pub async fn transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }
        if self.miso.is_none() || self.mosi.is_none() {
            return Err(Error::UnsupportedOperation);
        }
        self.transfer_inner(data, data).await
    }
}

impl<'d, T: Instance, W: Word> embedded_hal_async::spi::SpiBus<W> for Spi<'d, T, Async> {
    async fn write(&mut self, data: &[W]) -> Result<(), Self::Error> {
        self.write(data).await
    }

    async fn read(&mut self, data: &mut [W]) -> Result<(), Self::Error> {
        self.read(data).await
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, data: &mut [W]) -> Result<(), Self::Error> {
        self.transfer_in_place(data).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush().await
    }
}

impl<'d, T: Instance, M: PeriMode> Drop for Spi<'d, T, M> {
    fn drop(&mut self) {
        use crate::gpio::SealedPin;
        cleanup(T::REGS);
        #[cfg(spi_v4x7)]
        {
            use crate::interrupt::typelevel::Interrupt;
            T::Interrupt::disable();
        }

        self.sck.as_ref().map(|x| x.set_as_disconnected());
        self.mosi.as_ref().map(|x| x.set_as_disconnected());
        self.miso.as_ref().map(|x| x.set_as_disconnected());

        T::disable();
    }
}

// Get CTRL1.BR
#[inline]
fn calculate_baud_rate(clock: u32, frequency: u32) -> Option<BaudRate> {
    if clock == 0 || frequency == 0 {
        return None;
    }
    for (divisor, setting) in [
        (2u64, BaudRate::DIV_2),
        (4, BaudRate::DIV_4),
        (8, BaudRate::DIV_8),
        (16, BaudRate::DIV_16),
        (32, BaudRate::DIV_32),
        (64, BaudRate::DIV_64),
        (128, BaudRate::DIV_128),
        (256, BaudRate::DIV_256),
    ] {
        if u64::from(clock) <= u64::from(frequency) * divisor {
            return Some(setting);
        }
    }
    None
}

fn configure_high_speed(regs: Regs, clock: u32, divisor: BaudRate, receive: bool) {
    #[cfg(spi_v4x7)]
    let enabled = receive && divisor == BaudRate::DIV_2;
    #[cfg(not(spi_v4x7))]
    let enabled = receive && divisor == BaudRate::DIV_2 && clock / 2 >= 36_000_000;
    regs.hscr().modify(|w| {
        w.set_hsrxen(enabled);
        #[cfg(spi_v4x7)]
        w.set_hsrxen2(enabled && clock >= 120_000_000);
    });
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

fn cleanup(regs: Regs) {
    regs.ctlr2().modify(|reg| {
        reg.set_txdmaen(false);
        reg.set_rxdmaen(false);
        reg.set_errie(false);
    });
    let _ = regs.statr().read();
    regs.ctlr1().modify(|w| w.set_spe(false));
    flush_rx_fifo(regs);
    let _ = regs.datar().read();
    let _ = regs.statr().read();
    regs.statr().write(|w| w.set_crcerr(false));
    regs.ctlr1().modify(|w| {
        w.set_mstr(true);
        w.set_bidimode(false);
        w.set_bidioe(false);
        w.set_rxonly(false);
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
        self.blocking_flush()
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
            Self::BufferTooLong | Self::UnsupportedOperation => embedded_hal::spi::ErrorKind::Other,
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
    fn state() -> &'static State;
}

struct State {
    waker: AtomicWaker,
}

/// SPI instance trait.
#[allow(private_bounds)]
pub trait Instance:
    embassy_hal_internal::PeripheralType + crate::peripheral::RccPeripheral + crate::peripheral::RemapBound + SealedInstance
{
    /// SPI global interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (spi, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            const REGS: Regs = crate::pac::$inst;
            fn state() -> &'static State {
                static STATE: State = State { waker: AtomicWaker::new() };
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }
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
