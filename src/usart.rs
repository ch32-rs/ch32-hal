//! 4 USARTs
//!
//! - No split of BasicInstance and FullInstance

/*
Full-duplex or half-duplex synchronous or asynchronous communication
NRZ data format
Fractional baud rate generator, up to 3Mbps
Programmable data length
Configurable stop bits
Supports LIN, IrDA encoder, smart card
Supports DMA
Multiple interrupt sources
 */

use core::marker::PhantomData;

use crate::gpio::sealed::AFType;
use crate::gpio::{Pull, Speed};
use crate::{into_ref, pac, peripherals, Peripheral};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataBits {
    DataBits8 = 0,
    DataBits9 = 1,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum StopBits {
    #[doc = "1 stop bit"]
    STOP1 = 0,
    #[doc = "0.5 stop bits"]
    STOP0P5 = 0b01,
    #[doc = "2 stop bits"]
    STOP2 = 0b10,
    #[doc = "1.5 stop bits"]
    STOP1P5 = 0b11,
}

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: Parity,
}
impl Default for Config {
    /// 115200 8N1
    fn default() -> Self {
        Self {
            baudrate: 115200,
            data_bits: DataBits::DataBits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::ParityNone,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    // FE
    Framing,
    /// Noise error
    // NE
    Noise,
    /// RX buffer overrun
    // ORE
    Overrun,
    /// Parity check error
    // PE
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    BaudrateTooLow,
    BaudrateTooHigh,
}

pub struct UartTx<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UartTx<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,

        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, tx, config)
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, false)?;

        // TODO: async state

        Ok(Self { phantom: PhantomData })
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            while !rb.statr().read().tc() {} // wait tx complete
            rb.datar().write(|w| w.set_dr(c as u16));
        }
        Ok(())
    }

    pub fn blocking_flush(&mut self) -> Result<(), Error> {
        let rb = T::regs();

        while !rb.statr().read().txe() {} // wait tx ends
        Ok(())
    }
}

pub struct UartRx<'d, T: Instance> {
    phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> UartRx<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,

        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, rx, config)
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, rx);

        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, false, true)?;

        Ok(Self { phantom: PhantomData })
    }

    /// Read a single u8 if there is one avaliable
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        let rb = T::regs();

        if !rb.statr().read().rxne() {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(rb.datar().read().dr() as u8) // FIXME: how to handle 9 bits
        }
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        let rb = T::regs();

        for c in buffer {
            while !rb.statr().read().rxne() {} // wait rxne
            *c = rb.datar().read().dr() as u8; // FIXME: how to handle 9 bits
        }
        Ok(())
    }
}

/// Bidirectional UART Driver
pub struct Uart<'d, T: Instance> {
    tx: UartTx<'d, T>,
    rx: UartRx<'d, T>,
}

impl<'d, T: Instance> Uart<'d, T> {
    pub fn new<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        Self::new_inner(peri, tx, rx, config)
    }

    /// Half-duplex
    ///
    /// Note: Half duplex requires TX pin to have a pull-up resistor
    pub fn new_half_duplex_on_tx<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        T::enable_and_reset();

        into_ref!(_peri, tx);

        // 推挽复用输出(外加上拉)
        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, true)?;

        rb.ctlr3().modify(|w| w.set_hdsel(true)); // half duplex

        rb.statr().modify(|w| w.set_rxne(false)); // clear rxne
        let _ = rb.datar().read().dr(); // clear rxne

        Ok(Self {
            tx: UartTx { phantom: PhantomData },
            rx: UartRx { phantom: PhantomData },
        })
    }

    fn new_inner<const REMAP: u8>(
        _peri: impl Peripheral<P = T> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        into_ref!(_peri, tx, rx);

        tx.set_as_af_output(AFType::OutputPushPull, Speed::High);
        rx.set_as_input(Pull::None);
        T::set_remap(REMAP);

        let rb = T::regs();
        configure(rb, &config, true, true)?;

        Ok(Self {
            tx: UartTx { phantom: PhantomData },
            rx: UartRx { phantom: PhantomData },
        })
    }

    pub fn split(self) -> (UartTx<'d, T>, UartRx<'d, T>) {
        (self.tx, self.rx)
    }

    pub fn blocking_write(&mut self, buffer: &[u8]) -> Result<(), Error> {
        let rb = T::regs();

        for &c in buffer {
            while !rb.statr().read().txe() {} // wait tx complete
            rb.datar().write(|w| w.set_dr(c as u16));
        }
        Ok(())
    }

    /// Read a single u8 if there is one avaliable
    pub fn nb_read(&mut self) -> Result<u8, nb::Error<Error>> {
        self.rx.nb_read()
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        self.rx.blocking_read(buffer)
    }
}

pub(crate) mod sealed {
    // use embassy_sync::waitqueue::AtomicWaker;

    use super::*;
    use crate::interrupt;

    pub trait Instance {
        type Interrupt: interrupt::Interrupt;

        fn regs() -> &'static pac::usart::Usart;
    }
}

fn configure(rb: &pac::usart::Usart, config: &Config, enable_tx: bool, enable_rx: bool) -> Result<(), ConfigError> {
    if !enable_rx && !enable_tx {
        panic!("USART: At least one of RX or TX should be enabled");
    }

    rb.ctlr2().modify(|w| w.set_stop(config.stop_bits as u8));

    rb.ctlr1().modify(|w| {
        w.set_m(config.data_bits as u8 != 0);
        w.set_pce(config.parity != Parity::ParityNone);
        w.set_ps(config.parity == Parity::ParityOdd); // 1 for odd parity, 0 for even parity
        w.set_te(enable_tx);
        w.set_re(enable_rx);
    });

    // HCLK/(16*USARTDIV)
    // USARTDIV = DIV_M+(DIV_F/16)  via USART_BRR

    let apbclock = crate::rcc::clocks().hclk.0;

    let div_m = 25 * apbclock / (4 * config.baudrate);
    let mut tmpreg = (div_m / 100) << 4;

    let div_f = div_m - 100 * (tmpreg >> 4);
    tmpreg |= ((div_f * 16 + 50) / 100) & 0x0F;

    rb.brr().write(|w| w.0 = tmpreg);

    // enable uart
    rb.ctlr1().modify(|w| w.set_ue(true));

    Ok(())
}

// Peripheral traits

pub trait Instance:
    Peripheral<P = Self>
    + sealed::Instance
    + crate::peripheral::RccPeripheral
    + crate::peripheral::RemapPeripheral
    + 'static
    + Send
{
}

macro_rules! impl_uart {
    ($inst:ident, $irq:ident) => {
        impl sealed::Instance for crate::peripherals::$inst {
            type Interrupt = crate::interrupt::$irq;

            fn regs() -> &'static crate::pac::usart::Usart {
                &crate::pac::$inst
            }
        }
        impl Instance for peripherals::$inst {}
    };
}

impl_uart!(USART1, USART1);
impl_uart!(USART2, USART2);
impl_uart!(USART3, USART3);
impl_uart!(USART4, USART4);

macro_rules! pin_trait {
    ($signal:ident, $instance:path) => {
        pub trait $signal<T: $instance, const REMAP: u8>: crate::gpio::Pin {}
    };
}

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);
pin_trait!(CtsPin, Instance);
pin_trait!(RtsPin, Instance);
pin_trait!(CkPin, Instance);
