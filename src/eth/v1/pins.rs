use crate::gpio::{AFType, AnyPin, Pin, Pull, Speed};
use crate::Peri;

use super::Instance;

pin_trait!(RefClkPin, Instance);
pin_trait!(MdioPin, Instance);
pin_trait!(MdcPin, Instance);
pin_trait!(CrsDvPin, Instance);
pin_trait!(Rxd0Pin, Instance);
pin_trait!(Rxd1Pin, Instance);
pin_trait!(TxEnPin, Instance);
pin_trait!(Txd0Pin, Instance);
pin_trait!(Txd1Pin, Instance);

// Default RMII mapping only; remapped pin combinations are not supported.
pin_trait_impl!(crate::eth::RefClkPin, ETH, PA1, 0);
pin_trait_impl!(crate::eth::MdioPin, ETH, PA2, 0);
pin_trait_impl!(crate::eth::MdcPin, ETH, PC1, 0);
pin_trait_impl!(crate::eth::CrsDvPin, ETH, PA7, 0);
pin_trait_impl!(crate::eth::Rxd0Pin, ETH, PC4, 0);
pin_trait_impl!(crate::eth::Rxd1Pin, ETH, PC5, 0);
pin_trait_impl!(crate::eth::TxEnPin, ETH, PB11, 0);
pin_trait_impl!(crate::eth::Txd0Pin, ETH, PB12, 0);
pin_trait_impl!(crate::eth::Txd1Pin, ETH, PB13, 0);

/// Pins for the default RMII mapping, consumed and configured by
/// [`Ethernet::new_rmii`](super::Ethernet::new_rmii).
///
/// Signal traits are checked against the Ethernet instance by the constructor.
pub struct RmiiPins<
    'd,
    REFCLK: Pin,
    MDIO: Pin,
    MDC: Pin,
    CRSDV: Pin,
    RXD0: Pin,
    RXD1: Pin,
    TXEN: Pin,
    TXD0: Pin,
    TXD1: Pin,
> {
    /// REF_CLK (PA1).
    pub ref_clk: Peri<'d, REFCLK>,
    /// MDIO (PA2).
    pub mdio: Peri<'d, MDIO>,
    /// MDC (PC1).
    pub mdc: Peri<'d, MDC>,
    /// CRS_DV (PA7).
    pub crs_dv: Peri<'d, CRSDV>,
    /// RXD0 (PC4).
    pub rxd0: Peri<'d, RXD0>,
    /// RXD1 (PC5).
    pub rxd1: Peri<'d, RXD1>,
    /// TX_EN (PB11).
    pub tx_en: Peri<'d, TXEN>,
    /// TXD0 (PB12).
    pub txd0: Peri<'d, TXD0>,
    /// TXD1 (PB13).
    pub txd1: Peri<'d, TXD1>,
}

impl<'d, REFCLK: Pin, MDIO: Pin, MDC: Pin, CRSDV: Pin, RXD0: Pin, RXD1: Pin, TXEN: Pin, TXD0: Pin, TXD1: Pin>
    RmiiPins<'d, REFCLK, MDIO, MDC, CRSDV, RXD0, RXD1, TXEN, TXD0, TXD1>
{
    pub(super) fn configure(self) -> [Peri<'d, AnyPin>; 9] {
        self.ref_clk.set_as_input(Pull::None);
        self.mdio.set_as_af_output(AFType::OutputPushPull, Speed::High);
        self.mdc.set_as_af_output(AFType::OutputPushPull, Speed::High);
        self.crs_dv.set_as_input(Pull::None);
        self.rxd0.set_as_input(Pull::None);
        self.rxd1.set_as_input(Pull::None);
        self.tx_en.set_as_af_output(AFType::OutputPushPull, Speed::High);
        self.txd0.set_as_af_output(AFType::OutputPushPull, Speed::High);
        self.txd1.set_as_af_output(AFType::OutputPushPull, Speed::High);

        [
            self.ref_clk.into(),
            self.mdio.into(),
            self.mdc.into(),
            self.crs_dv.into(),
            self.rxd0.into(),
            self.rxd1.into(),
            self.tx_en.into(),
            self.txd0.into(),
            self.txd1.into(),
        ]
    }
}
