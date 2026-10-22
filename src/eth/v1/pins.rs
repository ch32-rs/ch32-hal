use crate::Peri;

/// Configure GPIO pins for RMII mode (default pin mapping, no remap).
///
/// This sets up the 9 RMII pins with appropriate modes:
/// - Input floating: REF_CLK (PA1), CRS_DV (PA7), RXD0 (PC4), RXD1 (PC5)
/// - AF push-pull 50MHz: MDIO (PA2), MDC (PC1), TX_EN (PB11), TXD0 (PB12), TXD1 (PB13)
///
/// Call this before [`Ethernet::new()`](super::Ethernet::new).
pub fn init_rmii_pins(
    _ref_clk: Peri<'_, impl crate::gpio::Pin>,
    _mdio: Peri<'_, impl crate::gpio::Pin>,
    _crs_dv: Peri<'_, impl crate::gpio::Pin>,
    _mdc: Peri<'_, impl crate::gpio::Pin>,
    _rxd0: Peri<'_, impl crate::gpio::Pin>,
    _rxd1: Peri<'_, impl crate::gpio::Pin>,
    _tx_en: Peri<'_, impl crate::gpio::Pin>,
    _txd0: Peri<'_, impl crate::gpio::Pin>,
    _txd1: Peri<'_, impl crate::gpio::Pin>,
) {
    use crate::gpio::{AFType, Pull, Speed};

    // Input pins: floating input
    _ref_clk.set_as_input(Pull::None);
    _crs_dv.set_as_input(Pull::None);
    _rxd0.set_as_input(Pull::None);
    _rxd1.set_as_input(Pull::None);

    // Output pins: AF push-pull, 50MHz
    _mdio.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _mdc.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _tx_en.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _txd0.set_as_af_output(AFType::OutputPushPull, Speed::High);
    _txd1.set_as_af_output(AFType::OutputPushPull, Speed::High);
}
