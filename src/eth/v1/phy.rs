use core::task::Context;

use super::{Smi, SmiError};

// ============================================================================
// PHY trait
// ============================================================================

/// Trait for Ethernet PHY drivers.
///
/// Methods propagate errors from PHY management transactions.
pub trait Phy {
    /// Reset the PHY and wait for it to come out of reset.
    fn phy_reset(&mut self, smi: &mut Smi) -> Result<(), SmiError>;
    /// Initialize the PHY (enable autonegotiation, etc).
    fn phy_init(&mut self, smi: &mut Smi) -> Result<(), SmiError>;
    /// Poll the link state. Returns true if link is up.
    fn poll_link(&mut self, smi: &mut Smi, cx: &mut Context) -> Result<bool, SmiError>;
    /// Duplex mode of the established link (called on each link-up).
    /// Returns true for full duplex. The default keeps the MAC in full duplex.
    fn full_duplex(&mut self, _smi: &mut Smi) -> Result<bool, SmiError> {
        Ok(true)
    }
    /// Swap the polarity of the receive pair and restart auto-negotiation.
    ///
    /// Called when the MAC has seen frames that fail their FCS and not a
    /// single one that passes, which is what a reversed pair looks like: the
    /// link comes up and link pulses are polarity independent, but every frame
    /// is inverted. Returns true if the PHY reconfigured itself. The default
    /// leaves the PHY alone.
    fn swap_polarity(&mut self, _smi: &mut Smi) -> Result<bool, SmiError> {
        Ok(false)
    }
}

/// MII PHY register constants.
mod phy_regs {
    pub const BCR: u8 = 0x00;
    pub const BSR: u8 = 0x01;
    pub const ANLPAR: u8 = 0x05;
    pub const ANLPAR_SELECTOR: u16 = 0x1F;
    pub const ANLPAR_SELECTOR_IEEE802_3: u16 = 0x01;

    pub const BCR_RESET: u16 = 1 << 15;
    pub const BCR_AUTONEG: u16 = 1 << 12;
    pub const BCR_AUTONEG_RESTART: u16 = 1 << 9;

    pub const BSR_LINK_UP: u16 = 1 << 2;
}

/// Generic PHY driver that works with IEEE 802.3 compliant PHYs.
///
/// For the CH32V307 built-in 10M PHY, use [`InternalPhy`] instead.
pub struct GenericPhy {
    phy_addr: u8,
}

impl GenericPhy {
    /// Create a new generic PHY driver.
    ///
    /// `phy_addr` is the SMI address of the PHY (0-31).
    pub fn new(phy_addr: u8) -> Self {
        Self { phy_addr }
    }
}

impl Phy for GenericPhy {
    fn phy_reset(&mut self, smi: &mut Smi) -> Result<(), SmiError> {
        smi.write(self.phy_addr, phy_regs::BCR, phy_regs::BCR_RESET)?;
        // Wait for reset bit to self-clear
        while smi.read(self.phy_addr, phy_regs::BCR)? & phy_regs::BCR_RESET != 0 {}
        Ok(())
    }

    fn phy_init(&mut self, smi: &mut Smi) -> Result<(), SmiError> {
        // Start auto-negotiation
        smi.write(
            self.phy_addr,
            phy_regs::BCR,
            phy_regs::BCR_AUTONEG | phy_regs::BCR_AUTONEG_RESTART,
        )
    }

    fn poll_link(&mut self, smi: &mut Smi, _cx: &mut Context) -> Result<bool, SmiError> {
        let bsr = smi.read(self.phy_addr, phy_regs::BSR)?;
        Ok(bsr & phy_regs::BSR_LINK_UP != 0)
    }
}

/// Driver for the CH32V307 built-in 10M PHY at SMI address 1.
pub struct InternalPhy {
    phy: GenericPhy,
}

impl InternalPhy {
    const PHY_ADDR: u8 = 1;
    /// WCH internal 10M PHY MDIX/polarity control register.
    const MDIX: u8 = 0x1E;
    /// WCH internal 10M PHY status register; bit 2 = full duplex.
    const STATUS: u8 = 0x10;
    const STATUS_FULL_DUPLEX: u16 = 1 << 2;
    /// Auto PN polarity detection (bits 3:2 = 0b10).
    const MDIX_PN_AUTO: u16 = 2 << 2;
    /// Receive polarity field (bits 3:2): 0b00 normal, 0b01 reversed.
    const MDIX_PN_MASK: u16 = 3 << 2;
    const MDIX_PN_REVERSED: u16 = 1 << 2;

    /// Create a driver for the built-in PHY.
    pub fn new() -> Self {
        Self {
            phy: GenericPhy::new(Self::PHY_ADDR),
        }
    }
}

impl Phy for InternalPhy {
    fn phy_reset(&mut self, smi: &mut Smi) -> Result<(), SmiError> {
        self.phy.phy_reset(smi)
    }

    fn phy_init(&mut self, smi: &mut Smi) -> Result<(), SmiError> {
        // Auto PN polarity detection is required for reliable link establishment.
        smi.write(Self::PHY_ADDR, Self::MDIX, Self::MDIX_PN_AUTO)?;
        self.phy.phy_init(smi)
    }

    fn poll_link(&mut self, smi: &mut Smi, cx: &mut Context) -> Result<bool, SmiError> {
        self.phy.poll_link(smi, cx)
    }

    fn full_duplex(&mut self, smi: &mut Smi) -> Result<bool, SmiError> {
        // As in WCH's driver (ETH_PHYLink): full duplex if the PHY reports it.
        // Otherwise half duplex if the partner auto-negotiated, full duplex if
        // it did not (no valid base page).
        if smi.read(Self::PHY_ADDR, Self::STATUS)? & Self::STATUS_FULL_DUPLEX != 0 {
            return Ok(true);
        }
        let anlpar = smi.read(Self::PHY_ADDR, phy_regs::ANLPAR)?;
        Ok(anlpar & phy_regs::ANLPAR_SELECTOR != phy_regs::ANLPAR_SELECTOR_IEEE802_3)
    }

    fn swap_polarity(&mut self, smi: &mut Smi) -> Result<bool, SmiError> {
        // Step the automatic detection one way or the other, as WCH does in
        // `WCHNET_PhyPNProcess`: auto and normal keep receiving inverted
        // frames, so the first swap forces the reversed setting and the next
        // one goes back to normal.
        let mdix = smi.read(Self::PHY_ADDR, Self::MDIX)?;
        let mdix = if mdix & Self::MDIX_PN_REVERSED != 0 {
            mdix & !Self::MDIX_PN_MASK
        } else {
            (mdix & !Self::MDIX_PN_MASK) | Self::MDIX_PN_REVERSED
        };
        smi.write(Self::PHY_ADDR, Self::MDIX, mdix)?;
        // The PHY only applies the new polarity when it negotiates again.
        self.phy.phy_init(smi)?;
        Ok(true)
    }
}
