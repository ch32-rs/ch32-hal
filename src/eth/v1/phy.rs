use core::task::Context;

use super::Smi;

// ============================================================================
// PHY trait
// ============================================================================

/// Trait for Ethernet PHY drivers.
pub trait Phy {
    /// Reset the PHY and wait for it to come out of reset.
    fn phy_reset(&mut self, smi: &mut Smi);
    /// Initialize the PHY (enable autonegotiation, etc).
    fn phy_init(&mut self, smi: &mut Smi);
    /// Poll the link state. Returns true if link is up.
    fn poll_link(&mut self, smi: &mut Smi, cx: &mut Context) -> bool;
}

/// MII PHY register constants.
mod phy_regs {
    pub const BCR: u8 = 0x00;
    pub const BSR: u8 = 0x01;

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
    fn phy_reset(&mut self, smi: &mut Smi) {
        smi.write(self.phy_addr, phy_regs::BCR, phy_regs::BCR_RESET);
        // Wait for reset bit to self-clear
        while smi.read(self.phy_addr, phy_regs::BCR) & phy_regs::BCR_RESET != 0 {}
    }

    fn phy_init(&mut self, smi: &mut Smi) {
        // Start auto-negotiation
        smi.write(
            self.phy_addr,
            phy_regs::BCR,
            phy_regs::BCR_AUTONEG | phy_regs::BCR_AUTONEG_RESTART,
        );
    }

    fn poll_link(&mut self, smi: &mut Smi, _cx: &mut Context) -> bool {
        let bsr = smi.read(self.phy_addr, phy_regs::BSR);
        bsr & phy_regs::BSR_LINK_UP != 0
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
    /// Auto PN polarity detection (bits 3:2 = 0b10).
    const MDIX_PN_AUTO: u16 = 2 << 2;

    /// Create a driver for the built-in PHY.
    pub fn new() -> Self {
        Self {
            phy: GenericPhy::new(Self::PHY_ADDR),
        }
    }
}

impl Phy for InternalPhy {
    fn phy_reset(&mut self, smi: &mut Smi) {
        self.phy.phy_reset(smi);
    }

    fn phy_init(&mut self, smi: &mut Smi) {
        // Auto PN polarity detection is required for reliable link establishment.
        smi.write(Self::PHY_ADDR, Self::MDIX, Self::MDIX_PN_AUTO);
        self.phy.phy_init(smi);
    }

    fn poll_link(&mut self, smi: &mut Smi, cx: &mut Context) -> bool {
        self.phy.poll_link(smi, cx)
    }
}
