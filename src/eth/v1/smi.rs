use super::eth;

// ============================================================================
// PHY management via SMI (Station Management Interface)
// ============================================================================

/// Station Management Interface for MII PHY register access.
pub struct Smi {
    pub(super) clock_range: u8,
}

impl Smi {
    /// Read a PHY register.
    pub fn read(&self, phy_addr: u8, reg: u8) -> u16 {
        eth().macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(false);
            w.set_mb(true);
        });
        while eth().macmiiar().read().mb() {}
        eth().macmiidr().read().md()
    }

    /// Write a PHY register.
    pub fn write(&self, phy_addr: u8, reg: u8, val: u16) {
        eth().macmiidr().write(|w| w.set_md(val));
        eth().macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(true);
            w.set_mb(true);
        });
        while eth().macmiiar().read().mb() {}
    }
}
