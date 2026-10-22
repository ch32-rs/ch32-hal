#[cfg(emac)]
use crate::pac::emac::Eth as EthRegs;
#[cfg(not(emac))]
use crate::pac::eth::Eth as EthRegs;

// ============================================================================
// PHY management via SMI (Station Management Interface)
// ============================================================================

/// Station Management Interface for MII PHY register access.
pub struct Smi {
    regs: EthRegs,
    clock_range: u8,
}

impl Smi {
    pub(super) fn new(regs: EthRegs, clock_range: u8) -> Self {
        Self { clock_range, regs }
    }

    /// Read a PHY register.
    pub fn read(&mut self, phy_addr: u8, reg: u8) -> u16 {
        self.regs.macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(false);
            w.set_mb(true);
        });
        while self.regs.macmiiar().read().mb() {}
        self.regs.macmiidr().read().md()
    }

    /// Write a PHY register.
    pub fn write(&mut self, phy_addr: u8, reg: u8, val: u16) {
        self.regs.macmiidr().write(|w| w.set_md(val));
        self.regs.macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(true);
            w.set_mb(true);
        });
        while self.regs.macmiiar().read().mb() {}
    }
}
