#[cfg(emac)]
use crate::pac::emac::Eth as EthRegs;
#[cfg(not(emac))]
use crate::pac::eth::Eth as EthRegs;

// ============================================================================
// PHY management via SMI (Station Management Interface)
// ============================================================================

// Bound register polling without requiring an Embassy time driver. This is a
// polling budget, not a wall-clock deadline.
const BUSY_POLL_LIMIT: usize = 10_000;

/// An error during a PHY management transaction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SmiError {
    /// The MAC management interface did not become idle within the polling budget.
    Timeout,
}

/// Station Management Interface for MII PHY register access.
pub struct Smi {
    regs: EthRegs,
    clock_range: u8,
}

impl Smi {
    pub(super) fn new(regs: EthRegs, clock_range: u8) -> Self {
        Self { clock_range, regs }
    }

    fn wait_until_idle(&self) -> Result<(), SmiError> {
        for _ in 0..BUSY_POLL_LIMIT {
            if !self.regs.macmiiar().read().mb() {
                return Ok(());
            }
        }
        Err(SmiError::Timeout)
    }

    /// Read a PHY register.
    ///
    /// Returns [`SmiError::Timeout`] if the management interface remains busy.
    pub fn read(&mut self, phy_addr: u8, reg: u8) -> Result<u16, SmiError> {
        self.wait_until_idle()?;
        self.regs.macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(false);
            w.set_mb(true);
        });
        self.wait_until_idle()?;
        Ok(self.regs.macmiidr().read().md())
    }

    /// Write a PHY register.
    ///
    /// Returns [`SmiError::Timeout`] if the management interface remains busy.
    pub fn write(&mut self, phy_addr: u8, reg: u8, val: u16) -> Result<(), SmiError> {
        self.wait_until_idle()?;
        self.regs.macmiidr().write(|w| w.set_md(val));
        self.regs.macmiiar().write(|w| {
            w.set_pa(phy_addr & 0x1F);
            w.set_mr(reg & 0x1F);
            w.set_cr(self.clock_range);
            w.set_mw(true);
            w.set_mb(true);
        });
        self.wait_until_idle()
    }
}
