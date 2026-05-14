//! Ethernet (ETH)
#![macro_use]

#[cfg_attr(eth_10m, path = "v10m/mod.rs")]
#[cfg_attr(eth_v1, path = "v1/mod.rs")]
mod _version;

pub mod generic_smi;

pub use self::_version::*;

#[allow(unused)]
const MTU: usize = 1514;

/// Station Management Interface (SMI) on an ethernet PHY
///
/// # Safety
///
/// The methods cannot move out of self
pub unsafe trait StationManagement {
    /// Read a register over SMI.
    fn smi_read(&mut self, phy_addr: u8, reg: u8) -> u16;
    /// Write a register over SMI.
    fn smi_write(&mut self, phy_addr: u8, reg: u8, val: u16);
}

/// Traits for an Ethernet PHY
///
/// # Safety
///
/// The methods cannot move S
pub unsafe trait PHY {
    /// Reset PHY and wait for it to come out of reset.
    fn phy_reset<S: StationManagement>(&mut self, sm: &mut S);
    /// Read PHY registers to check if link is established
    fn link_up<S: StationManagement>(&mut self, sm: &mut S) -> bool;
}

trait SealedInstance {
    fn regs() -> crate::pac::eth::Eth;
}

/// Ethernet instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + Send + 'static {}

impl SealedInstance for crate::peripherals::ETH {
    fn regs() -> crate::pac::eth::Eth {
        crate::pac::ETH
    }
}
impl Instance for crate::peripherals::ETH {}
