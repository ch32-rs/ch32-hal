//! PHY access for the CH32H417 gigabit MAC.
//!
//! The station management interface is part of the MAC block
//! (`MACMIIAR`/`MACMIIDR`), not a separate peripheral, so the internal
//! 10M/100M PHY needs no pins at all. Only an external RGMII PHY owns the
//! MDC/MDIO pads, and [`Sma`] keeps them.
//!
//! `MACMIIAR.CR` is always written as `0` (HCLK/42): it is the slowest of the
//! three encodings the H4 accepts and the one the EVT uses.

use super::{Error, MdcPin, MdioPin};
use crate::gpio::{AfType, OutputType, Pin, Speed};
use crate::{pac, peripherals, Peri};

const POLL_LIMIT: u32 = 1_000_000;
const RESET_ATTEMPTS: u16 = 1_000;

/// Reset value of `PHY_CFGR.PHYADDR_EN` selects address 1 for the internal PHY.
const INTERNAL_ADDRESS: u8 = 1;

// Clause 22 registers.
const REG_BMCR: u8 = 0x00;
const REG_BMSR: u8 = 0x01;
const REG_PHYIDR1: u8 = 0x02;
const REG_PHYIDR2: u8 = 0x03;
const REG_ANAR: u8 = 0x04;
const REG_ANLPAR: u8 = 0x05;
const REG_GB_CTRL: u8 = 0x09;
const REG_GB_STATUS: u8 = 0x0A;

// WCH internal PHY pages (EVT `ch32h417_eth.h`).
const REG_PAGE_SEL: u8 = 0x1F;
const REG_PHY_STATUS: u8 = 0x1A;
const REG_INTERRUPT_IND: u8 = 0x1E;
const REG_INTERRUPT_MASK: u8 = 0x13;
const PAGE0: u16 = 0;
const PAGE7: u16 = 7;

const BMCR_RESET: u16 = 1 << 15;
const BMCR_AN_ENABLE: u16 = 1 << 12;
const BMCR_AN_RESTART: u16 = 1 << 9;

const BMSR_LINK: u16 = 1 << 2;
const BMSR_AN_COMPLETE: u16 = 1 << 5;

/// WCH `PHY_STATUS` page 0 register.
const STATUS_SPEED_100: u16 = 1 << 3;
const STATUS_SPEED_10: u16 = 1 << 2;
const STATUS_FULL_DUPLEX: u16 = 1 << 1;

/// `ANLPAR` technology bits.
const LPA_100_FULL: u16 = 1 << 8;
const LPA_100_HALF: u16 = 1 << 7;
const LPA_10_FULL: u16 = 1 << 6;

/// `1000BASE-T status` link-partner bits.
const GB_STATUS_1000_FULL: u16 = 1 << 11;

/// Negotiated link speed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinkSpeed {
    Speed10,
    Speed100,
    Speed1000,
}

/// Negotiated link state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Link {
    Down,
    Up { speed: LinkSpeed, full_duplex: bool },
}

/// An Ethernet PHY behind the MAC's SMI.
pub trait Phy {
    /// Reset the PHY and start auto-negotiation.
    fn reset(&mut self) -> Result<(), Error>;
    /// Poll the link. Returns `None` if the state is unchanged since the last
    /// call, `Some` with the new state otherwise.
    fn poll_link(&mut self) -> Option<Link>;
    /// Clear a latched PHY interrupt, if the PHY has a usable one.
    fn clear_interrupt(&mut self) -> Result<(), Error> {
        Ok(())
    }
    /// PHY identifier, valid after [`Phy::reset`].
    fn phy_id(&self) -> u32;
}

fn wait(limit: u32, mut done: impl FnMut() -> bool, error: Error) -> Result<(), Error> {
    for _ in 0..limit {
        if done() {
            return Ok(());
        }
        core::hint::spin_loop();
    }
    Err(error)
}

fn smi_transfer(address: u8, register: u8, value: Option<u16>) -> Result<u16, Error> {
    let eth = pac::ETH;
    wait(POLL_LIMIT, || !eth.macmiiar().read().mb(), Error::SmiTimeout)?;
    if let Some(value) = value {
        eth.macmiidr().write(|w| w.set_md(value));
    }
    eth.macmiiar().write(|w| {
        w.set_cr(0);
        w.set_pa(address);
        w.set_mr(register);
        w.set_mw(value.is_some());
        w.set_mb(true);
    });
    wait(POLL_LIMIT, || !eth.macmiiar().read().mb(), Error::SmiTimeout)?;
    Ok(eth.macmiidr().read().md())
}

/// Raise the pad slew setting to 180 MHz for an alternate-function output.
///
/// `gpio::Speed` stops at 50 MHz because the other families do; the H4 pads
/// need the top setting for 125 MHz RGMII and for a clean MDC edge.
pub(super) fn set_pad_speed(pin: &impl Pin) {
    pac::GPIO(pin.port() as _)
        .speed()
        .modify(|w| w.set_speed(pin.pin() as usize, pac::gpio::vals::Speed::VERY_HIGH));
}

fn read_id(address: u8) -> Result<u32, Error> {
    let high = smi_transfer(address, REG_PHYIDR1, None)?;
    let low = smi_transfer(address, REG_PHYIDR2, None)?;
    Ok((u32::from(high) << 16) | u32::from(low))
}

/// MDC/MDIO pads for an external PHY.
pub struct Sma<'d> {
    _mdc: Peri<'d, crate::gpio::AnyPin>,
    _mdio: Peri<'d, crate::gpio::AnyPin>,
}

impl<'d> Sma<'d> {
    /// Configure `MDC` and `MDIO` as alternate-function outputs (AF1 on
    /// `PC0`/`PC1`, RM table 31-1) and take ownership of them.
    pub fn new(mdc: Peri<'d, impl MdcPin<peripherals::ETH>>, mdio: Peri<'d, impl MdioPin<peripherals::ETH>>) -> Self {
        let af = AfType::output(OutputType::PushPull, Speed::High);
        set_as_af!(mdc, af);
        set_as_af!(mdio, af);
        set_pad_speed(&*mdc);
        set_pad_speed(&*mdio);
        Self {
            _mdc: mdc.into(),
            _mdio: mdio.into(),
        }
    }

    pub fn read(&mut self, phy: u8, register: u8) -> Result<u16, Error> {
        smi_transfer(phy, register, None)
    }

    pub fn write(&mut self, phy: u8, register: u8, value: u16) -> Result<(), Error> {
        smi_transfer(phy, register, Some(value)).map(|_| ())
    }
}

/// The PHY integrated in the CH32H417.
///
/// It is 10M/100M only. Link and speed are read over SMI from page 0, the
/// same registers the EVT and the V4x7 driver use; `PHY_CFGR` reports the
/// same state and is used by the driver for the link-change interrupt.
pub struct InternalPhy {
    id: u32,
    link: Link,
}

impl InternalPhy {
    pub fn new() -> Self {
        Self {
            id: 0,
            link: Link::Down,
        }
    }

    fn poll(&mut self) -> Result<Link, Error> {
        let status = smi_transfer(INTERNAL_ADDRESS, REG_BMSR, None)?;
        let phy_status = smi_transfer(INTERNAL_ADDRESS, REG_PHY_STATUS, None)?;
        let anlpar = smi_transfer(INTERNAL_ADDRESS, REG_ANLPAR, None)?;
        let negotiated = status & BMSR_AN_COMPLETE != 0 || anlpar == 0;
        if status & BMSR_LINK == 0 || phy_status & (STATUS_SPEED_100 | STATUS_SPEED_10) == 0 || !negotiated {
            return Ok(Link::Down);
        }
        let speed = if phy_status & STATUS_SPEED_100 != 0 {
            LinkSpeed::Speed100
        } else {
            LinkSpeed::Speed10
        };
        Ok(Link::Up {
            speed,
            full_duplex: phy_status & STATUS_FULL_DUPLEX != 0,
        })
    }
}

impl Default for InternalPhy {
    fn default() -> Self {
        Self::new()
    }
}

impl Phy for InternalPhy {
    fn reset(&mut self) -> Result<(), Error> {
        // Power the PHY up and release its reset; the address stays at the
        // reset value of 1 because `PHYADDR_EN` is left clear.
        pac::ETH.phy_cfgr().modify(|w| {
            w.set_phy_pd(false);
            w.set_phy_rstn(true);
        });

        let mut id = 0;
        for _ in 0..RESET_ATTEMPTS {
            id = read_id(INTERNAL_ADDRESS)?;
            if id != 0 && id != u32::MAX {
                break;
            }
        }
        if id == 0 || id == u32::MAX {
            return Err(Error::PhyNotFound);
        }
        self.id = id;

        smi_transfer(INTERNAL_ADDRESS, REG_BMCR, Some(BMCR_RESET))?;
        wait(
            u32::from(RESET_ATTEMPTS),
            || matches!(smi_transfer(INTERNAL_ADDRESS, REG_BMCR, None), Ok(value) if value & BMCR_RESET == 0),
            Error::ResetTimeout,
        )?;

        // Enable the link-change interrupt on page 7 and clear the latched
        // status on page 0, as the EVT does.
        smi_transfer(INTERNAL_ADDRESS, REG_PAGE_SEL, Some(PAGE7))?;
        let mask = smi_transfer(INTERNAL_ADDRESS, REG_INTERRUPT_MASK, None)?;
        smi_transfer(
            INTERNAL_ADDRESS,
            REG_INTERRUPT_MASK,
            Some((mask | (1 << 13)) & !(1 << 9)),
        )?;
        smi_transfer(INTERNAL_ADDRESS, REG_PAGE_SEL, Some(PAGE0))?;
        smi_transfer(INTERNAL_ADDRESS, REG_INTERRUPT_IND, None)?;

        // The PHY only wants 10M/100M; leave the default advertisement and
        // restart auto-negotiation.
        smi_transfer(INTERNAL_ADDRESS, REG_BMCR, Some(BMCR_AN_ENABLE | BMCR_AN_RESTART))?;
        self.link = Link::Down;
        Ok(())
    }

    fn poll_link(&mut self) -> Option<Link> {
        let link = self.poll().unwrap_or(Link::Down);
        if link == self.link {
            None
        } else {
            self.link = link;
            Some(link)
        }
    }

    fn clear_interrupt(&mut self) -> Result<(), Error> {
        smi_transfer(INTERNAL_ADDRESS, REG_INTERRUPT_IND, None).map(|_| ())
    }

    fn phy_id(&self) -> u32 {
        self.id
    }
}

/// A generic IEEE 802.3 PHY on an external SMI bus.
pub struct GenericPhy<'d> {
    sma: Sma<'d>,
    address: Option<u8>,
    id: u32,
    link: Link,
}

impl<'d> GenericPhy<'d> {
    pub fn new(sma: Sma<'d>, address: u8) -> Self {
        Self {
            sma,
            address: Some(address),
            id: 0,
            link: Link::Down,
        }
    }

    /// Probe addresses 0..=31 during [`Phy::reset`].
    pub fn new_auto(sma: Sma<'d>) -> Self {
        Self {
            sma,
            address: None,
            id: 0,
            link: Link::Down,
        }
    }

    fn reset_at(&mut self, address: u8) -> Result<(), Error> {
        self.id = read_id(address)?;

        self.sma.write(address, REG_BMCR, BMCR_RESET)?;
        wait(
            u32::from(RESET_ATTEMPTS),
            || matches!(self.sma.read(address, REG_BMCR), Ok(value) if value & BMCR_RESET == 0),
            Error::ResetTimeout,
        )?;

        // Advertise 10/100 full and half duplex (802.3 selector in the low
        // bits) and 1000BASE-T full duplex only: RGMII has no half duplex.
        self.sma.write(address, REG_ANAR, 0x01E1)?;
        self.sma.write(address, REG_GB_CTRL, 1 << 9)?;
        self.sma.write(address, REG_BMCR, BMCR_AN_ENABLE | BMCR_AN_RESTART)?;

        self.address = Some(address);
        self.link = Link::Down;
        Ok(())
    }

    fn poll(&mut self) -> Result<Link, Error> {
        let address = self.address.ok_or(Error::PhyNotFound)?;
        // `BMSR` is latch-low; the second read returns the live value.
        let _ = self.sma.read(address, REG_BMSR)?;
        let status = self.sma.read(address, REG_BMSR)?;
        if status & BMSR_LINK == 0 {
            return Ok(Link::Down);
        }
        let gb_status = self.sma.read(address, REG_GB_STATUS)?;
        let anlpar = self.sma.read(address, REG_ANLPAR)?;
        let (speed, full_duplex) = if gb_status & GB_STATUS_1000_FULL != 0 {
            (LinkSpeed::Speed1000, true)
        } else if anlpar & LPA_100_FULL != 0 {
            (LinkSpeed::Speed100, true)
        } else if anlpar & LPA_100_HALF != 0 {
            (LinkSpeed::Speed100, false)
        } else if anlpar & LPA_10_FULL != 0 {
            (LinkSpeed::Speed10, true)
        } else {
            (LinkSpeed::Speed10, false)
        };
        Ok(Link::Up { speed, full_duplex })
    }
}

impl Phy for GenericPhy<'_> {
    fn reset(&mut self) -> Result<(), Error> {
        if let Some(address) = self.address {
            return self.reset_at(address);
        }
        for address in 0..32 {
            let id = read_id(address)?;
            if id == 0 || id == u32::MAX {
                continue;
            }
            match self.reset_at(address) {
                Ok(()) => return Ok(()),
                Err(Error::ResetTimeout) => {}
                Err(error) => return Err(error),
            }
        }
        Err(Error::PhyNotFound)
    }

    fn poll_link(&mut self) -> Option<Link> {
        let link = self.poll().unwrap_or(Link::Down);
        if link == self.link {
            None
        } else {
            self.link = link;
            Some(link)
        }
    }

    fn phy_id(&self) -> u32 {
        self.id
    }
}
