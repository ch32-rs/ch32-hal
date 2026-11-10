//! MAC management (MMC) counters.
//!
//! `ch32-metapac` does not describe the EMAC MMC block yet, so the two receive
//! counters the built-in 10M PHY needs are read at their offsets from the MAC
//! base; the block starts at `0x100` and `MMCRFCECR`/`MMCRGUFCR` sit at `0x194`
//! and `0x1C4`. Move this into the PAC once `ch32-data` describes the block.

#[cfg(emac)]
use crate::pac::emac::Eth as EthRegs;
#[cfg(not(emac))]
use crate::pac::eth::Eth as EthRegs;

/// Offset of the MMC block from the MAC base.
const MMC_BASE: usize = 0x100;
/// `MMCCR`: bit 0 resets every counter and clears itself.
const MMCCR: usize = 0x00;
/// `MMCRFCECR`: frames received with an FCS error.
const MMCRFCECR: usize = 0x94;
/// `MMCRGUFCR`: unicast frames received with a good FCS.
const MMCRGUFCR: usize = 0xC4;

/// How many polls to wait for `MMCCR.CR` to clear.
const RESET_POLL_LIMIT: usize = 1_000;

/// Receive counters accumulated since the last reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct RxCounters {
    /// Unicast frames addressed to this MAC that arrived with a good FCS.
    pub good_unicast: u32,
    /// Frames that passed the address filter but failed their FCS.
    pub crc_errors: u32,
}

/// The EMAC MMC counters.
pub(super) struct Mmc {
    regs: EthRegs,
}

impl Mmc {
    pub(super) const fn new(regs: EthRegs) -> Self {
        Self { regs }
    }

    fn addr(&self, offset: usize) -> usize {
        self.regs.as_ptr() as usize + MMC_BASE + offset
    }

    fn read(&self, offset: usize) -> u32 {
        // Safety: the offset is inside the MMC block, whose clock is enabled
        // with the rest of the MAC in `new_inner`.
        unsafe { core::ptr::read_volatile(self.addr(offset) as *const u32) }
    }

    /// Receive counters since the last reset.
    pub(super) fn rx(&self) -> RxCounters {
        RxCounters {
            good_unicast: self.read(MMCRGUFCR),
            crc_errors: self.read(MMCRFCECR),
        }
    }

    /// Restart every MMC counter.
    ///
    /// The counters only describe one link, so they are restarted at link-up
    /// and after every receive polarity swap.
    pub(super) fn reset(&self) {
        // Safety: as in `read`.
        unsafe { core::ptr::write_volatile(self.addr(MMCCR) as *mut u32, 1) };
        for _ in 0..RESET_POLL_LIMIT {
            if self.read(MMCCR) & 1 == 0 {
                break;
            }
        }
    }
}
