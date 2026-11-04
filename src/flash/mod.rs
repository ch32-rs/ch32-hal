//! Flash memory (FLASH)
//! CH32 FLASH is relatively close to STM32. This module is heavily inspired from embassy-stm32
use embedded_storage::nor_flash::{NorFlashError, NorFlashErrorKind};

#[cfg(not(flash_v4x7))]
mod common;
#[cfg(not(flash_v4x7))]
pub use common::*;

#[cfg(flash_v4x7)]
mod v4x7;
#[cfg(flash_v4x7)]
pub use v4x7::*;

// Re-declare the legacy geometry instead of re-exporting the PAC constants:
// the current metapac marks them deprecated in favor of `MEMORY_LAYOUT`, and
// re-exporting would push that warning into every non-V4 build (including
// downstream crates that deny warnings). The values are unchanged.
#[cfg(not(flash_v4x7))]
#[allow(deprecated)]
pub const FLASH_SIZE: usize = crate::pac::FLASH_SIZE;
#[cfg(not(flash_v4x7))]
#[allow(deprecated)]
pub const WRITE_SIZE: usize = crate::pac::WRITE_SIZE;

#[cfg(flash_v4x7)]
pub const FLASH_SIZE: usize = {
    let regions = crate::pac::MEMORY_LAYOUT.regions;
    let mut capacity = 0;
    let mut index = 0;
    while index < regions.len() {
        if matches!(regions[index].role, crate::pac::mem_layout::MemoryRole::Application) {
            core::assert!(regions[index].address as usize == 0x0800_0000 + capacity);
            capacity += regions[index].size as usize;
        }
        index += 1;
    }
    capacity
};

#[cfg(flash_v4x7)]
pub const WRITE_SIZE: usize =
    match crate::pac::MEMORY_LAYOUT.find_by_role(crate::pac::mem_layout::MemoryRole::Application) {
        Some(region) => match region.page_program_size() {
            Some(size) => size as usize,
            None => core::panic!("V4 flash requires page programming geometry"),
        },
        None => core::panic!("V4 flash requires application memory"),
    };

/// Read size (always 1)
pub const READ_SIZE: usize = 1;

/// Blocking flash mode typestate.
pub enum Blocking {}
/// Async flash mode typestate.
pub enum Async {}

/// Flash sector.
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashSector {
    /// Absolute start address.
    pub start: u32,
    /// Size in bytes.
    pub size: u32,
}

#[cfg_attr(flash_v3, path = "v3.rs")]
#[cfg_attr(not(flash_v3), path = "other.rs")]
#[cfg(not(flash_v4x7))]
mod family;

#[allow(unused_imports)]
#[cfg(not(flash_v4x7))]
pub use family::*;

/// Flash error
///
/// See CH32 Reference Manual for your chip for details.
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Prog,
    Size,
    Miss,
    Seq,
    Protected,
    Unaligned,
    Parallelism,
    #[cfg(flash_v4x7)]
    Busy,
    #[cfg(flash_v4x7)]
    Timeout,
    #[cfg(flash_v4x7)]
    Verify,
    #[cfg(flash_v4x7)]
    NotErased,
    #[cfg(flash_v4x7)]
    InvalidExecutionRegion,
}

impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Size => NorFlashErrorKind::OutOfBounds,
            Self::Unaligned => NorFlashErrorKind::NotAligned,
            _ => NorFlashErrorKind::Other,
        }
    }
}
