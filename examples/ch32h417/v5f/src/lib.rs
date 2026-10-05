//! Shared pieces for the V5F (hart 1) examples.
//!
//! Deliberately built on `ch32-metapac` alone — no `ch32-hal`, no embassy. hart 1
//! wakes into a chip the boot core has already brought up: there is one RCC
//! block shared by both cores, and both the HAL's `Peripherals::take()` singleton
//! and its PLL re-init assume they own the chip and run once. Examples therefore
//! drive registers through [`pac`] and, for printf, through [`sdi`].
//!
//! See `README.md` ("Writing a V5F half") for the rule and `docs/backlog.md` for
//! the deferred `ch32-hal` secondary-core entry point (`init_secondary()` /
//! `Clocks::noinit()`), which would let hart 1 use HAL *drivers* without
//! re-running the HAL's bring-up.

#![no_std]

/// `ch32-metapac` for the V5F hart, re-exported so examples have one import path.
pub use ch32_metapac as pac;

pub mod sdi;
