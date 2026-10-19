//! Shared pieces for the V5F (hart 1) examples.
//!
//! hart 1 wakes into a chip the boot core has already brought up, and there is
//! one RCC block shared by both cores, so this image never calls
//! `ch32_hal::init` — that re-programs the PLL and then waits for the switch.
//! `ch32_hal::init_secondary` only reads the clock tree back. Examples drive
//! registers through [`pac`] and, for printf, through [`sdi`].
//!
//! # Where the shared mailbox lives
//!
//! This image does **not** define the mailbox. The boot core's image does (one
//! `#[link_section = ".shared"]` item, see the `ch32h417-ipc` crate), placed at the
//! start of `SRAM_SHARED` by `shared.x`. Both harts declare that region at the
//! same address, so this half consumes [`ch32h417_ipc::MAILBOX_ADDR`] rather than
//! learning it from the other image.

#![no_std]

/// `ch32-metapac` for the V5F hart, re-exported so examples have one import path.
pub use ch32_hal::pac;

pub mod cache;
pub mod cpuid;
pub mod i2c;
pub mod sdi;

/// The shared mailbox, at [`ch32h417_ipc::MAILBOX_ADDR`].
///
/// `T` is the protocol this example stores there. Both halves of an example are
/// the same bin built against that same protocol.
pub fn mailbox<T: ch32h417_ipc::Protocol>() -> &'static ch32h417_ipc::Mailbox<T> {
    // safety: the address is the start of `SRAM_SHARED`, where `shared.x` places
    // the one mailbox the boot core defines.
    unsafe { ch32h417_ipc::at(ch32h417_ipc::MAILBOX_ADDR) }
}
