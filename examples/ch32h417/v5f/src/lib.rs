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
//!
//! # Where the shared mailbox lives
//!
//! This image does **not** define the mailbox. The boot core's image does (one
//! `#[link_section = ".shared"]` item, see the `ch32h417-ipc` crate), and `xtask`
//! post-processes the V3F ELF into `out/layout.txt`, which `build.rs` turns into
//! the constants included below. So the address is decided by the core that owns
//! the region, and the `const` assertion in [`mailbox`] fails the build if this
//! crate's idea of the structure disagrees with what the V3F actually placed.

#![no_std]

/// `ch32-metapac` for the V5F hart, re-exported so examples have one import path.
pub use ch32_metapac as pac;

pub mod cache;
pub mod cpuid;
pub mod sdi;

/// Layout of the shared region, as exported by the boot core's ELF.
pub mod layout {
    include!(concat!(env!("OUT_DIR"), "/layout.rs"));
}

/// The shared mailbox, at the address the boot core exported for it.
///
/// The `const` assertion is the cross-image check: the size the V3F reported for
/// `.shared` must equal this crate's `size_of::<Mailbox>()`, so a layout change
/// on one side fails the other side's build instead of corrupting memory.
pub fn mailbox() -> &'static ch32h417_ipc::Mailbox {
    const _: () = assert!(layout::MAILBOX_SIZE == ch32h417_ipc::MAILBOX_SIZE);
    // safety: the address comes from the V3F image that defines the mailbox, and
    // the size assertion above ties it to this crate's view of the structure.
    unsafe { ch32h417_ipc::at(layout::MAILBOX_ADDR) }
}
