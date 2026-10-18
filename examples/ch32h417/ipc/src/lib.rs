//! The CH32H417 dual-core mailbox: one definition, shared by both harts.
//!
//! Both example crates depend on this crate, so `MAILBOX` is placed at
//! `ORIGIN(SRAM_SHARED)` in *both* images by the same linker fragment
//! (`shared.x`), instead of each example carrying its own copy of
//! `0x2017_80xx`. `xtask report` links this crate too and reads the fields
//! through [`core::mem::offset_of!`], so the layout lives in exactly one place.
//!
//! # Why a fixed address at all
//!
//! The two harts are separate binaries with separate linker scripts; the only
//! thing that makes a shared variable the *same* variable on both sides is that
//! both place it in the same region. `shared.x` puts `.shared` at the start of
//! `SRAM_SHARED` (`0x2017_8000`, 32K, declared in both crates' `memory.x`), so
//! the linker guarantees the address.
//!
//! `.shared` is `NOLOAD`: it costs no flash and no startup code zeroes it, which
//! is what a mailbox wants — surviving a reset of one core while the other
//! keeps running. The boot core clears the whole structure before handing over.
//!
//! # What lives here
//!
//! Only the container. [`Mailbox`] carries a layout version and one payload of
//! type `T`; the payload is the example's own `#[repr(C)]` protocol and is
//! defined next to that example, not in this crate. One image still places
//! exactly one shared object, so `xtask` exports a single address and the other
//! hart consumes it.
//!
//! # Rules
//!
//! * Only a single `#[link_section = ".shared"]` item may exist per image, and it
//!   must be this crate's [`MAILBOX`] — the section is placed at the region start
//!   in declaration order, so a second one would shift the layout on one side.
//! * Writers clear [`Mailbox::layout_version`] to zero and set it to
//!   [`LAYOUT_VERSION`] when the payload is ready; a reader that finds another
//!   value knows the two images disagree and should not trust the payload.
//! * 32-bit aligned stores only: a single writer and a single reader need no
//!   locking (the examples keep to one writer per field), but anything richer
//!   needs release/acquire ordering around the flag.

#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

pub mod protocols;

/// Where the mailbox lives: the start of `SRAM_SHARED`.
///
/// Both harts declare that region at `ORIGIN = 0x20178000` in their `memory.x`,
/// and `shared.x` places the one `.shared` item at its start, so the address is
/// fixed by the linker scripts rather than discovered from an ELF. Moving the
/// region means changing both `memory.x` files and this constant together.
pub const MAILBOX_ADDR: usize = 0x2017_8000;

/// Bumped when the *container* changes shape. The payload is the example's own
/// type, so adding a field to one example does not change this.
///
/// Both images are flashed independently, so a stale half must be able to notice
/// it does not match.
pub const LAYOUT_VERSION: u32 = 1;

/// What an example stores in the mailbox.
///
/// The layout has to be fixed by `#[repr(C)]` rather than by rustc, because the
/// two harts and `xtask report` all read it. It cannot be `Copy`: the fields are
/// atomics, which are shared precisely because they are not copied.
pub trait Protocol {
    /// The value [`Mailbox::clear`] writes over the payload, and the one the
    /// defining side's static starts as.
    const ZERO: Self;
}

/// The shared structure: a version word and one payload.
///
/// Field order is the ABI. `T` is the example's protocol; the version word stays
/// in front of it so a reader can reject a block before touching the payload.
#[repr(C)]
pub struct Mailbox<T: Protocol> {
    /// [`LAYOUT_VERSION`] once the boot core has cleared the block; 0 while the
    /// block is being reset.
    pub layout_version: AtomicU32,
    /// The example's own data.
    pub payload: T,
}

impl<T: Protocol> Mailbox<T> {
    /// Initialiser for the boot core's static; the consuming side never
    /// constructs one, so this is gated with the `define` feature to keep the
    /// second core's build warning-free.
    #[cfg(feature = "define")]
    pub const fn new() -> Self {
        Self {
            layout_version: AtomicU32::new(0),
            payload: T::ZERO,
        }
    }

    /// Zero the payload and stamp the layout version. The boot core calls this
    /// before handing over, so a report describes the current run rather than
    /// whatever an earlier example left in shared RAM.
    pub fn clear(&self) {
        // Drop the version first, so a reader never trusts a half-cleared block.
        self.layout_version.store(0, Ordering::Relaxed);
        // Safety: the payload is a `repr(C)` block of atomics, so copying the
        // zero value over it byte by byte drops nothing. The version word above
        // tells a reader not to look until the store below.
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of!(self.payload) as *mut T, T::ZERO);
        }
        self.layout_version.store(LAYOUT_VERSION, Ordering::Relaxed);
    }

    /// Whether the block was written by an image with the same container layout.
    pub fn is_compatible(&self) -> bool {
        self.layout_version.load(Ordering::Relaxed) == LAYOUT_VERSION
    }
}

/// The mailbox, defined **once**, by the boot core (V3F): `shared.x` places
/// `.shared` at the start of `SRAM_SHARED` in that image. The second core never
/// places one; it reads [`MAILBOX_ADDR`], which names the same spot.
///
/// `T` is whatever protocol the example named in its `mailbox!` invocation, so
/// each example carries only the words it uses.
///
/// The section attribute only applies to the bare-metal targets: `xtask report`
/// builds this crate for the host, where it needs the field offsets (via
/// [`core::mem::offset_of!`]) and not the instance.
#[cfg(feature = "define")]
#[macro_export]
macro_rules! mailbox {
    ($payload:ty) => {
        #[cfg(target_os = "none")]
        #[link_section = ".shared"]
        #[no_mangle]
        pub static MAILBOX: $crate::Mailbox<$payload> = $crate::Mailbox::new();

        /// The mailbox this image defines.
        pub fn mailbox() -> &'static $crate::Mailbox<$payload> {
            &MAILBOX
        }
    };
}

/// View the mailbox at `addr`, for the core that receives the address.
///
/// # Safety
///
/// `addr` must be the address the defining core exported for a mailbox of this
/// `T`, and the two cores must agree on [`LAYOUT_VERSION`].
pub unsafe fn at<T: Protocol>(addr: usize) -> &'static Mailbox<T> {
    unsafe { &*(addr as *const Mailbox<T>) }
}
