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
//! keeps running. Every field starts as zero, and the boot core clears the whole
//! structure before handing over (see [`MAILBOX`]'s writer side).
//!
//! # Rules
//!
//! * Only a single `#[link_section = ".shared"]` item may exist per image, and it
//!   must be this crate's [`MAILBOX`] — the section is placed at the region start
//!   in declaration order, so a second one would shift the layout on one side.
//! * Writers clear [`Mailbox::layout_version`] to zero and set it to
//!   [`LAYOUT_VERSION`] when the fields are ready; a reader that finds another
//!   value knows the two images disagree and should not trust the payload.
//! * 32-bit aligned stores only: a single writer and a single reader need no
//!   locking (the examples keep to one writer per field), but anything richer
//!   needs release/acquire ordering around the flag.

#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

/// Bumped whenever the field layout below changes. Both images are flashed
/// independently, so a stale half must be able to notice it does not match.
pub const LAYOUT_VERSION: u32 = 3;

/// Number of probed CSRs in [`Mailbox::cpuid_values`].
pub const CPUID_CSRS: usize = 14;

/// Size of the structure, checked against the linker's own view of `.shared`.
pub const MAILBOX_SIZE: usize = core::mem::size_of::<Mailbox>();

/// Names of the CSRs hart 1 probes, in [`Mailbox::cpuid_values`] order. The
/// examples print them; `xtask report` labels them.
pub const CPUID_CSR_NAMES: [&str; CPUID_CSRS] = [
    "mvendorid",
    "marchid",
    "mimpid",
    "mhartid",
    "misa",
    "mstatus",
    "mtvec",
    "mepc",
    "mcause",
    "corecfgr",
    "intsyscr",
    "gintenr",
    "cache_strtg_ctlr",
    "cache_pmp_ovr",
];

/// Index of `misa` in [`CPUID_CSR_NAMES`].
pub const CPUID_MISA_INDEX: usize = 4;

/// Written by the boot core's `dualcore` half once hart 1 is alive.
pub const DUALCORE_MARKER: u32 = 0xDEAD_BEEF;

/// Written by hart 1's `cpuid` half once its whole block is in place.
pub const CPUID_DONE: u32 = 0xC0DE_0001;

/// Increments each core performs in the `atomics` example, on the *same* word.
pub const CAS_INCREMENTS: u32 = 1_000_000;

/// `cas_done` bits, one per core.
pub const CAS_DONE_V3F: u32 = 1 << 0;
pub const CAS_DONE_V5F: u32 = 1 << 1;

/// The shared structure. Field order is the ABI — append new fields at the end
/// and bump [`LAYOUT_VERSION`].
#[repr(C)]
pub struct Mailbox {
    /// [`LAYOUT_VERSION`] once the boot core has cleared the block; 0 while the
    /// block is being reset.
    pub layout_version: AtomicU32,
    /// `sdi_cpuid`: 1 while hart 1 owns the SDI console, 0 when the boot core does.
    pub sdi_token: AtomicU32,
    /// `sdi_cpuid`: hart 1's liveness counter.
    pub sdi_ticks: AtomicU32,
    /// `dualcore`: [`DUALCORE_MARKER`] once hart 1 has run.
    pub dualcore_marker: AtomicU32,
    /// `dualcore`: the boot core's loop counter.
    pub dualcore_counter: AtomicU32,
    /// `cpuid`: bit `i` set when `cpuid_values[i]` could be read.
    pub cpuid_present: AtomicU32,
    /// `cpuid`: hart 1's own Machine-mode CSRs, in `v5f/src/bin/cpuid.rs` order.
    pub cpuid_values: [AtomicU32; CPUID_CSRS],
    /// `cpuid`: ISA features the V5F image was built for.
    pub cpuid_buildcfg: AtomicU32,
    /// `cpuid`: [`CPUID_DONE`] when the block is complete.
    pub cpuid_done: AtomicU32,
    /// `cpuid`: how many CSRs hart 1 has probed, for diagnosing a hang.
    pub cpuid_progress: AtomicU32,
    /// `pingpong`: hart 1's round counter, incremented once per answered round.
    pub ping: AtomicU32,
    /// `atomics`: the contended word both cores increment with `fetch_add`; the
    /// total must equal `2 * CAS_INCREMENTS` for the hardware atomics to be
    /// usable across cores.
    pub cas_total: AtomicU32,
    /// `atomics`: [`CAS_DONE_V3F`] / [`CAS_DONE_V5F`], set once a core has
    /// finished its increments.
    pub cas_done: AtomicU32,
    /// `pingpong`: the boot core's echo — equal to [`Self::ping`] once it has
    /// answered that round, so `ping != pong` means "a round is outstanding".
    pub pong: AtomicU32,
}

impl Mailbox {
    /// Initialiser for the boot core's static; the consuming side never
    /// constructs one, so this is gated with the `define` feature to keep the
    /// second core's build warning-free.
    #[cfg(feature = "define")]
    const fn new() -> Self {
        Self {
            layout_version: AtomicU32::new(0),
            sdi_token: AtomicU32::new(0),
            sdi_ticks: AtomicU32::new(0),
            dualcore_marker: AtomicU32::new(0),
            dualcore_counter: AtomicU32::new(0),
            cpuid_present: AtomicU32::new(0),
            cpuid_values: [const { AtomicU32::new(0) }; CPUID_CSRS],
            cpuid_buildcfg: AtomicU32::new(0),
            cpuid_done: AtomicU32::new(0),
            cpuid_progress: AtomicU32::new(0),
            cas_total: AtomicU32::new(0),
            cas_done: AtomicU32::new(0),
            ping: AtomicU32::new(0),
            pong: AtomicU32::new(0),
        }
    }

    /// Zero every field and leave the layout unversioned; the boot core calls
    /// this before handing over, so the report describes the current run rather
    /// than whatever an earlier example left in shared RAM.
    pub fn clear(&self) {
        self.layout_version.store(0, Ordering::Relaxed);
        self.sdi_token.store(0, Ordering::Relaxed);
        self.sdi_ticks.store(0, Ordering::Relaxed);
        self.dualcore_marker.store(0, Ordering::Relaxed);
        self.dualcore_counter.store(0, Ordering::Relaxed);
        self.cpuid_present.store(0, Ordering::Relaxed);
        for slot in &self.cpuid_values {
            slot.store(0, Ordering::Relaxed);
        }
        self.cpuid_buildcfg.store(0, Ordering::Relaxed);
        self.cpuid_done.store(0, Ordering::Relaxed);
        self.cpuid_progress.store(0, Ordering::Relaxed);
        self.cas_total.store(0, Ordering::Relaxed);
        self.cas_done.store(0, Ordering::Relaxed);
        self.ping.store(0, Ordering::Relaxed);
        self.pong.store(0, Ordering::Relaxed);
        self.layout_version.store(LAYOUT_VERSION, Ordering::Relaxed);
    }

    /// Whether the block was written by an image with the same layout.
    pub fn is_compatible(&self) -> bool {
        self.layout_version.load(Ordering::Relaxed) == LAYOUT_VERSION
    }

    /// Read [`Self::cpuid_values`] into a plain array.
    pub fn cpuid_snapshot(&self) -> [u32; CPUID_CSRS] {
        let mut values = [0u32; CPUID_CSRS];
        for (out, slot) in values.iter_mut().zip(&self.cpuid_values) {
            *out = slot.load(Ordering::Relaxed);
        }
        values
    }
}

/// The mailbox, defined **once**, by the boot core (V3F): `shared.x` places
/// `.shared` at `ORIGIN(SRAM_SHARED)` in that image, and this symbol is what the
/// second core consumes — either as an address exported from the V3F ELF
/// (`xtask` post-processes it into `out/layout.rs`) or, later, as a
/// linker-provided symbol (`--defsym`/`PROVIDE`).
///
/// The section attribute only applies to the bare-metal targets: `xtask report`
/// builds this crate for the host, where it needs the field offsets (via
/// [`core::mem::offset_of!`]) and not the instance.
#[cfg(all(feature = "define", target_os = "none"))]
#[link_section = ".shared"]
#[no_mangle]
pub static MAILBOX: Mailbox = Mailbox::new();

/// Convenience accessor for the core that *defines* the mailbox — available only
/// with the `define` feature, which only the boot core's image enables.
#[cfg(feature = "define")]
pub fn mailbox() -> &'static Mailbox {
    &MAILBOX
}

/// View the mailbox at `addr`, for the core that receives the address.
///
/// # Safety
///
/// `addr` must be the address the defining core exported for this structure, and
/// the two cores must agree on [`LAYOUT_VERSION`].
pub unsafe fn at(addr: usize) -> &'static Mailbox {
    unsafe { &*(addr as *const Mailbox) }
}
