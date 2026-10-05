//! Probing hart 1's own Machine-mode CSRs into the shared mailbox.
//!
//! Shared by the V5F halves that need to describe themselves (`cpuid`,
//! `pingpong`). The boot core prints the block back out; see
//! `v3f/src/bin/cpuid.rs`.
//!
//! Each CSR is probed with a recoverable illegal-instruction handler, because
//! the QingKe V5 manual's CSR table lists CSRs the core does not implement; a
//! direct `csrr` of one of those would hang this image. The binary supplies the
//! handler (it overrides qingke-rt's weak `ExceptionHandler`) and only has to
//! call [`note_trap`] from it.

use core::sync::atomic::{AtomicU32, Ordering};

use ch32h417_ipc as ipc;
use ipc::Mailbox;

/// Set by the binary's `ExceptionHandler` when a `csrr` faulted.
static TRAP_HIT: AtomicU32 = AtomicU32::new(0);

/// Called from the binary's exception handler on a faulted CSR read.
pub fn note_trap() {
    TRAP_HIT.store(1, Ordering::Relaxed);
}

/// Read and clear the trap flag. Plain load/store rather than `swap`: hart 1's
/// atomics come from the critical-section fallback (the `unsafe-trust-wch-atomics`
/// feature is only enabled for the boot core), and this is single-threaded
/// per core anyway.
fn take_trap() -> bool {
    let hit = TRAP_HIT.load(Ordering::Relaxed) != 0;
    TRAP_HIT.store(0, Ordering::Relaxed);
    hit
}

/// Read one CSR, returning `(value, present)` — `present` is false when the
/// read raised an illegal instruction.
///
/// A macro rather than a function: `csrr` takes its CSR number as an immediate.
macro_rules! probe_csr {
    ($addr:literal) => {{
        let mut v: u32 = 0;
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $addr), inout(reg) v) };
        let trapped = take_trap();
        (v, !trapped)
    }};
}

/// ISA this image was built for (bit0 M, bit1 A, bit2 F, bit3 D, bit4 C, bit5 B)
/// — compare with what the hardware implements.
pub fn build_features() -> u32 {
    let mut f = 0u32;
    if cfg!(target_feature = "m") {
        f |= 1 << 0;
    }
    if cfg!(target_feature = "a") {
        f |= 1 << 1;
    }
    if cfg!(target_feature = "f") {
        f |= 1 << 2;
    }
    if cfg!(target_feature = "d") {
        f |= 1 << 3;
    }
    if cfg!(target_feature = "c") {
        f |= 1 << 4;
    }
    if cfg!(target_feature = "b") {
        f |= 1 << 5;
    }
    f
}

/// Fill `mailbox`'s CPU-ID block with this hart's own values.
///
/// The order matches [`ipc::CPUID_CSR_NAMES`] (and the boot core's printer). The
/// block is invalidated first, so a stale `DONE` from an earlier run cannot make
/// the boot core read a half-written block as if it were new.
pub fn report_self(mailbox: &Mailbox) {
    mailbox.cpuid_progress.store(0, Ordering::Relaxed);
    mailbox.cpuid_done.store(0, Ordering::Relaxed);

    let mut present_mask = 0u32;

    macro_rules! report {
        ($i:expr, $addr:literal) => {{
            let (value, present) = probe_csr!($addr);
            mailbox.cpuid_values[$i].store(value, Ordering::Relaxed);
            if present {
                present_mask |= 1 << $i;
            }
            // Advance the progress marker so a hang is localisable.
            mailbox.cpuid_progress.store($i + 1, Ordering::Relaxed);
        }};
    }

    report!(0, "0xf11"); // mvendorid
    report!(1, "0xf12"); // marchid
    report!(2, "0xf13"); // mimpid
    report!(3, "0xf14"); // mhartid
    report!(4, "0x301"); // misa
    report!(5, "0x300"); // mstatus
    report!(6, "0x305"); // mtvec
    report!(7, "0x341"); // mepc
    report!(8, "0x342"); // mcause
    report!(9, "0xbc0"); // corecfgr
    report!(10, "0x804"); // intsyscr
    report!(11, "0x800"); // gintenr
    report!(12, "0xbc2"); // cache_strtg_ctlr (QingKe V5-specific)
    report!(13, "0xbc3"); // cache_pmp_ovr   (QingKe V5-specific)

    mailbox.cpuid_buildcfg.store(build_features(), Ordering::Relaxed);
    mailbox
        .cpuid_present
        .store(present_mask, Ordering::Relaxed);
    // Last: tells the boot core the block above is complete.
    mailbox.cpuid_done.store(ipc::CPUID_DONE, Ordering::Relaxed);
}
