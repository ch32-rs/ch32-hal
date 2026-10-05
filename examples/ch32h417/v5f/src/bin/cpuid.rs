//! CPU-ID report — second core (V5F, hart 1).
//!
//! Probes its identity/ISA CSRs and stores the results in the shared-RAM
//! mailbox; the boot core prints them (`v3f/src/bin/cpuid.rs`).
//!
//! Deliberately minimal: metapac only, no `ch32-hal` and no embassy (there is
//! one shared RCC block and
//! the boot core has already programmed it, including this core's `FPRE`),
//! and no SDI or `core::fmt` — the V5F's flash-resident execution is slow, so
//! only the CSR probes and the stores happen here.
//!
//! Each CSR is probed with a recoverable illegal-instruction handler, because
//! the QingKe V5 manual's CSR table lists CSRs that the core does not
//! implement; a direct `csrr` of one of those would hang this image.

#![no_std]
#![no_main]

use ch32h417_ipc as ipc;
use ch32h417_v5f::mailbox;
use panic_halt as _;

/// Number of probed CSRs; keep in sync with the boot core's `CSR_NAMES`.
const N_CSRS: u32 = 14;

/// The probed CSRs must match the mailbox's value block.
const _: () = assert!(N_CSRS as usize == ipc::CPUID_CSRS);

static mut TRAP_HIT: u32 = 0;

/// Strong override of qingke-rt's weak `ExceptionHandler`: record the fault
/// and resume past the offending instruction.
#[no_mangle]
pub extern "C" fn ExceptionHandler() {
    let epc: u32;
    unsafe { core::arch::asm!("csrr {}, mepc", out(reg) epc) };
    let halfword = unsafe { core::ptr::read_volatile(epc as *const u16) } as u32;
    let len = if halfword & 0b11 == 0b11 { 4 } else { 2 };
    unsafe {
        core::ptr::write_volatile(&raw mut TRAP_HIT, 1);
        core::arch::asm!("csrw mepc, {}", in(reg) epc + len);
    }
}

/// Probe a CSR: returns `(value, present)`.
macro_rules! probe_csr {
    ($addr:literal) => {{
        let mut v: u32 = 0;
        unsafe { core::ptr::write_volatile(&raw mut TRAP_HIT, 0) };
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $addr), inout(reg) v) };
        let trapped = unsafe { core::ptr::read_volatile(&raw const TRAP_HIT) } != 0;
        (v, !trapped)
    }};
}

/// ISA this image was built for (bit0 M, bit1 A, bit2 F, bit3 D, bit4 C,
/// bit5 B) — compare with what the hardware implements.
fn build_features() -> u32 {
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

#[qingke_rt::entry]
fn main() -> ! {
    let mut present_mask = 0u32;
    // Invalidate the previous report first: the boot core clears the mailbox for
    // the `cpuid` example, but a generic `launcher` does not, and a stale
    // `DONE_MAGIC` would make `xtask report` show an old block as if it were new.
    unsafe {
        mailbox().cpuid_progress.store(0, core::sync::atomic::Ordering::Relaxed);
        mailbox().cpuid_done.store(0, core::sync::atomic::Ordering::Relaxed);
    }

    macro_rules! report {
        ($i:expr, $addr:literal) => {{
            let (v, present) = probe_csr!($addr);
            mailbox().cpuid_values[$i].store(v, core::sync::atomic::Ordering::Relaxed);
            if present {
                present_mask |= 1 << $i;
            }
            // Advance the progress marker so a hang is localisable.
            mailbox().cpuid_progress.store($i + 1, core::sync::atomic::Ordering::Relaxed);
        }};
    }

    // Keep the order in sync with `CSR_NAMES` in the boot core's copy.
    report!(0, "0xf11");  // mvendorid
    report!(1, "0xf12");  // marchid
    report!(2, "0xf13");  // mimpid
    report!(3, "0xf14");  // mhartid
    report!(4, "0x301");  // misa
    report!(5, "0x300");  // mstatus
    report!(6, "0x305");  // mtvec
    report!(7, "0x341");  // mepc
    report!(8, "0x342");  // mcause
    report!(9, "0xbc0");  // corecfgr
    report!(10, "0x804"); // intsyscr
    report!(11, "0x800"); // gintenr
    report!(12, "0xbc2"); // cache_strtg_ctlr (QingKe V5-specific)
    report!(13, "0xbc3"); // cache_pmp_ovr   (QingKe V5-specific)

    unsafe {
        mailbox().cpuid_buildcfg.store(build_features(), core::sync::atomic::Ordering::Relaxed);
        mailbox().cpuid_present.store(present_mask, core::sync::atomic::Ordering::Relaxed);
        // Last: tells the boot core the block above is complete.
        mailbox().cpuid_done.store(ipc::CPUID_DONE, core::sync::atomic::Ordering::Relaxed);
    }

    loop {}
}
