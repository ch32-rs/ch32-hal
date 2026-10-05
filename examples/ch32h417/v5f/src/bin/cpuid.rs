//! CPU-ID report — second core (V5F, hart 1).
//!
//! Probes its identity/ISA CSRs and stores the results in the shared-RAM
//! mailbox; the boot core prints them (`v3f/src/bin/cpuid.rs`).
//!
//! Deliberately minimal: no `hal::init()` (there is one shared RCC block and
//! the boot core has already programmed it, including this core's `FPRE`),
//! and no SDI or `core::fmt` — the V5F's flash-resident execution is slow, so
//! only the CSR probes and the stores happen here.
//!
//! Each CSR is probed with a recoverable illegal-instruction handler, because
//! the QingKe V5 manual's CSR table lists CSRs that the core does not
//! implement; a direct `csrr` of one of those would hang this image.

#![no_std]
#![no_main]

use panic_halt as _;

/// Number of probed CSRs; keep in sync with the boot core's `CSR_NAMES`.
const N_CSRS: u32 = 14;

/// hart 1's report block in the shared-RAM mailbox. Each offset is derived from
/// the previous one so changing [`N_CSRS`] keeps both cores in step.
const MB_V5F_PRESENT: *mut u32 = 0x2017_8010 as *mut u32;
const MB_V5F_VALUES: *mut u32 = 0x2017_8014 as *mut u32;
const MB_V5F_BUILDCFG: *mut u32 = MB_V5F_VALUES.wrapping_add(N_CSRS as usize) as *mut u32;
const MB_V5F_DONE: *mut u32 = MB_V5F_BUILDCFG.wrapping_add(1) as *mut u32;
/// "How far did I get" marker, so the boot core can localise a hang.
const MB_V5F_PROGRESS: *mut u32 = MB_V5F_DONE.wrapping_add(1) as *mut u32;

/// Completion flag, written last so the boot core never reads a partial block.
const DONE_MAGIC: u32 = 0xC0DE_0001;

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
    unsafe { core::ptr::write_volatile(MB_V5F_PROGRESS, 0) };

    macro_rules! report {
        ($i:expr, $addr:literal) => {{
            let (v, present) = probe_csr!($addr);
            unsafe { core::ptr::write_volatile(MB_V5F_VALUES.wrapping_add($i), v) };
            if present {
                present_mask |= 1 << $i;
            }
            // Advance the progress marker so a hang is localisable.
            unsafe { core::ptr::write_volatile(MB_V5F_PROGRESS, $i + 1) };
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
        core::ptr::write_volatile(MB_V5F_BUILDCFG, build_features());
        core::ptr::write_volatile(MB_V5F_PRESENT, present_mask);
        // Last: tells the boot core the block above is complete.
        core::ptr::write_volatile(MB_V5F_DONE, DONE_MAGIC);
    }

    loop {}
}
