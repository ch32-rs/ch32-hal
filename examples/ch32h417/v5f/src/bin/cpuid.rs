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

use ch32h417_v5f::{cpuid, mailbox};
use panic_halt as _;

/// Strong override of qingke-rt's weak `ExceptionHandler`: record the fault and
/// resume past the offending instruction, so probing a CSR the core does not
/// implement is recoverable (see `ch32h417_v5f::cpuid`).
#[no_mangle]
pub extern "C" fn ExceptionHandler() {
    let epc: u32;
    unsafe { core::arch::asm!("csrr {}, mepc", out(reg) epc) };
    let halfword = unsafe { core::ptr::read_volatile(epc as *const u16) } as u32;
    let len = if halfword & 0b11 == 0b11 { 4 } else { 2 };
    cpuid::note_trap();
    unsafe { core::arch::asm!("csrw mepc, {}", in(reg) epc + len) };
}

#[qingke_rt::entry]
fn main() -> ! {
    cpuid::report_self(mailbox());

    loop {}
}
