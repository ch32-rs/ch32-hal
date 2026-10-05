//! Ping-pong over shared memory — second core (V5F, hart 1).
//!
//! Woken by `v3f/src/bin/pingpong.rs`: reports its own CPU-ID CSRs into the
//! shared mailbox, then spends the rest of its life answering the boot core's
//! rounds. `ping` is this core's counter, `pong` the boot core's echo; a round
//! is outstanding while the two differ, and the next one only starts once the
//! echo has been seen, so the two cores cannot run away from each other.
//!
//! Writes only, no printing: hart 1 has no console of its own here, and
//! `xtask report` reads the counters back out of shared memory. See the boot
//! core's copy for how to watch it, and why attaching the console freezes this
//! core.
//!
//! Metapac only, no `ch32-hal` and no embassy — the boot core owns bring-up.

#![no_std]
#![no_main]

use ch32h417_v5f::{cpuid, mailbox};
use core::sync::atomic::Ordering;
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
    let mailbox = mailbox();

    // Describe ourselves first: that block is what the boot core prints.
    cpuid::report_self(mailbox);

    // Start round one, then answer the boot core's echoes. This core does not
    // pace the exchange — the boot core does, in its one-second loop.
    mailbox.ping.store(1, Ordering::Relaxed);
    let mut acked = 0u32;
    loop {
        let pong = mailbox.pong.load(Ordering::Relaxed);
        if pong != acked {
            acked = pong;
            mailbox.ping.store(pong.wrapping_add(1), Ordering::Relaxed);
        }
    }
}
