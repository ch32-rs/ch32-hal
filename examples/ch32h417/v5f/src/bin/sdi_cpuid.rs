//! SDI + CPU-id demo — second core (V5F, QingKe hart 1).
//!
//! Woken by the boot core at 0x00010000. It prints its own hart ID once
//! (`hart=C1`), hands the SDI console back to the boot core, and from then on
//! reports liveness by bumping a counter in shared SRAM that the boot core
//! prints. See `v3f/src/bin/sdi_cpuid.rs`.
//!
//! # Why this file uses metapac and not `ch32-hal`
//!
//! On CH32H417 there is a single RCC block shared by both harts, and the boot
//! core has already configured it — including this core's own `FPRE`
//! prescaler. So hart 1 deliberately initialises nothing global:
//!
//! * `ch32-hal`'s `init()` starts with `Peripherals::take()`, a process-wide
//!   singleton guard in RAM shared by both harts — a second call panics.
//! * its `rcc::init()` re-programs the PLL and then *blocks* on
//!   `CFGR0.SWS == PLL`, which would switch SYSCLK out from under the other
//!   core while it is executing.
//! * GPIO/AFIO, DMA and EXTI bring-up are global resources, already done.
//!
//! This crate therefore depends on `ch32-metapac` directly (re-exported as
//! `ch32h417_v5f::pac`) and prints through `ch32h417_v5f::sdi`. What hart 1 does
//! need is core-local: its own stack (set up by `qingke-rt`'s entry from
//! `REGION_STACK`) and its own vector table. Nothing here touches RCC, which is
//! exactly why the demo can run at all.
//!
//! See `README.md` ("Writing a V5F half"); `docs/backlog.md` tracks the
//! `ch32-hal` secondary-core entry that would let hart 1 use HAL drivers.

#![no_std]
#![no_main]

use ch32h417_v5f::sdi::SdiPrint;
use ch32h417_v5f::{cache, cpuid};
use ch32h417_v5f::sdi_try_println;
use ch32h417_ipc as ipc;
use ch32h417_v5f::mailbox;
use qingke::pfic::HartId;
use panic_halt as _;


/// Bounded spin so a missing boot core cannot wedge this one (~1s).
const HANDOFF_SPINS: u32 = 5_000_000;

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
    use core::sync::atomic::Ordering;

    // Progress marker: `report_self` below resets it to 0 and counts up as it
    // probes, so a *report* that shows 0x101 means hart 1 entered main and then
    // died before probing — the usual way "hart 1 never ran" is actually
    // "hart 1 ran and hung", which this distinguishes.
    mailbox().cpuid_progress.store(0x101, Ordering::Relaxed);
    // Hart 1 must enable its own instruction cache: it resets to disabled.
    cache::enable_icache();

    // Wait for the console handoff. Nothing global is initialised here — see
    // the module docs.
    let mut spins = 0u32;
    while mailbox().sdi_token.load(Ordering::Relaxed) != 1 {
        spins += 1;
        if spins > HANDOFF_SPINS {
            break;
        }
    }

    SdiPrint::enable();
    let me = HartId::current();
    sdi_try_println!("[V5F] hart={:?} (mhartid 1) is executing this code", me);

    // Probe our own CSRs and print the table *from this core* — the same block
    // `cpuid` publishes for `xtask report`, but over SDI: this is the variant to
    // use when you want to watch hart 1 rather than read shared memory later.
    //
    // Bounded prints (`sdi_try_println!`) so a missing console cannot wedge this
    // core: with the console open every line goes out, without it the lines are
    // dropped and the loop below keeps ticking.
    cpuid::report_self(mailbox());
    let values = mailbox().cpuid_snapshot();
    let present = mailbox().cpuid_present.load(Ordering::Relaxed);
    sdi_try_println!(
        "[V5F] image build features = {:#04x}",
        mailbox().cpuid_buildcfg.load(Ordering::Relaxed)
    );
    for (i, name) in ipc::CPUID_CSR_NAMES.iter().enumerate() {
        if present & (1 << i) != 0 {
            sdi_try_println!("  {:<18} = {:#010x}", name, values[i]);
        } else {
            sdi_try_println!("  {:<18}   absent (read faulted)", name);
        }
    }
    if present & (1 << ipc::CPUID_MISA_INDEX) != 0 {
        let misa = values[ipc::CPUID_MISA_INDEX];
        let mut letters = [0u8; 26];
        let mut len = 0;
        for (bit, letter) in (b'A'..=b'Z').enumerate() {
            if misa & (1 << bit) != 0 {
                letters[len] = letter;
                len += 1;
            }
        }
        sdi_try_println!(
            "  misa decode        MXL={} extensions={}",
            (misa >> 30) & 3,
            core::str::from_utf8(&letters[..len]).unwrap_or("")
        );
    }

    // Hand the console back to the boot core; it is the only SDI writer from
    // here on, so this core never has to arbitrate again.
    mailbox().sdi_token.store(0, Ordering::Relaxed);

    // Report liveness through shared SRAM. A single 32-bit aligned writer and
    // a single reader need no locking.
    loop {
        let ticks = mailbox().sdi_ticks.load(Ordering::Relaxed);
        mailbox().sdi_ticks.store(ticks.wrapping_add(1), Ordering::Relaxed);

        // The HAL's delay is not usable here (its calibration is filled in by
        // the boot core's `hal::init()` and it drives hart 0's systick), so a
        // plain nop loop is the heartbeat.
        for _ in 0..200_000 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}
