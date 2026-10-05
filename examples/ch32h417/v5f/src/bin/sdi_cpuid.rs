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
use ch32h417_v5f::sdi_println;
use ch32h417_ipc as ipc;
use ch32h417_v5f::mailbox;
use qingke::pfic::HartId;
use panic_halt as _;


/// Bounded spin so a missing boot core cannot wedge this one (~1s).
const HANDOFF_SPINS: u32 = 5_000_000;

#[qingke_rt::entry]
fn main() -> ! {
    // Wait for the console handoff. Nothing global is initialised here — see
    // the module docs.
    let mut spins = 0u32;
    while unsafe { mailbox().sdi_token.load(core::sync::atomic::Ordering::Relaxed) } != 1 {
        spins += 1;
        if spins > HANDOFF_SPINS {
            break;
        }
    }

    SdiPrint::enable();
    let me = HartId::current();
    sdi_println!("[V5F] hart={:?} (mhartid 1) is executing this code", me);

    // Hand the console back to the boot core; it is the only SDI writer from
    // here on, so this core never has to arbitrate again.
    unsafe { mailbox().sdi_token.store(0, core::sync::atomic::Ordering::Relaxed) };

    // Report liveness through shared SRAM. A single 32-bit aligned writer and
    // a single reader need no locking.
    loop {
        let ticks = mailbox().sdi_ticks.load(core::sync::atomic::Ordering::Relaxed);
        mailbox().sdi_ticks.store(ticks.wrapping_add(1), core::sync::atomic::Ordering::Relaxed);

        // The HAL's delay is not usable here (its calibration is filled in by
        // the boot core's `hal::init()` and it drives hart 0's systick), so a
        // plain nop loop is the heartbeat.
        for _ in 0..200_000 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}
