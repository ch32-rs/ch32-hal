//! SDI + CPU-id demo — second core (V5F, QingKe hart 1).
//!
//! Woken by the boot core at 0x08002000. It prints its own hart ID once
//! (`hart=C1`), hands the SDI console back to the boot core, and from then on
//! reports liveness by bumping a counter in shared SRAM that the boot core
//! prints. See `examples/ch32h417/src/bin/sdi_cpuid.rs`.
//!
//! # Why this file does not call `hal::init()`
//!
//! On CH32H417 there is a single RCC block shared by both harts, and the boot
//! core has already configured it — including this core's own `FPRE`
//! prescaler. So hart 1 deliberately skips clock setup:
//!
//! * `hal::init()` starts with `Peripherals::take()`, a process-wide
//!   singleton guard in RAM shared by both harts — a second call panics.
//! * `rcc::init()` re-programs the PLL and then *blocks* on
//!   `CFGR0.SWS == PLL`, which would switch SYSCLK out from under the other
//!   core while it is executing.
//! * GPIO/AFIO, DMA and EXTI bring-up are global resources, already done.
//!
//! What the second core does need is core-local: its own stack (set up by
//! `qingke-rt`'s entry from `REGION_STACK`) and its own vector table. Nothing
//! here touches RCC, which is exactly why the demo can run at all.

#![no_std]
#![no_main]

use hal::println;
use qingke::pfic::HartId;
use {ch32_hal as hal, panic_halt as _};

/// Cross-core mailbox in shared RAM (`RAM_SHARED`, declared with the same
/// address in both crates' `memory.x` — the CSDK's convention).
const PRINT_TURN: *mut u32 = 0x2017_8000 as *mut u32;
const V5F_TICKS: *mut u32 = 0x2017_8004 as *mut u32;

/// Bounded spin so a missing boot core cannot wedge this one (~1s).
const HANDOFF_SPINS: u32 = 5_000_000;

#[qingke_rt::entry]
fn main() -> ! {
    // Wait for the console handoff. No `hal::init()` — see the module docs.
    let mut spins = 0u32;
    while unsafe { core::ptr::read_volatile(PRINT_TURN) } != 1 {
        spins += 1;
        if spins > HANDOFF_SPINS {
            break;
        }
    }

    hal::debug::SDIPrint::enable();
    let me = HartId::current();
    println!("[V5F] hart={:?} (mhartid 1) is executing this code", me);

    // Hand the console back to the boot core; it is the only SDI writer from
    // here on, so this core never has to arbitrate again.
    unsafe { core::ptr::write_volatile(PRINT_TURN, 0) };

    // Report liveness through shared SRAM. A single 32-bit aligned writer and
    // a single reader need no locking.
    loop {
        let ticks = unsafe { core::ptr::read_volatile(V5F_TICKS) };
        unsafe { core::ptr::write_volatile(V5F_TICKS, ticks.wrapping_add(1)) };

        // `hal::delay::Delay` is not usable here: its calibration statics are
        // filled in by `hal::init()` on the boot core, and it drives hart 0's
        // systick counter. A plain nop loop is enough for a heartbeat.
        for _ in 0..200_000 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}
