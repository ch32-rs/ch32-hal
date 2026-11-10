//! Feasibility spike: one image, two harts (embassy-rp style).
//!
//! Reset still starts only hart 0 (V3F). This example does not flash a second
//! image at all: hart 0 copies its own `.highcode` into the zero-wait shared
//! block as usual, then wakes hart 1 into a 1 KiB-aligned trampoline that is
//! part of *this* image. Hart 1 sets its own `gp`/`sp`, gets its own handle on
//! the same `.data`/`.bss`, and runs `hart1_main` from the same `.text`.
//!
//! This is the core question behind ch32-rs/ch32-hal#251 ("is CH32H417 a
//! homogeneous dual core?"): can the second hart run the *same* program the
//! way `embassy_rp::multicore::spawn_core1` lets RP2040 core 1 do it?
//!
//! What is deliberately *not* solved here, and is what the notes have to
//! answer before this becomes an API:
//!
//! - `Peripherals::take()` is a single-image singleton. Hart 0 takes it, so
//!   hart 1 must not call `hal::init_secondary` (it would panic); peripheral
//!   ownership has to be handed over explicitly.
//! - `critical_section` is `SingleHartCriticalSection`: fine for per-hart
//!   state, wrong for anything both harts touch.
//! - Tracing (defmt-rtt, SDI) is single-producer today.
//!
//! Observation is through [`OBS`], placed at the start of `SRAM_SHARED` so its
//! address is fixed (`0x2017_8000`) and no symbol lookup is needed:
//!
//! ```text
//! WLINK_PROBE=1a86:8010:... cargo xtask flash --example homogeneous
//! sleep 10 && wlink dump 0x20178000 48 -o /var/tmp/obs.bin
//! ```
//!
//! Observed on the bench board (400/400/100 MHz HSI recipe): hart 1 reports
//! `mhartid = 1`, `HartId::C1`, `sp` inside `.bss`, `Delay.delay_ms(1000)`
//! costs exactly 400 000 168 `mcycle` ticks (1.000 s at 400 MHz) and `h1_seq`
//! advances 10 per 10 s. Both harts run this same `.text` from the zero-wait
//! shared block.
//!
//! **Read the block once.** On this board `wlink dump` leaves hart 0 halted
//! after the first read while hart 1 keeps running (the mirror image of the
//! SDI-console behaviour in AGENTS.md), so `h0_seq` freezing after the first
//! dump is the probe, not the image.

#![no_std]
#![no_main]

use core::sync::atomic::{fence, AtomicU32, Ordering};

use ch32_hal as hal;
use panic_halt as _;
use qingke::pfic::{self, HartId};

/// The cross-hart observation block.
///
/// `.shared` is placed at the start of `SRAM_SHARED` (`0x2017_8000`) by
/// `ipc/shared.x`, so this struct is the first word of that region. Nothing
/// else may claim `.shared` in this image.
#[repr(C)]
struct Obs {
    /// Hart 0's progress counter.
    h0_seq: AtomicU32,
    /// Hart 1's progress counter. The whole spike is about whether this moves.
    h1_seq: AtomicU32,
    /// What hart 1 read from `mhartid` (must be 1).
    h1_mhartid: AtomicU32,
    /// What `qingke::pfic::HartId::current()` (PFIC_SCTLR[16]) reports (1).
    h1_pfic_hart: AtomicU32,
    /// The wake address hart 0 programmed (must be 1 KiB aligned).
    h1_entry: AtomicU32,
    /// `mcause` if hart 1 trapped.
    h1_mcause: AtomicU32,
    /// `mepc` if hart 1 trapped.
    h1_mepc: AtomicU32,
    /// Bumped by hart 1 from its stack, to prove `sp` is the one we set.
    h1_stack_sp: AtomicU32,
    /// `rcc::clocks().v3f` / `.v5f` / `.hclk` / `.sysclk`, in Hz.
    clk_v3f: AtomicU32,
    clk_v5f: AtomicU32,
    clk_hclk: AtomicU32,
    clk_sysclk: AtomicU32,
    /// `mcycle` elapsed around the last `Delay.delay_ms(1000)` on each hart.
    h0_cycles: AtomicU32,
    h1_cycles: AtomicU32,
}

#[link_section = ".shared"]
static OBS: Obs = Obs {
    h0_seq: AtomicU32::new(0),
    h1_seq: AtomicU32::new(0),
    h1_mhartid: AtomicU32::new(0),
    h1_pfic_hart: AtomicU32::new(0),
    h1_entry: AtomicU32::new(0),
    h1_mcause: AtomicU32::new(0),
    h1_mepc: AtomicU32::new(0),
    h1_stack_sp: AtomicU32::new(0),
    clk_v3f: AtomicU32::new(0),
    clk_v5f: AtomicU32::new(0),
    clk_hclk: AtomicU32::new(0),
    clk_sysclk: AtomicU32::new(0),
    h0_cycles: AtomicU32::new(0),
    h1_cycles: AtomicU32::new(0),
};

#[inline(always)]
fn mcycle() -> u32 {
    let v: u32;
    unsafe { core::arch::asm!("csrr {}, mcycle", out(reg) v) };
    v
}

/// Hart 1's stack. Lives in `.bss`, i.e. in the shared block, because both
/// harts share one image and one address space. Hart 0's stack is the separate
/// `STACK` region, so the two never overlap.
#[repr(C, align(16))]
struct Hart1Stack([u8; 8 * 1024]);

#[used]
#[no_mangle]
static mut HART1_STACK: Hart1Stack = Hart1Stack([0; 8 * 1024]);

/// Written by the trap stub below; hart 0 folds it into `OBS` when it sees it.
#[used]
#[no_mangle]
static mut HART1_TRAP: [u32; 2] = [0; 2];

unsafe extern "C" {
    fn _hart1_trampoline();
}

// The wake entry. `PFIC_WAKEIP[1]` only keeps bits [31:10], so the linker
// puts this section on a 1 KiB boundary (see `v3f/link.x`).
//
// It must not touch the stack before it has set `sp`, and it must set `gp`
// before calling any Rust code: the two harts share `.data`, therefore the
// same `__global_pointer$`.
core::arch::global_asm!(
    r#"
    .section .hart1_entry, "ax", @progbits
    .align 10
    .global _hart1_trampoline
_hart1_trampoline:
    .option push
    .option norelax
    la gp, __global_pointer$
    la sp, HART1_STACK
    .option pop
    li t0, 8192
    add sp, sp, t0

    /* FS = Initial, Dirty cleared: the same two writes qingke-rt's startup
       does before `main`. Hart 1 never runs `_start`, so it has to do this. */
    li t0, 0x4000
    csrrc x0, mstatus, t0
    li t0, 0x2000
    csrrs x0, mstatus, t0

    /* Any trap on hart 1 goes here instead of to address 0 (which is the
       boot alias and would re-run `_start`, clobbering hart 0's data). */
    la t0, hart1_trap
    csrw mtvec, t0

    /* Pairs with the release fence hart 0 does before SETEVENT. */
    fence rw, rw
    call hart1_main
1:
    j 1b

    .align 2
hart1_trap:
    csrr t0, mcause
    csrr t1, mepc
    la t2, HART1_TRAP
    sw t0, 0(t2)
    sw t1, 4(t2)
2:
    j 2b
"#
);

#[no_mangle]
extern "C" fn hart1_main() -> ! {
    let mhartid: u32;
    unsafe { core::arch::asm!("csrr {}, mhartid", out(reg) mhartid) };
    OBS.h1_mhartid.store(mhartid, Ordering::Relaxed);
    OBS.h1_pfic_hart
        .store(HartId::current() as u32, Ordering::Relaxed);

    let sp: u32;
    unsafe { core::arch::asm!("mv {}, sp", out(reg) sp) };
    OBS.h1_stack_sp.store(sp, Ordering::Relaxed);

    // There is one `Peripherals` singleton per image, and hart 0 already took
    // it. Hart 1 can only `steal` it and promise to touch disjoint hardware;
    // this is the API gap `init_secondary` has today (it calls `take()` and
    // therefore panics when both harts share an image).
    let p = unsafe { hal::Peripherals::steal() };

    // LED1 = PF0, which hart 0 never configures in this example. The GPIO port
    // clock and the pin mux base state are shared, so no re-init is needed.
    let mut led = hal::gpio::Output::new(p.PF0, hal::gpio::Level::Low, Default::default());

    // `hal::delay::Delay` picks the calling hart's SysTick counter (index 1 on
    // V5F) and reads the shared `rcc::clocks()` static, so hart 1 gets a real
    // millisecond delay without `rcc::refresh`.
    loop {
        led.toggle();
        OBS.h1_seq.fetch_add(1, Ordering::Relaxed);
        let t0 = mcycle();
        hal::delay::Delay.delay_ms(1000);
        OBS.h1_cycles
            .store(mcycle().wrapping_sub(t0), Ordering::Relaxed);

        let mcause = unsafe { core::ptr::read_volatile(&raw const HART1_TRAP[0]) };
        if mcause != 0 {
            OBS.h1_mcause.store(mcause, Ordering::Relaxed);
        }
    }
}

#[ch32_hal::entry]
fn main() -> ! {
    // Boot-core bring-up: RCC, flash latency, AFIO/GPIO, EXTI. Hart 1 shares
    // all of that state through the image, so it does not redo it.
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi();
    let _p = hal::init(config);

    // `.shared` is NOLOAD, so its "initial" zeros never exist on the chip:
    // whatever the previous firmware left there is what the two harts see.
    // Clear the block explicitly before publishing the entry.
    for f in [
        &OBS.h0_seq,
        &OBS.h1_seq,
        &OBS.h1_mhartid,
        &OBS.h1_pfic_hart,
        &OBS.h1_mcause,
        &OBS.h1_mepc,
        &OBS.h1_stack_sp,
        &OBS.clk_v3f,
        &OBS.clk_v5f,
        &OBS.clk_hclk,
        &OBS.clk_sysclk,
        &OBS.h0_cycles,
        &OBS.h1_cycles,
    ] {
        f.store(0, Ordering::Relaxed);
    }

    let clocks = hal::rcc::clocks();
    OBS.clk_v3f.store(clocks.v3f.0, Ordering::Relaxed);
    OBS.clk_v5f.store(clocks.v5f.0, Ordering::Relaxed);
    OBS.clk_hclk.store(clocks.hclk.0, Ordering::Relaxed);
    OBS.clk_sysclk.store(clocks.sysclk.0, Ordering::Relaxed);

    let entry = _hart1_trampoline as unsafe extern "C" fn() as usize as u32;
    assert_eq!(entry & 0x3FF, 0, "wake entry must be 1 KiB aligned");
    OBS.h1_entry.store(entry, Ordering::Relaxed);

    // Everything above must be visible before hart 1 starts: a plain `fence`
    // here is what the mailbox convention is still missing today.
    fence(Ordering::SeqCst);
    unsafe { pfic::wake_other_core(entry) };

    // Reference: the same `Delay`, same 1 s request, on hart 0.
    loop {
        OBS.h0_seq.fetch_add(1, Ordering::Relaxed);
        hal::delay::Delay.delay_ms(1000);
    }
}
