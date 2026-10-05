//! Handover + CPU-ID report — boot core (V3F, hart 0).
//!
//! The boot core owns bring-up: `hal::init()` programs the whole clock tree
//! (RCC, flash latency, AFIO/GPIO, EXTI), exactly as the WCH CSDK's V3F half
//! does. It then hands the chip over by writing hart 1's entry to the PFIC wake
//! register, and parks in `wfi`. hart 1 runs `v5f/src/bin/cpuid.rs`, reads *its
//! own* identity/ISA CSRs and stores them in shared RAM; this half reads that
//! block back and prints it.
//!
//! ```text
//! cargo xtask flash --example cpuid     # hand over and let hart 1 probe itself
//! cargo xtask report                    # read hart 1's block back out
//! cargo xtask run   --example cpuid     # same, with the V3F's own console
//! ```
//!
//! The handover comes *before* the first `println!`: `SDIPrint::write_str` spins
//! until the debug module consumes `DATA0`, which only happens once wlink has
//! armed SDI print, so printing first would strand hart 1 unscheduled on a plain
//! `flash`. Reading the mailbox with `report` needs no console at all.
//!
//! hart 1 only reads CSRs and stores them: no formatting and no SDI, because
//! every wlink observation pauses the cores, and the WCH CSDK's model of running
//! V5F code from ITCM (`Link_v5f.ld` uses `.itcm_copy >ITCM AT>FLASH`) is not in
//! place here yet.
//!
//! # Privilege
//!
//! qingke-rt reaches `main` through `mret`, so `mstatus.MPP` picks the
//! privilege it runs in. Since 0.8.2 the default is Machine mode
//! (`mstatus = 0x7888`), which is why the Machine-mode CSRs below read
//! directly. The `u-mode` feature restores WCH's startup behaviour and returns
//! to User mode instead, where only the `URW` CSRs (`gintenr` 0x800,
//! `intsyscr` 0x804) are reachable — every other read raises an illegal
//! instruction, and a User-mode build would need an M-mode proxy to report
//! these values at all. This example does not enable `u-mode`.
//!
//! [`ExceptionHandler`] is overridden only to keep the report honest: a CSR
//! that the core does not implement shows up as "absent" instead of hanging in
//! the default handler.
//!
//! # Building and flashing
//!
//! ```text
//! cargo xtask flash --example cpuid
//! cargo xtask report
//! ```

#![no_std]
#![no_main]

use hal::{print, println};
use qingke::pfic::{self, HartId};
use {ch32_hal as hal, panic_halt as _};

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

/// Number of probed CSRs (see [`CSR_NAMES`]).
const N_CSRS: u32 = 14;

/// Start of the cross-core mailbox in shared RAM, and how much of it
/// `cargo xtask report` reads (token/ticks/marker/counter + the block below).
const MB_BASE: u32 = 0x2017_8000;
const MB_WORDS: u32 = 0x58 / 4;

/// Clears the mailbox before the handover: shared RAM survives a reset, so
/// without this `report` would show fields another example left behind.
fn clear_mailbox() {
    for i in 0..MB_WORDS {
        unsafe { core::ptr::write_volatile((MB_BASE + i * 4) as *mut u32, 0) };
    }
}

/// hart 1's report block in the shared-RAM mailbox (`RAM_SHARED` at
/// 0x20178000; the full map is documented in `dualcore.rs`). Each offset is
/// derived from the previous one so changing [`N_CSRS`] keeps both cores in
/// step.
const MB_V5F_PRESENT: *const u32 = 0x2017_8010 as *const u32;
const MB_V5F_VALUES: *const u32 = 0x2017_8014 as *const u32;
const MB_V5F_BUILDCFG: *const u32 = MB_V5F_VALUES.wrapping_add(N_CSRS as usize) as *const u32;
const MB_V5F_DONE: *const u32 = MB_V5F_BUILDCFG.wrapping_add(1) as *const u32;
/// hart 1's "how far did I get" marker (see the V5F copy).
const MB_V5F_PROGRESS: *const u32 = MB_V5F_DONE.wrapping_add(1) as *const u32;

/// Written by hart 1 once its whole block is in place.
const DONE_MAGIC: u32 = 0xC0DE_0001;

/// The probed CSRs, in report order. Index `i` corresponds to bit `i` of the
/// present mask and slot `i` of the value block. `csrr` takes an immediate, so
/// the matching addresses live in [`probe_all`].
const CSR_NAMES: [&str; N_CSRS as usize] = [
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

/// Index of `misa` in [`CSR_NAMES`]; it is decoded after the table.
const MISA_INDEX: usize = 4;

/// Set by [`ExceptionHandler`] when a probed read faulted.
static mut TRAP_HIT: u32 = 0;

/// `csrr` needs an immediate CSR number, hence the literal.
macro_rules! csrr {
    ($addr:literal) => {{
        let v: u32;
        unsafe { core::arch::asm!(concat!("csrr {}, ", $addr), out(reg) v) };
        v
    }};
}

/// Overrides qingke-rt's weak `ExceptionHandler`: record the fault and resume
/// past the instruction that caused it, so an unimplemented CSR is reported
/// instead of hanging.
#[no_mangle]
pub extern "C" fn ExceptionHandler() {
    let epc = csrr!("0x341"); // mepc

    // Compressed instructions are 2 bytes, everything else 4.
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
        unsafe { core::ptr::write_volatile(&raw mut TRAP_HIT, 0) };
        let mut v: u32 = 0;
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $addr), inout(reg) v) };
        let present = unsafe { core::ptr::read_volatile(&raw const TRAP_HIT) } == 0;
        (v, present)
    }};
}

/// Probes every CSR in [`CSR_NAMES`], returning `(values, present mask)`.
fn probe_all() -> ([u32; N_CSRS as usize], u32) {
    // `csrr` needs an immediate CSR number; keep this order in sync with
    // [`CSR_NAMES`].
    let probes = [
        probe_csr!("0xf11"),
        probe_csr!("0xf12"),
        probe_csr!("0xf13"),
        probe_csr!("0xf14"),
        probe_csr!("0x301"),
        probe_csr!("0x300"),
        probe_csr!("0x305"),
        probe_csr!("0x341"),
        probe_csr!("0x342"),
        probe_csr!("0xbc0"),
        probe_csr!("0x804"),
        probe_csr!("0x800"),
        probe_csr!("0xbc2"),
        probe_csr!("0xbc3"),
    ];

    let mut values = [0u32; N_CSRS as usize];
    let mut present = 0u32;
    for (i, (value, ok)) in probes.into_iter().enumerate() {
        values[i] = value;
        if ok {
            present |= 1 << i;
        }
    }
    (values, present)
}

/// ISA the *image* was built for (bit0 M, bit1 A, bit2 F, bit3 D, bit4 C,
/// bit5 B) — what the compiler is allowed to emit, as opposed to what the
/// hardware implements.
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

/// Decode a `misa` value: MXL plus the single-letter extensions.
fn print_misa_decode(misa: u32) {
    let mxl = (misa >> 30) & 0x3;
    print!(
        "  misa decode:  MXL={} ({})  ext:",
        mxl,
        if mxl == 1 { "32-bit" } else { "?" }
    );
    let letters = b"ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    for (i, c) in letters.iter().enumerate() {
        if misa & (1 << i) != 0 {
            print!(" {}", *c as char);
        }
    }
    println!();
}

/// Prints one CSR block: `values[i]` is shown when bit `i` of `present` is set,
/// otherwise the read is reported as absent. Decodes `misa` after the table.
fn print_csr_block(values: &[u32; N_CSRS as usize], present: u32) {
    println!("--- CSRs (probed) ---");
    for (i, name) in CSR_NAMES.iter().enumerate() {
        if present & (1 << i) != 0 {
            println!("{:<18} = {:#010x}", name, values[i]);
        } else {
            println!("{:<18}   absent (read faulted)", name);
        }
    }
    if present & (1 << MISA_INDEX) != 0 {
        print_misa_decode(values[MISA_INDEX]);
    }
}

/// Reads hart 1's mailbox block and prints it. Never returns: the SDI session
/// is kept alive whether or not hart 1 reported.
fn report_v5f() -> ! {
    println!("=== V5F (hart 1) ===");
    if unsafe { core::ptr::read_volatile(MB_V5F_DONE) } != DONE_MAGIC {
        println!(
            "no report from hart 1 (magic missing; its progress marker = {})",
            unsafe { core::ptr::read_volatile(MB_V5F_PROGRESS) }
        );
    } else {
        let mut values = [0u32; N_CSRS as usize];
        for (i, value) in values.iter_mut().enumerate() {
            *value = unsafe { core::ptr::read_volatile(MB_V5F_VALUES.wrapping_add(i)) };
        }
        println!(
            "image build features = {:#04x}",
            unsafe { core::ptr::read_volatile(MB_V5F_BUILDCFG) }
        );
        print_csr_block(&values, unsafe {
            core::ptr::read_volatile(MB_V5F_PRESENT)
        });
    }

    loop {
        hal::delay::Delay.delay_ms(5000);
    }
}

#[ch32_hal::entry]
fn main() -> ! {
    // Only the boot core runs the full bring-up: RCC, AFIO/GPIO and EXTI are
    // shared between both harts.
    let _p = hal::init(hal::Config::default());
    hal::debug::SDIPrint::enable();

    // Hand over to hart 1 before printing anything. `SDIPrint::write_str` spins
    // until the debug module consumes DATA0, which only happens when wlink arms
    // SDI print, so printing first would leave hart 1 unscheduled on a plain
    // `cargo xtask flash`. Waking first means the V5F probes itself either way;
    // `cargo xtask report` then reads its block back out of the mailbox.
    clear_mailbox();
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    println!("=== V3F (boot core) ===");
    println!("hart = {:?}  (PFIC_SCTLR.SCTLR[16])", HartId::current());
    println!("image build features = {:#04x}", build_features());
    println!("privilege = Machine (qingke-rt default; u-mode not enabled)");

    let (values, present) = probe_all();
    print_csr_block(&values, present);

    println!("handed over to hart 1 at {:#010x}", V5F_ENTRY);

    // Wait for hart 1 to fill its block (bounded, so a dead hart 1 cannot
    // wedge this one).
    let mut spins = 0u32;
    while unsafe { core::ptr::read_volatile(MB_V5F_DONE) } != DONE_MAGIC {
        spins += 1;
        if spins > 20_000_000 {
            break;
        }
    }

    report_v5f()
}
