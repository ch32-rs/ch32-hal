//! Read core identity CSRs and decode them in the CH585 cpuid example's style.
//! Run with `cargo run --release --bin cpuid` (flashes the target).
//!
//! Uses defmt RTT, no SDI or GPIO. WCH-style marchid/mimpid fields are printed
//! alongside raw values, without assuming CH585/V3C-specific meanings or
//! equating a core identity with a V467 package. MISA advertises extensions;
//! it does not validate instruction behavior or safe atomics.

#![no_std]
#![no_main]

use ch32v467_examples as board;
use defmt::info;
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

// CSR numbers, from the RISC-V privileged spec. `csrr` takes an immediate, so
// these appear as literals at each use rather than as `const`s.
//
//   mvendorid 0xf11   marchid 0xf12   mimpid 0xf13
//   mhartid   0xf14   misa    0x301   mepc   0x341

/// 0 = idle, 1 = CSR probe active, 2 = probe trapped.
static mut TRAP_HIT: u32 = 0;

/// Read a CSR. `csrr` takes an immediate, hence the literal argument.
macro_rules! csrr {
    ($addr:literal) => {{
        let v: u32;
        unsafe { core::arch::asm!(concat!("csrr {}, ", $addr), out(reg) v) };
        v
    }};
}

/// Overrides qingke-rt's weak `ExceptionHandler`.
///
/// Recover only from an illegal CSR read during a probe. Other faults halt.
#[unsafe(no_mangle)]
pub extern "C" fn ExceptionHandler() {
    let cause = csrr!("0x342");
    let probing = unsafe { core::ptr::read_volatile(&raw const TRAP_HIT) } == 1;
    if cause != 2 || !probing {
        loop {
            core::hint::spin_loop();
        }
    }
    let epc = csrr!("0x341"); // mepc
    let halfword = unsafe { core::ptr::read_volatile(epc as *const u16) };
    // csrr is CSRRS with rs1=x0 and is always a 32-bit instruction.
    if halfword & 0x707f != 0x2073 {
        loop {
            core::hint::spin_loop();
        }
    }

    unsafe {
        core::ptr::write_volatile(&raw mut TRAP_HIT, 2);
        core::arch::asm!("csrw mepc, {}", in(reg) epc + 4);
    }
}

/// Read a CSR, reporting whether it exists: `(value, present)`.
macro_rules! probe_csr {
    ($addr:literal) => {{
        unsafe { core::ptr::write_volatile(&raw mut TRAP_HIT, 1) };
        let mut v: u32 = 0;
        unsafe { core::arch::asm!(concat!("csrr {0}, ", $addr), inout(reg) v) };
        let present = unsafe { core::ptr::read_volatile(&raw const TRAP_HIT) } == 1;
        unsafe { core::ptr::write_volatile(&raw mut TRAP_HIT, 0) };
        (if present { v } else { 0 }, present)
    }};
}

/// The single-letter names of the extensions `misa` can advertise, indexed by
/// bit position: bit 0 is `A`, bit 1 `B`, ... bit 25 `Z`.
fn misa_letters(misa: u32) -> ([u8; 26], usize) {
    let mut buf = [0u8; 26];
    let mut n = 0;
    for bit in 0..26u32 {
        if misa & (1 << bit) != 0 {
            buf[n] = b'A' + bit as u8;
            n += 1;
        }
    }
    (buf, n)
}

/// Turn the three 5-bit vendor fields into the letters they encode.
///
/// The WCH-style encoding maps `A`..`Z` to 1..26. A zero field is skipped
/// rather than shown as a stray character, so a partially-used field reads
/// sensibly.
fn vendor_letters(v0: u32, v1: u32, v2: u32) -> ([u8; 3], usize) {
    let mut buf = [0u8; 3];
    let mut n = 0;
    for v in [v0, v1, v2] {
        if (1..=26).contains(&v) {
            buf[n] = b'A' + (v - 1) as u8;
            n += 1;
        }
    }
    (buf, n)
}

/// View a byte buffer as text, or `"?"` if it is not ASCII.
fn ascii(b: &[u8]) -> &str {
    core::str::from_utf8(b).unwrap_or("?")
}

/// Letter for a single WCH-style 5-bit code.
fn letter(code: u32) -> u8 {
    if (1..=26).contains(&code) {
        b'A' + (code - 1) as u8
    } else {
        b'?'
    }
}

#[entry]
fn main() -> ! {
    board::init();
    // Allow RTT attachment before emitting the identity report.
    board::delay_ms(500);

    info!("CH32V467 CPU identity (PAC-only, defmt RTT)");

    let (vendor, vendor_ok) = probe_csr!("0xf11");
    info!("mvendorid (0xf11) = {=u32:#010x} present={}", vendor, vendor_ok);
    if vendor_ok && vendor == 0 {
        info!("  -> zero vendor ID; not an absent CSR");
    }

    // --- marchid: vendor | arch | series | revision -----------------------
    let (arch, arch_ok) = probe_csr!("0xf12");
    info!("marchid   (0xf12) = {=u32:#010x} present={}", arch, arch_ok);
    if arch_ok && arch != 0 {
        let (v0, v1, v2) = ((arch >> 26) & 0x1f, (arch >> 21) & 0x1f, (arch >> 16) & 0x1f);
        let a = (arch >> 10) & 0x1f; // architecture letter
        let s = (arch >> 5) & 0x1f; // series digit
        let r = arch & 0x1f; // silicon revision letter

        let (vbuf, vn) = vendor_letters(v0, v1, v2);
        info!("  vendor      = {}", ascii(&vbuf[..vn]));
        // Series is a *number*, so print the raw 5-bit value rather than a
        // letter. Version and Arch are letters.
        info!("  arch/series/rev = {}{}{}", letter(a) as char, s, letter(r) as char);
        info!(
            "  raw fields  = V0={=u32:#x} V1={=u32:#x} V2={=u32:#x} Arch={=u32:#x} Serial={=u32:#x} Ver={=u32:#x}",
            v0, v1, v2, a, s, r
        );
    }

    // --- mimpid: vendor | reserved | minor --------------------------------
    let (imp, imp_ok) = probe_csr!("0xf13");
    info!("mimpid    (0xf13) = {=u32:#010x} present={}", imp, imp_ok);
    if imp_ok && imp != 0 {
        let (v0, v1, v2) = ((imp >> 26) & 0x1f, (imp >> 21) & 0x1f, (imp >> 16) & 0x1f);
        let minor = imp & 0xff;
        let (vbuf, vn) = vendor_letters(v0, v1, v2);
        info!("  vendor      = {}", ascii(&vbuf[..vn]));
        info!("  minor       = {}", minor);
        info!("  -> raw implementation minor; not a chip/package identifier");
    }

    let (hart, hart_ok) = probe_csr!("0xf14");
    info!("mhartid   (0xf14) = {=u32} present={}", hart, hart_ok);

    let (misa, misa_ok) = probe_csr!("0x301");
    info!("misa      (0x301) = {=u32:#010x} present={}", misa, misa_ok);

    if misa_ok {
        // MXL (bits 31:30): 1 = RV32, 2 = RV64.
        match (misa >> 30) & 0x3 {
            1 => info!("  XLEN        = 32"),
            2 => info!("  XLEN        = 64"),
            other => info!("  XLEN        = reserved ({})", other),
        }

        let (buf, n) = misa_letters(misa);
        match core::str::from_utf8(&buf[..n]) {
            Ok(s) => info!("  extensions  = {}", s),
            Err(_) => info!("  extensions  = <not ascii>"),
        }

        info!("  I (base)    = {}", misa & (1 << 8) != 0);
        info!("  M (mul/div) = {}", misa & (1 << 12) != 0);
        info!("  C (16-bit)  = {}", misa & (1 << 2) != 0);
        info!("  A (atomics) = {}", misa & (1 << 0) != 0);
        info!("  F (float)   = {}", misa & (1 << 5) != 0);
        info!("  V (vector)  = {}", misa & (1 << 21) != 0);
        info!("  U (user)    = {}", misa & (1 << 20) != 0);
        // Advertised bits do not prove instruction encodings or atomic behavior.
        info!("  B (bitmanip)= {}", misa & (1 << 1) != 0);
        info!("  X (custom)  = {}", misa & (1 << 23) != 0);
    }

    info!("CPU identity report complete; no ISA behavior tests performed");
    let mut ticks = 0u32;
    loop {
        ticks = ticks.wrapping_add(1);
        board::heartbeat(ticks);
        board::delay_ms(1000);
    }
}
