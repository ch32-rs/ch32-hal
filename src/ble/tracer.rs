//! BLE register-access tracer for TX/RX debugging and Phase-2 bisect.
//!
//! Records `(t_us, op, addr, val)` tuples in a ring buffer during a TX burst
//! (or any BLE register sequence), then dumps them as CSV for diff comparison
//! against EVT C traces.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ch32_hal::ble::tracer;
//!
//! unsafe {
//!     tracer::reset();
//!     // Run your register sequence here, inserting trace calls:
//!     tracer::w(0x40024100 + 0x08, 0x8E89_BED6);   // ADV AA write
//!     tracer::r(0x40024100 + 0x00, lle_read(0x00)); // lle_read result
//!     tracer::dump();
//! }
//! ```
//!
//! For cleaner integration, define thin wrappers in your caller:
//!
//! ```rust,ignore
//! unsafe fn tw(off: usize, v: u32) {
//!     tracer::w(LLE_BASE as u32 + off as u32, v);
//!     lle_write(off, v);
//! }
//! unsafe fn tr(off: usize) -> u32 {
//!     let v = lle_read(off);
//!     tracer::r(LLE_BASE as u32 + off as u32, v);
//!     v
//! }
//! ```
//!
//! # Output format
//!
//! ```text
//! # t_us,op,addr,val  (N entries)
//! 0,W,0x40024108,0x8e89bed6
//! 1,R,0x40024100,0x14001365
//! ```
//!
//! - `t_us`  — microseconds relative to last `reset()` call (96 MHz → ÷96)
//! - `op`    — `W` (write) or `R` (read)
//! - `addr`  — absolute 32-bit register address (hex)
//! - `val`   — 32-bit value written or read (hex)
//!
//! # Notes
//!
//! - The ring buffer holds `CAP` (256) entries; older entries are overwritten on overflow.
//! - All functions are `unsafe`; caller guarantees single-threaded access.
//! - `dump()` requires `SDIPrint::enable()` to have been called before use.
//! - `mcycle` (CSR 0xB00) is read directly via inline assembly. At 96 MHz,
//!   the 32-bit counter wraps every ~44 s; relative timestamps stay valid for
//!   any burst shorter than that.

/// Ring buffer capacity. 256 entries covers a full TX burst (13 steps × ~4 ops each).
const CAP: usize = 256;

/// A single trace entry (16 bytes, cache-line-friendly).
#[derive(Clone, Copy)]
struct Entry {
    /// mcycle count relative to the last `reset()` baseline.
    t_cycles: u32,
    /// 0 = write, 1 = read.
    op: u8,
    _pad: [u8; 3],
    /// Absolute register address.
    addr: u32,
    /// Value written or read.
    val: u32,
}

impl Entry {
    const ZERO: Self = Self {
        t_cycles: 0,
        op: 0,
        _pad: [0; 3],
        addr: 0,
        val: 0,
    };
}

// SAFETY: all access is guarded by `unsafe fn`; callers guarantee single-threaded
// access (no concurrent ISR during TX burst).
static mut TRACE_BUF: [Entry; CAP] = [Entry::ZERO; CAP];
static mut TRACE_LEN: usize = 0;
static mut TRACE_BASE: u32 = 0;

/// Read the low 32 bits of the `mcycle` performance counter (CSR 0xB00).
///
/// Available in all RISC-V machine-mode contexts. At 96 MHz, resolution ≈ 10 ns.
#[inline(always)]
fn mcycle() -> u32 {
    let v: u32;
    // SAFETY: bare-metal machine-mode context; mcycle CSR always accessible.
    unsafe {
        core::arch::asm!("csrr {}, mcycle", out(reg) v, options(nomem, nostack));
    }
    v
}

/// Push one entry into the ring buffer (internal helper).
///
/// # Safety
///
/// Single-threaded access required.
#[inline(always)]
unsafe fn push(op: u8, addr: u32, val: u32) {
    let t = mcycle().wrapping_sub(TRACE_BASE);
    let idx = TRACE_LEN % CAP;
    // Use raw pointer write to avoid `&mut` reference to `static mut`.
    core::ptr::addr_of_mut!(TRACE_BUF[idx]).write(Entry {
        t_cycles: t,
        op,
        _pad: [0; 3],
        addr,
        val,
    });
    TRACE_LEN += 1;
}

/// Clear the ring buffer and record the current `mcycle` as the time baseline.
///
/// Call this immediately before the register sequence you want to capture.
///
/// # Safety
///
/// Single-threaded access required.
pub unsafe fn reset() {
    TRACE_LEN = 0;
    TRACE_BASE = mcycle();
}

/// Record a register **write** (`op = W`).
///
/// Does **not** perform the actual hardware write — the caller must still call
/// `write_volatile` or the appropriate `lle_write`/`bb_write`/`rfend_write`.
///
/// Typical usage pattern:
/// ```rust,ignore
/// tracer::w(LLE_BASE as u32 + 0x08, v);
/// lle_write(0x08, v);
/// ```
///
/// # Safety
///
/// Single-threaded access required.
#[inline(always)]
pub unsafe fn w(addr: u32, val: u32) {
    push(0, addr, val);
}

/// Record a register **read** result (`op = R`).
///
/// Does **not** perform the actual hardware read — the caller must still call
/// `read_volatile` or the appropriate `lle_read`/`bb_read`/`rfend_read`.
///
/// Typical usage pattern:
/// ```rust,ignore
/// let v = lle_read(0x00);
/// tracer::r(LLE_BASE as u32 + 0x00, v);
/// ```
///
/// # Safety
///
/// Single-threaded access required.
#[inline(always)]
pub unsafe fn r(addr: u32, val: u32) {
    push(1, addr, val);
}

/// Dump all recorded entries as CSV to SDI (via `crate::println!`).
///
/// Output header: `# t_us,op,addr,val  (N entries)`
/// Each entry:    `t_us,op,0xXXXXXXXX,0xXXXXXXXX`
///
/// If the buffer wrapped (more than `CAP` entries were recorded), only the
/// most recent `CAP` entries are printed, in chronological order.
///
/// # Safety
///
/// Single-threaded access required. `SDIPrint::enable()` must have been called.
pub unsafe fn dump() {
    let total = TRACE_LEN;
    let count = total.min(CAP);
    // If total > CAP, ring has wrapped; oldest surviving entry is at index (total % CAP).
    let start = if total > CAP { total % CAP } else { 0 };

    crate::println!("# t_us,op,addr,val  ({} entries{})",
        count,
        if total > CAP { ", ring wrapped" } else { "" },
    );

    for i in 0..count {
        // SAFETY: index arithmetic keeps idx in [0, CAP).
        let e = core::ptr::addr_of!(TRACE_BUF[(start + i) % CAP]).read();
        let t_us = e.t_cycles / 96; // 96 MHz → µs
        let op = if e.op == 0 { 'W' } else { 'R' };
        crate::println!("{},{},{:#010x},{:#010x}", t_us, op, e.addr, e.val);
    }
}

/// Return the number of entries recorded since the last `reset()`.
///
/// If greater than `CAP`, the ring has wrapped.
///
/// # Safety
///
/// Single-threaded access required.
#[inline(always)]
pub unsafe fn len() -> usize {
    TRACE_LEN
}

/// Dump a snapshot of a contiguous register block as CSV to SDI.
///
/// Reads every 4-byte word from `base + from_off` to `base + to_off` (inclusive, step 4)
/// and prints each as `0xAAAAAAAA,0xVVVVVVVV`. Output header line names the block.
///
/// Designed for post-init / pre-TX state capture to diff against EVT register state.
/// Use three calls to cover all three BLE register blocks:
///
/// ```rust,ignore
/// tracer::dump_regs("BB   ", 0x40024100, 0x00, 0x80);  // gptrBBReg
/// tracer::dump_regs("LLE  ", 0x40024200, 0x00, 0x80);  // gptrLLEReg
/// tracer::dump_regs("RFEND", 0x40025000, 0x00, 0xD0);  // gptrRFENDReg
/// ```
///
/// Output format (for easy diff/import):
/// ```text
/// # snap BB    base=0x40024100 off=0x00..0x80
/// 0x40024100,0x14001365
/// 0x40024104,0x00000001
/// ...
/// ```
///
/// # Safety
///
/// Reads hardware registers via `read_volatile`. Call only after `ble_phy_init()`.
/// `SDIPrint::enable()` must have been called before use.
pub unsafe fn dump_regs(name: &str, base: u32, from_off: usize, to_off: usize) {
    crate::println!("# snap {} base={:#010x} off={:#04x}..{:#04x}",
        name, base, from_off, to_off);
    let mut off = from_off;
    while off <= to_off {
        let addr = base + off as u32;
        let val = core::ptr::read_volatile(addr as *const u32);
        crate::println!("{:#010x},{:#010x}", addr, val);
        off += 4;
    }
}
