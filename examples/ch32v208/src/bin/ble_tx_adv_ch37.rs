//! BLE ADV_NONCONN_IND ch37 minimal air validation — CH32V208, Task #16.
//!
//! Single-variable experiment: ch37 only, fixed AdvA, Flags + Mfr payload.
//! Based directly on the confirmed-working DTM TX register sequence (`ble_dtm_tx.rs`)
//! with the minimal changes required for ADV format:
//!   - Access address: 0x8E89_BED6 (BLE ADV AA, not DTM 0x71764129)
//!   - Channel field: logical ch=37 | bit6=1 (whitening on, BLE spec §3.2)
//!   - PDU format: ADV_NONCONN_IND header + AdvA(6B) + AD payload
//!   - TX buf ptr: BB+0x70 (same as DTM)
//!   - PLL formula: same as DTM / RX — `freq_khz` direct, no offset (0xFE0F_C000 mask)
//!
//! DTM TX confirmation baseline: `ble_dtm_tx.rs` produced visible GFSK bursts on
//! spectrum analyzer (2402/2426/2480 MHz). ADV TX uses the same 13-step sequence.
//!
//! # Validation (Cindy)
//!
//! Primary: nRF Connect → Scanner → AdvA=C2:21:43:65:87:12 + "cba" appears
//! Secondary: second CH32 running `ble_rx_listener` → pdu type=2 ch37 AdvA match
//!
//! # Register naming (same as all BLE files — WCH naming swapped)
//!
//! ```text
//! LLE_BASE = 0x40024100  (WCH: gptrBBReg)   → lle_* accessors
//! BB_BASE  = 0x40024200  (WCH: gptrLLEReg)  → bb_*  accessors
//! RFEND_BASE = 0x40025000 (WCH: gptrRFENDReg) → rfend_* accessors
//! ```
//!
//! # TX register sequence (15 steps, from lib `ll_advertise_tx` d.asm RE — Lucy 2026-05-02)
//!
//! Steps 1-11 are setup; steps 12-15 are the commit+trigger sequence.
//! Previous versions used bit 11 as GO — **WRONG**; lib ADV uses bit 23.
//! Previous versions put BB+0x00=2 before GO — **WRONG**; lib puts it last (trigger).
//!
//! ```text
//!  1. LLE+0x2C bits[30:25]=ch37, bits[1:0]=01 — TX arm + channel field in cfg reg
//!  2. BB+0x64 = 160              — event timeout counter
//!  3. RFEND+0x44 = PLL dividers  — int_div + frac_div for 2402 MHz
//!     RFEND+0x2C bit1 = 1       — PLL lock set
//!  4. LLE+0x00 bits[8:7] = 10b  — TX path select (clear then set)
//!  5. RFEND+0x08 |= 0x330000    — PA / LNA / TX path enable
//!  6. BB+0x50 = 90              — TX pre-delay timer
//!  7. RFEND+0x2C bit1 = 0       — PLL lock release (channel-tracking mode)
//!  8. LLE+0x00 bits[6:0] = 37 | 0x40  — logical ch37, bit6=1 (whitening)
//!  9. LLE+0x08 = 0x8E89_BED6   — ADV access address
//! 10. LLE+0x04 = 0x55_5555     — ADV CRC seed
//! 11. BB+0x70 = &TX_BUF         — TX buffer pointer (NOT 0x74 which is RX DMA buf)
//! 12. BB+0x04 |= 1              — ADV-mode flag (lib DTM clears this bit)
//! 13. LLE+0x00 |= 0x800000      — ADV GO bit (bit 23, NOT bit 11; lib l39276)
//! 14. LLE+0x2C bits[1:0] = 00   — clear TX arm (commit all config)
//! 15. BB+0x00 = 2               — TX trigger — FINAL write (lib L39288)
//! ```

#![no_std]
#![no_main]

use core::ptr::{addr_of, read_volatile, write_volatile};
use {ch32_hal as hal, panic_halt as _};

extern "C" {
    fn BLE_IPCoreInit();
    // Task #20 forensic (2026-05-04):
    // IRQ64 stays masked in Path C (`PATHC_ENABLE_LLE_IRQ=false`). The lib body
    // is retained explicitly by `_KEEP_LLE_IRQ_HANDLER` below; removing both the
    // call and symbol retention GC'd ~840B and failed the 60s BLE gate (cba=0).
    fn LLE_IRQSubHandler();
    fn BB_IRQLibHandler();
    fn llAdvertiseCreateCore();
    fn llAdvertiseSet();
    fn llAdvertiseStart();
    fn llAdvTraverseallChannel();
    static mut gPaControl: u32;
    static mut dtmFlag: u8;
    // V_T2_b bisect: gBleIPPara now migrated to Rust; gBleLlPara still lib BSS COMMON
    static mut gBleLlPara: u8;
}

// Task #22: Keep BB_IRQLibHandler in the binary even though bb_irq_lib_handler()
// (Rust) is now called instead. Without this anchor, --gc-sections removes a large
// libwchble.a subtree and shifts timing-sensitive code. LLE_IRQSubHandler is an
// independent IRQ64 body and is anchored separately by `_KEEP_LLE_IRQ_HANDLER`.
// Remove once full lib removal is validated.
#[used]
static _KEEP_BB_IRQ_LIB_HANDLER: unsafe extern "C" fn() = BB_IRQLibHandler;

// Task #23: BLE_IPCoreInit anchor — same reason as above. Both anchors needed
// until complete lib removal validation.
#[used]
static _KEEP_BLE_IP_CORE_INIT: unsafe extern "C" fn() = BLE_IPCoreInit;

// Task #20: keep the lib LLE IRQ body live without calling it from the IRQ64
// wrapper. The direct call is unreachable in Path C because IRQ64 is masked, but
// removing the symbol entirely shifts layout and kills ADV TX. Explicit retention
// passed the 60s gate (cba=78), so this static is the deliberate anchor.
#[used]
static _KEEP_LLE_IRQ_HANDLER: unsafe extern "C" fn() = LLE_IRQSubHandler;

// Task #25 D-final.1: retain 4 llAdvertise* lib symbols for code-layout
// anchoring. These functions are dead code in Path C — the vtable at
// gBleLlPara+0x68..0x74 is never dereferenced at runtime (TMOS scheduler
// never runs). Removing them causes -22440B linker GC (Iron Law #22).
// D-1a.0b size-neutral sentinel gate (median=64) confirmed content dead.
#[used]
static _KEEP_LL_ADV_CREATE_CORE: unsafe extern "C" fn() = llAdvertiseCreateCore;
#[used]
static _KEEP_LL_ADV_SET: unsafe extern "C" fn() = llAdvertiseSet;
#[used]
static _KEEP_LL_ADV_START: unsafe extern "C" fn() = llAdvertiseStart;
#[used]
static _KEEP_LL_ADV_TRAVERSE: unsafe extern "C" fn() = llAdvTraverseallChannel;

// Task #25 D-final.1: 8B rodata pad to keep BIN size = baseline 51588B.
// The 4 anchors above add ~16B net; sentinel vtable writes (below) have
// same instruction count as fn-ptr writes; pad compensates the difference.
#[used]
#[link_section = ".rodata"]
static _PHASE_D_PAD: [u8; 8] = [0u8; 8];

// Task #23 Variant B bisect anchor: keeps ble_ip_core_init() body in flash even
// when the call site uses the FFI path. Without this, Rust DCE removes the function
// and the SRAM layout collapses back to baseline (0x20001738), making the bisect
// equivalent to a baseline re-run rather than a single-variable layout test.
#[used]
static _KEEP_RUST_IP_CORE_INIT: unsafe fn() = hal::ble::ble_ip_core_init;

// Phase D+1 T1: MMIO register base pointer globals — Rust-defined.
// Previously COMMON BSS in libwchble.a, written by BLE_IPCoreInit at runtime.
// Hardcoded to the CH32V208 hardware constants (confirmed from ble_ip_core_init()).
// WCH naming note: gptrBBReg=0x40024100 is our lle_*/timer range (confusing but correct);
//                  gptrLLEReg=0x40024200 is our bb_*/CTRL-GO range.
// ble_ip_core_init() still writes these (redundant; same value), which is harmless.
#[no_mangle]
pub static mut gptrBBReg:    u32 = 0x4002_4100; // WCH "BB"   = our lle_* timer/IRQ range
#[no_mangle]
pub static mut gptrLLEReg:   u32 = 0x4002_4200; // WCH "LLE"  = our bb_* CTRL/GO range
#[no_mangle]
pub static mut gptrAESReg:   u32 = 0x4002_4300; // AES crypto block
#[no_mangle]
pub static mut gptrRFENDReg: u32 = 0x4002_5000; // RF/PLL analog calibration block

// Phase D+1 T2: BSS struct globals — Rust-defined.
// Previously COMMON BSS in libwchble.a, zeroed at startup by linker.
// Sizes verified from lib nm (lib reports: ble=64, gBleLlPara=296, gBleIPPara=40).
// gBleIPPara uses conservative +24B margin: diagnostic dump reads 0x40=64B and
// lib anchor functions (BB_IRQLibHandler/BLE_IPCoreInit/LLE_IRQSubHandler) have
// not been fully audited for gBleIPPara[40..] access — unaudited risk surface.
//
// CRITICAL: use [u32; N/4] not [u8; N]. Lib code accesses these as u32* structs.
// [u8; N] has alignment=1; linker places it at unaligned addresses (mod4=3 observed)
// causing misaligned u32 reads → undefined behaviour → cba=0.
// [u32; N/4] forces 4-byte alignment (same as lib COMMON BSS natural alignment).
#[no_mangle]
pub static mut ble:        [u32; 16] = [0; 16]; // 64B, u32 for 4-byte alignment
#[no_mangle]
pub static mut gBleIPPara: [u32; 16] = [0; 16]; // 64B (lib=40, +24B margin), u32 for 4-byte alignment — V_T2_b

// Phase D+1 T2: rodata size-neutral pad. Moving ble/gBleLlPara/gBleIPPara from lib
// COMMON BSS to Rust BSS caused linker GC to drop -72B vs baseline. This pad restores
// BIN to 51588B (Iron Law #22: layout shift → cba=0). Remove or adjust in T8 cleanup.
#[used]
#[link_section = ".rodata"]
static _T2_PAD: [u8; 64] = [0u8; 64]; // V_T2_b: ble+gBleIPPara migration → BIN -60B, pad to restore 51588B

// ── Register bases ────────────────────────────────────────────────────────────

const LLE_BASE:   usize = 0x40024100;
const BB_BASE:    usize = 0x40024200;
const RFEND_BASE: usize = 0x40025000;

#[inline(always)]
unsafe fn lle_read(off: usize) -> u32 { read_volatile((LLE_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn lle_write(off: usize, v: u32) { write_volatile((LLE_BASE + off) as *mut u32, v); }
#[inline(always)]
unsafe fn bb_read(off: usize) -> u32 { read_volatile((BB_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn bb_write(off: usize, v: u32) { write_volatile((BB_BASE + off) as *mut u32, v); }
#[inline(always)]
unsafe fn rfend_read(off: usize) -> u32 { read_volatile((RFEND_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn rfend_write(off: usize, v: u32) { write_volatile((RFEND_BASE + off) as *mut u32, v); }

unsafe fn write_ll_u8(base: *mut u8, off: usize, v: u8) {
    write_volatile(base.add(off), v);
}

unsafe fn write_ll_u16(base: *mut u8, off: usize, v: u16) {
    write_volatile(base.add(off) as *mut u16, v);
}

unsafe fn write_ll_u32(base: *mut u8, off: usize, v: u32) {
    write_volatile(base.add(off) as *mut u32, v);
}

#[link_section = ".bss"]
static mut RUST_ADV_CTX: [u8; 192] = [0; 192];

/// Safe prefix of WCH LL_Init, extracted from ll.o before ll_get_sca_own and
/// later TMOS/role-dependent calls. This seeds the LL frequency/timing state
/// without entering the full LL_Init path, which hangs in the standalone probe.
unsafe fn ll_init_safe_prefix() {
    let p = core::ptr::addr_of_mut!(gBleLlPara) as *mut u8;

    core::ptr::write_bytes(core::ptr::addr_of_mut!(gBleLlPara) as *mut u8, 0, 296);

    write_ll_u32(p, 0x00, 0x0000_0004);
    write_ll_u32(p, 0x14, 0x07d7_000d);
    write_ll_u32(p, 0x18, 0x0d0d_b140);
    write_ll_u32(p, 0x1c, 0x07d7_b140);
    write_ll_u16(p, 0x20, 0xb140);
    write_ll_u16(p, 0x22, 27);
    write_ll_u32(p, 0x24, 0x0148_0528);
    write_ll_u32(p, 0x28, 0x001b_0528);
    write_ll_u32(p, 0x2c, 0x001b_0528);
    write_ll_u32(p, 0x30, 0x0000_0528);
    write_ll_u32(p, 0x34, 0x0015_f900);

    write_ll_u32(p, 0xe0, 31);
    write_ll_u32(p, 0xe4, 0);
    write_ll_u32(p, 0xd0, 0x072d_79ff);
    write_ll_u32(p, 0xd4, 0x0000_1b9e);
    write_ll_u32(p, 0xd8, 0x072d_79ff);
    write_ll_u32(p, 0xdc, 0x0000_1b9e);

    write_ll_u8(p, 0xc1, 0);
    write_ll_u8(p, 0xc9, 0);
    write_ll_u16(p, 0x106, 0xdfff);
    write_ll_u16(p, 0x108, 0xdfff);
    write_ll_u8(p, 0x10a, 31);
    write_ll_u8(p, 0x39, 0);
    write_ll_u16(p, 0x3a, 0x0f0f);
    write_ll_u32(p, 0x3c, 0x0101_0f0f);
    write_ll_u32(p, 0x40, 0x0003_01cc);

    // ll_get_sca_own is skipped; BSS default is 0. Keep the following
    // role-independent writes from the same prefix.
    write_ll_u8(p, 0x03, 0);
    write_ll_u16(p, 0x7e, 460);
    write_ll_u8(p, 0x3f, 1);
    write_ll_u8(p, 0x89, 0);
    write_ll_u16(p, 0x42, 3);
    write_ll_u32(p, 0x44, 0);
    write_ll_u32(p, 0x48, 0x0607_1440);

    // Equivalent of LL_AdvertiseEnalbe's callback table plus the ADV core list
    // head set up by llAdvertiseCreateCore. Use Rust-local addresses.
    //
    // Task #21 forensic (2026-05-03): these four vtable slots look unused in
    // Path C's explicit control flow, but hardware gate showed they must stay
    // populated. Removing the writes makes `cba` disappear even when a
    // `#[used]` anchor keeps llAdvertise* symbols and the binary layout at the
    // 42e57ef baseline. Direct deletion also GC's ~47KB of lib text and shifts
    // gBleLlPara, which is another unsafe layout change. Keep these writes until
    // the hidden init/IRQ consumer is fully identified.
    //
    // Task #21 R2 empirical lock (2026-05-04): to isolate SYMBOL PRESENCE from
    // CONTENT, R2 added `#[used] static` anchors for all 4 llAdvertise* symbols
    // (keeping them in the binary) but commented out these 4 vtable writes
    // (zeroing slots 0x68/0x6c/0x70/0x74). Gate: cba=0 (60s scan).
    // Conclusion: lib init/IRQ path reads the FUNCTION ADDRESSES from the vtable,
    // not just checks whether the symbols exist. `#[used] static` retention is
    // insufficient here — the CONTENT (concrete Rust fn pointers) must be written.
    // Boundary of the `#[used] static` pattern: effective when lib checks symbol
    // presence / code layout (task #20 LLE_IRQSubHandler, cba=78); ineffective
    // when lib dereferences a function-pointer field (task #21 vtable dispatch).
    //
    // 2026-05-04 15:38 UPDATE (Task #25 D-1a.0b — above conclusion REVISED):
    // D-1a.0b padding-neutral test (4 #[used] anchors + 4 sentinel writes +
    // 8B rodata pad, BIN exact 51588B): 3-round 60s cba=[68,64,54], median=64.
    // R2 cba=0 was the -40B BIN delta triggering Iron Law #22 layout shift,
    // NOT vtable content consumption. In Path C the vtable content at
    // gBleLlPara+0x68..0x74 is never dereferenced (TMOS scheduler never runs;
    // LL_ProcessEvent is never invoked, so the jalr dispatch never fires).
    // Cross-validation: anchor-less sentinel caused -22440B GC, proving
    // 4 llAdvertise* + transitive call tree (~22kB) has no other Rust ref.
    // `#[used] static` retention IS effective here — size-neutral suffices.
    // See notes/ch32-rs/ll-advertise-create-core-disasm.md §6 for full audit.
    let adv_ctx = core::ptr::addr_of_mut!(RUST_ADV_CTX) as u32;
    write_ll_u32(p, 0x58, adv_ctx);
    write_ll_u32(p, 0x5c, adv_ctx);
    write_ll_u32(p, 0x60, adv_ctx);
    write_ll_u32(p, 0x64, adv_ctx);
    // D-final.1: vtable dispatch slots marked dead. Path C never invokes
    // LL_ProcessEvent (TMOS scheduler not running), so these slots are
    // never dereferenced. Sentinel value 0x12345678 = invalid fn ptr.
    // Symbols kept live via _KEEP_LL_ADV_* anchors above (layout anchor).
    write_ll_u32(p, 0x68, 0x1234_5678); // llAdvertiseCreateCore — dead in Path C
    write_ll_u32(p, 0x6c, 0x1234_5678); // llAdvertiseSet         — dead in Path C
    write_ll_u32(p, 0x70, 0x1234_5678); // llAdvertiseStart       — dead in Path C
    write_ll_u32(p, 0x74, 0x1234_5678); // llAdvTraverseallChannel — dead in Path C
    write_ll_u32(p, 0x7c, 0x01cc_0001);
    write_ll_u32(p, 0x88, 0x0000_0700);
    write_ll_u32(p, 0xc0, 0x0000_0300);
    write_ll_u32(p, 0xc8, 0x0000_0300);
}

unsafe fn seed_ble_bd_addr() {
    let bd: [u8; 6] = [0x12, 0x87, 0x65, 0x43, 0x21, 0xC2];
    let p = core::ptr::addr_of_mut!(ble) as *mut u8;
    for (i, b) in bd.iter().enumerate() {
        write_volatile(p.add(0x18 + i), *b);
    }
    // LL_AddrInit copies ble[0x18..0x1d] into gBleLlPara+0xE8 through
    // tmos_memcpy. Do the effective store directly to avoid the tmos bb+0x0C guard.
    let ll = core::ptr::addr_of_mut!(gBleLlPara) as *mut u8;
    for (i, b) in bd.iter().enumerate() {
        write_volatile(ll.add(0xE8 + i), *b);
    }
}

// ── Isolation mode selector (for bisect testing; change and recompile) ────────
//
//  0 = fix2 FULL  — fix1 (BB+0x04|=1, bit23, BB+0x00=2 last) + BLE_SetPHYTxMode (current)
//  1 = iso ORDERING only — only move BB+0x00=2 last; no BB+0x04 or bit23 change
//  2 = iso BB+0x04 only  — only add ADV-mode flag; keep original ordering
//  3 = iso BIT23 only    — only bit23 re-set; keep original ordering
//  4 = BASELINE OLD      — original step 11.5+12+13 (control / known-bad)
//
// To run an isolation test: set ISO_MODE below, cargo build, flash, check HackRF.
const ISO_MODE: u8 = 0; // ← change this

// ── Tier A trigger-pre full register dump ─────────────────────────────────────
//
// When DUMP_TIER_A=true, dump all 3 BLE register windows (LLE/BB/RFEND, 256B each)
// immediately before Step 15 (BB+0x00=2 TX trigger) on the FIRST burst only.
// Output format: `# snap <label> base=0x... off=0x00..0xfc` + one `0xaddr,0xval` line per word.
//
// Cindy's EVT-side Tier A will hook the same point in ll_advertise_tx.
// Comparing both outputs reveals any register not written by our 15-step sequence
// that EVT has in a non-default state at trigger time.
const DUMP_TIER_A: bool = false;
const PATHC_REPLACE_TIER0: bool = true;
const PATHC_USE_RUST_IP_CORE_INIT: bool = true;
const Y200_SNAPSHOT: bool = false;
const Z37_SNAPSHOT: bool = false;
const Z37_TARGET_BURST: u32 = 1; // zero-based: capture burst 2.
const GBLELL_SNAPSHOT: bool = false;
const PATHC_LIB_IRQ: bool = true;
const PATHC_ENABLE_LLE_IRQ: bool = false;
const PATHC_MANUAL_L6: bool = false;
// Task #22 V1.1 probe: true = Rust bb_irq_lib_handler (testing), false = FFI BB_IRQLibHandler (V_A3 reference).
// Set false to build V_A3+probe for comparative trace without changing the ISR register snapshot.
const PATHC_USE_RUST_BB_IRQ: bool = true;
// Task #22 V1.2 bisect: controls how much of the Rust BB handler runs.
//   0 = reads only (blk/ip4/bb08 recorded, no writes — confirm no exception from just reading)
//   1 = + W1C bit6 (write 0x60 to WCH_BBR+0x38, first peripheral write)
//   2 = + .L6 block writes (WCH_LLER+0x08=0x2000, WCH_LLER+0x64=timer)
//   3 = full handler (all paths, same as V1.1)
const BB_HANDLER_STAGE: u8 = 5; // 5+ = full hal::ble::bb_irq_lib_handler()
const PATHC_BIT0_PULSE: bool = false;
static mut TIER_A_DONE: bool = false;
static mut Y200_DUMPED: bool = false;
static mut Y200_PRINTED: bool = false;
static mut Z37_DUMPED: bool = false;
static mut Z37_PRINTED: bool = false;
static mut Z37_HIT: bool = false;
static mut Z37_BB08: u32 = 0;
static mut Z37_TIMEOUT_LEFT: u32 = 0;
static mut BB08_TRACE_DONE: bool = false;
static mut BB08_TRACE_PRINTED: bool = false;
static mut BB08_TRACE: [u32; 200] = [0; 200];
static mut GBLELL_DUMPED: bool = false;
static mut GBLELL_PRINTED: bool = false;
static mut GBLELL_SNAPSHOT_WORDS: [u32; 74] = [0; 74];
static mut ADV_CTX_DUMPED: bool = false;
static mut ADV_CTX_PRINTED: bool = false;
static mut TX_PTR_SNAPSHOT: u32 = 0;
static mut TX_PTR_DUMP: [u8; 64] = [0; 64];
static mut ADV_CTX_DUMP: [u8; 192] = [0; 192];
static mut Y3_BB00_LOGGED: bool = false;
static mut Y3_BB64_LOGGED: bool = false;
static mut Y4_039_LOGGED: bool = false;
static mut BB_IRQ_ENTRY: u32 = 0;
static mut BB_IRQ_EXIT: u32 = 0;
static mut LLE_IRQ_ENTRY: u32 = 0;
static mut LLE_IRQ_EXIT: u32 = 0;
// V1.2 staged bisect: counter + per-stage statics (observable from main loop even if IRQ storms).
// Stage 0 (reads-only): auto-disables BB IRQ after BB_STUB_MAX_ENTRIES to let main loop print.
const BB_STUB_MAX_ENTRIES: u32 = 50;
static mut BB_STUB_ENTER_COUNT: u32 = 0;
static mut BB_STUB_BLK: [u32; 4] = [0; 4];   // LLE_BASE+0x38 sampled at handler start
static mut BB_STUB_IP4: [u8; 4] = [0; 4];    // gBleIPPara[4] at handler start
static mut BB_STUB_BB08: [u32; 4] = [0; 4];  // BB_BASE+0x08 at handler start (= WCH_LLER+0x08)
static mut BB_IRQ_STATUS: [u32; 4] = [0; 4];
static mut BB_IRQ_IP4: [u8; 4] = [0; 4];
static mut BB_IRQ_LLE38: [u32; 4] = [0; 4];
// V1.1 probe: BB_BASE+0x38 = 0x40024238 (WCH_LLER+0x38, naming-inversion sanity check).
// We expect this differs from LLE_BASE+0x38 (0x40024138); if alt38 shows bit6 when lle38=0,
// that pinpoints a base inversion in bb_irq_lib_handler.
static mut BB_IRQ_ALT38: [u32; 4] = [0; 4];
static mut BB_IRQ_SNAP_COUNT: usize = 0;
static mut MANUAL_L6_HIT: bool = false;
static mut MANUAL_L6_WAIT: u32 = 0;
static mut MANUAL_L6_BEFORE: u32 = 0;
static mut MANUAL_L6_AFTER: u32 = 0;
static mut MANUAL_L6_IP4_BEFORE: u8 = 0;
static mut MANUAL_L6_IP4_AFTER: u8 = 0;

#[link_section = ".bss"]
static mut Y200_BB: [u32; 64] = [0; 64];
#[link_section = ".bss"]
static mut Y200_LLE: [u32; 64] = [0; 64];
#[link_section = ".bss"]
static mut Y200_RFEND: [u32; 52] = [0; 52];

// ── BLE ADV constants ─────────────────────────────────────────────────────────

const ADV_AA:       u32 = 0x8E89_BED6;
const ADV_CRC_INIT: u32 = 0x55_5555;
const ADV_CH37_FREQ_KHZ: u32 = 2_402_000;

/// Device address, LE order.
/// Over-the-air: C2:21:43:65:87:12 (reversed from this array).
const ADDR: [u8; 6] = [0x12, 0x87, 0x65, 0x43, 0x21, 0xC2];

/// Match EVT broadcaster header style: public-address ADV_NONCONN_IND (TxAdd=0).
const TXADD_RANDOM: bool = false;

/// Capture TX bursts as register traces. The trace is printed after the burst
/// finishes so SDI output does not perturb the trigger sequence.
const TRACE_FIRST_BURST: bool = true;
const TRACE_EVERY_N: u32 = 100;
static mut TRACE_ARMED: bool = false;

// ── B experiment — BB+0x74 alignment ─────────────────────────────────────────
//
// B_EXPERIMENT=true: override BB+0x74 to point LLE_DMA_BUF_B at 0x20008234.
//   Target readback: (0x20008234 >> 2) & 0xFF = 0x8D, matching EVT.
//   Current baseline: LLE_DMA_BUF at ~0x200000D0 → readback 0x34.
//   build.rs places .lle_dma_b at 0x20008234 via lle_dma_b.x linker script.
// B_EXPERIMENT=false: baseline (lle_dev_init keeps LLE_DMA_BUF, readback 0x34).
const B_EXPERIMENT: bool = false;

/// B experiment DMA buffer — placed at 0x20008234 by build.rs linker script.
/// Size 1024B (256×u32). Used only when B_EXPERIMENT=true.
/// NOLOAD: not zero-initialised from flash; LLE uses it as scratch, so this is fine.
#[link_section = ".lle_dma_b"]
static mut LLE_DMA_BUF_B: [u32; 256] = [0u32; 256];

// ── Trigger-POST timing scan ──────────────────────────────────────────────────
//
// Reads BB+0x08 + BB+0x1C at 4 absolute time points after GO (BB+0x00=2).
// At 96 MHz: 1µs ≈ 96 cycles.
//   +100µs ≈  9 600 cycles
//   +200µs ≈ 19 200 cycles (9 600 more)
//   +400µs ≈ 38 400 cycles (19 200 more)
//   +521µs ≈ 50 016 cycles (11 616 more)
// Reads collected into locals first; printed after to avoid SDI perturbing timing.
// Runs on first burst only (TRIGGER_POST_PRINTED gate).
static mut TRIGGER_POST_PRINTED: bool = false;
const X1_POLLED_W1C: bool = true;

// ── TX buffer ─────────────────────────────────────────────────────────────────

/// TX buffer: header(2B) + AdvA(6B) + AD payload (max 31B).
/// Filled once in main before the TX loop.
///
/// A3 experiment (2026-05-02) CLOSED: placed at 0x20004344, BB+0x70 readback=0x10D1,
/// formula (val>>2)&0x1FFF confirmed 13-bit. bleak=0 → BB+0x70 addressing is NOT the
/// root cause. Reverted to default linker placement (BSS, readback ~0x54f).
static mut TX_BUF: [u8; 39] = [0u8; 39];

#[ch32_hal::interrupt]
fn BB() {
    unsafe {
        BB_IRQ_ENTRY = BB_IRQ_ENTRY.wrapping_add(1);
        // Task #22 V1.2 staged bisect.
        // PATHC_USE_RUST_BB_IRQ=false → FFI BB_IRQLibHandler (V_A3 reference)
        // PATHC_USE_RUST_BB_IRQ=true  → Rust handler at BB_HANDLER_STAGE:
        //   0: reads-only stub (no writes; expects IRQ storm, auto-disables after BB_STUB_MAX_ENTRIES)
        //   1: + W1C bit6 (write 0x60 to LLE_BASE+0x38)
        //   2: + .L6 SRAM writes (gBleIPPara[4,5] only; NO hardware writes)
        //   3: + WCH_LLER+0x08=0x2000 write (BB_BASE+0x08 = 0x40024208, advances PHY 0x33→0x37)
        //      also reads ip[16] as u32 (alignment check) but does NOT write WCH_LLER+0x64
        //   4: + WCH_LLER+0x64=timer write (BB_BASE+0x64 = 0x40024264, full .L6)
        //   5+: full Rust handler via hal::ble::bb_irq_lib_handler() (same as V1.1)
        if PATHC_USE_RUST_BB_IRQ {
            match BB_HANDLER_STAGE {
                0 => {
                    // Reads-only stub. Captures blk/ip4/bb08 for up to 4 IRQs, then
                    // auto-disables BB IRQ so main loop can print the results.
                    let blk  = read_volatile((LLE_BASE + 0x38) as *const u32);
                    let ip4v = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
                    let b08  = read_volatile((BB_BASE + 0x08) as *const u32);
                    let idx = BB_STUB_ENTER_COUNT as usize;
                    if idx < 4 {
                        BB_STUB_BLK[idx]  = blk;
                        BB_STUB_IP4[idx]  = ip4v;
                        BB_STUB_BB08[idx] = b08;
                    }
                    BB_STUB_ENTER_COUNT = BB_STUB_ENTER_COUNT.wrapping_add(1);
                    // Without W1C of bit6, hardware immediately reasserts IRQ → storm.
                    // Disable after BB_STUB_MAX_ENTRIES so main loop can run and print.
                    if BB_STUB_ENTER_COUNT >= BB_STUB_MAX_ENTRIES {
                        qingke::pfic::disable_interrupt(63);
                    }
                }
                1 => {
                    // Stage 1: + W1C bit6 (first peripheral write).
                    let blk = read_volatile((LLE_BASE + 0x38) as *const u32);
                    if blk & (1 << 6) != 0 {
                        write_volatile((LLE_BASE + 0x38) as *mut u32, 0x60); // W1C bits 5+6
                    }
                }
                2 => {
                    // Stage 2: + .L6 SRAM writes (gBleIPPara[4,5] only).
                    let blk = read_volatile((LLE_BASE + 0x38) as *const u32);
                    if blk & (1 << 6) != 0 {
                        write_volatile((LLE_BASE + 0x38) as *mut u32, 0x60);
                        let ip4v = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
                        if ip4v & 0x40 == 0 {
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(5), 1u8);
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(4), 0xC0u8);
                        }
                    }
                }
                3 => {
                    // Stage 3a: + WCH_LLER+0x08=0x2000 (BB_BASE+0x08 = 0x40024208).
                    // Also reads ip[16] as u32 to test alignment. Does NOT write WCH_LLER+0x64 yet.
                    // If hang here: write(0x40024208, 0x2000) triggers IRQ re-entry or fault.
                    let blk = read_volatile((LLE_BASE + 0x38) as *const u32);
                    if blk & (1 << 6) != 0 {
                        write_volatile((LLE_BASE + 0x38) as *mut u32, 0x60);
                        let ip4v = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
                        if ip4v & 0x40 == 0 {
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(5), 1u8);
                            // WCH_LLER+0x08 = BB_BASE+0x08 = 0x40024208: advance PHY 0x33→0x37
                            write_volatile((BB_BASE + 0x08) as *mut u32, 0x2000);
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(4), 0xC0u8);
                            // Read ip[16] as u32 (alignment test; no hardware write yet)
                            let _timer = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(16) as *const u32);
                        }
                    }
                }
                4 => {
                    // Stage 3b: + WCH_LLER+0x64=timer (BB_BASE+0x64 = 0x40024264). Full .L6.
                    // If hang here but not in stage 3a: write(0x40024264, timer) is the fault.
                    let blk = read_volatile((LLE_BASE + 0x38) as *const u32);
                    if blk & (1 << 6) != 0 {
                        write_volatile((LLE_BASE + 0x38) as *mut u32, 0x60);
                        let ip4v = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
                        if ip4v & 0x40 == 0 {
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(5), 1u8);
                            write_volatile((BB_BASE + 0x08) as *mut u32, 0x2000); // WCH_LLER+0x08
                            write_volatile((core::ptr::addr_of_mut!(gBleIPPara) as *mut u8).add(4), 0xC0u8);
                            let timer = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(16) as *const u32);
                            write_volatile((BB_BASE + 0x64) as *mut u32, timer); // WCH_LLER+0x64
                        }
                    }
                }
                _ => {
                    // Stage 5+: full Rust handler via library function (same as V1.1)
                    hal::ble::bb_irq_lib_handler();
                }
            }
        } else {
            BB_IRQLibHandler();
        }
        // Bug A removed: post-handler W1C of lle38 bits[4,6,7] deleted.
        // WCH BB_IRQLibHandler does NOT have this extra W1C. It was a historical
        // debug artifact that raced against .L4/.L8: if bit4/7 arrived after the
        // .L4/.L8 re-reads inside bb_irq_lib_handler, this W1C would clear them
        // without writing ip[4]=1, causing ip[4] to stay at 0xC0 (stuck).
        // With it removed, late bit4/7 cause a second IRQ entry where .L4/.L8
        // correctly catches them — exactly matching WCH library behavior.
        BB_IRQ_EXIT = BB_IRQ_EXIT.wrapping_add(1);
    }
}

#[ch32_hal::interrupt]
fn LLE() {
    unsafe {
        LLE_IRQ_ENTRY = LLE_IRQ_ENTRY.wrapping_add(1);
        // IRQ64 is masked in Path C. Keep the vector stub and counters alive, but
        // leave the lib body as an explicit retention anchor above.
        LLE_IRQ_EXIT = LLE_IRQ_EXIT.wrapping_add(1);
    }
}

/// Build minimal ADV_NONCONN_IND PDU into TX_BUF.
///
/// AD payload matches EVT Broadcaster shape, but name is reversed to identify Rust:
/// Flags(3B) + Manufacturer "ble"(5B) + Shortened Local Name "cba"(5B) = 13B.
/// Total payload = AdvA(6) + AD(13) = 19B → header Length = 19.
unsafe fn build_adv_pdu() -> usize {
    let mut pos = 2usize; // skip header bytes, fill below

    // AdvA: 6 bytes LE
    TX_BUF[pos..pos + 6].copy_from_slice(&ADDR);
    pos += 6;

    // AD: Flags = BR/EDR Not Supported (EVT Broadcaster uses 0x04).
    TX_BUF[pos]     = 2;    // length
    TX_BUF[pos + 1] = 0x01; // type: Flags
    TX_BUF[pos + 2] = 0x04; // value
    pos += 3;

    // AD: Manufacturer specific, EVT-compatible bytes "ble".
    TX_BUF[pos]     = 4;    // length: type + 3 bytes
    TX_BUF[pos + 1] = 0xFF; // type: Manufacturer Specific Data
    TX_BUF[pos + 2] = b'b';
    TX_BUF[pos + 3] = b'l';
    TX_BUF[pos + 4] = b'e';
    pos += 5;

    // AD: Shortened Local Name "cba" (type 0x08), reversed from EVT "abc".
    let name = b"cba";
    TX_BUF[pos]     = (name.len() + 1) as u8; // length
    TX_BUF[pos + 1] = 0x08;                    // type: Shortened Local Name
    TX_BUF[pos + 2..pos + 2 + name.len()].copy_from_slice(name);
    pos += 2 + name.len();

    let payload_len = pos - 2; // AdvA + AD, excludes header bytes

    // Header byte 0: ADV_NONCONN_IND (type=0x02), EVT-style TxAdd=0 by default.
    TX_BUF[0] = 0x02 | if TXADD_RANDOM { 1 << 6 } else { 0 };
    // Header byte 1: Length (6-bit)
    TX_BUF[1] = payload_len as u8 & 0x3F;

    pos // total bytes written
}

// ── PLL programming (same formula as DTM TX / RX, no offset) ─────────────────

/// Program RFEND PLL for `freq_khz` and assert channel lock (RFEND+0x2C bit1=1).
///
/// Formula from RF_DevSetChannel (d.asm L71418, Lucy 2026-04-30):
///   int_div  = (freq_khz / 64000) & 0x1F
///   frac_div = ((freq_khz % 64000) << 10) / 250
///
/// Mask 0xFE0F_C000 preserves bits[31:25] (read-back from calibration) and
/// bits[17:14] (other PLL fields), zeroes bits[24:20] (int) and bits[13:0] (frac).
unsafe fn set_channel_freq(freq_khz: u32) {
    let int_div  = (freq_khz / 64_000) & 0x1F;
    let frac_div = ((freq_khz % 64_000) << 10) / 250;
    let pll = rfend_read(0x44);
    rfend_write(0x44, (pll & 0xFE0F_C000) | (int_div << 20) | (frac_div & 0x3FFF));
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v | (1 << 1)); // assert channel lock
}

// ── PHY TX mode setup ─────────────────────────────────────────────────────────

/// Configure BB/LLE for 1 Mbps BLE TX (default Advertising PHY).
///
/// Direct translation of `BLE_SetPHYTxMode(phy=1, pdu_len_plus_1)` 1Mbps branch,
/// re-decoded by Lucy 2026-05-02 from d.asm L91784-91858.
///
/// **Step 3/7 are 32-bit lw/sw** (NOT byte-level lbu/sb). Lucy's earlier description
/// (msg=ca7c17e9) was incorrect; the corrected decode (msg=ca653fe3, d.asm L91834-91838)
/// confirms `lw a0, 12(base)` / `sw a0, 12(base)` — full 32-bit save and restore.
///
/// The strobe (bit13 of BB+0x0C) is a no-op in both EVT and our code because
/// BB+0x0C = 0 post-init in both cases (EVT confirmed by Lucy). Step 4
/// (`bb_write(0x08, 0x2000)` = PHY rate lock) is the critical step; without it
/// the radio may modulate at the wrong PHY rate (nRF can't decode).
///
/// # Parameters
///
/// `pdu_body_len`: BLE PDU body length (header byte 1), NOT including the 2-byte
///   PDU header. Internally incremented by 1 before the slot-timer formula.
///   For our PDU: AdvA(6)+Flags(3)+Name(9) = 18 → len_plus_1=19 → BB+0x64=556.
///
/// # Safety
///
/// Must be called in the trigger sequence: after BB+0x04|=1, before BB+0x00=2.
unsafe fn ble_set_phy_tx_mode_1mbps(pdu_body_len: u8) {
    let ip = core::ptr::addr_of!(gBleIPPara) as *const u8;
    hal::println!(
        "# PATHC_TIMER pre-setphy ip16={:#010x} bb64={:#010x}",
        read_volatile(ip.add(16) as *const u32),
        bb_read(0x64)
    );

    // gPaControl path: CH32V208 has integrated PA; enabled via RFEND+0x08 in adv_tx_burst.

    // Step 1: clear PHY rate bits[13:12] of LLE+0x00 (1Mbps = 00b).
    //   lib asm: lw + and !0x3000 + sw  (= our 0xFFFF_CFFF mask)
    let v = lle_read(0x00);
    trace_r(LLE_BASE as u32 + 0x00, v);
    let v = v & !0x3000;
    trace_w(LLE_BASE as u32 + 0x00, v);
    lle_write(0x00, v);

    // Step 3: 32-bit save + clear bit13 (strobe; no-op since BB+0x0C=0 post-init in both EVT+us).
    //   lib asm L91834: lw a0, 12(lib_LLE_base)  (32-bit load — NOT lbu; corrected msg=ca653fe3)
    let saved_0c = bb_read(0x0C);
    trace_r(BB_BASE as u32 + 0x0C, saved_0c);
    let v = saved_0c & !0x2000;
    trace_w(BB_BASE as u32 + 0x0C, v);
    bb_write(0x0C, v);

    // Step 4: PHY rate lock — marks 1Mbps mode AND W1C-clears bit13 (timer IRQ).
    //   lib asm: li a4, 0x2000; sw a4, 8(lib_LLE_base)
    //   This is the critical step: without it nRF will not decode the packet.
    trace_w(BB_BASE as u32 + 0x08, 0x2000);
    bb_write(0x08, 0x2000);

    // Step 5: gBleIPPara[4] = 0x80 — restored (was skipped; now mirrors WCH BLE_SetPHYTxMode).
    // Root cause fix (task #22 lifecycle probe): if the post-handler W1C in BB() clears a
    // "late" bit4/7 in lle38 without the ip[4]=1 re-arm (because .L4/.L8 re-read found 0),
    // ip[4] stays at 0xC0. Since step5 was skipped, ip[4]=0xC0 persisted to the next burst,
    // blocking .L6 (0xC0 & 0x40 ≠ 0). WCH's BLE_SetPHYTxMode resets ip[4]=0x80 every burst,
    // so it is immune. Adding step5 back makes our path immune too.
    write_volatile((addr_of!(gBleIPPara) as *mut u8).add(4), 0x80u8);

    // Step 6: TX slot timer (lib asm L91852-91856, exact decode by Lucy msg=83cacb1f).
    //   formula: (((pdu_len_plus_1 + 11) << 2) + (gBleIPPara[0xa] + 158)) << 1
    //   gBleIPPara[0xa]=0 confirmed by Lucy (BSS-init, only written by RX cancel path with zero).
    //   len_plus_1=19 → ((30<<2) + 158) << 1 = (120+158)<<1 = 556
    let len_plus_1 = (pdu_body_len & 0x3F) as u32 + 1;
    let timer = (((len_plus_1 + 11) << 2) + 158) << 1;
    trace_w(BB_BASE as u32 + 0x64, timer);
    bb_write(0x64, timer);
    hal::println!(
        "# PATHC_TIMER post-setphy timer={} ip16={:#010x} bb64={:#010x}",
        timer,
        read_volatile(ip.add(16) as *const u32),
        bb_read(0x64)
    );

    // Step 7: 32-bit restore (no-op strobe since saved_0c=0 in both EVT and us post-init).
    //   lib asm L91838: sw a0, 12(lib_LLE_base)  (32-bit store — NOT sb; corrected msg=ca653fe3)
    trace_w(BB_BASE as u32 + 0x0C, saved_0c);
    bb_write(0x0C, saved_0c);

    // Step 8: RFEND+0x8C lower-byte clear — DEFERRED (not from BLE_SetPHYTxMode per Lucy d.asm).
    //   Lucy (2026-05-02 msg=8f98bdc7): BLE_SetPHYTxMode & ll_advertise_tx do NOT write RFEND+0x8C.
    //   EVT 0x10e→0x100 change source unknown (cal chain / hardware / IRQ path).
    //   A3 single-variable: leave RFEND+0x8C at init value (0x105) to isolate BB+0x70 DMA address.
    //   Test separately after A3 result is known.
    //
    // let v8c = rfend_read(0x8C);
    // rfend_write(0x8C, v8c & 0xFFFF_FF00);
}

#[inline(always)]
unsafe fn trace_on() -> bool {
    TRACE_ARMED
}

#[inline(always)]
unsafe fn trace_r(addr: u32, val: u32) {
    if trace_on() {
        hal::ble::tracer::r(addr, val);
    }
}

#[inline(always)]
unsafe fn trace_w(addr: u32, val: u32) {
    if trace_on() {
        hal::ble::tracer::w(addr, val);
    }
}

#[inline(always)]
unsafe fn trace_bb_read(off: usize) -> u32 {
    let v = bb_read(off);
    trace_r(BB_BASE as u32 + off as u32, v);
    v
}

// ── ADV TX burst ──────────────────────────────────────────────────────────────

/// Fire one ADV_NONCONN_IND burst on ch37 (2402 MHz).
///
/// Returns (state_pre_go, state_post_go, irq_post_go):
///   state_pre/post: BB+0x1C LLE state (108=Sleep, non-108 post = HW accepted GO)
///   irq_post_go: BB+0x08 bits[31:0] immediately after GO strobe
unsafe fn adv_tx_burst_ch37(burst_idx: u32) -> (u32, u32, u32) {
    if TRACE_ARMED {
        hal::ble::tracer::reset();
    }

    // Step 1: TX arm — LLE+0x2C bits[30:25]=ble[0x14] state (EVT=9), bits[1:0]=01.
    let cfg = lle_read(0x2C);
    trace_r(LLE_BASE as u32 + 0x2C, cfg);
    let v = (cfg & 0x81FF_FFFF) | (9u32 << 25) | 0x1;
    trace_w(LLE_BASE as u32 + 0x2C, v);
    lle_write(0x2C, v);

    // Step 2: BB+0x64 initial value (overwritten by ble_set_phy_tx_mode_normal in trigger sequence).
    // Kept for compatibility with ISO_MODE=1/2/3/4 which don't call SetPHYTxMode.
    trace_w(BB_BASE as u32 + 0x64, 160);
    bb_write(0x64, 160);

    // Step 3: PLL program + lock.
    // H20 fix: EVT does NOT rewrite RFEND+0x44 per-burst.
    // EVT runtime shows RFEND+0x44 = 0x0060C000 (2440 MHz calibration anchor) during ADV at 2402 MHz.
    // The channel register (step 8, LLE+0x00 bits[5:0]=37) selects the actual RF frequency in PLL
    // tracking mode. Calling set_channel_freq here changes RFEND+0x44 to 0x0050E000 (wrong for ADV)
    // and forces a PLL re-lock cycle (bit1=1→0) that the PLL cannot settle before TX fires.
    // EVT relies on the calibration-time RFEND+0x44 value throughout; we must do the same.
    // set_channel_freq(ADV_CH37_FREQ_KHZ);  // REMOVED: disrupts PLL calibration anchor per-burst

    // Step 4: TX path select — bits[8:7] = 10b (clear first, then set bit8).
    let ctrl = lle_read(0x00);
    trace_r(LLE_BASE as u32 + 0x00, ctrl);
    let v = ctrl & !0x180;
    trace_w(LLE_BASE as u32 + 0x00, v);
    lle_write(0x00, v);
    let ctrl = lle_read(0x00);
    trace_r(LLE_BASE as u32 + 0x00, ctrl);
    let v = (ctrl & !0x180) | 0x100;
    trace_w(LLE_BASE as u32 + 0x00, v);
    lle_write(0x00, v);

    // Step 5: PA / LNA / TX path enable.
    let ana = rfend_read(0x08);
    trace_r(RFEND_BASE as u32 + 0x08, ana);
    let v = ana | 0x0033_0000;
    trace_w(RFEND_BASE as u32 + 0x08, v);
    rfend_write(0x08, v);

    // Step 6: TX pre-delay timer.
    trace_w(BB_BASE as u32 + 0x50, 90);
    bb_write(0x50, 90);

    // Step 7: PLL lock release → channel-tracking mode.
    let v = rfend_read(0x2C);
    trace_r(RFEND_BASE as u32 + 0x2C, v);
    let v = v & !(1 << 1);
    trace_w(RFEND_BASE as u32 + 0x2C, v);
    rfend_write(0x2C, v);

    // Step 8: Channel field — logical ch37, bit6=0.
    //   ADV uses logical channel (37), confirmed by Lucy re ll_advertise_process asm L40091:
    //     li a1, 37  → writes logical 37, not physical idx 0.
    //   bit6 semantics: code writes 0x25 (bit6=0, ch=37).
    //   EVT pAdvCtx[10]=0x26 (bit6=0, ch=38) — bit6=0 in EVT too (Lucy note e1cfc4c1).
    //   Session-5 test: bit6=1 (0x65) also gives 0 hit. Both variants fail.
    //   Lucy: "代码可能本身就对" — bit6=0 might be correct.
    let ctrl = lle_read(0x00);
    trace_r(LLE_BASE as u32 + 0x00, ctrl);
    let v = (ctrl & !0x7F) | 37u32; // 0x25: ch=37, bit6=0
    trace_w(LLE_BASE as u32 + 0x00, v);
    lle_write(0x00, v);

    // Step 9: ADV access address.
    trace_w(LLE_BASE as u32 + 0x08, ADV_AA);
    lle_write(0x08, ADV_AA);

    // Step 10: CRC seed.
    trace_w(LLE_BASE as u32 + 0x04, ADV_CRC_INIT);
    lle_write(0x04, ADV_CRC_INIT);

    // Step 11: TX buffer DMA pointer → BB+0x70.
    // Hardware applies (written_value >> 2) & 0xfff on writes; readback = byte_addr[15:2].
    // Write the full SRAM pointer; hardware stores (0x153c >> 2) = 0x54f, DMA reads from TX_BUF.
    // A2-test (2026-05-02): write 0x344 → readback 0xD1 (formula confirmed), but bleak=0 still.
    // DMA pointer semantics confirmed — correct addr (tx_ptr→0x54f) is the right approach.
    let tx_ptr = addr_of!(TX_BUF) as u32;
    let tx_slot = tx_ptr;
    trace_w(BB_BASE as u32 + 0x70, tx_slot);
    bb_write(0x70, tx_slot);

    // Pre-trigger snapshot (before any GO writes).
    let state_pre = bb_read(0x1C); // expect 108 = Sleep
    trace_r(BB_BASE as u32 + 0x1C, state_pre);

    // ── Trigger sequence — controlled by ISO_MODE ─────────────────────────────
    match ISO_MODE {
        1 => {
            // iso ORDERING only: move BB+0x00=2 last, but no BB+0x04 or bit23 change
            // Isolates Finding #2 (dominant bug candidate)
            lle_write(0x00, lle_read(0x00) & !0x800); // clear bit11 edge
            lle_write(0x00, lle_read(0x00) | 0x800);  // set bit11 (NOP per Lucy, but keep original)
            let cfg = lle_read(0x2C);
            lle_write(0x2C, cfg & !0x3); // clear TX arm
            bb_write(0x00, 2);            // TX trigger LAST (was mid-sequence before)
        }
        2 => {
            // iso BB+0x04 only: add ADV-mode flag, keep original ordering (mid BB+0x00=2)
            bb_write(0x04, bb_read(0x04) | 0x1); // ADV-mode flag only change
            bb_write(0x00, 2);                    // TX trigger MID (original position)
            lle_write(0x00, lle_read(0x00) & !0x800);
            lle_write(0x00, lle_read(0x00) | 0x800); // bit11 (NOP)
            let cfg = lle_read(0x2C);
            lle_write(0x2C, cfg & !0x3);
        }
        3 => {
            // iso BIT23 only: add bit23 re-set, keep original ordering
            bb_write(0x00, 2);                          // TX trigger MID (original)
            lle_write(0x00, lle_read(0x00) | 0x0080_0000); // bit23 re-set only change
            let cfg = lle_read(0x2C);
            lle_write(0x2C, cfg & !0x3);
        }
        4 => {
            // BASELINE OLD — original step 11.5+12+13 (control / known-bad)
            bb_write(0x00, 2);           // step 11.5: BB arm MID (original wrong position)
            lle_write(0x00, lle_read(0x00) & !0x800);
            lle_write(0x00, lle_read(0x00) | 0x800); // step 12: bit11 GO (NOP)
            let cfg = lle_read(0x2C);
            lle_write(0x2C, cfg & !0x3); // step 13: clear TX arm
        }
        _ => {
            // fix2 FULL (ISO_MODE=0) — fix1 + BLE_SetPHYTxMode (Lucy msg=44b37684, 2d303e7b)
            // Step 12: ADV-mode flag — BB+0x04 bit0 = 1 (lib L39233)
            let v = bb_read(0x04);
            trace_r(BB_BASE as u32 + 0x04, v);
            let v = v | 0x1;
            trace_w(BB_BASE as u32 + 0x04, v);
            bb_write(0x04, v);
            // Step 12.5: PHY mode setup — mirrors ll_advertise_tx offset 0x20A jalr BLE_SetPHYTxMode(1).
            // Previously missing entirely; identified as root cause of "self-diag OK + air silent".
            // a1 = (PDU.length_byte & 0x3F) + 1 (Lucy correction msg=685f2e47; NOT channel index).
            // TX_BUF[1] is PDU header byte 1 = body length (already masked in build_adv_pdu).
            //
            // BB+0x0C pre-arm REMOVED (2026-05-02, Lucy msg=ca653fe3):
            // EVT post-init has BB+0x0C=0, strobe is no-op in both EVT and us. Pre-arming 0x2000
            // (bit13=1) was based on incorrect byte-level hypothesis — removed.
            if PATHC_BIT0_PULSE {
                // EVT pulses LLE+0x2C bit0 before BLE_SetPHYTxMode, then clears it
                // in the post-SetPHYTxMode commit step. Keep gated because this
                // introduces an extra IRQ/stall state in the standalone probe.
                let cfg = lle_read(0x2C);
                trace_r(LLE_BASE as u32 + 0x2C, cfg);
                let cfg = (cfg & !0x3) | 0x1;
                trace_w(LLE_BASE as u32 + 0x2C, cfg);
                lle_write(0x2C, cfg);
            }
            ble_set_phy_tx_mode_1mbps(TX_BUF[1]);
            // Step 13: BLE enable defensive re-set — LLE+0x00 bit23 (lib L39276)
            let v = lle_read(0x00);
            trace_r(LLE_BASE as u32 + 0x00, v);
            // EVT Tier-A trigger-pre has bit12 set as well:
            //   EVT  LLE+0x00 = 0x14001325
            //   Rust LLE+0x00 = 0x14000325
            // Keep bit23 behavior and add bit12 as the only remaining non-cal register delta.
            let v = v | 0x0080_1000;
            trace_w(LLE_BASE as u32 + 0x00, v);
            lle_write(0x00, v);
            let t1_lle00 = lle_read(0x00);
            trace_r(LLE_BASE as u32 + 0x00, t1_lle00);
            // Step 14: Clear TX arm (lib L39281)
            let cfg = lle_read(0x2C);
            trace_r(LLE_BASE as u32 + 0x2C, cfg);
            let cfg = cfg & !0x3;
            trace_w(LLE_BASE as u32 + 0x2C, cfg);
            lle_write(0x2C, cfg);
            let t2_lle00 = lle_read(0x00);
            trace_r(LLE_BASE as u32 + 0x00, t2_lle00);
            trace_bb_read(0x1C);
            trace_bb_read(0x40);
            trace_bb_read(0x44);
            trace_bb_read(0x60);
            trace_bb_read(0x68);
            // ── Tier A: trigger-pre full register dump (first burst only) ─────
            // Dumps all 3 BLE register windows immediately before the TX GO strobe.
            // EVT-side equivalent: Cindy hooks ll_advertise_tx just before BB+0x00=2.
            // Comparing both sides reveals any register EVT has in non-default state
            // that our 15-step sequence doesn't explicitly set.
            if DUMP_TIER_A && !TIER_A_DONE {
                TIER_A_DONE = true;
                hal::println!("# TIER_A trigger-pre full dump (first burst, before BB+0x00=2)");
                hal::ble::tracer::dump_regs("TierA-LLE  ", LLE_BASE as u32,   0x00, 0xFC);
                hal::ble::tracer::dump_regs("TierA-BB   ", BB_BASE as u32,    0x00, 0xFC);
                hal::ble::tracer::dump_regs("TierA-RFEND", RFEND_BASE as u32, 0x00, 0xD0);
            }
            // Step 15: TX trigger — FINAL (lib L39288)
            // PRE snapshot: BB+0x08 + BB+0x1C immediately before GO strobe (t=0 baseline).
            let pre_bb08 = bb_read(0x08);
            let pre_bb1c = bb_read(0x1C);
            trace_r(BB_BASE as u32 + 0x08, pre_bb08);
            trace_r(BB_BASE as u32 + 0x1C, pre_bb1c);

            // Probe: EVT keeps RFEND+0x9C bit8 clear during TX-active. Rust carried bit8 set.
            let rf9c = rfend_read(0x9C);
            trace_r(RFEND_BASE as u32 + 0x9C, rf9c);
            let rf9c_tx = rf9c & !0x100;
            trace_w(RFEND_BASE as u32 + 0x9C, rf9c_tx);
            rfend_write(0x9C, rf9c_tx);

            if !ADV_CTX_PRINTED {
                ADV_CTX_PRINTED = true;
                ADV_CTX_DUMPED = true;
                let adv_base = core::ptr::addr_of!(RUST_ADV_CTX) as usize;
                let tx_base = core::ptr::addr_of!(TX_BUF) as usize;
                hal::println!(
                    "# PATHC_PREGO regs bb70_readback={:#010x} bb70_addr=0x{:08x} lle08_aa={:#010x} lle04_crc={:#010x} lle00={:#010x} bb04={:#010x} rust_adv_ctx=0x{:08x} tx_buf=0x{:08x}",
                    bb_read(0x70),
                    0x2000_0000u32 + ((bb_read(0x70) & 0x3fff) << 2),
                    lle_read(0x08),
                    lle_read(0x04),
                    lle_read(0x00),
                    bb_read(0x04),
                    adv_base,
                    tx_base
                );
                hal::ble::tracer::dump_regs("PATHC-PREGO-BB   ", BB_BASE as u32, 0x00, 0xFC);
                hal::ble::tracer::dump_regs("PATHC-PREGO-LLE  ", LLE_BASE as u32, 0x00, 0xFC);
                hal::ble::tracer::dump_regs("PATHC-PREGO-RFEND", RFEND_BASE as u32, 0x00, 0xD0);
                dump_state_region("RUST_ADV_CTX_PREGO", adv_base, 192);
                dump_state_region("TX_BUF_PREGO", tx_base, 64);
            }

            if PATHC_LIB_IRQ {
                // Keep interrupts disabled during SDI dumps. Enable immediately before GO
                // so the lib handlers only see the TX-trigger IRQ sequence.
                hal::println!("# PATHC_IRQ_MARK pre-clear");
                let ip = core::ptr::addr_of_mut!(gBleIPPara) as *mut u8;
                write_volatile(ip.add(4), 0x80);
                write_volatile(ip.add(5), 0);
                // EVT pre-GO holds BB+0x64 at 0x308. Match the observed GO-time value.
                write_volatile(ip.add(16) as *mut u32, 776);
                hal::println!(
                    "# PATHC_TIMER pre-go ip16={:#010x} bb64={:#010x}",
                    read_volatile(ip.add(16) as *const u32),
                    bb_read(0x64)
                );
                bb_write(0x08, 0x0000_FFFF);
                lle_write(0x38, 0x0000_00F0);
                qingke::pfic::unpend_interrupt(63);
                qingke::pfic::unpend_interrupt(64);
                hal::println!("# PATHC_IRQ_MARK pre-enable");
                qingke::pfic::enable_interrupt(63);
                hal::println!("# PATHC_IRQ_MARK post-enable63");
                if PATHC_ENABLE_LLE_IRQ {
                    qingke::pfic::enable_interrupt(64);
                    hal::println!("# PATHC_IRQ_MARK post-enable64");
                } else {
                    hal::println!("# PATHC_IRQ_MARK skip-enable64");
                }
            }

            if PATHC_LIB_IRQ {
                // BB+0x64 is an active down-counter; SDI logging above can let it
                // expire before GO. Refresh it as the final pre-GO MMIO write.
                let ip = core::ptr::addr_of_mut!(gBleIPPara) as *mut u8;
                write_volatile(ip.add(16) as *mut u32, 776);
                bb_write(0x08, 0x0000_2000);
                bb_write(0x64, 776);
            }
            trace_w(BB_BASE as u32 + 0x00, 2);
            bb_write(0x00, 2); // T=0: GO strobe
            hal::println!("# PATHC_IRQ_MARK post-go");
            for alive in 0..3 {
                qingke::riscv::asm::delay(240);
                hal::println!("# PATHC_ALIVE post-go {}", alive);
            }

            if PATHC_MANUAL_L6 {
                let mut waited = 0u32;
                let mut status = 0u32;
                while waited < 4_000 {
                    status = bb_read(0x08);
                    if (status & 0xFF00_0000) == 0x3300_0000 {
                        break;
                    }
                    waited += 1;
                }
                MANUAL_L6_WAIT = waited;
                MANUAL_L6_BEFORE = status;
                MANUAL_L6_IP4_BEFORE = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
                if (status & 0xFF00_0000) == 0x3300_0000 {
                    MANUAL_L6_HIT = true;
                    // EVT reaches 0x37 between +100us and +200us. A write immediately
                    // after the first 0x33 sample can precede PHY warmup completion.
                    qingke::riscv::asm::delay(2_400); // ~100us at 96MHz / 4 cycles per iter.
                    trace_w(BB_BASE as u32 + 0x08, 0x0000_2000);
                    bb_write(0x08, 0x0000_2000);
                    trace_w(BB_BASE as u32 + 0x64, 556);
                    bb_write(0x64, 556);
                    MANUAL_L6_AFTER = bb_read(0x08);
                }
                MANUAL_L6_IP4_AFTER = read_volatile((core::ptr::addr_of!(gBleIPPara) as *const u8).add(4));
            }

            if !BB08_TRACE_DONE && burst_idx == Z37_TARGET_BURST {
                BB08_TRACE_DONE = true;
                for j in 0..200 {
                    BB08_TRACE[j] = bb_read(0x08);
                    qingke::riscv::asm::delay(120); // ~1.25us at 96MHz.
                }
            }

            if Z37_SNAPSHOT && !Z37_DUMPED && burst_idx == Z37_TARGET_BURST {
                Z37_DUMPED = true;
                let mut timeout = 192_000u32;
                let mut bb08 = 0u32;
                while timeout != 0 {
                    bb08 = bb_read(0x08);
                    if (bb08 & 0xFF00_0000) == 0x3700_0000 {
                        Z37_HIT = true;
                        break;
                    }
                    timeout -= 1;
                }
                Z37_BB08 = bb08;
                Z37_TIMEOUT_LEFT = timeout;
                if Z37_HIT {
                    for j in 0..64 {
                        Y200_BB[j] = read_volatile((BB_BASE + j * 4) as *const u32);
                    }
                    for j in 0..64 {
                        Y200_LLE[j] = read_volatile((LLE_BASE + j * 4) as *const u32);
                    }
                    for j in 0..52 {
                        Y200_RFEND[j] = read_volatile((RFEND_BASE + j * 4) as *const u32);
                    }
                }
            }

            // X1: poll the low-bit W1C status handled by lib IRQ code, without enabling IRQs.
            // This tests whether HW waits for low IRQ acknowledgement before moving 0x33→0x37.
            let mut x1_bb_ack_count = 0u32;
            let mut x1_lle_ack_count = 0u32;
            let mut x1_last_bb_mask = 0u32;
            let mut x1_last_lle_mask = 0u32;
            let mut y3_bb00_changed_iter = 0xFFFF_FFFFu32;
            let mut y3_bb00_changed_val = 0u32;
            let mut y3_bb64_changed_iter = 0xFFFF_FFFFu32;
            let mut y3_bb64_changed_val = 0u32;
            let (post_08_100, post_1c_100, post_08_200, post_1c_200) = if X1_POLLED_W1C {
                let mut p08_100 = 0u32;
                let mut p1c_100 = 0u32;
                let mut p08_200 = 0u32;
                let mut p1c_200 = 0u32;
                for i in 0..100 {
                    if i < 4 {
                        hal::println!("# PATHC_ALIVE x1-enter {}", i);
                    }
                    qingke::riscv::asm::delay(48); // ~2µs at 96MHz.

                    let bb_status = bb_read(0x08);
                    if i < 4 {
                        hal::println!("# PATHC_ALIVE x1-after-bb08 {} {:#010x}", i, bb_status);
                    }
                    if (bb_status & 0xFF00_0000) == 0x3900_0000 && !Y4_039_LOGGED {
                        Y4_039_LOGGED = true;
                        hal::println!("# Y4: Rust bb08 hit 0x39 iter={} full_word={:#010x}", i, bb_status);
                    }
                    let bb_mask = bb_status & 0xFFFF;
                    if bb_mask != 0 {
                        bb_write(0x08, bb_mask);
                        x1_bb_ack_count += 1;
                        x1_last_bb_mask = bb_mask;
                    }
                    if i < 4 {
                        hal::println!("# PATHC_ALIVE x1-after-bb-ack {}", i);
                    }

                    let lle_status = lle_read(0x38);
                    let lle_mask = lle_status & 0xF0;
                    if lle_mask != 0 {
                        lle_write(0x38, lle_mask);
                        x1_lle_ack_count += 1;
                        x1_last_lle_mask = lle_mask;
                    }
                    if i < 4 {
                        hal::println!("# PATHC_ALIVE x1-after-lle-ack {} lle38={:#010x}", i, lle_status);
                    }

                    let bb00_now = bb_read(0x00);
                    if bb00_now != 2 && y3_bb00_changed_iter == 0xFFFF_FFFF {
                        y3_bb00_changed_iter = i as u32;
                        y3_bb00_changed_val = bb00_now;
                    }
                    if bb00_now != 2 && !Y3_BB00_LOGGED {
                        Y3_BB00_LOGGED = true;
                        hal::println!("# Y3: bb00 changed iter={} value={:#010x}", i, bb00_now);
                    }
                    let bb64_now = bb_read(0x64);
                    if bb64_now != 0 && y3_bb64_changed_iter == 0xFFFF_FFFF {
                        y3_bb64_changed_iter = i as u32;
                        y3_bb64_changed_val = bb64_now;
                    }
                    if bb64_now != 0 && !Y3_BB64_LOGGED {
                        Y3_BB64_LOGGED = true;
                        hal::println!("# Y3: bb64 first_nonzero iter={} value={:#010x}", i, bb64_now);
                    }

                    if i == 49 {
                        p08_100 = bb_read(0x08);
                        p1c_100 = bb_read(0x1C);
                    } else if i == 99 {
                        p08_200 = bb_read(0x08);
                        p1c_200 = bb_read(0x1C);
                        if Y200_SNAPSHOT && !Y200_DUMPED {
                            Y200_DUMPED = true;
                            // Snapshot first, print later. SDI printing here shifts the timing window.
                            for j in 0..64 {
                                Y200_BB[j] = read_volatile((BB_BASE + j * 4) as *const u32);
                            }
                            for j in 0..64 {
                                Y200_LLE[j] = read_volatile((LLE_BASE + j * 4) as *const u32);
                            }
                            for j in 0..52 {
                                Y200_RFEND[j] = read_volatile((RFEND_BASE + j * 4) as *const u32);
                            }
                        }
                    }
                }
                (p08_100, p1c_100, p08_200, p1c_200)
            } else {
                qingke::riscv::asm::delay(2_400);
                let p08_100 = bb_read(0x08);
                let p1c_100 = bb_read(0x1C);
                qingke::riscv::asm::delay(2_400);
                let p08_200 = bb_read(0x08);
                let p1c_200 = bb_read(0x1C);
                (p08_100, p1c_100, p08_200, p1c_200)
            };
            qingke::riscv::asm::delay(4_800);
            let post_08_400 = bb_read(0x08);
            let post_1c_400 = bb_read(0x1C);
            qingke::riscv::asm::delay(2_904);
            let post_08_521 = bb_read(0x08);
            let post_1c_521 = bb_read(0x1C);

            // Print on first burst only. Absolute time labels for Lucy's parser.
            if !TRIGGER_POST_PRINTED {
                TRIGGER_POST_PRINTED = true;
                hal::println!("# TRIGGER_POST_SCAN t=0_pre:  bb08={:#010x} bb1c={:#010x}",
                    pre_bb08, pre_bb1c);
                hal::println!("# TRIGGER_POST_SCAN t=+100us: bb08={:#010x} bb1c={:#010x}",
                    post_08_100, post_1c_100);
                hal::println!("# TRIGGER_POST_SCAN t=+200us: bb08={:#010x} bb1c={:#010x}",
                    post_08_200, post_1c_200);
                hal::println!("# TRIGGER_POST_SCAN t=+400us: bb08={:#010x} bb1c={:#010x}",
                    post_08_400, post_1c_400);
                hal::println!("# TRIGGER_POST_SCAN t=+521us: bb08={:#010x} bb1c={:#010x}",
                    post_08_521, post_1c_521);
                hal::println!("# X1_POLLED_W1C enabled={} bb_ack_count={} bb_last={:#010x} lle_ack_count={} lle_last={:#010x}",
                    X1_POLLED_W1C, x1_bb_ack_count, x1_last_bb_mask, x1_lle_ack_count, x1_last_lle_mask);
                hal::println!("# Y3_TRIGGER_ACK bb00_changed_iter={} bb00_val={:#010x} bb64_changed_iter={} bb64_val={:#010x}",
                    y3_bb00_changed_iter, y3_bb00_changed_val, y3_bb64_changed_iter, y3_bb64_changed_val);
                hal::println!("# PATHC_MANUAL_L6 enabled={} hit={} wait={} before={:#010x} after={:#010x}",
                    PATHC_MANUAL_L6, MANUAL_L6_HIT, MANUAL_L6_WAIT, MANUAL_L6_BEFORE, MANUAL_L6_AFTER);
                hal::println!("# PATHC_MANUAL_L6_IP4 before={:#04x} after={:#04x}",
                    MANUAL_L6_IP4_BEFORE, MANUAL_L6_IP4_AFTER);
                // V1.1 probe: lle38=LLE_BASE+0x38=0x40024138 (WCH_BBR+0x38, gate register in bb_irq_lib_handler)
                //             alt38=BB_BASE+0x38=0x40024238 (WCH_LLER+0x38, naming-inversion check)
                // If alt38 has bit6 set when lle38 does not → base names swapped in bb_irq_lib_handler.
                // If both stay 0x001e800f → neither base has bit6; revisit asm decode.
                hal::println!("# PATHC_BB_IRQ_SNAPSHOT count={} s0={:#010x}/ip4={:#04x}/lle38={:#010x}/alt38={:#010x} s1={:#010x}/ip4={:#04x}/lle38={:#010x}/alt38={:#010x} s2={:#010x}/ip4={:#04x}/lle38={:#010x}/alt38={:#010x} s3={:#010x}/ip4={:#04x}/lle38={:#010x}/alt38={:#010x}",
                    BB_IRQ_SNAP_COUNT,
                    BB_IRQ_STATUS[0], BB_IRQ_IP4[0], BB_IRQ_LLE38[0], BB_IRQ_ALT38[0],
                    BB_IRQ_STATUS[1], BB_IRQ_IP4[1], BB_IRQ_LLE38[1], BB_IRQ_ALT38[1],
                    BB_IRQ_STATUS[2], BB_IRQ_IP4[2], BB_IRQ_LLE38[2], BB_IRQ_ALT38[2],
                    BB_IRQ_STATUS[3], BB_IRQ_IP4[3], BB_IRQ_LLE38[3], BB_IRQ_ALT38[3]);
            }
            if Y200_DUMPED && !Y200_PRINTED {
                Y200_PRINTED = true;
                hal::println!("# Y200 snapshot Rust (+200us)");
                for j in 0..64 {
                    hal::println!("Y200-BB    +0x{:02x} = {:#010x}", j * 4, Y200_BB[j]);
                }
                for j in 0..64 {
                    hal::println!("Y200-LLE   +0x{:02x} = {:#010x}", j * 4, Y200_LLE[j]);
                }
                for j in 0..52 {
                    hal::println!("Y200-RFEND +0x{:02x} = {:#010x}", j * 4, Y200_RFEND[j]);
                }
            }
            if Z37_DUMPED && !Z37_PRINTED {
                Z37_PRINTED = true;
                hal::println!("# Z37 Rust poll-stop burst={} hit={} bb08={:#010x} timeout_left={}",
                    Z37_TARGET_BURST + 1, Z37_HIT, Z37_BB08, Z37_TIMEOUT_LEFT);
                if Z37_HIT {
                    for j in 0..64 {
                        hal::println!("Z37-BB    +0x{:02x} = {:#010x}", j * 4, Y200_BB[j]);
                    }
                    for j in 0..64 {
                        hal::println!("Z37-LLE   +0x{:02x} = {:#010x}", j * 4, Y200_LLE[j]);
                    }
                    for j in 0..52 {
                        hal::println!("Z37-RFEND +0x{:02x} = {:#010x}", j * 4, Y200_RFEND[j]);
                    }
                }
            }
            if BB08_TRACE_DONE && !BB08_TRACE_PRINTED {
                BB08_TRACE_PRINTED = true;
                hal::println!("# BB08_TRACE Rust burst={} samples=200 step_us≈1.25", Z37_TARGET_BURST + 1);
                let mut last = 0u32;
                for j in 0..200 {
                    let v = BB08_TRACE[j];
                    let phy = v & 0xFF00_0000;
                    if j == 0 || phy != last {
                        hal::println!("# BB08_TRANSITION idx={} approx_us_x100={} {:#010x}", j, j * 125, v);
                        last = phy;
                    }
                    hal::println!("BB08_TRACE[{j:03}] = {v:#010x}");
                }
            }
            if GBLELL_SNAPSHOT && !GBLELL_DUMPED {
                GBLELL_DUMPED = true;
                let p = core::ptr::addr_of!(gBleLlPara) as *const u32;
                for j in 0..74 {
                    GBLELL_SNAPSHOT_WORDS[j] = read_volatile(p.add(j));
                }
            }
            if GBLELL_DUMPED && !GBLELL_PRINTED {
                GBLELL_PRINTED = true;
                let base = core::ptr::addr_of!(gBleLlPara) as usize;
                hal::println!("# GBLELL snapshot Rust base=0x{:08x} len=296", base);
                for j in 0..74 {
                    hal::println!("GBLELL +0x{:02x} = {:#010x}", j * 4, GBLELL_SNAPSHOT_WORDS[j]);
                }
            }
            if !ADV_CTX_DUMPED {
                ADV_CTX_DUMPED = true;
                let adv = core::ptr::addr_of!(RUST_ADV_CTX) as *const u8;
                for j in 0..192 {
                    ADV_CTX_DUMP[j] = read_volatile(adv.add(j));
                }

                let tx_readback = bb_read(0x70);
                let tx_addr = 0x2000_0000u32 + ((tx_readback & 0x3fff) << 2);
                TX_PTR_SNAPSHOT = tx_addr;
                if (0x2000_0000..0x2001_0000).contains(&tx_addr) {
                    let tx = tx_addr as *const u8;
                    for j in 0..64 {
                        TX_PTR_DUMP[j] = read_volatile(tx.add(j));
                    }
                }
            }
            if ADV_CTX_DUMPED && !ADV_CTX_PRINTED {
                ADV_CTX_PRINTED = true;
                let adv_base = core::ptr::addr_of!(RUST_ADV_CTX) as usize;
                hal::println!("# PATHC_ADVCTX snapshot Rust base=0x{:08x} len=192", adv_base);
                hal::print!("STATE RUST_ADV_CTX 0x{:08x} 192 ", adv_base);
                for j in 0..192 {
                    hal::print!("{:02x}", ADV_CTX_DUMP[j]);
                }
                hal::println!();
                hal::println!("# PATHC_TXPTR bb70_addr=0x{:08x} len=64", TX_PTR_SNAPSHOT);
                hal::print!("STATE TXPTR_BUF 0x{:08x} 64 ", TX_PTR_SNAPSHOT);
                for j in 0..64 {
                    hal::print!("{:02x}", TX_PTR_DUMP[j]);
                }
                hal::println!();
            }
            // Record POST values in tracer for trace dump
            trace_r(BB_BASE as u32 + 0x08, post_08_521);
            trace_r(BB_BASE as u32 + 0x1C, post_1c_521);
        }
    }

    // Post-trigger snapshot.
    let state_post = bb_read(0x1C); // non-108 → HW state machine left Sleep
    let irq_post   = bb_read(0x08); // bits[15:0] → TX completion IRQ flags
    trace_r(BB_BASE as u32 + 0x1C, state_post);
    trace_r(BB_BASE as u32 + 0x08, irq_post);

    (state_pre, state_post, irq_post)
}

/// Wait for TX completion using the shared BLE TX done primitive.
///
/// settle_loops = 50_000 ≈ 350µs at 96 MHz — covers ADV_NONCONN_IND on-air time:
///   pre-delay 90µs + preamble+AA+PDU(6+3+2+9+3=23B at 1Mbps) 184µs ≈ 274µs → fits in 350µs.
unsafe fn wait_tx_done() -> bool {
    hal::ble::ble_tx_wait_done(10_000, 50_000)
}

// ── BLE stack RAM state dumper ────────────────────────────────────────────────

/// Dump a region of BLE stack RAM in STATE format.
///
/// Output line: `STATE <label> 0x<addr_hex> <len> <hexbytes...>`
///
/// Format is designed for line-by-line diff against the EVT Broadcaster
/// SRAM dump (Cindy's SDI patch / wlink halt-dump) at matching symbol
/// addresses.  One line per region, no separators, hexbytes run together
/// (same as typical hex dump tools — easier to paste into Python diffing).
///
/// # Safety
/// `addr` must point to valid, readable SRAM. No bounds or alignment check
/// is performed; the caller is responsible for symbol address correctness.
unsafe fn dump_state_region(label: &str, addr: usize, len: usize) {
    hal::print!("STATE {} 0x{:08x} {} ", label, addr, len);
    let ptr = addr as *const u8;
    for i in 0..len {
        hal::print!("{:02x}", core::ptr::read_volatile(ptr.add(i)));
    }
    hal::println!();
}

unsafe fn dump_current_ble_symbols(tag: &str) {
    hal::println!("# BLE_SYMBOL_DUMP {} begin", tag);
    dump_state_region("sym_ble", core::ptr::addr_of!(ble) as usize, 64);
    dump_state_region("sym_gBleLlPara", core::ptr::addr_of!(gBleLlPara) as usize, 0x80);
    dump_state_region("sym_gBleIPPara", core::ptr::addr_of!(gBleIPPara) as usize, 0x40);
    dump_state_region("sym_gptrLLEReg", core::ptr::addr_of!(gptrLLEReg) as usize, 4);
    dump_state_region("sym_gptrBBReg", core::ptr::addr_of!(gptrBBReg) as usize, 4);
    dump_state_region("sym_gptrRFENDReg", core::ptr::addr_of!(gptrRFENDReg) as usize, 4);
    dump_state_region("sym_gptrAESReg", core::ptr::addr_of!(gptrAESReg) as usize, 4);
    dump_state_region("sym_gPaControl", core::ptr::addr_of!(gPaControl) as usize, 4);
    dump_state_region("sym_dtmFlag", core::ptr::addr_of!(dtmFlag) as usize, 1);
    hal::println!("# BLE_SYMBOL_DUMP {} end", tag);
}

unsafe fn dump_ip_core_mmio(tag: &str) {
    hal::println!("# IP_CORE_MMIO_DUMP {} begin", tag);
    match tag {
        "after_ip_core_init" => {
            hal::ble::tracer::dump_regs("IP-after_ip-BB   ", BB_BASE as u32, 0x00, 0x80);
            hal::ble::tracer::dump_regs("IP-after_ip-LLE  ", LLE_BASE as u32, 0x00, 0x80);
            hal::ble::tracer::dump_regs("IP-after_ip-RFEND", RFEND_BASE as u32, 0x00, 0xD0);
            hal::ble::tracer::dump_regs("IP-after_ip-AES  ", 0x4002_4300, 0x00, 0x30);
        }
        "after_ll_init_safe_prefix" => {
            hal::ble::tracer::dump_regs("IP-after_ll-BB   ", BB_BASE as u32, 0x00, 0x80);
            hal::ble::tracer::dump_regs("IP-after_ll-LLE  ", LLE_BASE as u32, 0x00, 0x80);
            hal::ble::tracer::dump_regs("IP-after_ll-RFEND", RFEND_BASE as u32, 0x00, 0xD0);
            hal::ble::tracer::dump_regs("IP-after_ll-AES  ", 0x4002_4300, 0x00, 0x30);
        }
        _ => {}
    }
    hal::println!("# IP_CORE_MMIO_DUMP {} end", tag);
}

/// Dump all known BLE stack RAM globals for EVT-vs-Rust comparison.
///
/// **Addresses are STUBS** from Lucy's stub-link map (msg 7e201119, 2026-05-02).
/// They will shift in the real EVT Broadcaster ELF.  After Cindy runs
/// `nm Broadcaster.elf | grep "^20"`, replace every address constant below
/// with the matching real value (search for the matching symbol name).
///
/// Symbols (stub → update after nm):
///
/// | symbol      | stub addr  | size |
/// |-------------|------------|------|
/// | ble         | 0x200003dc |  64B |
/// | gBleLlPara  | 0x20000438 | 296B |
/// | gBleIPPara  | 0x20000590 |  40B |
/// | rfConfig    | 0x200005bc |  32B |
/// | rfInfo      | 0x200005dc |  36B |
/// | pAdvCtx     | heap ptr   | 192B |
///
/// pAdvCtx is heap-allocated.  Its pointer is stored inside gBleLlPara at
/// some byte offset; Lucy's asm trace puts it around +0x3c/+0x3e.  This
/// function reads the u32 at `gBleLlPara + PADV_PTR_OFF`, validates it lies
/// in SRAM (0x20000000..0x20008000), then dumps 192 bytes from that address.
/// If the pointer is zero or out-of-range the BLE stack has not yet run
/// `llAdvertiseCreateCore` / heap allocation (expected in Rust path — we
/// never call BLE_LibInit).
///
/// Hook point (both sides): after `ble_phy_init()` / `BLE_LibInit()` returns,
/// before the first TX burst / first RF_Tx call.
unsafe fn dump_ble_ram_state() {
    // ── Real EVT addresses from `nm Broadcaster.elf` (Cindy, 2026-05-02) ─────
    // These are EVT-side BSS addresses.  In the Rust binary the same SRAM
    // range may hold stack / other data — expect mostly zeros in Rust output.
    // That contrast (EVT non-zero vs Rust zero) identifies fields to bake in.
    const BLE_ADDR:         usize = 0x20002150; // ble global (64B)    — ble[0x14] = LL state
    const BLECLOCK_ADDR:    usize = 0x20002198; // bleClock_t (16B)
    const GBLLHOSTPARA_ADDR:usize = 0x20002310; // gBleHostPara (10B)
    const GBLELGPARA_ADDR:  usize = 0x20002320; // gBleLlPara (296B)   — pAdvCtx_ptr at +0x58
    const GPACONTROL_ADDR:  usize = 0x2000246c; // gPaControl (4B)
    const GPTRBB_ADDR:      usize = 0x20002468; // gptrBBReg (4B)
    const GBLEIPPARA_ADDR:  usize = 0x20002474; // gBleIPPara (40B)
    const GPTRLLE_ADDR:     usize = 0x2000249c; // gptrLLEReg (4B)
    const RFCONFIG_ADDR:    usize = 0x200024a0; // rfConfig (32B)
    const RFINFO_ADDR:      usize = 0x200024c0; // rfInfo (36B)
    const GPTRFEND_ADDR:    usize = 0x200024f0; // gptrRFENDReg (4B)
    // single-byte calibration globals
    const NGA2440_ADDR:     usize = 0x200024e4;
    const NGA2480_ADDR:     usize = 0x200024ec;
    const NCO2440_ADDR:     usize = 0x200024f4;
    const NCO2480_ADDR:     usize = 0x200024f5;
    const NCO2401_ADDR:     usize = 0x200001b2;
    const NGA2401_ADDR:     usize = 0x200001b3;
    // IRQ callback pointers (Cindy msg 52edd8f3, 2026-05-02).
    // EVT: BLE_LibInit registers a lib handler here.  Rust: expect 0 (BLE_LibInit never called).
    // If EVT non-zero and Rust 0, and TX requires lib ISR acknowledgment, this is root cause.
    const GLLE_IRQ_HANDLER_ADDR: usize = 0x20000124; // g_LLE_IRQLibHandlerLocation (4B ptr)
    const PFN_IRQ_PROCESS_ADDR:  usize = 0x200021a8; // pfnIrqProcessHandle (4B ptr)
    // Full heap arena — optional, very verbose (7168B × 2 hex = ~14K chars/line).
    // Set DUMP_MEM_BUF=true only when you need Lucy to grep pAdvCtx data offline.
    const DUMP_MEM_BUF: bool = false;
    const MEM_BUF_ADDR:     usize = 0x20000230;
    const MEM_BUF_LEN:      usize = 0x1c00; // 7168B

    // Byte offset within gBleLlPara where pAdvCtx heap pointer is stored.
    // Lucy asm trace (msg d1f87c42, 2026-05-02): llAdvertiseCreateCore writes
    // the malloc'd 192B block to gBleLlPara+0x58/+0x5c/+0x60 (linked-list head).
    const PADV_PTR_OFF: usize = 0x58;

    hal::println!("# BLE_RAM_DUMP begin (EVT real addrs from nm Broadcaster.elf, 2026-05-02)");

    dump_state_region("ble",         BLE_ADDR,          64);
    dump_state_region("bleClock_t",  BLECLOCK_ADDR,     16);
    dump_state_region("gBleHostPara",GBLLHOSTPARA_ADDR, 10);
    dump_state_region("gBleLlPara",  GBLELGPARA_ADDR,  296);
    dump_state_region("gPaControl",  GPACONTROL_ADDR,    4);
    dump_state_region("gptrBBReg",   GPTRBB_ADDR,        4);
    dump_state_region("gBleIPPara",  GBLEIPPARA_ADDR,   40);
    dump_state_region("gptrLLEReg",  GPTRLLE_ADDR,       4);
    dump_state_region("rfConfig",    RFCONFIG_ADDR,     32);
    dump_state_region("rfInfo",      RFINFO_ADDR,       36);
    dump_state_region("gptrRFENDReg",GPTRFEND_ADDR,      4);
    dump_state_region("nGA2440",     NGA2440_ADDR,       1);
    dump_state_region("nGA2480",     NGA2480_ADDR,       1);
    dump_state_region("nCO2440",     NCO2440_ADDR,       1);
    dump_state_region("nCO2480",     NCO2480_ADDR,       1);
    dump_state_region("nCO2401",     NCO2401_ADDR,       1);
    dump_state_region("nGA2401",     NGA2401_ADDR,       1);
    dump_state_region("g_LLE_IRQLibHandlerLocation", GLLE_IRQ_HANDLER_ADDR, 4);
    dump_state_region("pfnIrqProcessHandle",         PFN_IRQ_PROCESS_ADDR,  4);

    // pAdvCtx pointer-chase: gBleLlPara+0x58 holds heap ptr (Lucy asm d1f87c42).
    // In Rust: BLE stack never ran → ptr = 0 → expected "out of SRAM" log.
    let padv_ptr_addr = GBLELGPARA_ADDR + PADV_PTR_OFF;
    let padv_ptr = core::ptr::read_volatile(padv_ptr_addr as *const u32) as usize;
    hal::println!("# pAdvCtx_ptr @ 0x{:08x} = 0x{:08x}", padv_ptr_addr, padv_ptr);
    if padv_ptr >= 0x2000_0000 && padv_ptr < 0x2000_8000 {
        dump_state_region("pAdvCtx", padv_ptr, 192);
    } else {
        hal::println!("# pAdvCtx ptr out of SRAM — BLE stack not initialized (expected in Rust path)");
    }

    // Full heap arena — optional (verbose). Set DUMP_MEM_BUF=true when Lucy needs to
    // grep pAdvCtx data or TX buf offline. In Rust path, MEM_BUF range is unused BSS.
    if DUMP_MEM_BUF {
        dump_state_region("MEM_BUF", MEM_BUF_ADDR, MEM_BUF_LEN);
    }

    hal::println!("# BLE_RAM_DUMP end");
}


// ── H_PADVCTX experiment ──────────────────────────────────────────────────────
//
// Hypothesis (Lucy msg f2e2549f, 2026-05-02):
//   EVT's llAdvertiseCreateCore malloc's 192B at 0x20000b4c and fills structured
//   advertising context.  Rust never calls BLE_LibInit → pAdvCtx = all zeros.
//   If hardware DMA reads pAdvCtx fields directly (bypassing our bb_*(0x70) TX buf
//   pointer path), TX would silently fail.  This experiment tests that hypothesis.
//
//   true  = inject EVT pAdvCtx bytes before first TX  (hypothesis under test)
//   false = baseline, pAdvCtx stays zero              (A4 known-bad)
//
// Prior bleak test (Cindy) ran with 1-byte-misaligned byte array → evidence degraded.
// Re-running with corrected bytes (see EVT_PADVCTX below, misalignment at [40] fixed).
// Set true for clean single-variable re-test; false = A4 baseline.
// H_PADVCTX ELIMINATED (2026-05-02 session 6): Cindy confirmed rb[8]=0x01 + PADVCTX_MATCH=True
// + bleak 30s 0 hits. Hardware does NOT DMA-read pAdvCtx. Keeping false as A4 baseline.
const INJECT_PADVCTX: bool = false;

/// Heap address where EVT's llAdvertiseCreateCore stores the 192B ADV context.
const EVT_PADVCTX_ADDR: usize = 0x2000_0b4c;

/// EVT pAdvCtx bytes — Lucy's corrected parse from state_dump.txt (msg f2e2549f, 2026-05-02).
/// Key fields: [22]=0xff (SeqN), [80..83]=0x00,0x00,0x00,0x08, [104]=0x09 (ble[0x14]).
/// NOTE: prior version at this location had a 1-byte alignment error starting at [40].
///       This version matches the raw hex dump exactly.
const EVT_PADVCTX: [u8; 192] = [
    // [0..15]
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x26,0x91,0x01,0x00,0x02,0x07,
    // [16..31]
    0x02,0x13,0x00,0x00,0x00,0x00,0xff,0x00,0x03,0x27,0x26,0x00,0x0d,0x00,0x10,0x00,
    // [32..47]
    0xa0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x05,0x00,0x20,0xe8,0x04,0x00,0x20,
    // [48..63]
    0x00,0x00,0x00,0x00,0x01,0x00,0x3e,0x4c,0x88,0x26,0x3b,0x38,0x00,0x00,0x00,0x00,
    // [64..79]
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x03,0x00,0x20,
    // [80..95]
    0x30,0x02,0x00,0x20,0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // [96..111]
    0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x00,0x00,0x00,0x43,0x02,0x00,0x00,
    // [112..127]
    0x00,0x00,0x00,0x00,0x00,0x00,0x16,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0x00,0x00,
    // [128..143]
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // [144..159]
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // [160..175]
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    // [176..191]
    0x00,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
];

unsafe fn inject_evt_padvctx() {
    if !INJECT_PADVCTX {
        hal::println!("H_PADVCTX: skip (INJECT_PADVCTX=false)");
        return;
    }
    hal::println!("H_PADVCTX: INJ_START dst=0x{:08x} len={}", EVT_PADVCTX_ADDR, EVT_PADVCTX.len());
    // Use write_volatile byte-by-byte to guarantee LTO cannot elide the stores.
    let dst = EVT_PADVCTX_ADDR as *mut u8;
    for i in 0..EVT_PADVCTX.len() {
        core::ptr::write_volatile(dst.add(i), EVT_PADVCTX[i]);
    }
    // Read-back spot check: EVT_PADVCTX[8]=0x01 — if this reads 0 the store didn't land.
    let rb = core::ptr::read_volatile(dst.add(8));
    hal::println!("H_PADVCTX: INJ_DONE rb[8]=0x{:02x} (expect 0x01 — FAIL if 0)", rb);
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    // 96 MHz from HSE 32 MHz crystal — required for RFEND calibration (same as DTM TX).
    // At 8 MHz HSI the calibration PLL times out (CO=0, all tables zero → no RF output).
    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config {
            hse: Some(hal::rcc::Hse {
                freq: hal::time::Hertz(32_000_000),
                mode: hal::rcc::HseMode::Oscillator,
            }),
            sys: hal::rcc::Sysclk::PLL,
            pll_src: hal::rcc::PllSource::HSE,
            pll: Some(hal::rcc::Pll {
                prediv: hal::rcc::PllPreDiv::DIV4,
                mul:    hal::rcc::PllMul::MUL12,
            }),
            pllx: None,
            ahb_pre:  hal::rcc::AHBPrescaler::DIV1,
            apb1_pre: hal::rcc::APBPrescaler::DIV1,
            apb2_pre: hal::rcc::APBPrescaler::DIV1,
            ls:       hal::rcc::LsConfig::default_lsi(),
            hspll_src: hal::rcc::HsPllSource::HSE,
            hspll: Some(hal::rcc::HsPll {
                pre: hal::rcc::HsPllPrescaler::DIV2,
            }),
        },
        ..Default::default()
    });

    let iso_name = match ISO_MODE {
        0 => "fix2_full (BB+0x04|=1 + bit23 + BB=2 last + SetPHYTxMode)",
        1 => "iso_ordering_only (BB=2 last, no BB04/bit23)",
        2 => "iso_bb04_only (ADV-mode flag, original ordering)",
        3 => "iso_bit23_only (bit23 re-set, original ordering)",
        4 => "baseline_old (known-bad: BB=2 mid + bit11)",
        _ => "unknown",
    };
    hal::println!("BLE TX ADV ch37 — Task #16 | ISO_MODE={} {}", ISO_MODE, iso_name);
    hal::println!("AdvA: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} (random static)",
        ADDR[5], ADDR[4], ADDR[3], ADDR[2], ADDR[1], ADDR[0]);
    hal::println!("PDU: ADV_NONCONN_IND, ch37=2402MHz, EVT-format Flags=0x04, Mfr=\"ble\", Name=\"cba\"");

    unsafe {
        // ── Phase B: staged BLE PHY init with register snapshots between stages ──
        //
        // Replaces the single ble_phy_init() call.  Each STAGE* snapshot captures
        // BB/LLE/RFEND state so the output can be diff'd against EVT Broadcaster
        // register dumps at matching points (e.g. "EVT after BLE_IPCoreInit" vs
        // our "S3 after bb_dev_init", "EVT after WCHBLE_Init" vs our "S4").
        //
        // Lucy d.asm findings (2026-05-02):
        //   BB+0x2C, BB+0x34, RFEND+0x28, RFEND+0x38 differences are NOT in
        //   BB_DevInit/RFEND_DevInit — they are downstream writers (LL_Init /
        //   GAPRole_BroadcasterInit / TMOS_Init).  Phase B confirms which stage
        //   diverges and which registers are actually touched by dev_inits.

        hal::ble::ble_hw_preamble(); // HSE 32 MHz + RCC BLE/CRC clocks

        // S0: baseline after clocks, before any hardware init.
        hal::ble::tracer::dump_regs("S0 BB   ", BB_BASE as u32,    0x00, 0x80);
        hal::ble::tracer::dump_regs("S0 LLE  ", LLE_BASE as u32,   0x00, 0x80);
        hal::ble::tracer::dump_regs("S0 RFEND", RFEND_BASE as u32, 0x00, 0xD0);

        if PATHC_REPLACE_TIER0 {
            // Path C Plan C: RF_RoleInit hangs in the TMOS registration path here,
            // so call the TMOS-free IP-core init plus LL_Init directly.
            if PATHC_USE_RUST_IP_CORE_INIT {
                hal::println!("PathC PlanC: before ble_ip_core_init()");
                hal::ble::ble_ip_core_init();
                hal::println!("PathC PlanC: after ble_ip_core_init()");
            } else {
                // Task #23 bisect: FFI path with Rust replacement retained in flash.
                hal::println!("PathC bisect: before BLE_IPCoreInit() FFI");
                BLE_IPCoreInit();
                hal::println!("PathC bisect: after BLE_IPCoreInit() FFI");
            }
            dump_current_ble_symbols("after_ip_core_init");
            dump_ip_core_mmio("after_ip_core_init");
            hal::println!("PathC PlanD: before ll_init_safe_prefix()");
            ll_init_safe_prefix();
            hal::println!("PathC PlanD: after ll_init_safe_prefix()");
            dump_current_ble_symbols("after_ll_init_safe_prefix");
            dump_ip_core_mmio("after_ll_init_safe_prefix");
            seed_ble_bd_addr();
            hal::println!("PathC PlanD: seeded ble[0x18..0x1d] BD addr");
            hal::println!("PathC PlanB2: LL helper calls skipped");
            hal::println!("PathC PlanD: BLE IP-core init + LL_Init safe prefix replaced Tier-0 init");
        } else {
        hal::ble::lle_dev_init();

        // B experiment: override BB+0x74 with LLE_DMA_BUF_B address (0x20008234).
        // lle_dev_init() just wrote LLE_DMA_BUF (~0x200000D0) → readback 0x34.
        // This write replaces it: (0x20008234 >> 2) & 0xFF = 0x8D → matches EVT.
        if B_EXPERIMENT {
            let b_addr = core::ptr::addr_of!(LLE_DMA_BUF_B) as u32;
            bb_write(0x74, b_addr);
            let rb74 = bb_read(0x74);
            hal::println!("B_EXPERIMENT: LLE_DMA_BUF_B addr={:#010x} BB+0x74_readback={:#010x} (expect 0x8d)",
                b_addr, rb74);
        }

        // S1: after LLE init (WCH lle_dev_init / LLE_DevInit).
        hal::ble::tracer::dump_regs("S1 BB   ", BB_BASE as u32,    0x00, 0x80);
        hal::ble::tracer::dump_regs("S1 LLE  ", LLE_BASE as u32,   0x00, 0x80);
        hal::ble::tracer::dump_regs("S1 RFEND", RFEND_BASE as u32, 0x00, 0xD0);

        hal::ble::rfend_dev_init();
        // S2: after RFEND init (WCH rfend_dev_init / RFEND_DevInit).
        hal::ble::tracer::dump_regs("S2 BB   ", BB_BASE as u32,    0x00, 0x80);
        hal::ble::tracer::dump_regs("S2 LLE  ", LLE_BASE as u32,   0x00, 0x80);
        hal::ble::tracer::dump_regs("S2 RFEND", RFEND_BASE as u32, 0x00, 0xD0);

        hal::ble::bb_dev_init(hal::ble::BB_RF_FLAG_1M);
        // S3: after BB init — Lucy confirms BB_DevInit sets BB+0x2C=0x80010EC8|(rf_flag<<25)
        // and BB+0x34=0x1D0(464).  EVT differences at these offsets come from downstream
        // writers (not visible in S3).  Compare S3 with EVT's post-BLE_IPCoreInit dump.
        hal::ble::tracer::dump_regs("S3 BB   ", BB_BASE as u32,    0x00, 0x80);
        hal::ble::tracer::dump_regs("S3 LLE  ", LLE_BASE as u32,   0x00, 0x80);
        hal::ble::tracer::dump_regs("S3 RFEND", RFEND_BASE as u32, 0x00, 0xD0);

        hal::ble::ble_reg_init(false); // RF calibration — always last (Phase B RX path, keep clears)

        // Path C probe: let WCH's IP-core init populate hidden BLE hardware/RAM latches.
        // ip.o disassembly shows this calls only LLE_DevInit/RFEND_DevInit/BB_DevInit/BLE_RegInit.
        BLE_IPCoreInit();
        hal::println!("PathC: BLE_IPCoreInit() called");
        }

        // H19 probe: EVT BLE_LibInit/GAPRole runtime has RFEND+0x04 bits 12 and 8 set.
        // Test whether these analog TX-path enable bits are the missing broadcast gate.
        rfend_write(0x04, rfend_read(0x04) | 0x0000_1100);

        // ── post_stack_patch: apply EVT Broadcaster post-WCHBLE_Init values ─────
        //
        // Phase B confirmed (2026-05-02, Cindy):
        //   S0 (stale EVT state) already had EVT target values in LLE+0x2C/0x34.
        //   bb_dev_init then OVERWROTE them with lib-correct-but-wrong-for-ADV values.
        //
        // The real writer is an unidentified BLE stack function (candidate: BLE_LibInit
        // or GAPRole_BroadcasterInit) that reconfigures LLE+0x2C/0x34 for ADV mode.
        // We don't call that function; apply its output directly.
        //
        // EVT target values (from regdiff_fix2_vs_evt_postinit_clean_20260502_1322.csv):
        //
        //   LLE+0x2C (BB_CFG): bb_dev_init writes 0x92010EC8 (rf_flag=9).
        //     EVT ends at 0x80009178 — different lower bits, likely ADV-mode CFG.
        //     These lower bits survive into the TX arm step (mask 0x81FF_FFFF keeps them).
        // H18 Test A4: disable post_stack_patch; keep dev_init + ble_reg_init baseline.
        // lle_write(0x2C, 0x80009178);
        //
        //   LLE+0x34 (TIMING): bb_dev_init writes 0x1D0 (464).
        //     EVT ends at 0x3 — dramatically different, likely ADV scheduling slot count.
        // lle_write(0x34, 0x3);
        //
        //   RFEND+0x28 (CFG0): RFEND_DevInit writes 0x480; ble_reg_init adds bit12 → 0x1480.
        //     EVT ends at 0x047f — completely different, likely analog CFG overwrite by stack.
        // H18 Test A2: keep local ble_reg_init RFEND+0x28 calibration bits.
        // rfend_write(0x28, 0x047f);
        //
        //   RFEND+0x38 (calibration anchors + PLL bits): our RMW stores nCO/nGA with bit31=1.
        //     EVT ends at 0x4000d320 (bit31=0, bit30=1) — downstream writer swaps bit31→bit30.
        //     NOTE: this also uses EVT's calibration values (nCO2440=32, GA encoding differs).
        //     If this patch causes calibration regression, revert and test LLE patches alone.
        // H18 Test A: keep local ble_reg_init calibration for RFEND+0x38.
        // Directly copying EVT's board-specific value can distort modulation gain.
        // rfend_write(0x38, 0x4000d320);

        // ── Tier 2 patches (added 2026-05-02 after Tier 1 air test: NOT visible) ──
        //
        // Lucy's Tier 2 list (BB_BASE=0x40024100=our LLE_BASE, LLE_BASE=0x40024200=our BB_BASE):
        //
        //   LLE+0x00 bit28: bb_dev_init leaves bit28=1; EVT has bit28=0.
        // lle_write(0x00, lle_read(0x00) & !0x1000_0000);
        //
        //   LLE+0x80 bit28: mirror register, same clear.
        // lle_write(0x80, lle_read(0x80) & !0x1000_0000);
        //
        //   BB+0x08 bit13: EVT has bit13=0 post-init; W1C clear it.
        //   BB+0x08 is the LLE IRQ register; bit13 = timer IRQ (W1C).
        //   Must write 1 to bit13 (not 0) to clear — writing 0 has no effect on W1C.
        //   Our S4 value is 0x00002000 (only bit13 set), so writing 0x2000 clears it.
        // bb_write(0x08, 0x0000_2000); // W1C: write 1 → clears bit13
        //
        //   BB+0x0C = 0: matches EVT Broadcaster post-init dump.
        // bb_write(0x0C, 0);
        //
        //   BB+0x50 = 0x11: pre-delay / scan timer. TX burst overwrites this to 90 anyway.
        // bb_write(0x50, 0x0000_0011);
        //
        //   BB+0x74 = word-indexed DMA pointer register (RX DMA buf ptr, not TX-critical).
        //   EVT readback: 0x8C (word index). Hardware stores writes as byte_addr/4.
        //   To get readback 0x8C, must write 0x8C*4=0x230 (Cindy analysis: 0x23*4=0x8C).
        //   Not on TX critical path — write best-effort; actual RX buf setup uses its own ptr.
        // bb_write(0x74, 0x0000_0230); // word-indexed: 0x230 → readback 0x8C to match EVT

        // ── Tier 3 patches (2026-05-02, after Cindy Tier 2 S4 diff confirmation) ─────
        //
        // RFEND structural analog registers still differing from EVT post-WCHBLE_Init
        // per phaseC_tier2_vs_evt_s4_diff_20260502_1403.csv:
        //   0x4002502c / 0x40025030 / 0x4002503c /
        //   0x40025048 / 0x4002504c / 0x40025050 / 0x40025054 / 0x40025058
        //
        // These are RFEND PLL/VCO/filter/bias analog config registers (PLL/VCO block,
        // loop filter, CFG5, RF0..RF3). rfend_dev_init() configures them from WCH asm;
        // EVT's WCHBLE_Init downstream writers apply further overrides. Tier 3 applies
        // EVT's final observed values directly.
        //
        // RFEND+0x2C note: bit1 = PLL lock, toggled per-burst by set_channel_freq()
        // (asserts bit1) + step 7 (clears bit1). Writing structural config here (bit1=0)
        // is safe — set_channel_freq does read-modify-write (OR bit1), preserving these bits.
        // H18 Test A3: disable Tier 3 RFEND overrides. These values mix
        // architectural config with EVT-board calibration/gain results.
        // rfend_write(0x2C, 0x0222_2000); // PLL/VCO structural config
        // rfend_write(0x30, 0x4334_4407); // loop filter config
        // rfend_write(0x3C, 0x6B00_62C0); // CFG5 (rfend_dev_init A5-fix partial; EVT final)
        // rfend_write(0x48, 0x4401_0209); // RF0 analog (full EVT value; overrides rfend_dev_init RMWs)
        // rfend_write(0x4C, 0x0222_1700); // RF1 bias/filter (full EVT value)
        // rfend_write(0x50, 0x0041_A010); // RF2 (rfend_dev_init set bit14; EVT full value)
        // rfend_write(0x54, 0x0000_110A); // RF2b (rfend_dev_init bits[3:0]+bit7+bit12; EVT full value)
        // rfend_write(0x58, 0x0021_0300); // RF3 (not touched by rfend_dev_init; EVT stack writer)

        // S4: full init + post_stack_patch.  Should now match EVT post-WCHBLE_Init.
        hal::ble::tracer::dump_regs("S4 BB   ", BB_BASE as u32,    0x00, 0x80);
        hal::ble::tracer::dump_regs("S4 LLE  ", LLE_BASE as u32,   0x00, 0x80);
        hal::ble::tracer::dump_regs("S4 RFEND", RFEND_BASE as u32, 0x00, 0xD0);

        if !PATHC_LIB_IRQ {
            qingke::pfic::enable_interrupt(63);
            qingke::pfic::enable_interrupt(64);
        }

        // Calibration check — read CO from RFEND+0x90 (filled by ble_reg_init).
        let rfend90 = read_volatile(0x4002_5090 as *const u32);
        let co = rfend90 & 0x3F;
        hal::println!("PHY init done: co={co} (expect 23-54; 0 = calibration failed)");
        if co == 0 {
            hal::println!("WARNING: co=0 — RF calibration failed. No output expected.");
        }

        // ── BLE stack RAM state dump ──────────────────────────────────────────
        // Hook point: after ble_phy_init() equivalent, before first TX burst.
        // EVT equivalent: after Broadcaster_Init() / BLE_LibInit() returns.
        //
        // In Rust we never call BLE_LibInit, so all EVT BSS addresses below
        // should read as zero (BSS-cleared) or whatever stack data is at those
        // addresses in our different binary layout.  The EVT side shows non-zero
        // structured state — diff identifies fields to bake into Rust inject.
        //
        // Format: STATE <name> 0x<addr> <len> <hex>  (one line per region)
        // Same format as EVT state_dump.txt — paste both into Lucy's diff tool.
        dump_ble_ram_state();
        inject_evt_padvctx();
        dump_state_region("pAdvCtx_after_inject", EVT_PADVCTX_ADDR, 192);

        // Step 2: Build TX PDU (once; same for every burst).
        let pdu_len = build_adv_pdu();
        hal::println!("PDU built: {}B total (header=2 + payload={})", pdu_len, pdu_len - 2);
        hal::println!("  hdr=[{:#04x},{:#04x}] AdvA=[{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}]",
            TX_BUF[0], TX_BUF[1],
            TX_BUF[7], TX_BUF[6], TX_BUF[5], TX_BUF[4], TX_BUF[3], TX_BUF[2]);

        let mut tx_n: u32 = 0;
        let mut ok_n: u32 = 0;

        loop {
            if (TRACE_FIRST_BURST && tx_n == 0) || (TRACE_EVERY_N != 0 && tx_n != 0 && tx_n % TRACE_EVERY_N == 0) {
                TRACE_ARMED = true;
            }

            let (state_pre, state_post, irq_post) = adv_tx_burst_ch37(tx_n);

            if TRACE_ARMED {
                // Dense post-trigger samples for 0x40024208 status/IRQ candidates.
                // 96 MHz: 960 cycles ≈ 10us, 3840 cycles ≈ 40us more (≈50us total).
                qingke::riscv::asm::delay(960);
                let bb08_10us = bb_read(0x08);
                trace_r(BB_BASE as u32 + 0x08, bb08_10us);
                qingke::riscv::asm::delay(3_840);
                let bb08_50us = bb_read(0x08);
                trace_r(BB_BASE as u32 + 0x08, bb08_50us);
            }

            let done = wait_tx_done();

            if TRACE_ARMED {
                let lle00_post = lle_read(0x00);
                let bb08_post = bb_read(0x08);
                let bb64_post = bb_read(0x64);
                trace_r(LLE_BASE as u32 + 0x00, lle00_post);
                trace_r(BB_BASE as u32 + 0x08, bb08_post);
                trace_r(BB_BASE as u32 + 0x64, bb64_post);

                hal::println!("# txbuf addr={:#010x} len={} first21", addr_of!(TX_BUF) as u32, TX_BUF[1] as usize + 2);
                hal::println!(
                    "# txbuf {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                    TX_BUF[0], TX_BUF[1], TX_BUF[2], TX_BUF[3], TX_BUF[4], TX_BUF[5], TX_BUF[6],
                    TX_BUF[7], TX_BUF[8], TX_BUF[9], TX_BUF[10], TX_BUF[11], TX_BUF[12], TX_BUF[13],
                    TX_BUF[14], TX_BUF[15], TX_BUF[16], TX_BUF[17], TX_BUF[18], TX_BUF[19], TX_BUF[20],
                );
                hal::ble::tracer::dump();
                TRACE_ARMED = false;
            }

            tx_n += 1;
            if done { ok_n += 1; }

            // Print first 5, then every 100.
            if tx_n <= 5 || tx_n % 100 == 0 {
                let irq08  = bb_read(0x08);
                let ctrl00 = lle_read(0x00);
                let lle2c  = lle_read(0x2C);
                hal::println!(
                    "tx#{tx_n}: done={done} state={state_pre}→{state_post} \
                     irq_post={irq_post:#010x} irq_now={irq08:#010x} \
                     ctrl00={ctrl00:#010x} lle2c={lle2c:#010x} ok={ok_n}/{tx_n}",
                );
                let y3_bb00_logged = Y3_BB00_LOGGED;
                let y3_bb64_logged = Y3_BB64_LOGGED;
                hal::println!(
                    "# Y3_RUN total_bursts={} bb00_logged={} bb64_logged={}",
                    tx_n, y3_bb00_logged, y3_bb64_logged,
                );
                let bb_irq_entry = BB_IRQ_ENTRY;
                let bb_irq_exit = BB_IRQ_EXIT;
                let lle_irq_entry = LLE_IRQ_ENTRY;
                let lle_irq_exit = LLE_IRQ_EXIT;
                hal::println!(
                    "# PATHC_IRQ counters BB={}/{} LLE={}/{}",
                    bb_irq_entry, bb_irq_exit, lle_irq_entry, lle_irq_exit,
                );
                // V1.2 staged bisect: print stub results (visible only if BB_HANDLER_STAGE=0 or
                // stage 1/2 that returns cleanly — confirms handler can enter+exit without fault).
                // Stage 0 (no W1C): expect BB_STUB_ENTER_COUNT=BB_STUB_MAX_ENTRIES (50), then auto-disable.
                // Stage 1+: expect BB_STUB_ENTER_COUNT=0 (stub statics unused; full path runs).
                hal::println!(
                    "# BB_STUB stage={} enter_count={} blk0={:#010x}/ip4={:#04x}/bb08={:#010x} blk1={:#010x}/ip4={:#04x}/bb08={:#010x}",
                    BB_HANDLER_STAGE,
                    BB_STUB_ENTER_COUNT,
                    BB_STUB_BLK[0], BB_STUB_IP4[0], BB_STUB_BB08[0],
                    BB_STUB_BLK[1], BB_STUB_IP4[1], BB_STUB_BB08[1],
                );
                // NOTE: all SDI diagnostics (done/state/irq) are self-report only and
                // may not reflect actual air TX. Acceptance gate = nRF/HackRF visible packet.
                //   done=true      → ble_tx_wait_done() settle delay elapsed (always true)
                //   state_post≠108 → HW left Sleep state after trigger (informational)
                //   irq_post bits[15:0] → TX completion IRQ flags (bits[29+25] always-set)
            }

            // ~100 ms inter-packet gap.
            // At 96 MHz, ~4 cycles/iter: 2_400_000 iters ≈ 100 ms.
            qingke::riscv::asm::delay(2_400_000);
        }
    }
}
