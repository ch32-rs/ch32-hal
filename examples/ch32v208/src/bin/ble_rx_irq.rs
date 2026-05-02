//! BLE RX IRQ-driven passive scanner — CH32V208, Phase 2 / Task #13.
//!
//! Embassy async + PFIC interrupt harness replacing the polling loop from
//! `ble_rx_listen.rs`. Scan windows are now timer-driven via the LLE interrupt
//! (bit13 = window timeout) instead of a spinning poll.
//!
//! # Architecture
//!
//! ```text
//! PFIC LLE IRQ (IRQn 64) fires:
//!   bit13 → timer reload + OUR_S10 → S_HOP + signal Embassy task
//!   bit2  → RX done (frame ready in RX_BUF) → OUR_S10 → S_RX_DONE + signal
//!   end of ISR: if lle_ctrl00==0x07 → OUR_S10 = S_IDLE
//!   bit1/bit15 → cleared by broad W1C
//!
//! PFIC BB IRQ (IRQn 63) → W1C LLE+0x38 (BB block status) + count
//!
//! Embassy `ble_scanner_task`:
//!   loop {
//!     await LLE_IRQ_SIG
//!     if OUR_S10 ∈ {S_RX_DONE, S_HOP}: rx_traverse_v8(next_ch)
//!     if irq & bit2: decode PDU snapshot
//!   }
//! ```
//!
//! # Init sequence (from EVT MCU.c + ch32-hal confirmed)
//!
//! 1. `ble_phy_init()` — handles HSE, CRC/BLEC/BLES clocks, dev_init, RF calibration
//! 2. `ble_set_phy_rx_mode_normal()` — observer-mode BB config (dtmFlag=0 path)
//! 3. Clear stale LLE IRQ bits: `bb_write(0x08, 0xF00F)` (Cindy recommendation)
//! 4. PFIC enable BB (63) then LLE (64) (EVT MCU.c:158-159 order)
//! 5. `rx_cold_init()` — LL_ScanSetRF equivalent, RX GO
//!
//! # Register naming convention (swapped vs WCH SDK — see ble_rx_listen.rs)
//!
//! ```text
//! LLE_BASE = 0x40024100  (WCH: gptrBBReg)   → lle_read / lle_write
//! BB_BASE  = 0x40024200  (WCH: gptrLLEReg)  → bb_read  / bb_write
//! ```
//!
//! # v8 changelog (2026-05-02, multi-channel passive traverse)
//!
//! v7 Embassy Signal safety validated (v7 board: sig#75500/30s, 0 STUCK, 1149 PDUs).
//! v8 adds multi-channel channel hopping via `rx_traverse_v8` (passive .L546 path).
//!
//! ## Key findings from Lucy Path B (`<llScanTraverseaChannel>` d.asm, 2026-05-02)
//!
//! **Root cause of v6/v6b failures — THREE ERRORS in old `rx_traverse`:**
//! 1. Wrong accessor: `bb_write(0x00, ...)` targets RX-GO trigger (gptrLLEReg+0),
//!    not scan-active state (`lle_write(0x00, ...)` = gptrBBReg+0). Every bit7/bit8
//!    write was firing spurious RX-GO signals → HW state machine drift → 0x07 stuck.
//! 2. Active-scan state machine steps applied to passive scan — lib dispatch
//!    `s0[33]==2` gate routes to `.L544` (12-step active) vs `.L546` (2-step passive).
//!    Passive Observer mode should use `.L546`.
//! 3. No `s0[10]` state gate — lib only calls traverse when `s0[10] ∈ {162, 166}`.
//!
//! **Passive traverse `.L546` is just 2 writes:**
//! 1. `rfend_*(0x2C) &= ~0x02` — PLL lock release (HW auto-tracks channel → freq)
//! 2. `lle_*(0x00) = (c & !0x7F) | channel` — channel field write
//!
//! **PLL channel-tracking mode:** cold_init ends with `rfend_*(0x2C) bit1 = 0` which
//! puts PLL into channel-tracking mode (HW auto-maps `lle_*(0x00)[5:0]` → freq).
//! No per-hop `set_channel_freq` needed. RFEND+0x44 is never written in traverse.
//!
//! ## v8 single variable: multi-channel traverse + `OUR_S10` state gate
//!
//! - `OUR_S10: AtomicU8` tracks simplified scan state (IDLE/RX_DONE/HOP/TRAVERSING)
//! - ISR transitions: bit2 → S_RX_DONE, bit13 → S_HOP (from S_RX_DONE), end → S_IDLE if stuck
//! - Task: traverse only when `OUR_S10 ∈ {S_RX_DONE, S_HOP}`; cold re-init when S_IDLE
//! - `rx_traverse_v8(channel)` replaces old `rx_traverse(logical_ch, freq_khz)`
//!
//! ## v8 validation criteria (Cindy)
//!
//! - `snap_ch` cycles 0/1/2 (ch37/ch38/ch39), each ≥30 frames in 30s
//! - `lle_ctrl00=0x1` stable (no ★STUCK)
//! - Valid PDUs (Mfr/Flags) decoded on all three channels
//! - `HB:` heartbeat continues every 5s → executor alive
//!
//! # v7 changelog (2026-05-02, Embassy Signal safety validation)
//!
//! v6c skipped by team consensus — throttle only masks timing, not root cause.
//! v7 single variable: re-enable `LLE_IRQ_SIG.signal(irq)` in `fn LLE()`.
//! Board result: sig#75500/30s, HB×6, 1149 PDUs, 0 STUCK. Signal safe in HWSTK.
//!
//! # v6b changelog (2026-05-02, deferred channel traverse)
//!
//! v6 (rx_traverse in ISR) FAILED: PLL/RFEND writes in HWSTK fast-IRQ context push
//! `lle_ctrl00` to `0x07` STUCK after first hop (same signature as polling #3.11/#3.12).
//! v6b fix: ISR bit13 only sets `PENDING_TRAVERSE` (AtomicBool); task executes
//! `rx_traverse()` on next 50 ms poll tick in safe Embassy context. `CHANNEL_IDX`
//! rotation moved to task as well, so `SNAPSHOT_CH_IDX` remains correct at bit2 time.
//! Signal still disabled, `lle_*(0x2C)` tail still disabled — single new variable.
//!
//! # v5 changelog (2026-05-02, PDU decode)
//!
//! Timer-poll task (50 ms) replaces `wait().await` Signal consumer.
//! Signal remains disabled in LLE ISR (single-variable isolation per Cindy).
//! LLE ISR bit2 path: snapshot `RX_BUF[0..50]` + `SNAPSHOT_CH_IDX` after ACK.
//! Task decodes PDU type/len/AdvA and AD structures (Flags/Mfr/Name length).
//! v5 board result (2026-05-02): 517 decode lines, Apple/Samsung/MS confirmed,
//! lle_ctrl00=0x1 stable, lle2c=0x92010ec8 safe, HW dewhiten re-confirmed.
//!
//! # v4 changelog (2026-05-02, Lucy `apply_rx_state_ack()` spec)
//!
//! Added `apply_rx_state_ack()` to LLE ISR bit2 path:
//! - Computes `lle1c` value via Lucy's formula (d.asm L71240 0x270–0x2BA)
//! - Writes `bb_*(0x1C)` = WCH gptrLLEReg+0x1C (HW state-machine ACK)
//! - Clears stale RX flag: `lle_*(0x2C) = (val & ~3) | 1`
//! - Tracks lle1c histogram: 0x5d/0x6c/other
//!
//! Corrected: bit0 semantics = BB-side RX-done preprocess hook (not cold-init signal).
//! `b0 == b2 == bb_count` in v3 → all three fire on same CRC-OK RX event (Lucy 2026-05-02).
//!
//! # v3 changelog (2026-05-02, Cindy Run A/B findings)
//!
//! **Run A root cause**: BB stub (v2) had no W1C for LLE+0x38 (BB block status).
//! BB(63) fired continuously, starving LLE(64) which stayed PFIC-pending forever.
//! Fix: `BB()` now does `lle_write(0x38, lle_read(0x38))` before incrementing count.
//!
//! **Run B root cause**: LLE handler (v2) only W1C'd bits 2/1/15. Many other bits
//! (3,5,6,7,10,11,13,16,24,27,28,29) in LLE+0x08 remained set, causing PFIC to
//! immediately re-trigger LLE after mret — "handler enters but never returns" symptom.
//! Fix: broad W1C `bb_write(0x08, irq)` clears ALL asserted bits at ISR entry.
//!
//! **v3 scope**: minimal isolation — confirm LLE IRQ enters/returns cleanly.
//! PDU decode, channel advance (rx_traverse), and canonical timer reload (.L562)
//! are deferred to v4 once ISR stability is proven.
//!
//! # v4 TODOs (after v3 board validation confirms stable LLE IRQ)
//!
//! - Restore RX_BUF snapshot copy on bit2 + PDU decode (Flags/Mfr/Name)
//! - Restore channel advance (rx_traverse) in task on bit13
//! - Restore canonical rx_window_timer_reload() (.L562 masked sequence)
//! - `LLE+0x1C` state value write (lle_irq_process bit2 path, Lucy phase 1)
//! - `s0[10]` dispatch gating (Lucy Risk 1 + Risk 3)
//! - `#[highcode]` on `fn LLE()` if flash-latency A/B test shows degraded RX
//!
//! # Ship baseline reference
//! `ble_rx_listen.rs` (commit 30f36a5, patch #3.10 + #3.13) = polling MVP.
//! This file is the IRQ-driven successor.

#![no_std]
#![no_main]

use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use {ch32_hal as hal, panic_halt as _};

// ── Register base addresses (swapped WCH naming — see module doc) ─────────────

/// WCH gptrBBReg: CTRL/GO, BB_CFG, ACCESS_ADDR, CRC_INIT, PHY mode regs.
const LLE_BASE: usize = 0x40024100;
/// WCH gptrLLEReg: scan ctrl, IRQ status, window timer, DMA buffer ptr.
const BB_BASE: usize = 0x40024200;
/// WCH gptrRFENDReg: PLL dividers, channel lock, PA/LNA bias.
const RFEND_BASE: usize = 0x40025000;

// ── Register accessors ─────────────────────────────────────────────────────────

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

// ── BLE constants ──────────────────────────────────────────────────────────────

const ADV_AA: u32 = 0x8E89_BED6;
const ADV_CRC_INIT: u32 = 0x55_5555;

/// ADV channels: (logical_ch, freq_khz, label).
const ADV_CHANNELS: [(u8, u32, &str); 3] = [
    (37, 2_402_000, "ch37/2402MHz"),
    (38, 2_426_000, "ch38/2426MHz"),
    (39, 2_480_000, "ch39/2480MHz"),
];

/// Per-window timer ticks written to LLE+0x64.
/// Value 406 (0x196) from d.asm `llScanProcess` L63159 — matches WCH lib.
/// (Polling version used 400_000 to keep receiver alive longer; IRQ version
///  relies on bit13 firing each window, so 406 is correct.)
const SCAN_WINDOW_TICKS: u32 = 406;

// ── RX DMA buffer ──────────────────────────────────────────────────────────────

/// DMA RX buffer. HW writes the dewhitened PDU directly at offset 0.
/// `buf[1]` becoming non-zero signals a completed DMA write (length byte).
#[link_section = ".bss"]
static mut RX_BUF: [u8; 280] = [0u8; 280];

// ── Shared state ───────────────────────────────────────────────────────────────

/// Simplified scan state for `OUR_S10` (mirrors lib's `s0[10]` dispatch values).
/// lib `<llScanProcess>` only calls traverse when `s0[10] ∈ {162, 166}`.
/// Using same values to make cross-referencing with d.asm clear.
const S_IDLE:       u8 = 0;   // HW not in scan (lle_ctrl00=0x07 stuck or pre-init)
const S_RX_DONE:    u8 = 162; // Frame received, ready for traverse to next channel
const S_HOP:        u8 = 166; // Timer expired without frame, ready for channel hop
const S_TRAVERSING: u8 = 1;   // Traverse in progress (reentry guard)

/// Simplified scan-state machine: ISR updates, task reads and acts.
/// Initialized to S_RX_DONE so first timer tick immediately triggers a traverse
/// rather than waiting for a frame first.
static OUR_S10: AtomicU8 = AtomicU8::new(S_RX_DONE);

/// Current ADV channel index (0=ch37, 1=ch38, 2=ch39).
/// Advanced by task in state gate before calling rx_traverse_v8.
/// Read by ISR bit2 snapshot to record which channel the frame arrived on.
/// AtomicUsize load/store safe on single-core.
static CHANNEL_IDX: AtomicUsize = AtomicUsize::new(0);

/// Guards cold-boot init — only runs once.
static SCAN_INITED: AtomicBool = AtomicBool::new(false);

/// BB IRQ occurrence counter (diagnostic; BB path unused for scanning).
/// Non-atomic is safe here: BB IRQ is non-reentrant on CH32 single-core.
static mut BB_IRQ_COUNT: u32 = 0;

/// LLE IRQ fire count — diagnostic for confirming ISR entry/exit.
/// Non-atomic safe: LLE ISR is non-reentrant on CH32 single-core.
static mut LLE_IRQ_COUNT: u32 = 0;

/// Most recent raw LLE+0x08 IRQ status captured by LLE ISR.
static mut LAST_LLE_IRQ: u32 = 0;

/// Per-bit LLE IRQ distribution counters (Lucy 2026-05-02 diagnostic).
/// Expected stable-state (passive scan, empty room):
///   bit13 >> bit2 >> bit15; bit0 ≥ 1 (cold-init / RX GO signal).
/// All non-atomic: LLE ISR is non-reentrant on CH32 single-core.
static mut LLE_BIT0_COUNT: u32 = 0;  // cold-init / RX GO startup signal
static mut LLE_BIT1_COUNT: u32 = 0;  // unknown (rarely fires in scan mode)
static mut LLE_BIT2_COUNT: u32 = 0;  // RX done (rare in empty room)
static mut LLE_BIT13_COUNT: u32 = 0; // window timeout (dominant in passive scan)
static mut LLE_BIT15_COUNT: u32 = 0; // spurious / IRQ bus
static mut LLE_OTHER_COUNT: u32 = 0; // all other bits combined

// ── lle1c histogram (v4 validation — Lucy 2026-05-02) ─────────────────────────

/// Counts of each lle1c value written by `apply_rx_state_ack()`.
/// v4 validation criterion: at least 0x5D (93) and 0x6C (108) both appear.
/// (Cindy: lle1c should NOT be constant 0x6c anymore — formula drives variation.)
/// 0x5D = 93: rxflag=0, lle50.bit5=0 path (most common passive scan case)
static mut LLE1C_HIST_93: u32 = 0;
/// 0x6C = 108: sw_state bits[13:12] != 0 path (idle/post-RX default)
static mut LLE1C_HIST_108: u32 = 0;
/// Other lle1c values: 97/101/105/107 (rxflag 1–4 paths, rare in passive scan)
static mut LLE1C_HIST_OTHER: u32 = 0;

// ── Frame snapshot (ISR → task, avoids DMA/channel race) ──────────────────────

/// Snapshot of RX_BUF[0..50] captured by the LLE ISR at bit2 time, BEFORE any
/// channel advance. The task reads this instead of RX_BUF directly to avoid:
///   (a) DMA overwriting RX_BUF before the task runs
///   (b) CHANNEL_IDX already advanced when bit2+bit13 are in the same IRQ
///
/// Single-producer (ISR) / single-consumer (task) on single-core CH32 — safe.
static mut RX_SNAPSHOT: [u8; 50] = [0u8; 50];

/// ADV channel index at the time the snapshot was captured (pre-advance).
static mut SNAPSHOT_CH_IDX: usize = 0;

/// Set by LLE ISR bit13 to request a channel traverse from the task.
/// ISR cannot call rx_traverse() directly — PLL/RFEND writes in HWSTK fast-IRQ
/// context push lle_ctrl00 to 0x07 STUCK (confirmed v6 board run 2026-05-02).
/// Task clears this flag after executing rx_traverse() in safe poll context.
static PENDING_TRAVERSE: AtomicBool = AtomicBool::new(false);

/// Generation counter: ISR increments after every complete snapshot copy.
/// Task compares to detect new frames without signal.
/// Non-atomic is safe: single-core CH32, ISR is the sole writer.
static mut SNAPSHOT_GEN: u32 = 0;

// ── IRQ → Embassy signaling ────────────────────────────────────────────────────

/// LLE IRQ signal: carries irq08 status bits from `LLE()` ISR to `ble_scanner_task`.
///
/// `Signal` is last-value: only the most recent unread value is retained.
/// For scan workloads this is acceptable — the task must process frames fast
/// enough not to miss consecutive bit2 events. If back-to-back frames are a
/// concern, replace with `Channel<_, u32, N>`.
///
/// v4b: `.signal()` re-enabled in `fn LLE()` to test Embassy ISR→task wake safety.
/// If `HB:` heartbeat stops after Signal is added → CriticalSection deadlocks in HWSTK.
/// If `HB:` continues and task wakes → Signal is safe, proceed to v5 (RX decode).
static LLE_IRQ_SIG: Signal<CriticalSectionRawMutex, u32> = Signal::new();

// ── PHY helpers (verbatim from ble_rx_listen.rs) ──────────────────────────────

/// Program the RFEND PLL to `freq_khz` and assert channel-lock (RFEND+0x2C bit1=1).
///
/// From RF_DevSetChannel() in libwchble.a V1.40.
unsafe fn set_channel_freq(freq_khz: u32) {
    let int_div = (freq_khz / 64_000) & 0x1F;
    let frac_div = ((freq_khz % 64_000) << 10) / 250;
    let pll = rfend_read(0x44);
    rfend_write(0x44, (pll & 0xFE0F_C000) | (int_div << 20) | (frac_div & 0x3FFF));
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v | (1 << 1));
}

/// Configure BB for normal-mode BLE reception (observer / scanner).
///
/// From BLE_SetPHYRxMode() .L94 branch (dtmFlag=0), decoded by Lucy 2026-05-01.
/// Must be called after `ble_phy_init()`.
unsafe fn ble_set_phy_rx_mode_normal() {
    lle_write(0x20, 0x0009_0083);
    lle_write(0x14, 0x0810_1901);
    lle_write(0x18, 0x0003_1624);
    lle_write(0x28, 0x0000_28BE);
    lle_write(0x24, 0x0100_6310);
    lle_write(0x10, 0x0032_22D0);
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x3000) | 0x1000);
}

/// Mirror of `<lle_irq_process>` bit2-path tail (Lucy spec 2026-05-02).
/// Source: `notes/ch32-rs/scan-ctx-layout.md` §"bit2's lle1c calculation"
///         d.asm L71240 instructions 0x270–0x2BA
///
/// # What this does
///
/// Called when bit2 (RX done) fires. Computes the `lle1c` state-machine acknowledgment
/// value and writes it to `bb_*(0x1C)`, then clears the stale RX flag in `lle_*(0x2C)`.
/// This is the HW state-machine ack that prevents `lle00 = 0x07` stuck state.
///
/// # Register naming (CRITICAL — same confusion as elsewhere)
///
/// - `sw_state = lle_*(0x00)` = **WCH gptrBBReg+0** = `LLE_BASE+0` — NOT `bb_*(0x00)`
/// - `lle1c write = bb_*(0x1C)` = WCH gptrLLEReg+0x1C — this is the scan state register
/// - `lle_*(0x2C)` = **WCH gptrBBReg+0x2C** = `LLE_BASE+0x2C` — NOT `bb_*(0x2C)`
///
/// # v4 simplification
///
/// `rxflag` is hardcoded to 0. True source is `*(*(scan_ctx+36)+1)` per d.asm — we don't
/// yet have a Rust scan_ctx mirror. Passive scan default is `rxflag=0` → writes `lle1c=93`
/// (0x5D) in the common case.
///
/// # v4 validation (Cindy)
///
/// `lle1c` should show ≥ 2 distinct values (not constant 0x6c anymore):
/// - `0x5D` (93): most common passive RX, rxflag=0 + lle50.bit5=0
/// - `0x6C` (108): sw_state bits[13:12] != 0, idle/post-RX default
/// - Others (0x61/0x65/0x69/0x6B): rxflag 1–4 paths (rare in passive scan)
#[inline]
unsafe fn apply_rx_state_ack() {
    // sw_state: lle_*(0x00) = WCH gptrBBReg+0 = LLE_BASE+0
    let sw_state = lle_read(0x00);
    let lle1c_value: u32 = if (sw_state >> 12) & 0x3 != 0 {
        108  // 0x6C — idle/post-RX default (matches LLE_DevInit boot value)
    } else {
        let lle50 = bb_read(0x50);
        let rxflag: u8 = 0; // v4 hardcode: passive scan rxflag=0 default
        if lle50 & 0x20 == 0 {
            // lle50 bit5 == 0 → rxflag-driven dispatch
            match rxflag {
                0 => 93,   // 0x5D — most common passive RX path
                1 => 97,   // 0x61
                2 => 101,  // 0x65
                3 => 105,  // 0x69
                _ => 107,  // 0x6B
            }
        } else {
            // lle50 bit5 == 1
            if rxflag != 0 { 107 } else { 93 }
        }
    };

    // Track lle1c histogram for v4 validation
    match lle1c_value {
        93  => LLE1C_HIST_93  += 1,
        108 => LLE1C_HIST_108 += 1,
        _   => LLE1C_HIST_OTHER += 1,
    }

    // Write lle1c = bb_*(0x1C) = WCH gptrLLEReg+0x1C
    bb_write(0x1C, lle1c_value);

    // Stale RX flag clear + ACK: lle_*(0x2C) = (val & ~3) | 1
    // lle_*(0x2C) = WCH gptrBBReg+0x2C = LLE_BASE+0x2C (NOT bb_*(0x2C))
    // d.asm: a3 = lw 44(a2); a3 &= -4; a3 |= 1; sw a3, 44(a2)
    //
    // v4a ISOLATION: lle_*(0x2C) write DISABLED.
    // Cindy v4 result: b2 dropped from 3962 to 1 after first RX; bb_blk changed
    // from 0x00bea407 to 0x001ec106 on first bit2. Suspected culprit: this write
    // disrupts HW scan state-machine. Testing with only bb_write(0x1C) to isolate.
    // If b2 recovers to v3 levels → lle_*(0x2C) write needs timing/condition fix.
    // If b2 still = 1 → bb_write(0x1C, lle1c_value) itself is the disruption.
    //
    // let lle_2c = lle_read(0x2C);
    // lle_write(0x2C, (lle_2c & !0b11) | 1);
}

/// Per-window timer reload: canonical `llScanProcess .L562` sequence.
/// Deferred to v4 (v3 uses single-write reload in ISR; masked sequence for v4).
#[allow(dead_code)]
///
/// d.asm sequence (L63159, patch #3.10, Lucy 2026-05-01):
///   mask lle0C bit13 → W1C bit13 → write lle64=406 → restore lle0C.
///
/// Masking bit13 prevents a spurious IRQ during the reload.
/// Must be called BEFORE `rx_traverse` (Plan C ordering).
unsafe fn rx_window_timer_reload() {
    let lle0c_orig = bb_read(0x0C);
    bb_write(0x0C, lle0c_orig & !0x2000);
    bb_write(0x08, 0x2000);
    bb_write(0x64, SCAN_WINDOW_TICKS);
    bb_write(0x0C, lle0c_orig);
}

/// Per-window channel advance: passive scan `.L546` minimal path.
///
/// Direct transcription of d.asm `<llScanTraverseaChannel>` L62961+0x10e-0x136.
/// Passive Observer mode (`s0[33] != 2`) uses this 2-write path; the 12-step
/// `.L544` path is active-scan only (SCAN_REQ/RSP, not used here).
///
/// ## Why only 2 writes?
///
/// Passive RX chain stays live across channel hops — no RF state-machine reset
/// needed. HW is in channel-tracking mode (RFEND+0x2C bit1=0) since cold_init
/// ended with `rfend_write(0x2C, v & !0x02)`. Writing the channel field and
/// pulsing PLL release is sufficient for HW to retune to the new frequency.
///
/// ## Why NO `set_channel_freq` per hop?
///
/// `set_channel_freq` (1) programs RFEND+0x44 dividers (unnecessary — HW maps
/// `lle_*(0x00)[5:0]` → freq automatically) and (2) asserts PLL lock (bit1=1),
/// undoing the tracking mode. `<llScanTraverseaChannel>` has no RFEND+0x44 write.
///
/// ## Edge case: 39→37 wraparound
///
/// lib `.L545` may call a full `LL_ScanSetRF`-equivalent on wraparound.
/// v8 first pass skips special handling. If board shows 39→37 frame loss
/// or `lle_ctrl00=0x07` on wraparound, add `set_channel_freq(ADV_CHANNELS[0].1)`
/// prepend for that hop only.
unsafe fn rx_traverse_v8(channel: u8) {
    // .L546 step 1: PLL lock release — triggers channel-tracking mode retune
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !0x02);

    // .L546 step 2: channel field write — HW auto-maps channel[5:0] → RF freq
    // Uses lle_*(0x00) = WCH gptrBBReg+0 (scan-active state register).
    // NOT bb_*(0x00) which is WCH gptrLLEReg+0 (RX-GO trigger — different register!).
    let c = lle_read(0x00);
    lle_write(0x00, (c & !0x7F) | (channel as u32 & 0x3F));
}

/// DEPRECATED — three known errors, replaced by `rx_traverse_v8`.
///
/// Errors:
/// 1. `bb_write(0x00, ...)` targets RX-GO trigger (gptrLLEReg+0), not scan-active
///    state (`lle_write(0x00, ...)` = gptrBBReg+0). Bit7/bit8 writes fired spurious
///    RX-GO signals → HW state-machine drift → 0x07 stuck after ~34 frames.
/// 2. Uses active-scan `.L544` state transitions (bit7→bit8 cycle, PRE_DELAY reset)
///    which are unnecessary for passive Observer mode (`.L546` path).
/// 3. `set_channel_freq` per hop: asserts PLL lock, undoing channel-tracking mode.
#[allow(dead_code)]
unsafe fn rx_traverse_v6(logical_ch: u8, freq_khz: u32) {
    set_channel_freq(freq_khz);
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x180) | 0x80);
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x180) | 0x100);
    bb_write(0x50, 89);
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !0x02);
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x7F) | (logical_ch as u32 & 0x3F));
}

/// Cold-boot RX init: full `LL_ScanSetRF` (L62824) + RX GO. Call once.
///
/// Steps match d.asm exactly. Verbatim from ble_rx_listen.rs (patch #3.10).
unsafe fn rx_cold_init(logical_ch: u8, freq_khz: u32) {
    set_channel_freq(freq_khz);

    bb_write(0x64, 0);           // LL_ScanSetRF step 1: LLE+0x64 = 0
    bb_write(0x08, 0xF00F);      // step 2: W1C stale IRQ bits (cold-boot clear)

    lle_write(0x08, ADV_AA);     // step 3a: ACCESS_ADDR
    lle_write(0x04, ADV_CRC_INIT); // step 3b: CRC seed

    let v = bb_read(0x04);
    bb_write(0x04, v | 0x1);     // step 4: LLE+0x04 |= 1 (scan mode enable)

    bb_write(0x74, addr_of!(RX_BUF) as u32); // step 5: DMA buffer pointer

    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !(1 << 1)); // step 6: release PLL lock

    // step 7: BB+0x00 bits[5:0] = logical_ch (channel BEFORE bit8)
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | (logical_ch as u32 & 0x3F));

    // step 8: BB+0x00 bits[8:7] = 10b (RX-mode marker, AFTER channel)
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x180) | 0x100);

    // step 9: RFEND+0x08 |= 0x330000 (PA/LNA/RX path enable)
    let ana = rfend_read(0x08);
    rfend_write(0x08, ana | 0x0033_0000);

    bb_write(0x50, 89);          // step 10: PRE_DELAY = 89

    // Initial scan window (IRQ version: use d.asm value 406, not 400_000 from polling)
    bb_write(0x64, SCAN_WINDOW_TICKS);

    bb_write(0x00, 1);           // step 11: RX GO — last write, pulls HW into active scan
}

// ── PFIC interrupt handlers ────────────────────────────────────────────────────

/// LLE PFIC interrupt handler (IRQn 64) — v3 minimal isolation.
///
/// # v3 changes from v2 (Cindy Run A/B findings, 2026-05-02)
///
/// **Problem 1 (run A)**: BB stub had no W1C → BB(63) IRQ storm → LLE(64) never ran.
/// **Problem 2 (run B)**: v2 LLE handler only cleared bits 2/1/15. Many other bits
/// (3,5,6,7,10,11,13,16,24,27,28,29) in LLE+0x08 remained set, causing PFIC to
/// immediately re-trigger LLE after mret — "handler never returns" symptom.
///
/// **Fix**: Broad W1C `bb_write(0x08, irq)` clears ALL asserted bits at entry.
/// This is safe for passive scanning — we don't depend on other bit paths.
///
/// # ISR philosophy (v3 diagnostic)
///
/// Minimal: broad W1C → diagnostic record → simple timer reload (bit13) → Signal.
/// No RX_BUF snapshot loop (50×read_volatile deferred to v4).
/// No rx_traverse (channel advance deferred to v4).
/// Goal: confirm LLE IRQ enters and exits cleanly (LLE_IRQ_COUNT grows in heartbeat).
///
/// # bit13 timer reload (simplified vs v2)
///
/// v2 used the masked .L562 sequence (mask lle0C → W1C bit13 → reload → restore).
/// v3 uses a single write: broad W1C already cleared bit13 before the reload.
/// No new bit13 can fire between the broad W1C and the `bb_write(0x64, ...)` —
/// the mask/restore was only needed when bit13 was handled separately.
///
/// # TODO (v4, pending Lucy phase 1 / v2-api-contract)
///
/// - Restore RX_BUF snapshot copy on bit2
/// - Restore canonical rx_window_timer_reload() (.L562 masked sequence)
/// - rx_traverse for channel advance (Lucy Risk 2 — after ISR stability proven)
/// - LLE+0x1C state write (lle_irq_process bit2 path, Lucy phase 1)
/// - s0[10] dispatch gating (Lucy Risk 1)
#[ch32_hal::interrupt]
fn LLE() {
    let irq = unsafe { bb_read(0x08) };

    // Broad W1C: clear ALL asserted bits to prevent immediate re-entry.
    // v2 only cleared bits 2/1/15; remaining bits caused PFIC to re-trigger LLE
    // after every mret → "handler stuck / never returns" symptom in run B.
    unsafe { bb_write(0x08, irq); }

    // Diagnostic record (read by heartbeat and ble_scanner_task).
    unsafe {
        LAST_LLE_IRQ = irq;
        LLE_IRQ_COUNT += 1;
        // Per-bit distribution (Lucy 2026-05-02 revised semantics):
        //   bit0 = BB-side RX-done preprocess hook (NOT cold-init signal!).
        //          b0 == b2 == bb_count in v3 → these three are same physical RX event.
        //          Lucy corrected §14: bit0 fires on every CRC-OK RX, not just cold-init.
        //   bit13 >> bit2 in passive empty-room scan.
        if irq & (1 << 0)  != 0 { LLE_BIT0_COUNT  += 1; }
        if irq & (1 << 1)  != 0 { LLE_BIT1_COUNT  += 1; }
        if irq & (1 << 2)  != 0 { LLE_BIT2_COUNT  += 1; }
        if irq & (1 << 13) != 0 { LLE_BIT13_COUNT += 1; }
        if irq & (1 << 15) != 0 { LLE_BIT15_COUNT += 1; }
        let known = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 13) | (1 << 15);
        if irq & !known != 0    { LLE_OTHER_COUNT += 1; }
    }

    // ★ v4/v5: apply_rx_state_ack() + snapshot copy on bit2.
    // apply_rx_state_ack() is the HW state-machine ACK (prevents lle00=0x07 stuck state).
    // Snapshot is captured immediately after ACK — BEFORE any channel advance — so the
    // task always sees the PDU that triggered this bit2 event.
    //
    // IMPORTANT: bit2 must run BEFORE bit13 within the same ISR call (ordering invariant).
    // If bit13 ran first and the task advanced CHANNEL_IDX, SNAPSHOT_CH_IDX would label
    // the frame with the new (wrong) channel. Order bit2→bit13 is correct.
    if irq & (1 << 2) != 0 {
        unsafe { apply_rx_state_ack(); }

        // v5: copy RX_BUF[0..50] → RX_SNAPSHOT (DMA buffer → stable task-readable copy).
        // Bump gen via write_volatile AFTER copy — prevents compiler from hoisting the
        // gen write above the read_volatile loop. Task reads gen via read_volatile too.
        unsafe {
            SNAPSHOT_CH_IDX = CHANNEL_IDX.load(Ordering::Relaxed);
            let src = addr_of!(RX_BUF) as *const u8;
            for i in 0..50usize {
                RX_SNAPSHOT[i] = read_volatile(src.add(i));
            }
            let g = read_volatile(addr_of!(SNAPSHOT_GEN));
            write_volatile(addr_of_mut!(SNAPSHOT_GEN), g.wrapping_add(1));
        }

        // v8: state gate transition — frame received → S_RX_DONE (ready to hop).
        // If S_HOP: keep S_HOP (simultaneous bit2+bit13: timer takes priority for hop).
        let s = OUR_S10.load(Ordering::Relaxed);
        if s != S_HOP {
            OUR_S10.store(S_RX_DONE, Ordering::Relaxed);
        }
    }

    // bit13: timer reload + v8 state gate transition.
    // ISR reloads the scan window timer and signals S_HOP; the task executes the
    // actual channel hop in safe Embassy context (rx_traverse in ISR caused 0x07
    // stuck in v6 due to RFEND writes in HWSTK fast-IRQ context).
    if irq & (1 << 13) != 0 {
        unsafe { bb_write(0x64, SCAN_WINDOW_TICKS); }
        // v8: timer → S_HOP only if currently S_RX_DONE (lib semantics: 162→166).
        // If S_IDLE: HW not scanning, keep IDLE. If already S_HOP or S_TRAVERSING:
        // don't overwrite (preserve reentry guard or queued hop).
        let s = OUR_S10.load(Ordering::Relaxed);
        if s == S_RX_DONE {
            OUR_S10.store(S_HOP, Ordering::Relaxed);
        }
        PENDING_TRAVERSE.store(true, Ordering::Relaxed); // kept for compat/diagnostics
    }

    // v8: end-of-ISR stuck detection — override OUR_S10 to S_IDLE if HW stuck.
    // lle_ctrl00 = bb_read(0x00) = WCH gptrLLEReg+0 (scan state: 1=active, 7=stuck).
    // Checked after bit2/bit13 so transitions don't race the stuck detection.
    let ctrl00 = unsafe { bb_read(0x00) };
    if ctrl00 == 0x07 {
        OUR_S10.store(S_IDLE, Ordering::Relaxed);
    }

    // Signal: carries irq bits to task for diagnostic + bit2 PDU decode trigger.
    LLE_IRQ_SIG.signal(irq);
}

/// BB PFIC interrupt handler (IRQn 63).
///
/// EVT C: `BB_IRQHandler()` → `BB_IRQLibHandler()` (d.asm L69270, 0x114 size).
/// Passive scanning uses the LLE path; BB is the connection/advertiser path.
///
/// # Critical fix (v3 — Cindy Run A finding, 2026-05-02)
///
/// **MUST W1C LLE+0x38 (BB block status register) to prevent IRQ storm.**
/// v2 only incremented BB_IRQ_COUNT, leaving BB+0x38 asserted. This caused
/// BB(63) to fire continuously (PFIC IACTR0 word1 bit31 stuck active), entirely
/// starving LLE(64) which was stuck in PFIC IPR pending state.
///
/// The W1C write `lle_write(0x38, lle_read(0x38))` clears the BB block IRQ
/// status register (WCH gptrBBReg+0x38 = LLE_BASE+0x38 in our naming).
/// Confirmed working in Cindy Run B: after the fix, BB released and LLE
/// became active (PFIC IACTR0 word2 bit0 = IRQ64 active).
#[ch32_hal::interrupt]
fn BB() {
    unsafe {
        // W1C BB block status: required to release BB IRQ and allow LLE to run.
        lle_write(0x38, lle_read(0x38));
        BB_IRQ_COUNT += 1;
    }
}

// ── Embassy async scanner task ─────────────────────────────────────────────────

/// Signal-driven scanner task — v8 multi-channel traverse.
///
/// # v8 design
///
/// Driven by `LLE_IRQ_SIG.wait().await`. On each wake:
/// 1. Check `OUR_S10` state gate: traverse if `∈ {S_RX_DONE, S_HOP}`.
/// 2. Decode PDU snapshot if bit2 was set in the waking IRQ.
///
/// Channel hops via `rx_traverse_v8` — 2-write passive `.L546` path from d.asm.
/// No `set_channel_freq` per hop (PLL in channel-tracking mode since cold_init).
///
/// # Validation criteria
///
/// - `snap_ch` cycles 0/1/2 (ch37/ch38/ch39), each ≥30 frames/30s
/// - `lle_ctrl00=0x1` stable (no ★STUCK)
/// - Valid PDUs on all three channels
/// - `HB:` every 5s → executor alive
///
/// # TODO (v9+)
///
/// - S_IDLE cold re-init path (LL_ScanSetRF equivalent) for sustained scanning
/// - lle_*(0x2C) tail with correct timing (Lucy §17)
#[embassy_executor::task]
async fn ble_scanner_task() {
    let mut prev_lle_cnt: u32 = 0;
    let mut prev_gen:     u32 = unsafe { read_volatile(addr_of!(SNAPSHOT_GEN)) };
    let mut sig_n:        u32 = 0;
    let mut traverse_n:   u32 = 0;

    loop {
        let irq = LLE_IRQ_SIG.wait().await;
        sig_n += 1;

        let lle_cnt    = unsafe { LLE_IRQ_COUNT };
        let bb_cnt     = unsafe { BB_IRQ_COUNT };
        let b0         = unsafe { LLE_BIT0_COUNT };
        let b2         = unsafe { LLE_BIT2_COUNT };
        let b13        = unsafe { LLE_BIT13_COUNT };
        let bx         = unsafe { LLE_OTHER_COUNT };
        let h5d        = unsafe { LLE1C_HIST_93 };
        let h6c        = unsafe { LLE1C_HIST_108 };
        let lle_ctrl00 = unsafe { bb_read(0x00) };
        let lle64      = unsafe { bb_read(0x64) };
        let lle1c      = unsafe { bb_read(0x1C) };
        let bb_blk     = unsafe { lle_read(0x38) };
        let bb_sw00    = unsafe { lle_read(0x00) };
        let lle_2c     = unsafe { lle_read(0x2C) };
        let s10        = OUR_S10.load(Ordering::Relaxed);

        let lle_delta = lle_cnt.wrapping_sub(prev_lle_cnt);
        let stuck     = lle_ctrl00 == 0x07;

        // Print first 5 signals then every 500th (~10 prints/s at 5.4kHz bit13 rate)
        if sig_n <= 5 || sig_n % 500 == 0 {
            hal::println!(
                "sig#{sig_n}: irq={irq:#010x} lle={lle_cnt}(+{lle_delta}) bb={bb_cnt} \
                 s10={s10} trav={traverse_n} lle_ctrl00={lle_ctrl00:#010x}{} \
                 lle64={lle64:#010x} lle1c={lle1c:#04x} bb_sw00={bb_sw00:#010x} \
                 lle2c={lle_2c:#010x} bb_blk={bb_blk:#010x}",
                if stuck { " ★STUCK" } else { "" },
            );
            hal::println!(
                "  irq_hist: b0={b0} b2={b2} b13={b13} other={bx} \
                 lle1c_hist: 93={h5d} 108={h6c}"
            );
        }

        // ── PDU decode on bit2 events (before traverse, mirrors ISR bit2→bit13 order) ──

        if irq & (1 << 2) != 0 {
            let gen = unsafe { read_volatile(addr_of!(SNAPSHOT_GEN)) };
            if gen != prev_gen {
                prev_gen = gen;
                let snap_ch = unsafe { SNAPSHOT_CH_IDX };
                let snap    = unsafe { RX_SNAPSHOT };

                let pdu_type = snap[0] & 0x0F;
                let pdu_len  = snap[1] & 0x3F;
                let ch_label = ADV_CHANNELS[snap_ch].2;

                if pdu_len >= 6 {
                    hal::println!(
                        "[gen={gen} ch={ch_label}] type={pdu_type:#x} len={pdu_len} \
                         AdvA={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                        snap[7], snap[6], snap[5], snap[4], snap[3], snap[2],
                    );
                    if matches!(pdu_type, 0x00 | 0x02 | 0x06) {
                        let ad_end = (8 + pdu_len as usize).min(50);
                        let mut off = 8usize;
                        while off + 1 < ad_end {
                            let ad_len   = snap[off] as usize;
                            if ad_len == 0 { break; }
                            let data_end = off + 1 + ad_len;
                            if data_end > 50 { break; }
                            let ad_type  = snap[off + 1];
                            match ad_type {
                                0x01 if ad_len >= 2 => {
                                    hal::println!("  flags={:#04x}", snap[off + 2]);
                                }
                                0xFF if ad_len >= 3 => {
                                    let company = u16::from_le_bytes([snap[off + 2], snap[off + 3]]);
                                    hal::println!("  mfr={company:#06x} datalen={}", ad_len - 3);
                                }
                                0x08 | 0x09 if ad_len >= 1 => {
                                    hal::println!("  name=<{}B>", ad_len - 1);
                                }
                                _ => {}
                            }
                            off = data_end;
                        }
                    }
                } else {
                    hal::println!(
                        "[gen={gen} ch={ch_label}] raw hdr=[{:#04x},{:#04x}] \
                         type={pdu_type:#x} len={pdu_len}",
                        snap[0], snap[1],
                    );
                }
            }
        }

        // ── v8 state gate: channel traverse ──────────────────────────────────────────

        // lib `<llScanProcess>` only calls traverse when `s0[10] ∈ {162, 166}`.
        // S_RX_DONE (162): frame received, hop to next channel.
        // S_HOP (166): timer fired without frame, hop anyway.
        // S_IDLE (0): HW stuck (lle_ctrl00=0x07) — cold re-init needed (deferred).
        // S_TRAVERSING (1): already traversing — skip (reentry guard).
        let s = OUR_S10.load(Ordering::Relaxed);
        if s == S_RX_DONE || s == S_HOP {
            // Set TRAVERSING before the traverse call (reentry guard for signal bursts).
            OUR_S10.store(S_TRAVERSING, Ordering::Relaxed);

            // Advance channel BEFORE traverse: CHANNEL_IDX must reflect the new channel
            // BEFORE the next bit2 fires so SNAPSHOT_CH_IDX is labelled correctly.
            let new_ch_idx = CHANNEL_IDX.load(Ordering::Relaxed).wrapping_add(1) % 3;
            CHANNEL_IDX.store(new_ch_idx, Ordering::Relaxed);
            let (logical_ch, _, ch_label) = ADV_CHANNELS[new_ch_idx];

            unsafe { rx_traverse_v8(logical_ch); }
            traverse_n += 1;

            OUR_S10.store(S_RX_DONE, Ordering::Relaxed);

            // Log every 1000th traverse for channel hop visibility
            if traverse_n <= 3 || traverse_n % 1000 == 0 {
                hal::println!("  trav#{traverse_n}: → {ch_label}");
            }
        } else if s == S_IDLE && !stuck {
            // Spurious S_IDLE without 0x07: ISR may have briefly set IDLE during
            // a glitch — just print a warning, don't reinit yet.
            if sig_n <= 5 || sig_n % 500 == 0 {
                hal::println!("  ⚠ S_IDLE but ctrl00≠0x07 (transient?)");
            }
        }
        // S_IDLE + stuck: cold re-init path (TODO v9 — print and let HB show it)

        prev_lle_cnt = lle_cnt;
    }
}

// ── Entry point ────────────────────────────────────────────────────────────────

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();

    // 96 MHz from HSE 32 MHz crystal — required for RFEND calibration.
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
                mul: hal::rcc::PllMul::MUL12,
            }),
            pllx: None,
            ahb_pre: hal::rcc::AHBPrescaler::DIV1,
            apb1_pre: hal::rcc::APBPrescaler::DIV1,
            apb2_pre: hal::rcc::APBPrescaler::DIV1,
            ls: hal::rcc::LsConfig::default_lsi(),
            hspll_src: hal::rcc::HsPllSource::HSE,
            hspll: Some(hal::rcc::HsPll {
                pre: hal::rcc::HsPllPrescaler::DIV2,
            }),
        },
        ..Default::default()
    });

    hal::println!("BLE RX IRQ harness — Phase 2 / Task #13");
    hal::println!("Embassy async + PFIC LLE(64)/BB(63) interrupt-driven scan");

    unsafe {
        // Step 1: Full BLE PHY init (handles HSE, CRC/BLEC/BLES clocks,
        //         lle_dev_init, rfend_dev_init, bb_dev_init, RF calibration).
        //         CRC clock is included — no separate enable needed (Cindy confirmed,
        //         ch32-hal src/ble/mod.rs AHBPCENR |= 0x0003_0040 = bit6+bit16+bit17).
        hal::ble::ble_phy_init();

        // Step 2: Observer-mode BB config (BLE_SetPHYRxMode dtmFlag=0 path).
        ble_set_phy_rx_mode_normal();

        // Calibration snapshot (matches ble_rx_listen.rs output for cross-check).
        let rfend90 = read_volatile(0x4002_5090 as *const u32);
        let rfend38 = read_volatile(0x4002_5038 as *const u32);
        let co_t1_0 = read_volatile(0x4002_50A0 as *const u32);
        hal::println!(
            "RFEND cal: rfend90=0x{rfend90:08x} co={} rfend38=0x{rfend38:08x} CO1[0..7]=0x{co_t1_0:08x}",
            rfend90 & 0x3F
        );

        // Step 3: Clear stale LLE IRQ bits before enabling PFIC.
        // Prevents a pre-init stale bit13/bit2 from firing the handler
        // before cold_init has set up the scan context (Cindy recommendation).
        bb_write(0x08, 0xF00F); // W1C all 8 known mask bits

        // Step 4: Enable PFIC interrupts (EVT MCU.c order: BB=63 first, LLE=64 second).
        // Both enabled after ble_phy_init + stale clear, before scan arm.
        qingke::pfic::enable_interrupt(hal::pac::Interrupt::BB as u8);  // 63
        qingke::pfic::enable_interrupt(hal::pac::Interrupt::LLE as u8); // 64

        // Step 5: Cold-boot scan arm (channel 37, 2402 MHz).
        let (logical_ch, freq_khz, _) = ADV_CHANNELS[0];
        rx_cold_init(logical_ch, freq_khz);
        SCAN_INITED.store(true, Ordering::Relaxed);

        hal::println!("PHY init done — IRQ scan active (LLE=64, BB=63)");
    }

    // Spawn the async PDU decode + stats task.
    spawner.spawn(ble_scanner_task()).unwrap();

    // Main task: periodic heartbeat to confirm Embassy executor is alive.
    // Prints LLE/BB IRQ counts and register snapshot to monitor for hangs.
    loop {
        embassy_time::Timer::after_secs(5).await;
        let lle_cnt    = unsafe { LLE_IRQ_COUNT };
        let bb_cnt     = unsafe { BB_IRQ_COUNT };
        let last_irq   = unsafe { LAST_LLE_IRQ };
        let lle_ctrl00 = unsafe { bb_read(0x00) };  // WCH gptrLLEReg+0, scan-state
        let lle64      = unsafe { bb_read(0x64) };
        let lle1c      = unsafe { bb_read(0x1C) };
        let bb_blk     = unsafe { lle_read(0x38) };
        hal::println!(
            "HB: lle_irq={lle_cnt} bb_irq={bb_cnt} last={last_irq:#010x} \
             lle_ctrl00={lle_ctrl00:#010x} lle64={lle64:#010x} lle1c={lle1c:#04x} \
             bb_blk={bb_blk:#010x}"
        );
    }
}

// panic-halt is used (imported via `panic_halt as _` above).
