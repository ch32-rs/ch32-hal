//! BLE passive-scan listener API — CH32V208WBU6.
//!
//! Provides a safe async interface over the raw IRQ-driven scan harness validated
//! in `ble_rx_irq.rs` (v8, commit 01e0850).  The caller retains ownership of the
//! two `#[interrupt]` handlers but delegates their bodies here.
//!
//! # Quick start
//!
//! ```rust,ignore
//! use ch32_hal::ble::listener::{AdvFilter, BleListenerState};
//!
//! // Declare state as a static (required — `next_pdu()` is async).
//! static LISTENER: BleListenerState = BleListenerState::new();
//!
//! // Wire up interrupt handlers in your application crate:
//! #[ch32_hal::interrupt]
//! fn LLE() { LISTENER.on_lle_irq(); }
//!
//! #[ch32_hal::interrupt]
//! fn BB()  { LISTENER.on_bb_irq(); }
//!
//! // Embassy async task:
//! #[embassy_executor::task]
//! async fn scanner() {
//!     LISTENER.start(AdvFilter::default());
//!     loop {
//!         let pdu = LISTENER.next_pdu().await;
//!         for ad in pdu.ad_iter() {
//!             if ad.ad_type == 0x01 { /* Flags */ }
//!         }
//!     }
//! }
//! ```
//!
//! # Hardware contract summary
//!
//! Based on API contract v0.3 (Lucy 2026-05-02), v8 board validation (1163 frames,
//! 0 STUCK, 30 s) and Cindy 10 m soak (21 573 frames, 0 STUCK, 618 s).
//!
//! ## Register naming (swapped vs WCH SDK — same as `ble_rx_irq.rs`)
//!
//! ```text
//! LLE_BASE = 0x40024100  (WCH: gptrBBReg)   → lle_* accessors
//! BB_BASE  = 0x40024200  (WCH: gptrLLEReg)  → bb_*  accessors
//! ```
//!
//! The "lle / bb" label swap is a historical artifact and is documented in
//! `ble_rx_irq.rs` and the API contract §1.  All accessor choices in this file
//! match those in `ble_rx_irq.rs` exactly; register offsets refer to the physical
//! block identified by the project label.

#![allow(dead_code)]

use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};

// ── Register base addresses ───────────────────────────────────────────────────

/// WCH gptrBBReg: scan state, ACCESS_ADDR, CRC_INIT, mode regs.
const LLE_BASE: usize = 0x40024100;
/// WCH gptrLLEReg: IRQ status (W1C), scan window timer, DMA ptr, ctrl.
const BB_BASE: usize = 0x40024200;
/// WCH gptrRFENDReg: PLL dividers, channel-lock control, PA/LNA bias.
const RFEND_BASE: usize = 0x40025000;

#[inline(always)]
unsafe fn lle_read(off: usize) -> u32 {
    read_volatile((LLE_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn lle_write(off: usize, v: u32) {
    write_volatile((LLE_BASE + off) as *mut u32, v);
}
#[inline(always)]
unsafe fn bb_read(off: usize) -> u32 {
    read_volatile((BB_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn bb_write(off: usize, v: u32) {
    write_volatile((BB_BASE + off) as *mut u32, v);
}
#[inline(always)]
unsafe fn rfend_read(off: usize) -> u32 {
    read_volatile((RFEND_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn rfend_write(off: usize, v: u32) {
    write_volatile((RFEND_BASE + off) as *mut u32, v);
}

// ── BLE ADV constants ─────────────────────────────────────────────────────────

const ADV_AA: u32 = 0x8E89_BED6;
const ADV_CRC_INIT: u32 = 0x55_5555;

/// BLE advertising channels: (logical_channel, freq_khz).
pub const ADV_CHANNELS: [(u8, u32); 3] = [
    (37, 2_402_000),
    (38, 2_426_000),
    (39, 2_480_000),
];

/// Per-window timer ticks written to `bb_*(0x64)`.
///
/// Value 406 = `llScanProcess` d.asm L63159 `case 0` (bits[13:12] of `lle_ctrl00` == 0).
/// See API contract §5.
const SCAN_WINDOW_TICKS: u32 = 406;

// ── Scan state constants (mirrors lib s0[10] dispatch values, §4) ─────────────

/// HW not scanning: `lle_ctrl00 == 0x07` stuck or pre-init.
const S_IDLE: u8 = 0;
/// Frame received — ready to hop (`llScanProcess .L562` target).
const S_RX_DONE: u8 = 162;
/// Timer expired without frame — hop needed (`llScanProcess .L573` target).
const S_HOP: u8 = 166;
/// Channel hop in progress: reentry guard (task sets before `rx_traverse`).
const S_TRAVERSING: u8 = 1;

// ── PFIC IRQ numbers (CH32V208WBU6) ──────────────────────────────────────────

/// PFIC IRQ number for the BB baseband interrupt.
const IRQ_BB: u8 = 63;
/// PFIC IRQ number for the LLE link-layer interrupt.
const IRQ_LLE: u8 = 64;

// ── SyncUnsafeCell ────────────────────────────────────────────────────────────

/// An `UnsafeCell<T>` that is `Sync` by contract.
///
/// # Safety
///
/// Only safe on **single-core** CH32V208 where BLE interrupt handlers are
/// **non-reentrant**.  All accesses use `read_volatile` / `write_volatile` to
/// prevent the compiler from caching values across interrupt boundaries.
struct SyncUnsafeCell<T>(UnsafeCell<T>);

// SAFETY: single-core, non-reentrant BLE ISR context.
unsafe impl<T: Send> Sync for SyncUnsafeCell<T> {}

impl<T: Copy> SyncUnsafeCell<T> {
    const fn new(v: T) -> Self {
        Self(UnsafeCell::new(v))
    }

    /// Volatile read (prevents compiler register-caching of ISR-written values).
    #[inline(always)]
    unsafe fn read(&self) -> T {
        read_volatile(self.0.get())
    }

    /// Volatile write (prevents compiler hoisting across ISR boundaries).
    #[inline(always)]
    unsafe fn write(&self, v: T) {
        write_volatile(self.0.get(), v);
    }
}

impl SyncUnsafeCell<u32> {
    /// Increment by 1 (wrapping).  ISR-exclusive — not atomic.
    #[inline(always)]
    unsafe fn inc(&self) {
        let v = read_volatile(self.0.get());
        write_volatile(self.0.get(), v.wrapping_add(1));
    }
}

// ── Public API types ──────────────────────────────────────────────────────────

/// A decoded BLE advertising PDU.
///
/// Populated from the DMA snapshot taken by the ISR at bit2 time (RX done),
/// **before** any channel hop.  HW dewhitens in scan mode — no SW LFSR needed.
///
/// # Field layout (from raw DMA buffer at offset 0)
///
/// ```text
/// buf[0]    PDU header byte 0: bits[3:0] = type, bit6 = TxAdd, bit7 = RxAdd
/// buf[1]    PDU header byte 1: bits[5:0] = total PDU length (AdvA + AD)
/// buf[2..8] AdvA — 6 bytes, LE order as received
/// buf[8..]  AD payload — buf[1]&0x3F − 6 bytes (max 31 for legacy ADV)
/// ```
#[derive(Clone, Copy)]
pub struct AdvPdu {
    /// Receive channel: 37, 38, or 39.
    pub ch: u8,

    /// PDU type field (header byte 0, bits[3:0]).
    ///
    /// **Default accepted** (`AdvFilter::accept_legacy = true`):
    /// `0`=ADV_IND, `2`=ADV_NONCONN_IND, `4`=SCAN_RSP, `6`=ADV_SCAN_IND.
    ///
    /// **Anomaly (counted, not delivered)**:
    /// `3`=SCAN_REQ — `buf[2..8]` is ScanA (scanner addr), not AdvA; semantically
    /// incompatible with the `adv_a` field, so filtered by default.
    /// `7`=ADV_EXT_IND (BLE 5.0+), `8–15` = reserved/extended.
    pub adv_type: u8,

    /// Advertiser address (AdvA), raw bytes as received (little-endian).
    ///
    /// Display as `[5]:[4]:[3]:[2]:[1]:[0]` for colon-separated MAC notation.
    pub adv_a: [u8; 6],

    /// Length of `ad` payload in bytes (`pdu_len − 6`).  Valid range: 0..=31.
    pub ad_len: u8,

    /// AD (Advertising Data) payload.  Only `ad[0..ad_len]` is valid.
    pub ad: [u8; 31],
}

impl AdvPdu {
    /// Decode a 50-byte DMA snapshot into an `AdvPdu`.
    ///
    /// Returns `Some((pdu, is_anomaly))` when `pdu_len ≥ 6` (contains a full AdvA).
    /// `is_anomaly` is true when `type ∉ {0,2,3,4,6}` OR `len > 37` (§11 filter).
    /// Returns `None` if the snapshot is too short to contain a valid AdvA.
    fn from_snapshot(snap: &[u8; 50], ch: u8) -> Option<(Self, bool)> {
        let pdu_type = snap[0] & 0x0F;
        let pdu_len = snap[1] & 0x3F;

        if pdu_len < 6 {
            return None; // Too short to contain AdvA
        }

        // §11 / Cindy review: legacy accept = {0,2,4,6} only.
        // type=3 (SCAN_REQ) has layout ScanA+AdvA — buf[2..8] is ScanA, not AdvA,
        // making the `adv_a` field semantically misleading.  Treat as anomaly.
        // type=0  ADV_IND          (connectable undirected)
        // type=2  ADV_NONCONN_IND  (non-connectable undirected)
        // type=4  SCAN_RSP         (scan response — AdvA at same offset, AD payload)
        // type=6  ADV_SCAN_IND     (scannable undirected)
        let is_anomaly = !matches!(pdu_type, 0 | 2 | 4 | 6) || pdu_len > 37;

        // AdvA: buf[2..8], stored as-received (LE order)
        let adv_a = [snap[2], snap[3], snap[4], snap[5], snap[6], snap[7]];

        // AD payload: buf[8 .. 8 + (pdu_len − 6)], capped at 31 bytes
        let ad_payload_bytes = (pdu_len - 6) as usize;
        let ad_available = (50usize.saturating_sub(8)).min(ad_payload_bytes).min(31);

        let mut ad = [0u8; 31];
        ad[..ad_available].copy_from_slice(&snap[8..8 + ad_available]);

        Some((
            Self {
                ch,
                adv_type: pdu_type,
                adv_a,
                ad_len: ad_available as u8,
                ad,
            },
            is_anomaly,
        ))
    }

    /// Iterate over AD structures in this PDU.
    ///
    /// # Example
    /// ```rust,ignore
    /// for ad in pdu.ad_iter() {
    ///     match ad.ad_type {
    ///         0x01 => { /* Flags */ }
    ///         0xFF => { /* Manufacturer specific */ }
    ///         0x09 => { /* Complete Local Name */ }
    ///         _ => {}
    ///     }
    /// }
    /// ```
    pub fn ad_iter(&self) -> AdIterator<'_> {
        AdIterator {
            data: &self.ad[..self.ad_len as usize],
            pos: 0,
        }
    }
}

/// A single entry in the Advertising Data structures.
pub struct AdEntry<'a> {
    /// AD type byte (e.g. `0x01` = Flags, `0xFF` = Manufacturer specific).
    pub ad_type: u8,
    /// AD data (excludes the length and type bytes; length = `len_field − 1`).
    pub data: &'a [u8],
}

/// Iterator over AD structures within an [`AdvPdu`].
pub struct AdIterator<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Iterator for AdIterator<'a> {
    type Item = AdEntry<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.data.len() {
            return None;
        }
        let len = self.data[self.pos] as usize;
        if len == 0 {
            return None; // zero-length AD = end sentinel
        }
        let type_pos = self.pos + 1;
        let end_pos = self.pos + 1 + len;
        if type_pos >= self.data.len() || end_pos > self.data.len() {
            return None; // truncated structure
        }
        let ad_type = self.data[type_pos];
        let data = &self.data[type_pos + 1..end_pos];
        self.pos = end_pos;
        Some(AdEntry { ad_type, data })
    }
}

/// Filter configuration for the listener.
///
/// Configured at [`BleListenerState::start`] time, read-only during scanning.
#[derive(Clone, Copy, Debug)]
pub struct AdvFilter {
    /// Accept legacy ADV PDUs (type ∈ {0,2,3,4,6} && len ≤ 37) and deliver them
    /// via [`next_pdu`](BleListenerState::next_pdu).  Default: `true`.
    pub accept_legacy: bool,

    /// Count anomaly frames (type 7–15 OR len > 37) in the diagnostic stats.
    /// Anomaly frames are **never** delivered via `next_pdu` regardless of this flag.
    /// Default: `true` (count for diagnostics).
    pub count_anomalies: bool,
}

impl Default for AdvFilter {
    fn default() -> Self {
        Self {
            accept_legacy: true,
            count_anomalies: true,
        }
    }
}

/// Diagnostic snapshot from the listener.
///
/// Obtained via [`BleListenerState::stats`].  All counters are `u32` — see §11:
/// at 34.9 frames/s a `u16` anomaly counter rolls over in ~30 min; `u32` lasts 3.9 years.
#[derive(Clone, Copy, Debug, Default)]
pub struct ListenerStats {
    /// Number of LLE IRQ fires (all IRQ bits combined).
    pub lle_irq_count: u32,
    /// Number of BB IRQ fires.
    pub bb_irq_count: u32,
    /// Number of `Signal::signal()` calls (IRQ → task wakes).
    pub sig_count: u32,
    /// Number of successful channel traversals.
    pub trav_count: u32,
    /// Number of legacy ADV frames successfully delivered via `next_pdu`.
    pub frame_count: u32,
    /// Number of anomaly frames counted (never delivered).
    pub anomaly_count: u32,
    /// Number of `lle_ctrl00 == 0x07` stuck observations (§10 failure mode 1).
    pub stuck_count: u32,
}

// ── BleListenerState ──────────────────────────────────────────────────────────

/// All static state for the BLE passive-scan listener.
///
/// Declare a single instance as a `static` and wire up the interrupt handlers:
///
/// ```rust,ignore
/// static LISTENER: BleListenerState = BleListenerState::new();
///
/// #[ch32_hal::interrupt]
/// fn LLE() { LISTENER.on_lle_irq(); }
///
/// #[ch32_hal::interrupt]
/// fn BB()  { LISTENER.on_bb_irq(); }
/// ```
///
/// All `pub` methods are safe to call; `unsafe` is encapsulated inside.
pub struct BleListenerState {
    // ── Scan state machine (ISR ↔ task) ────────────────────────────────────
    /// Simplified `s0[10]` state.  ISR writes; task reads (§4).
    our_s10: AtomicU8,
    /// Current channel index (0=ch37, 1=ch38, 2=ch39).  Task writes before
    /// traverse; ISR reads at bit2 time to label the snapshot.
    channel_idx: AtomicUsize,

    // ── IRQ → task wake (Embassy Signal) ──────────────────────────────────
    /// Carries the raw `bb_*(0x08)` IRQ bits from `on_lle_irq` to `next_pdu`.
    /// Signal coalesce semantics (latest-wins) confirmed benign over 1.6 M
    /// iterations in the 10-minute soak — no accumulating signal loss (§11).
    irq_sig: Signal<CriticalSectionRawMutex, u32>,

    // ── Frame snapshot (ISR writes, task reads — single-producer/consumer) ─
    /// DMA RX buffer.  HW writes decoded PDU bytes here.
    /// `[u32; 70]` (= 280 bytes) ensures 4-byte alignment required by HW DMA.
    rx_buf: SyncUnsafeCell<[u32; 70]>,
    /// Task-readable 50-byte snapshot captured by ISR at bit2 time, before hop.
    rx_snapshot: SyncUnsafeCell<[u8; 50]>,
    /// Channel index at the time `rx_snapshot` was taken (pre-traverse).
    snapshot_ch_idx: SyncUnsafeCell<usize>,
    /// Monotonically increasing generation counter.  ISR increments after each
    /// snapshot copy; task compares to detect new frames.
    snapshot_gen: SyncUnsafeCell<u32>,

    // ── ISR diagnostic counters (ISR-exclusive, non-atomic) ─────────────────
    lle_irq_count: SyncUnsafeCell<u32>,
    bb_irq_count: SyncUnsafeCell<u32>,
    last_lle_irq: SyncUnsafeCell<u32>,
    lle_bit0_count: SyncUnsafeCell<u32>,
    lle_bit2_count: SyncUnsafeCell<u32>,
    lle_bit13_count: SyncUnsafeCell<u32>,
    lle_other_count: SyncUnsafeCell<u32>,
    lle1c_hist_93: SyncUnsafeCell<u32>,
    lle1c_hist_108: SyncUnsafeCell<u32>,

    // ── Stats counters ────────────────────────────────────────────────────
    sig_count: SyncUnsafeCell<u32>,
    trav_count: SyncUnsafeCell<u32>,
    frame_count: SyncUnsafeCell<u32>,
    anomaly_count: SyncUnsafeCell<u32>,
    stuck_count: SyncUnsafeCell<u32>,

    // ── Filter config (set at start, read during scan) ─────────────────────
    accept_legacy: AtomicBool,
    count_anomalies: AtomicBool,
}

impl BleListenerState {
    /// Create a new zeroed listener state.  Suitable for `static` declaration.
    pub const fn new() -> Self {
        Self {
            our_s10: AtomicU8::new(S_RX_DONE),
            channel_idx: AtomicUsize::new(0),
            irq_sig: Signal::new(),
            rx_buf: SyncUnsafeCell::new([0u32; 70]),
            rx_snapshot: SyncUnsafeCell::new([0u8; 50]),
            snapshot_ch_idx: SyncUnsafeCell::new(0),
            snapshot_gen: SyncUnsafeCell::new(0),
            lle_irq_count: SyncUnsafeCell::new(0),
            bb_irq_count: SyncUnsafeCell::new(0),
            last_lle_irq: SyncUnsafeCell::new(0),
            lle_bit0_count: SyncUnsafeCell::new(0),
            lle_bit2_count: SyncUnsafeCell::new(0),
            lle_bit13_count: SyncUnsafeCell::new(0),
            lle_other_count: SyncUnsafeCell::new(0),
            lle1c_hist_93: SyncUnsafeCell::new(0),
            lle1c_hist_108: SyncUnsafeCell::new(0),
            sig_count: SyncUnsafeCell::new(0),
            trav_count: SyncUnsafeCell::new(0),
            frame_count: SyncUnsafeCell::new(0),
            anomaly_count: SyncUnsafeCell::new(0),
            stuck_count: SyncUnsafeCell::new(0),
            accept_legacy: AtomicBool::new(true),
            count_anomalies: AtomicBool::new(true),
        }
    }

    // ── Interrupt handlers ────────────────────────────────────────────────────

    /// Body of the BB interrupt handler.
    ///
    /// W1C the BB block status register (`lle_*(0x38)`) to prevent an IRQ storm
    /// that starves the LLE interrupt.  Required fix from Cindy Run A (v3).
    ///
    /// Wire up in your crate:
    /// ```rust,ignore
    /// #[ch32_hal::interrupt]
    /// fn BB() { LISTENER.on_bb_irq(); }
    /// ```
    pub fn on_bb_irq(&self) {
        unsafe {
            // W1C BB block status: required to release BB IRQ (prevents IRQ storm).
            // gptrBBReg+0x38 = LLE_BASE+0x38 in project naming.
            lle_write(0x38, lle_read(0x38));
            self.bb_irq_count.inc();
        }
    }

    /// Body of the LLE interrupt handler.
    ///
    /// Implements the passive-scan IRQ dispatch (§6 W1C contract, §4 state gate,
    /// §7 steady-state invariants).  Must be called from a `#[ch32_hal::interrupt]`
    /// context (HWSTK fast-IRQ).
    ///
    /// Wire up in your crate:
    /// ```rust,ignore
    /// #[ch32_hal::interrupt]
    /// fn LLE() { LISTENER.on_lle_irq(); }
    /// ```
    pub fn on_lle_irq(&self) {
        // §6: read ALL asserted bits then write exactly what was read (broad W1C).
        // "Write exactly the bits read, no read-modify-write to add extra bits" — doing
        // so would silently clear unrelated pending events (e.g. bb_write(0x08, 0x06)
        // clears bit1+bit2 but leaves other bits pending, causing re-entry on those bits).
        let irq = unsafe { bb_read(0x08) };
        unsafe { bb_write(0x08, irq) }; // W1C: clear exactly what we read, nothing more

        // Diagnostic counters (ISR-exclusive, non-atomic).
        unsafe {
            self.last_lle_irq.write(irq);
            self.lle_irq_count.inc();
            if irq & (1 << 0) != 0 {
                self.lle_bit0_count.inc();
            }
            if irq & (1 << 2) != 0 {
                self.lle_bit2_count.inc();
            }
            if irq & (1 << 13) != 0 {
                self.lle_bit13_count.inc();
            }
            let known = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 13) | (1 << 15);
            if irq & !known != 0 {
                self.lle_other_count.inc();
            }
        }

        // ── bit2: RX done (frame ready in DMA buffer) ─────────────────────────
        // §4: bit2 path MUST run before bit13 within the same ISR call.
        // Reason: correct state machine priority for simultaneous events.
        //   bit2-first: set S_RX_DONE → bit13 then promotes to S_HOP. Final = S_HOP ✓
        //   bit13-first: set S_HOP → bit2 sees S_HOP and skips overwrite. Final = S_HOP ✓
        // Both produce S_HOP, but the bit2-first order correctly captures
        // `snapshot_ch_idx = channel_idx` BEFORE the state transitions that wake the task
        // (the task cannot advance `channel_idx` mid-ISR, but the ordering makes intent clear).
        if irq & (1 << 2) != 0 {
            // apply_rx_state_ack: HW state-machine ACK.
            // Prevents lle_ctrl00 = 0x07 stuck state (§7 hard contract).
            // Source: lle_irq_process bit2 path, d.asm L71240.
            unsafe {
                let sw_state = lle_read(0x00); // bb_sw00 = gptrBBReg+0
                let lle1c_value: u32 = if (sw_state >> 12) & 0x3 != 0 {
                    108 // 0x6C — idle/post-RX default
                } else {
                    let lle50 = bb_read(0x50);
                    // rxflag hardcoded to 0 (passive scan default, §v4 simplification)
                    if lle50 & 0x20 == 0 {
                        93 // 0x5D — most common passive RX path (rxflag=0, lle50.bit5=0)
                    } else {
                        93 // lle50.bit5=1, rxflag=0 → same result
                    }
                };
                match lle1c_value {
                    93 => self.lle1c_hist_93.inc(),
                    _ => self.lle1c_hist_108.inc(),
                }
                bb_write(0x1C, lle1c_value); // WCH gptrLLEReg+0x1C: state-machine ACK
            }

            // Snapshot: copy DMA buffer bytes 0..50 → rx_snapshot, then bump gen.
            // Bump gen AFTER copy (prevents task from reading partial snapshot).
            unsafe {
                self.snapshot_ch_idx
                    .write(self.channel_idx.load(Ordering::Relaxed));
                let src = self.rx_buf.0.get().cast::<u8>();
                let dst = self.rx_snapshot.0.get().cast::<u8>();
                for i in 0..50usize {
                    write_volatile(dst.add(i), read_volatile(src.add(i)));
                }
                self.snapshot_gen.inc();
            }

            // §4 state transition: bit2 → S_RX_DONE, unless S_HOP (timer priority).
            let s = self.our_s10.load(Ordering::Relaxed);
            if s != S_HOP {
                self.our_s10.store(S_RX_DONE, Ordering::Relaxed);
            }
        }

        // ── bit13: scan window timer expired ─────────────────────────────────
        if irq & (1 << 13) != 0 {
            // Reload timer (§5: SCAN_WINDOW_TICKS = 406 = case 0).
            unsafe { bb_write(0x64, SCAN_WINDOW_TICKS) };

            // §4 state transition: S_RX_DONE → S_HOP (lib 162 → 166 semantics).
            // Keep S_IDLE if HW is not scanning (don't restart from ISR).
            // Keep S_TRAVERSING (task in-flight — don't overwrite reentry guard).
            let s = self.our_s10.load(Ordering::Relaxed);
            if s == S_RX_DONE {
                self.our_s10.store(S_HOP, Ordering::Relaxed);
            }
        }

        // ── End-of-ISR: stuck detection (§10 failure mode 1) ─────────────────
        // lle_ctrl00 = bb_read(0x00) = WCH gptrLLEReg+0 (scan state: 1=active, 7=stuck).
        // ISR only sets our_s10 → S_IDLE here.  stuck_count is incremented by the TASK
        // (in next_pdu / traverse_next) when it observes S_IDLE, so transient 0x07
        // reads during PLL re-tune do not spam the counter (§11 / Cindy Suggestion 1).
        let ctrl00 = unsafe { bb_read(0x00) };
        if ctrl00 == 0x07 {
            self.our_s10.store(S_IDLE, Ordering::Relaxed);
            // NOTE: stuck_count is NOT incremented here — see above comment.
        }

        // Signal task: carries raw irq bits for bit2/bit13 dispatch.
        self.irq_sig.signal(irq);
    }

    // ── Scan control ──────────────────────────────────────────────────────────

    /// Start passive scanning on all three ADV channels (ch37 → 38 → 39 → …).
    ///
    /// Runs the full `LL_ScanSetRF` cold-init sequence (§7: must restore every
    /// steady-state invariant).  Enables PFIC interrupts last (§10 ordering).
    ///
    /// **Pre-condition**: `ble_phy_init()` and `ble_set_phy_rx_mode_normal()` must
    /// have been called before `start()`.  See `ble_rx_irq.rs` `main()` for the
    /// correct init order.
    pub fn start(&self, filter: AdvFilter) {
        // Store filter config
        self.accept_legacy
            .store(filter.accept_legacy, Ordering::Relaxed);
        self.count_anomalies
            .store(filter.count_anomalies, Ordering::Relaxed);

        // §10 / §4: SW state reset BEFORE HW cold_init.
        // First ISR after cold_init lands in S_RX_DONE → first task iteration traverses.
        self.our_s10.store(S_RX_DONE, Ordering::Relaxed);
        self.channel_idx.store(0, Ordering::Relaxed);

        // Full cold-init (§7: re-runs entire LL_ScanSetRF sequence).
        let (ch, freq_khz) = ADV_CHANNELS[0];
        unsafe { self.rx_cold_init(ch, freq_khz) };

        // §10 start(), last step: enable PFIC interrupts AFTER HW is fully ready.
        // BB first → LLE second (EVT MCU.c:158-159 order).
        unsafe {
            qingke::pfic::enable_interrupt(IRQ_BB);
            qingke::pfic::enable_interrupt(IRQ_LLE);
        }
    }

    /// Stop passive scanning.
    ///
    /// Follows the 5-step stop sequence from §10 v0.3.  Safe to call from any
    /// context; spins briefly if a channel hop is in progress (step 2).
    pub fn stop(&self) {
        // Step 1: disable PFIC LLE/BB IRQs — prevents new ISRs from firing.
        unsafe {
            qingke::pfic::disable_interrupt(IRQ_LLE);
            qingke::pfic::disable_interrupt(IRQ_BB);
        }

        // Step 2: wait for any in-flight traverse to complete.
        // S_TRAVERSING is set by the task before the 2-write .L546 sequence and
        // cleared immediately after.  Spin here is bounded — the 2 writes are fast.
        while self.our_s10.load(Ordering::Relaxed) == S_TRAVERSING {
            core::hint::spin_loop();
        }

        // Step 3: stop scan window timer (bb_*(0x64) = 0).
        unsafe { bb_write(0x64, 0) };

        // Step 4: reset SW state so the next start() resumes cleanly.
        self.our_s10.store(S_RX_DONE, Ordering::Relaxed);

        // Step 5: PFIC re-enable is in start(), not here (§10 ordering).
        // See start() last two lines for the BB→LLE re-enable sequence.
    }

    // ── Async PDU reception ───────────────────────────────────────────────────

    /// Wait for the next decoded legacy ADV PDU.
    ///
    /// On every LLE wake this method:
    /// 1. Calls [`traverse_next`](Self::traverse_next) — a no-op unless
    ///    `our_s10 ∈ {S_RX_DONE, S_HOP}`, so both bit13-timer and post-bit2 hops
    ///    are handled without the caller needing a separate traverse loop.
    /// 2. Checks for stuck state (`our_s10 == S_IDLE`) and increments `stuck_count`
    ///    (task-side debounce per §11 / Cindy Suggestion 1).
    /// 3. Decodes the latest DMA snapshot when `snapshot_gen` has advanced.
    ///
    /// Returns the first legacy ADV PDU (type ∈ {0,2,4,6} && len ≤ 37) that passes
    /// the configured `AdvFilter`.  Anomaly frames are counted and discarded.
    ///
    /// # Snapshot decode gate: `snapshot_gen`, not `irq & bit2`
    ///
    /// Embassy `Signal` is latest-value: if the ISR fires bit2 (commits snapshot)
    /// then bit13 (overwrites the signal payload) before the task resumes, the task
    /// wakes with an `irq` that has **no bit2 set** — but `snapshot_gen` has already
    /// advanced.  Gating decode on `irq & bit2` would silently drop that frame.
    ///
    /// The correct gate is `snapshot_gen != prev_gen`: decode whenever the ISR has
    /// committed a new snapshot, regardless of which IRQ bits are in the current
    /// signal value.  `irq` is retained only for diagnostic counters.
    ///
    /// # Requirements
    ///
    /// * `self` must be `'static` (declared as a `static`) so the future can be
    ///   spawned by `embassy_executor`.
    /// * `start()` must have been called before `next_pdu()`.
    pub async fn next_pdu(&self) -> AdvPdu {
        // Initialize prev_gen from the current generation counter so we don't
        // re-deliver a frame that arrived before this call.
        let mut prev_gen: u32 = unsafe { self.snapshot_gen.read() };

        loop {
            // Wait for the ISR to signal any LLE event.
            // irq is kept for diagnostic increments only; NOT used as decode gate.
            let _irq = self.irq_sig.wait().await;
            unsafe { self.sig_count.inc() };

            // ── Step 1: always attempt traverse ──────────────────────────────
            // Handles bit13 (timer hop) and post-bit2 (frame-received hop).
            // traverse_next() is a no-op if our_s10 ∉ {S_RX_DONE, S_HOP}.
            self.traverse_next();

            // ── Step 2: stuck detection (task-side debounce) ─────────────────
            // ISR sets S_IDLE on ctrl00==0x07.  We count here to avoid inflating
            // stuck_count on transient 0x07 PLL re-tune glitches (§11 / §10).
            if self.our_s10.load(Ordering::Relaxed) == S_IDLE {
                unsafe { self.stuck_count.inc() };
                // Application layer owns watchdog / restart decision (§10 failure mode 1).
                continue;
            }

            // ── Step 3: decode if ISR has committed a new snapshot ────────────
            // Gate on snapshot_gen, NOT on irq bits (see "Snapshot decode gate" above).
            let gen = unsafe { self.snapshot_gen.read() };
            if gen == prev_gen {
                continue; // No new snapshot this wake.
            }
            prev_gen = gen;

            // Snapshot channel captured pre-traverse by ISR at bit2 time.
            let ch_idx = unsafe { self.snapshot_ch_idx.read() };
            let ch = ADV_CHANNELS[ch_idx].0;

            let mut snap = [0u8; 50];
            let src = self.rx_snapshot.0.get().cast::<u8>();
            for i in 0..50usize {
                snap[i] = unsafe { read_volatile(src.add(i)) };
            }

            // Decode and filter.
            if let Some((pdu, is_anomaly)) = AdvPdu::from_snapshot(&snap, ch) {
                if is_anomaly {
                    // §11: anomaly frames never delivered, optionally counted.
                    if self.count_anomalies.load(Ordering::Relaxed) {
                        unsafe { self.anomaly_count.inc() };
                    }
                    continue;
                }
                if self.accept_legacy.load(Ordering::Relaxed) {
                    unsafe { self.frame_count.inc() };
                    return pdu;
                }
            }
            // Too short or filtered — wait for next signal.
        }
    }

    /// Perform a single channel hop using the passive `.L546` 2-write path.
    ///
    /// Must be called from the Embassy task (not from the ISR — writing RFEND
    /// registers in HWSTK fast-IRQ context caused 0x07 stuck in v6).
    ///
    /// Sets `OUR_S10 = S_TRAVERSING` before the 2-write sequence and
    /// `S_RX_DONE` after, as required by the §10 stop sequence.
    ///
    /// Returns the channel label string for logging (e.g. `"ch38/2426MHz"`).
    pub fn traverse_next(&self) -> Option<u8> {
        let s = self.our_s10.load(Ordering::Relaxed);
        if s != S_RX_DONE && s != S_HOP {
            return None; // §4 gate: only traverse in {S_RX_DONE, S_HOP}
        }

        self.our_s10.store(S_TRAVERSING, Ordering::Relaxed);

        // Advance channel index BEFORE the traverse so the next bit2 labels the
        // snapshot with the new channel (ISR reads channel_idx at bit2 time).
        let new_idx = self
            .channel_idx
            .load(Ordering::Relaxed)
            .wrapping_add(1)
            % 3;
        self.channel_idx.store(new_idx, Ordering::Relaxed);
        let (logical_ch, _) = ADV_CHANNELS[new_idx];

        unsafe { self.rx_traverse(logical_ch) };
        unsafe { self.trav_count.inc() };

        self.our_s10.store(S_RX_DONE, Ordering::Relaxed);
        Some(logical_ch)
    }

    // ── Diagnostics ──────────────────────────────────────────────────────────

    /// Return a diagnostic stats snapshot.
    ///
    /// Safe to call from any Embassy task context between `next_pdu` calls.
    pub fn stats(&self) -> ListenerStats {
        ListenerStats {
            lle_irq_count: unsafe { self.lle_irq_count.read() },
            bb_irq_count: unsafe { self.bb_irq_count.read() },
            sig_count: unsafe { self.sig_count.read() },
            trav_count: unsafe { self.trav_count.read() },
            frame_count: unsafe { self.frame_count.read() },
            anomaly_count: unsafe { self.anomaly_count.read() },
            stuck_count: unsafe { self.stuck_count.read() },
        }
    }

    // ── Private HW helpers ────────────────────────────────────────────────────
}

// ── Public standalone HW helpers ─────────────────────────────────────────────

/// Configure BB/LLE for normal-mode BLE reception (Observer / passive scanner).
///
/// Transcription of `BLE_SetPHYRxMode()` `.L94` branch (`dtmFlag=0`), decoded by
/// Lucy 2026-05-01 from libwchble.a.  Must be called **after** [`crate::ble::ble_phy_init`]
/// and **before** [`BleListenerState::start`].
///
/// # Safety
///
/// Must be called after `ble_phy_init()`.  Caller must ensure no concurrent BLE
/// register access (single-core CH32V208: call from init context before IRQs are enabled).
pub unsafe fn ble_set_phy_rx_mode_normal() {
    lle_write(0x20, 0x0009_0083);
    lle_write(0x14, 0x0810_1901);
    lle_write(0x18, 0x0003_1624);
    lle_write(0x28, 0x0000_28BE);
    lle_write(0x24, 0x0100_6310);
    lle_write(0x10, 0x0032_22D0);
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x3000) | 0x1000);
}

impl BleListenerState {
    /// Program the RFEND PLL to `freq_khz` and assert channel-lock (RFEND+0x2C bit1=1).
    ///
    /// Used during cold-init's manual-lock phase only.  After cold-init the PLL is
    /// released to channel-tracking mode — do NOT call this per-hop (§2 hard contract).
    unsafe fn set_channel_freq(&self, freq_khz: u32) {
        let int_div = (freq_khz / 64_000) & 0x1F;
        let frac_div = ((freq_khz % 64_000) << 10) / 250;
        let pll = rfend_read(0x44);
        rfend_write(
            0x44,
            (pll & 0xFE0F_C000) | (int_div << 20) | (frac_div & 0x3FFF),
        );
        let v = rfend_read(0x2C);
        rfend_write(0x2C, v | (1 << 1));
    }

    /// Passive channel hop — `.L546` 2-write path (§3).
    ///
    /// Transcription of `<llScanTraverseaChannel>` d.asm L62961 offset 0x10e–0x136.
    /// Works because cold-init left RFEND+0x2C bit1=0 (channel-tracking mode): HW
    /// auto-maps `lle_*(0x00)[5:0]` → RF frequency; no RFEND+0x44 write needed.
    ///
    /// Must be called from task context, NOT from the LLE ISR (RFEND writes in
    /// HWSTK fast-IRQ cause 0x07 stuck — confirmed v6 board 2026-05-02).
    unsafe fn rx_traverse(&self, channel: u8) {
        // Step 1: PLL lock release pulse → triggers channel-tracking mode re-tune.
        let v = rfend_read(0x2C);
        rfend_write(0x2C, v & !0x02);

        // Step 2: channel field write → HW auto-maps bits[5:0] → RF frequency.
        // lle_*(0x00) = WCH gptrBBReg+0 (scan-active state register).
        // Mask ~0x7F preserves bits[31:7]; channel masked to 0x3F per lib.
        let c = lle_read(0x00);
        lle_write(0x00, (c & !0x7F) | (channel as u32 & 0x3F));
    }

    /// Cold-boot RX init: full `LL_ScanSetRF` equivalent + RX GO trigger.
    ///
    /// Restores all §7 steady-state invariants.  Must be called exactly once
    /// per scan session (or after `stop()` to restart).  Steps match d.asm L62824.
    unsafe fn rx_cold_init(&self, logical_ch: u8, freq_khz: u32) {
        // Initial frequency + manual PLL lock (will be released at step 6).
        self.set_channel_freq(freq_khz);

        bb_write(0x64, 0); // step 1: stop scan window timer
        bb_write(0x08, 0xF00F); // step 2: W1C stale IRQ bits (cold-boot clear)

        lle_write(0x08, ADV_AA); // step 3a: ACCESS_ADDR = BLE adv AA
        lle_write(0x04, ADV_CRC_INIT); // step 3b: CRC seed

        let v = bb_read(0x04);
        bb_write(0x04, v | 0x1); // step 4: scan mode enable (LLE+0x04 bit0)

        // step 5: DMA buffer pointer — rx_buf is a [u32;70] field, always 4B aligned.
        let rx_buf_addr = self.rx_buf.0.get() as u32;
        bb_write(0x74, rx_buf_addr);

        // step 6: release PLL lock → channel-tracking mode
        let v = rfend_read(0x2C);
        rfend_write(0x2C, v & !(1 << 1));

        // step 7: channel field (before bit8 mode marker)
        let ctrl = lle_read(0x00);
        lle_write(0x00, (ctrl & !0x7F) | (logical_ch as u32 & 0x3F));

        // step 8: bits[8:7] = 10b (RX-mode marker, written AFTER channel)
        let ctrl = lle_read(0x00);
        lle_write(0x00, (ctrl & !0x180) | 0x100);

        // step 9: RFEND PA/LNA/RX path enable
        let ana = rfend_read(0x08);
        rfend_write(0x08, ana | 0x0033_0000);

        bb_write(0x50, 89); // step 10: PRE_DELAY = 89
        bb_write(0x64, SCAN_WINDOW_TICKS); // load scan window timer (406 ticks)

        bb_write(0x00, 1); // step 11: RX GO — pulls HW into active scan (LAST write)
    }
}
