//! BLE RX passive scanner for CH32V208 — ship baseline (2026-05-02).
//!
//! Rotates through ADV channels 37/38/39 (2402/2426/2480 MHz) listening for BLE
//! advertising packets and decodes AdvA, Flags, Manufacturer Specific, and Name ADs.
//!
//! Proven: three-channel passive scan, raw PDU at DMA offset 0, HW dewhitens in scan
//! mode (no SW dewhiten needed), real Flags/Mfr decodes (Apple 0x004c, MS 0x0006,
//! Samsung 0x0075). See `docs/ble_rx_development.md` for full development history.
//!
//! # Register name convention (intentionally swapped vs WCH lib naming)
//!
//! ```text
//! LLE_BASE = 0x40024100  (WCH: gptrBBReg)   → accessed via lle_read/lle_write
//! BB_BASE  = 0x40024200  (WCH: gptrLLEReg)  → accessed via bb_read/bb_write
//! ```
//! This swap is kept for consistency with the d.asm decode workflow.
//!
//! # RX architecture: dual-track (cold boot + per-window)
//!
//! Cold boot (`LL_ScanSetRF`, L62824): full register init + RX GO (`bb_write(0x00, 1)`).
//! Run once to pull HW into active scan.
//!
//! Per-window (`llScanProcess .L562` → `llScanTraverseaChannel`, L62961):
//! Timer reload BEFORE channel traverse. W1C bit13, bit7→bit8 transitions in LLE+0x00,
//! PRE_DELAY + RFEND PA reset, channel update. No RX GO — HW is already live.
//!
//! **Critical**: lle00, lle74, lle04 are HW-managed. Writing them causes regression.
//!
//! # Expected output
//!
//! ```text
//! RX#1 ch37/2402MHz: type=0 len=23 AdvA=28:af:42:30:5c:65 [Flags=0x1a,Mfr=0x004c]
//! ```
//!
//! # If no packets appear
//!
//! Check: antenna connected, PHY calibration shows CO non-zero, BLE environment active.

#![no_std]
#![no_main]

use core::ptr::{addr_of, read_volatile, write_volatile};
use core::sync::atomic::{AtomicBool, Ordering};
use {ch32_hal as hal, panic_halt as _};

/// Guards cold-boot init — only LL_ScanSetRF runs once; subsequent windows use traverse.
static SCAN_INITED: AtomicBool = AtomicBool::new(false);

// ── Register base addresses (swapped WCH naming — see module doc) ────────────

/// WCH gptrBBReg: CTRL/GO, BB_CFG, ACCESS_ADDR, CRC_INIT.
const LLE_BASE: usize = 0x40024100;
/// WCH gptrLLEReg: timing, IRQ, TX timer, DMA buffer pointer.
const BB_BASE: usize = 0x40024200;
/// WCH gptrRFENDReg: PLL dividers, channel lock, PA bias.
const RFEND_BASE: usize = 0x40025000;

// ── Register accessors ────────────────────────────────────────────────────────

#[inline(always)]
unsafe fn lle_read(off: usize) -> u32  { read_volatile((LLE_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn lle_write(off: usize, v: u32) { write_volatile((LLE_BASE + off) as *mut u32, v); }
#[inline(always)]
unsafe fn bb_read(off: usize) -> u32   { read_volatile((BB_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn bb_write(off: usize, v: u32) { write_volatile((BB_BASE + off) as *mut u32, v); }
#[inline(always)]
unsafe fn rfend_read(off: usize) -> u32  { read_volatile((RFEND_BASE + off) as *const u32) }
#[inline(always)]
unsafe fn rfend_write(off: usize, v: u32) { write_volatile((RFEND_BASE + off) as *mut u32, v); }

// ── BLE software dewhitening ──────────────────────────────────────────────────

/// Dewhiten a BLE PDU in-place using the spec LFSR (x^7 + x^4 + 1).
///
/// **NOT needed in scan mode**: CH32V208 HW dewhitens automatically before DMA write
/// (confirmed 2026-05-01; raw PDU at DMA offset 0 is already dewhitened).
/// Kept as reference implementation and for potential future use (e.g. promiscuous modes
/// where HW dewhiten may not apply).
///
/// Seed = `channel | 0x40`. Right-shift Fibonacci LFSR, output from bit 0.
/// Feedback = bit0 XOR bit4. BLE bits are LSB-first on-air.
#[allow(dead_code)]
fn dewhiten_inplace(data: &mut [u8], channel: u8) {
    let mut lfsr = channel | 0x40; // 7-bit LFSR seed (bit6 always 1)
    for byte in data.iter_mut() {
        let mut dw = 0u8;
        for bit in 0..8u8 {
            let out = lfsr & 1;
            let feedback = (lfsr ^ (lfsr >> 4)) & 1; // bit0 XOR bit4
            lfsr = (lfsr >> 1) | (feedback << 6);
            let data_bit = (*byte >> bit) & 1;
            dw |= (data_bit ^ out) << bit;
        }
        *byte = dw;
    }
}

// ── BLE ADV constants ─────────────────────────────────────────────────────────

/// BLE advertising access address (fixed by spec, all ADV channels).
const ADV_AA: u32 = 0x8E89_BED6;
/// BLE advertising CRC init (fixed by spec).
const ADV_CRC_INIT: u32 = 0x55_5555;

/// ADV channels: (logical_ch, freq_khz, label).
const ADV_CHANNELS: [(u8, u32, &str); 3] = [
    (37, 2_402_000, "ch37/2402MHz"),
    (38, 2_426_000, "ch38/2426MHz"),
    (39, 2_480_000, "ch39/2480MHz"),
];

/// HW scan-window countdown written to LLE+0x64 before each RX window.
///
/// HW counts down from SCAN_WINDOW; when it hits 0 the window is "done"
/// (frame received OR timeout). Used as the primary poll-exit signal (Plan A2).
///
/// Critical (Lucy, patch #3.6): do NOT pair with W1C of irq08 bit1/bit2.
/// bit1/bit2 are sticky "RX-pending" state indicators — NOT per-frame triggers.
/// In WCH's IRQ model, W1C is done only as part of IRQ dispatch, not standalone.
/// Clearing them in polling mode (patch #3.5) tells HW "all RX consumed, idle"
/// → full SLEEP (SCN dropped 2575→21). Leaving them set keeps state machine active.
///
/// At ~30 cycles/iter × 400_000 / 96 MHz ≈ 125 ms per channel (3 ch ≈ 375 ms/rot).
const SCAN_WINDOW: u32 = 400_000;

// ── RX DMA buffer ─────────────────────────────────────────────────────────────

/// DMA RX buffer: PDU header (2B) + AdvA (6B) + payload (≤31B) + CRC (3B) + margin.
/// Pre-filled to 0; `buf[1]` becoming non-zero signals a completed DMA write.
#[link_section = ".bss"]
static mut RX_BUF: [u8; 280] = [0u8; 280];

// ── PLL channel programming ───────────────────────────────────────────────────

/// Program the RFEND PLL to `freq_khz` and assert channel-lock (RFEND+0x2C bit1=1).
///
/// From RF_DevSetChannel() in libwchble.a V1.40 (same as used in ble_dtm_tx.rs):
///   int_div  = (freq_khz / 64000) & 0x1F       → RFEND+0x44 bits\[24:20\]
///   frac_div = (freq_khz % 64000) * 1024 / 250  → RFEND+0x44 bits\[13:0\]
unsafe fn set_channel_freq(freq_khz: u32) {
    let int_div  = (freq_khz / 64_000) & 0x1F;
    let frac_div = ((freq_khz % 64_000) << 10) / 250;
    let pll = rfend_read(0x44);
    rfend_write(0x44, (pll & 0xFE0F_C000) | (int_div << 20) | (frac_div & 0x3FFF));
    // Assert channel lock (PLL latches the new dividers).
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v | (1 << 1));
}

// ── BLE PHY RX mode setup ─────────────────────────────────────────────────────

/// Configure BB for normal-mode BLE reception (observer / scanner).
///
/// Derived from `BLE_SetPHYRxMode()` in libwchble.a V1.40, `.L94` branch
/// (`dtmFlag bit1=0`, offset 0xae–0x118), decoded by Lucy 2026-05-01.
///
/// Call once after `ble_phy_init()`, before the scan loop.
/// These are static mode-setup registers — not per-channel.
///
/// # Register mapping (Lucy's standard naming → this file's swapped naming)
/// Lucy's `bb_write(reg, val)` = `lle_write(reg, val)` here (gptrBBReg = LLE_BASE).
unsafe fn ble_set_phy_rx_mode_normal() {
    lle_write(0x20, 0x0009_0083); // PHY mode (DTM mode uses 0x90086)
    lle_write(0x14, 0x0810_1901); // RX correlation threshold #1 (DTM: 0x8301FF1)
    lle_write(0x18, 0x0003_1624); // RX correlation threshold #2 (DTM: 0x31619)
    lle_write(0x28, 0x0000_28BE); // RX symbol detect (DTM: 0x28DE)
    lle_write(0x24, 0x0100_6310); // RX equalizer (same in DTM)
    lle_write(0x10, 0x0032_22D0); // RX matched filter coeffs, dtmFlag=0 path

    // BB+0x00 bits[13:12] = 01b (RX enable; LL_ReceiverTest a0!=1 path).
    // asm: "a4 &= 0xFFFFCFFF; a4 |= 0x1000"
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x3000) | 0x1000);
}

// ── RX arm / GO — dual-track: cold boot vs per-window ────────────────────────

/// Per-window timer reload: canonical `llScanProcess .L562` sequence. Call before traverse.
///
/// d.asm sequence (L63159, patch #3.10, Lucy 2026-05-01):
///   50: lw a3, 12(a2)           # a3 = lle0C (= 0xF00F)
///   52: and a5, a5, a3          # a5 = ~0x2000 & 0xF00F = 0xD00F
///   54: sw a5, 12(a2)           # lle0C = 0xD00F  — mask bit13 IRQ temporarily
///   5E: sw a4, 8(a5)            # W1C bit13 (lle08 = 0x2000)
///   82: sw a4, 100(a5)          # lle64 = 406  — reload window timer
///   84: sw a3, 12(a5)           # lle0C = 0xF00F  — restore IRQ mask
///
/// Masking bit13 before the reload prevents a spurious bit13 IRQ from firing during
/// the timer reload window. 406 (0x196) is the d.asm value; 0x107a was HW mid-countdown.
///
/// Canonical order: .L562 (timer reload) → llScanTraverseaChannel (channel switch).
/// `rx_arm` calls this BEFORE `rx_traverse` to match that ordering.
unsafe fn rx_window_timer_reload() {
    let lle0c_orig = bb_read(0x0C);           // should be 0xF00F (LLE_DevInit default)
    bb_write(0x0C, lle0c_orig & !0x2000);     // 0xD00F: temporarily mask bit13 IRQ
    bb_write(0x08, 0x2000);                   // W1C bit13 (canonical .L562 pre-reload ack)
    bb_write(0x64, 406);                      // 0x196: reload per-window timer
    bb_write(0x0C, lle0c_orig);               // 0xF00F: restore IRQ mask
}

/// Arm the BLE receiver: cold-boot on first call, per-window traverse on subsequent calls.
///
/// Architecture (patch #3.4, Lucy d.asm, 2026-05-01):
///
/// Cold boot (`LL_ScanSetRF`, L62824): full register init + RX GO.
///   Run once to pull HW from init state into active RX.
///
/// Per-window — canonical two-step (Plan C, patch #3.10/#3.13):
///   Step 1: `rx_window_timer_reload()` — equivalent to `llScanProcess .L562`.
///     Reload lle64=406 with lle0C mask/restore BEFORE channel switch.
///   Step 2: `rx_traverse()` — equivalent to `llScanTraverseaChannel`.
///     W1C bit13 + bit7/bit8 transitions + channel update. No RX GO.
///   No HW-managed register writes: lle00/lle74 are fully HW-owned (#3.11/#3.12 lesson).
///
/// Root cause of #3.3 failure:
///   LL_ScanSetRF every window over-reset the state machine. HW entered SLEEP (state=0x6c)
///   after the first real timeout and ignored all subsequent GO commands.
unsafe fn rx_arm(logical_ch: u8, freq_khz: u32) {
    if !SCAN_INITED.load(Ordering::Relaxed) {
        SCAN_INITED.store(true, Ordering::Relaxed);
        rx_cold_init(logical_ch, freq_khz);
    } else {
        // Per-window canonical two-step (Plan C, patch #3.10):
        //   Step 1: timer reload (.L562) → Step 2: channel traverse (.L544).
        // No HW-managed register writes (lle00/lle74 are HW-only — #3.11/#3.12 lesson).
        rx_window_timer_reload(); // .L562: mask 0x0C + W1C bit13 + lle64=406 + restore
        rx_traverse(logical_ch, freq_khz);
    }
}

/// Cold-boot RX init: full `LL_ScanSetRF` (L62824) + RX GO. Call once.
unsafe fn rx_cold_init(logical_ch: u8, freq_khz: u32) {
    // Pre: program PLL to target channel frequency (sets RFEND+0x2C bit1=1 = channel lock).
    set_channel_freq(freq_khz);

    // LL_ScanSetRF step 1: LLE+0x64 = 0.
    bb_write(0x64, 0);

    // LL_ScanSetRF step 2: LLE+0x08 W1C — clear stale IRQ bits.
    // Use 0xF00F (lle_dev_init mask: bits 0-3 + 12-15) for cold-boot clearing.
    bb_write(0x08, 0xF00F);

    // LL_ScanSetRF step 3: BB+0x08 = ADV AA, BB+0x04 = CRC seed.
    lle_write(0x08, ADV_AA);
    lle_write(0x04, ADV_CRC_INIT);

    // LL_ScanSetRF step 4: LLE+0x04 |= 1 (scan mode enable; DTM ReceiverTest CLEARS it).
    let v = bb_read(0x04);
    bb_write(0x04, v | 0x1);

    // LL_ScanSetRF step 5: DMA buffer pointer (LLE+0x74 = MEMAddr = &RX_BUF).
    bb_write(0x74, addr_of!(RX_BUF) as u32);

    // LL_ScanSetRF step 6: RFEND+0x2C bit1 = 0 (release PLL lock — PLL holds freq).
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !(1 << 1));

    // LL_ScanSetRF step 7: BB+0x00 bits[5:0] = logical_ch. Channel BEFORE bit8.
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | (logical_ch as u32 & 0x3F));

    // LL_ScanSetRF step 8: BB+0x00 bits[8:7] = 10b (RX-mode marker, AFTER channel).
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x180) | 0x100);

    // LL_ScanSetRF step 9: RFEND+0x08 |= 0x330000 (PA/LNA/RX path enable).
    let ana = rfend_read(0x08);
    rfend_write(0x08, ana | 0x0033_0000);

    // LL_ScanSetRF step 10: LLE+0x50 = 89 (PRE_DELAY).
    bb_write(0x50, 89);

    // Write scan-window countdown before GO (polling-model addition).
    bb_write(0x64, SCAN_WINDOW);

    // LL_ScanSetRF step 11: RX GO — always the last write. Pulls HW into active RX.
    bb_write(0x00, 1);
}

/// Per-window RX advance: `llScanTraverseaChannel` style (L62961). No full re-init.
///
/// Channel state-machine transition only — timer reload is NOT done here.
/// `rx_arm` calls `rx_window_timer_reload()` BEFORE this function, matching the
/// canonical `llScanProcess .L562` → `llScanTraverseaChannel` ordering (Plan C, #3.10).
///
/// Does NOT write RX GO (bit0=1) — HW is already live from the cold-boot GO.
unsafe fn rx_traverse(logical_ch: u8, freq_khz: u32) {
    // Update RFEND PLL to new channel frequency.
    set_channel_freq(freq_khz);

    // llScanTraverseaChannel: W1C bit13 only (not full 0xF00F).
    // The IRQ handler normally W1Cs bit13 (scan-window-done event) per window.
    // We have no handler, so do it manually. Bits 1/2 (RX-done) are NOT cleared here
    // — they will be set naturally when HW completes a reception.
    bb_write(0x08, 0x2000);

    // State-machine transition step 1: set bit7 in LLE+0x00.
    // llScanTraverseaChannel clears bits[8:7] then sets bit7 first (.L544 step 1).
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x180) | 0x80);

    // State-machine transition step 2: set bit8 (second transition after bit7).
    // (.L544 step 4: clear bits[8:7] again, set bit8)
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x180) | 0x100);

    // PRE_DELAY = 89 and RFEND PA reset (clear RFEND+0x2C bit1).
    bb_write(0x50, 89);
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !0x02);

    // Final write: set new channel bits[5:0], preserve bit7+bit8 from above.
    // (.L544 end: andi a4,-128; or a4,a4,channel; sw a4,0(a5))
    // — keeps bits[8:7] from the state transitions, replaces bits[6:0] with channel.
    let c = bb_read(0x00);
    bb_write(0x00, (c & !0x7F) | (logical_ch as u32 & 0x3F));
    // No RX GO write: HW is already in active scan from cold-boot.
    // No LLE+0x1C write: 0x6c is EVT's normal idle/post-RX state (EVT SDI confirmed).
    // No timer reload: done before this function in rx_window_timer_reload() (Plan C).
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    // 96 MHz from HSE 32 MHz crystal (same config as ble_dtm_tx.rs).
    // Required: RFEND calibration PLL fails at default 8 MHz HSI.
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

    hal::println!("BLE RX scanner — CH32V208 bedrock test");
    hal::println!("Passive scan: ADV ch37/38/39 — AdvA + Flags + Mfr + Name");

    unsafe {
        let rcc_ctlr  = read_volatile(0x4002_1000 as *const u32);
        let rcc_cfgr0 = read_volatile(0x4002_1004 as *const u32);
        hal::println!("RCC: ctlr={:#010x} cfgr0={:#010x} hserdy={} pllrdy={} sws={}",
            rcc_ctlr, rcc_cfgr0,
            (rcc_ctlr >> 17) & 1, (rcc_ctlr >> 25) & 1, (rcc_cfgr0 >> 2) & 3);

        hal::ble::ble_phy_init();

        // Configure BB for normal-mode BLE scanning (BLE_SetPHYRxMode, dtmFlag=0 path).
        // Must be called after ble_phy_init() — overwrites some bb_dev_init defaults.
        ble_set_phy_rx_mode_normal();

        // Calibration result dump (same as ble_dtm_tx.rs).
        let rfend90  = read_volatile(0x4002_5090 as *const u32);
        let rfend38  = read_volatile(0x4002_5038 as *const u32);
        let co_t1_0  = read_volatile(0x4002_50A0 as *const u32);
        hal::println!("RFEND cal: rfend90=0x{rfend90:08x} co={} rfend38=0x{rfend38:08x} CO1[0..7]=0x{co_t1_0:08x}",
            rfend90 & 0x3F);

        hal::println!("PHY init done — entering scan loop");

        let mut rx_count  = 0u32;
        let mut scan_count = 0u32;
        let mut ch_idx    = 0usize;
        let mut struct_ok_count = 0u32;   // frames passing structural PDU gate

        loop {
            let (logical_ch, freq_khz, ch_label) = ADV_CHANNELS[ch_idx];

            // Clear RX buffer, pre-clear BB IRQ (W1C), then arm the receiver.
            core::ptr::write_bytes(RX_BUF.as_mut_ptr(), 0, RX_BUF.len());
            lle_write(0x38, 0xFFFF_FFFF); // clear stale BB IRQ events before new window
            rx_arm(logical_ch, freq_khz);
            scan_count += 1;

            // Poll until frame arrives OR HW scan window expires (patch #3.13).
            //
            // Two exit conditions:
            //
            //   1. RX_BUF[50] != 0 (wtrl position): DMA trailer written — packet received.
            //      WCH writes 0x80 at RX_BUF[RX_BUF[1]] as the LAST DMA write after a full
            //      packet. RX_BUF[50] is the trailer position for typical ADV_IND.
            //
            //   2. LLE+0x08 bit13 set: HW scan-window timer expired (lle64 → 0).
            //      Root cause of SCN=20 ceiling (Lucy hypothesis, patch #3.13):
            //        • lle64=406 loaded by rx_window_timer_reload(); HW counts down to 0.
            //        • When lle64=0, HW sets bit13 (scan-window-timeout IRQ).
            //        • Previous loop had NO bit13 exit — loop spun full 2M iters (~625ms).
            //        • W1C of bit13 + next reload happened ~625ms LATE each window.
            //        • After ~20 windows the accumulated latency stalls HW permanently.
            //        • Fix: exit promptly on bit13 → reload issued within a few µs of
            //          window expiry → HW can restart scan window on schedule.
            //
            // NOTE: bit13 may already be set stale from the prior window. The per-window W1C
            // block at the bottom handles the clear. rx_window_timer_reload() and rx_traverse()
            // both W1C bit13 before poll start. The poll exits on a FRESH bit13 assertion fired
            // during this window's lle64 countdown, not on stale events (they were already W1C'd).
            //
            // We do NOT poll irq08 bit1/bit2 (patch #3.5 lesson, Lucy 2026-05-01):
            //   bit1/bit2 are sticky "RX-pending" state indicators, NOT per-frame triggers.
            //   W1C'ing them signals "all consumed" → HW enters full SLEEP.
            for _ in 0..2_000_000u32 {
                if read_volatile(addr_of!(RX_BUF).cast::<u8>().add(50)) != 0 { break; } // packet
                if bb_read(0x08) & 0x2000 != 0 { break; }  // bit13: HW scan-window timeout
                core::hint::spin_loop();
            }
            // After poll: RX_BUF[1] = HW DMA length byte; non-zero = DMA actually wrote data.
            // Checking here (not mid-loop) avoids re-arming while DMA is in flight (#3.2 lesson).
            let got_frame = read_volatile(addr_of!(RX_BUF).cast::<u8>().add(1)) != 0;

            if got_frame {

                rx_count += 1;

                // W1C BB IRQ status register (separate from LLE IRQ handled in W1C block below).
                lle_write(0x38, lle_read(0x38));

                // Snapshot 50 raw bytes from DMA buffer (covers PDU header + AdvA + payload).
                // Read AFTER got_frame check so DMA is complete. pdu_off=0: HW dewhitens in
                // scan mode — raw PDU starts at DMA buffer byte 0 (confirmed 2026-05-01).
                let mut raw50 = [0u8; 50];
                for i in 0..50usize {
                    raw50[i] = read_volatile(addr_of!(RX_BUF).cast::<u8>().add(i));
                }

                // PDU layout (DMA offset 0, HW-dewhitened):
                //   [0]      PDU header byte 0: bits[3:0]=type, bit6=TxAdd, bit7=RxAdd
                //   [1]      PDU header byte 1: bits[5:0]=len, bits[7:6]=RFU (must be 0)
                //   [2..8]   AdvA (6 bytes, little-endian on-air → print big-endian)
                //   [8..]    AD structures
                let mut pdu = [0u8; 41];
                pdu.copy_from_slice(&raw50[..41]);

                let pdu_type = pdu[0] & 0x0F;
                let real_len = ((pdu[1] & 0x3F) as usize).min(37);
                let rfu_ok   = (pdu[1] & 0xC0) == 0;

                // Structural gate: common passive-scan ADV types (0=ADV_IND, 2=ADV_NONCONN_IND,
                // 4=SCAN_RSP, 5=ADV_DIRECT_IND, 6=ADV_SCAN_IND), RFU==0, min payload (AdvA=6B).
                let struct_ok = matches!(pdu_type, 0 | 2 | 4 | 5 | 6)
                    && rfu_ok && real_len >= 8;

                if struct_ok { struct_ok_count += 1; }

                if struct_ok {
                    // AdvA: bytes 2..8, little-endian on-air → print MSB-first.
                    hal::print!("RX#{rx_count} {ch_label}: type={pdu_type} len={real_len} AdvA=");
                    for i in (2..8).rev() {
                        hal::print!("{:02x}", pdu[i]);
                        if i > 2 { hal::print!(":"); }
                    }

                    // AD structures: iterate pdu[8 .. 2+real_len].
                    let ad_end = (2 + real_len).min(pdu.len());
                    let mut p = 8usize;
                    let mut any = false;
                    while p + 1 < ad_end {
                        let ad_len = pdu[p] as usize;
                        if ad_len == 0 || p + 1 + ad_len > ad_end { break; }
                        let ad_type = pdu[p + 1];
                        if !any { hal::print!(" ["); any = true; } else { hal::print!(","); }
                        match ad_type {
                            0x01 => hal::print!("Flags={:#04x}", pdu[p + 2]),
                            0x08 => {
                                hal::print!("SName=\"");
                                for &c in &pdu[p+2..p+1+ad_len] {
                                    if c.is_ascii_graphic() || c == b' ' { hal::print!("{}", c as char); }
                                    else { hal::print!("\\x{c:02x}"); }
                                }
                                hal::print!("\"");
                            }
                            0x09 => {
                                hal::print!("Name=\"");
                                for &c in &pdu[p+2..p+1+ad_len] {
                                    if c.is_ascii_graphic() || c == b' ' { hal::print!("{}", c as char); }
                                    else { hal::print!("\\x{c:02x}"); }
                                }
                                hal::print!("\"");
                            }
                            0xFF if ad_len >= 3 => {
                                let company = u16::from_le_bytes([pdu[p+2], pdu[p+3]]);
                                hal::print!("Mfr={company:#06x}");
                            }
                            _ => hal::print!("T{ad_type:#04x}({}B)", ad_len - 1),
                        }
                        p += 1 + ad_len;
                    }
                    if any { hal::print!("]"); }
                    hal::println!();
                }

            } else if scan_count <= 5 || scan_count % 50 == 0 {
                // Keepalive: scan stats + register snapshot every 50 windows (first 5 individually).
                // lle00=0x07 = HW exited scan mode (stuck signature). lle74=0x4c = stalled.
                // EVT baseline: lle00=1 (scan active), lle74=0x83.
                let lle_irq = bb_read(0x08);
                let lle64   = bb_read(0x64);
                let lle00   = bb_read(0x00);
                let lle74   = bb_read(0x74);
                hal::println!("  scan#{scan_count} rx={rx_count} struct={struct_ok_count} {ch_label}: \
                    lle_irq={lle_irq:#010x} lle64={lle64:#010x} lle00={lle00:#010x} lle74={lle74:#04x}");
            }

            // Canonical per-window W1C: replicates lle_irq_process @ L71240 (Lucy d.asm).
            // Rules confirmed from d.asm:
            //   - Each bit W1C'd independently — NEVER merge (e.g. writing 0x06 for bit1+bit2
            //     broke state machine in patch #3.5: SCN 2575→21)
            //   - bit2: frame received (write 4)
            //   - bit1: alt RX path (write 2; no state write in canonical handler)
            //   - bit13: scan-window-timeout (write 0x2000)
            //   - bit15: timer event (write 0x8000)
            //   - LLE+0x64 written by rx_window_timer_reload() BEFORE traverse (patch #3.10)
            //   - NO LLE+0x1C write: 0x6c is normal idle state (EVT SDI confirmed)
            {
                let irq = bb_read(0x08);
                if irq & (1 << 2)  != 0 { bb_write(0x08, 4);      }
                if irq & (1 << 1)  != 0 { bb_write(0x08, 2);      }
                if irq & (1 << 13) != 0 { bb_write(0x08, 0x2000); }
                if irq & (1 << 15) != 0 { bb_write(0x08, 0x8000); }
            }

            ch_idx = (ch_idx + 1) % ADV_CHANNELS.len();
        }
    }
}
