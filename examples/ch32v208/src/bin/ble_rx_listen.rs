//! BLE RX bedrock scanner for CH32V208.
//!
//! Rotates through ADV channels 37/38/39 (2402/2426/2480 MHz) listening for any
//! BLE advertising packet. If ANY packet is received, the full PHY RX chain is proven:
//!   HSE 32 MHz crystal → RFEND PLL lock → BB/LLE state machine → DMA → UART
//!
//! # Register name convention (same as ble_dtm_tx.rs — intentionally swapped)
//!
//! ```
//! const LLE_BASE = 0x40024100  → WCH gptrBBReg  (BB engine regs)
//! const BB_BASE  = 0x40024200  → WCH gptrLLEReg  (link-layer timing/buffer regs)
//! ```
//! This swapped naming is intentional for compatibility with the WCH asm decode flow.
//!
//! # Key RX sequences (Lucy d.asm, 2026-05-01)
//!
//! Two functions discovered — patch #3.4 dual-track:
//!
//! ## Cold boot: LL_ScanSetRF (L62824) — run ONCE
//! Full register init + final RX GO (LLE+0x00 = 1 via bb_write).
//! Pulls HW out of default state into active RX for the first window.
//!
//! ## Per-window: llScanTraverseaChannel (L62961) — run every subsequent window
//! Minimal state-machine transition: W1C bit13, bit7→bit8 transitions in LLE+0x00,
//! PRE_DELAY + RFEND PA reset, then `(ctrl & !0x7F) | channel` final write.
//! **Does NOT write RX GO (bit0=1)** — HW is already live; full re-init would push
//! it back to SLEEP.
//!
//! Root cause of patch #3.3 failure: running LL_ScanSetRF every window over-resets
//! the state machine. HW entered SLEEP after the startup burst and never re-emerged.
//!
//! # Expected output (success)
//!
//! ```
//! RX#1 ch37/2402MHz: type=0 len=X AdvA=AA:BB:CC:DD:EE:FF [Flags=0x06,Name="device"]
//! ```
//!
//! # If no packets appear
//!
//! Check: antenna connected, PHY calibration printed (CO non-zero), RF busy environment.

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

/// Dewhiten a BLE PDU in-place using the spec LFSR (polynomial x^7 + x^4 + 1).
///
/// # Why needed
///
/// CH32V208 HW does NOT dewhiten before DMA write (confirmed 2026-05-01).
/// Despite BB+0x00 bits[6:0] = channel|0x40 being the HW LFSR seed register,
/// WCH's BLE library has zero SW dewhiten calls because the TMOS stack operates
/// at a higher abstraction level that never sees raw DMA bytes.
/// Empirical proof (Cindy bucket analysis, 1614 frames):
///   - raw bit21-only bucket at offset 9 with SW dewhiten → type=90.4%, RFU=86.6%
///   - raw bit21-only bucket at offset 9 WITHOUT dewhiten → ~62.5% (random baseline)
///
/// # LFSR spec
///
/// State: 7 bits. Seed = `channel | 0x40` (bit6 always 1).
/// Each bit: output = state[0] (LSB); new input to state[6] = state[0] XOR state[4].
/// BLE PDU bits are transmitted LSB first, so we apply the LFSR LSB-first within each byte.
///
/// # Polynomial derivation
///
/// Polynomial x^7 + x^4 + 1, right-shift Fibonacci LFSR (output from bit 0):
///   feedback = bit0 XOR bit4   (taps at positions 0 and 4 from the output end)
///   new_state = (state >> 1) | (feedback << 6)
///
/// NOT `bit6 XOR bit3` — that is the left-shift (output from bit6) variant and is WRONG here.
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

/// Scan window duration written to LLE+0x64 just before RX GO (polling model addition).
/// LL_ScanSetRF (IRQ-driven Observer) writes 0 here (no countdown needed; IRQ fires on done).
/// In our polling model we need a window bound: HW counts down from this value; == 0 means
/// window expired. At ~30 cycles/iter × 400_000 iters / 96 MHz ≈ 125 ms per channel.
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

/// Arm the BLE receiver: cold-boot on first call, per-window traverse on subsequent calls.
///
/// Architecture (patch #3.4, Lucy d.asm, 2026-05-01):
///
/// Cold boot (`LL_ScanSetRF`, L62824): full register init + RX GO.
///   Run once to pull HW from init state into active RX.
///
/// Per-window (`llScanTraverseaChannel`, L62961): minimal state-machine transition.
///   W1C bit13 → bit7 transition → bit8 transition → channel update → done.
///   **No RX GO (bit0=1)** — HW is already live; writing full re-init pushed HW to SLEEP.
///
/// Root cause of #3.3 failure:
///   LL_ScanSetRF every window over-reset the state machine. HW entered SLEEP (state=0x6c)
///   after the first real timeout and ignored all subsequent GO commands.
unsafe fn rx_arm(logical_ch: u8, freq_khz: u32) {
    if !SCAN_INITED.load(Ordering::Relaxed) {
        SCAN_INITED.store(true, Ordering::Relaxed);
        rx_cold_init(logical_ch, freq_khz);
    } else {
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
/// Follows the per-window state-machine transition sequence from d.asm, which does NOT
/// write RX GO (bit0=1). HW remains live from the cold-boot GO; the traverse just
/// switches channel and nudges the state machine without resetting it.
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
    hal::println!("Rotating ADV ch37/38/39 (LL_ScanSetRF path) window={}  Looking for any advertiser...", SCAN_WINDOW);

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
        let mut struct_ok_count = 0u32;   // structural gate passed
        let mut crc_fail_count  = 0u32;   // struct_ok but bit21=0 (no DTM PRBS sync)

        loop {
            let (logical_ch, freq_khz, ch_label) = ADV_CHANNELS[ch_idx];

            // Clear RX buffer, pre-clear BB IRQ (W1C), then arm the receiver.
            core::ptr::write_bytes(RX_BUF.as_mut_ptr(), 0, RX_BUF.len());
            lle_write(0x38, 0xFFFF_FFFF); // clear stale BB IRQ events before new window
            rx_arm(logical_ch, freq_khz);
            scan_count += 1;

            // Poll until HW signals frame complete or scan window expires.
            //
            // Exit conditions:
            //   A. irq08 bit1|bit2 — HW RX-done (WCH IRQSubHandler .L5 path, d.asm L69423).
            //      This is the ONLY valid "got frame" signal. RX_BUF[1] != 0 was removed
            //      in patch #3.3 (Lucy 2026-05-01): that fires on partial DMA (mid-write)
            //      and causes us to re-arm while HW is still writing, aborting in-flight
            //      frames and eventually killing the state machine after ~13 hits.
            //   B. bb_read(0x64) == 0 — scan window countdown expired (no packet this window).
            //      LLE+0x64 written SCAN_WINDOW before GO; HW decrements it; 0 = window over.
            //   C. 2M iteration safety backstop (software guard, should never fire in practice).
            let mut got_frame = false;
            for _ in 0..2_000_000u32 {
                // irq08 bit1 OR bit2 = RX done (WCH IRQSubHandler .L5 RX-done path).
                if (bb_read(0x08) & 0x06) != 0 {
                    got_frame = true;
                    break;
                }
                if bb_read(0x64) == 0 { break; } // LLE window countdown expired
                core::hint::spin_loop();
            }

            if got_frame {
                // Settle: wait for DMA to finish writing the rest of the packet.
                // At 1 Mbps a max ADV (2+37B PDU + 3B CRC = 42B) takes 336 µs on-air.
                // 40_000 nops @ 96 MHz ≈ 417 µs — covers full packet + margin.
                for _ in 0..40_000u32 { core::hint::spin_loop(); }

                rx_count += 1;

                // Read BB IRQ (0x40024100+0x38) + LLE IRQ (0x40024200+0x08) before W1C.
                // Lucy: WCH IRQSubHandler uses LLE+0x08 bit1/bit2 for RX done, not BB+0x38.
                let bb_irq  = lle_read(0x38);
                let irq08   = bb_read(0x08);   // LLE IRQ status: bit1/bit2 = RX done path
                lle_write(0x38, bb_irq);

                // Snapshot 50 raw bytes from DMA buffer (covers prefix + max PDU + margin).
                // Done BEFORE any decode so SCN scan and decode see the same snapshot.
                let mut raw50 = [0u8; 50];
                for i in 0..50usize {
                    raw50[i] = read_volatile(addr_of!(RX_BUF).cast::<u8>().add(i));
                }

                // ── SCN: per-offset LFSR scan ─────────────────────────────────────────────
                // Runs for EVERY got_frame event (outside CRC gate) so no frames are swallowed.
                // For each offset 4..=12: fresh LFSR (seed=logical_ch|0x40), decode 2 bytes →
                //   tT = PDU type (0-9 = valid BLE ADV); rR = bits[7:6]==00 (RFU valid).
                // Also tests ±1 channel seed at offset 9 (channel-drift detection).
                {
                    // Helper closure: dewhiten 2 bytes from raw50 at 'off' with given seed ch.
                    let dw2 = |off: usize, ch: u8| -> (u8, u8) {
                        let mut lfsr = ch | 0x40;
                        let (mut b0, mut b1) = (0u8, 0u8);
                        for bit in 0..8u8 {
                            let out = lfsr & 1;
                            let fb = (lfsr ^ (lfsr >> 4)) & 1;
                            lfsr = (lfsr >> 1) | (fb << 6);
                            b0 |= (((raw50[off] >> bit) & 1) ^ out) << bit;
                        }
                        for bit in 0..8u8 {
                            let out = lfsr & 1;
                            let fb = (lfsr ^ (lfsr >> 4)) & 1;
                            lfsr = (lfsr >> 1) | (fb << 6);
                            b1 |= (((raw50[off + 1] >> bit) & 1) ^ out) << bit;
                        }
                        (b0 & 0x0F, ((b1 & 0xC0) == 0) as u8)
                    };

                    hal::print!("SCN#{rx_count} {ch_label} bb={bb_irq:#010x}:");
                    for off in 4usize..=12 {
                        let (t, r) = dw2(off, logical_ch);
                        hal::print!(" o{}=t{}r{}", off, t, r);
                    }
                    let ch_m1 = logical_ch.saturating_sub(1);
                    let ch_p1 = (logical_ch + 1).min(39);
                    let (tm1, rm1) = dw2(9, ch_m1);
                    let (t0,  r0)  = dw2(9, logical_ch);
                    let (tp1, rp1) = dw2(9, ch_p1);
                    hal::println!(" |o9: ch{}=t{}r{} ch{}=t{}r{} ch{}=t{}r{}",
                        ch_m1, tm1, rm1, logical_ch, t0, r0, ch_p1, tp1, rp1);
                }

                // ── Decode: structural PDU validity gate ─────────────────────────────────
                //
                // patch #2.5 (2026-05-01): bit21 confirmed NOT CRC-OK in non-DTM mode —
                // it was a DTM PRBS sync event (bit21=0/3438 after removing DTM bits).
                // Lucy: real CRC-OK bit is still unknown; hunting in d.asm bb_irq_handler.
                // Meanwhile: structural gate only, add struct_ok_but_crc_fail counter.
                // Bits 1,6 in bb_irq at 0x001e8046/66 may carry the real CRC status.
                let _bit21 = (bb_irq >> 21) & 1; // keep for stats; NOT used as gate
                //
                // Channel-specific PDU offset (confirmed 2026-05-01):
                //   ch37 → 8-byte HW prefix → PDU at DMA offset 8
                //   ch38, ch39 → 9-byte HW prefix → PDU at DMA offset 9
                let pdu_off: usize = if logical_ch == 37 { 8 } else { 9 };
                let mut pdu = [0u8; 41];
                pdu.copy_from_slice(&raw50[pdu_off..pdu_off + 41]);
                // HW does NOT dewhiten before DMA (confirmed 2026-05-01, LFSR 35/35 verified).
                dewhiten_inplace(&mut pdu, logical_ch);

                let pdu_type = pdu[0] & 0x0F;
                let real_len = ((pdu[1] & 0x3F) as usize).min(37); // ADV max 37B
                let tx_add   = (pdu[0] >> 6) & 1;
                let rx_add   = (pdu[0] >> 7) & 1;
                let rfu_ok   = (pdu[1] & 0xC0) == 0;

                // Structural gate: common ADV types, RFU==0, legal length.
                let struct_ok = matches!(pdu_type, 0 | 2 | 4 | 5 | 6)
                    && rfu_ok && real_len >= 8;
                let valid_frame = struct_ok;

                // Lucy trailer byte: WCH rf_rx_basic_rxProcess checks MEMAddr[MEMAddr[1]]==0x80.
                // raw50[1] = RX_BUF[1] = second byte of HW metadata prefix = WCH's "length field".
                // Trailer at RX_BUF[raw50[1]] = raw50[raw50[1]] (if index in range).
                // Also check pdu[real_len+2] = raw50[pdu_off+real_len+2] (Lucy's formula for PDU-relative).
                let wch_len   = raw50[1] as usize;
                let wch_trl   = if wch_len < raw50.len() { raw50[wch_len] } else { 0xFF };
                let pdu_trl   = if (real_len + 2) < pdu.len() { pdu[real_len + 2] } else { 0xFF };

                if struct_ok { struct_ok_count += 1; }
                if struct_ok && _bit21 == 0 { crc_fail_count += 1; }

                if valid_frame {
                    // AdvA: pdu[2..8] little-endian (on-air LSB first → print MSB first).
                    hal::print!("RX#{rx_count} {ch_label} type={pdu_type} len={real_len} \
                        tx={tx_add} rx={rx_add} bb={bb_irq:#010x} irq08={irq08:#010x} \
                        b21={_bit21} wlen={wch_len} wtrl={wch_trl:#04x} ptrl={pdu_trl:#04x} AdvA=");
                    for i in (2..8).rev() {
                        hal::print!("{:02x}", pdu[i]);
                        if i > 2 { hal::print!(":"); }
                    }

                    // AD structures from pdu[8 .. 2+real_len] (payload after 6-byte AdvA).
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
                            _ => hal::print!("T{ad_type:#04x}({B}B)", B = ad_len - 1),
                        }
                        p += 1 + ad_len;
                    }
                    if any { hal::print!("]"); }

                    // Diagnostic dumps for trailer byte analysis:
                    // prefix = raw50[0..pdu_off] (HW metadata), bytes_after = raw50[pdu_off+real_len+2..]
                    let dump_end = (2 + real_len).min(pdu.len());
                    hal::print!(" ad0={:02x} ad1={:02x}", pdu[8], pdu[9]);
                    hal::print!(" pfx={:02x?}", &raw50[..pdu_off]);
                    let after_start = pdu_off + 2 + real_len;
                    if after_start + 6 <= raw50.len() {
                        hal::print!(" after={:02x?}", &raw50[after_start..after_start+6]);
                    }
                    hal::println!(" pdu={:02x?}", &pdu[..dump_end]);
                } else {
                    // Failed structural gate — suppress most; keep first 5 + every 200.
                    if rx_count <= 5 || rx_count % 200 == 0 {
                        hal::println!("  disc#{rx_count} {ch_label} type={pdu_type} \
                            len={real_len} rfu={} bb={bb_irq:#010x}",
                            rfu_ok as u8);
                    }
                }

            } else if scan_count <= 10 || scan_count % 30 == 0 {
                // Keepalive: print scan stats + register snapshot.
                // First 10 scans printed individually (Lucy: observe state 0-3s post cold-boot).
                // Then every 30 scans.
                let bb38 = lle_read(0x38);  // BB IRQ status (real CRC/event register)
                let bb08 = bb_read(0x08);   // LLE IRQ status (for comparison)
                let bb64 = bb_read(0x64);   // LLE+0x64: 0=idle, non-zero=RX in progress
                let bb1c = bb_read(0x1C);   // LLE state machine
                let lle00 = bb_read(0x00);  // LLE+0x00: GO/state-bits (watch for SLEEP drift)
                hal::println!("  scan#{scan_count} rx={rx_count} struct={struct_ok_count} {ch_label}: no frame | \
                    bb_irq={bb38:#010x} lle_irq={bb08:#010x} rxbusy={bb64:#010x} state={bb1c:#04x} lle00={lle00:#010x}");
            }

            ch_idx = (ch_idx + 1) % ADV_CHANNELS.len();
        }
    }
}
