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
//! # Key RX vs TX differences (from Lucy's asm decode of LL_ReceiverTest)
//!
//! - LLE+0x00 (`bb_write(0x00, ...)`) = 1 for RX (TX = 2); this IS the RX GO trigger
//! - BB+0x00 bits\[8:7\] = 10b for both TX and RX (NOT a TX/RX selector; it's BB-enable)
//! - BB+0x50 (LLE+0x50, `bb_write(0x50, ...)`) pre-delay = 89 (TX = 90)
//! - LLE+0x64 (hardware countdown) = window timeout; check == 0 for scan done
//!
//! # Expected output (success)
//!
//! ```
//! RX#1 ch37/2402MHz: raw_hdr=0x.. type=0 len=X AdvA=AA:BB:CC:DD:EE:FF raw=[..]
//! ```
//!
//! # If no packets appear
//!
//! Check: antenna connected, PHY calibration printed (CO non-zero), RF busy environment.
//! Try increasing RX_WINDOW_TIMEOUT or the outer retry loop count.

#![no_std]
#![no_main]

use core::ptr::{addr_of, read_volatile, write_volatile};
use {ch32_hal as hal, panic_halt as _};

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

/// Hardware RX window timeout loaded into LLE+0x64 before each GO.
/// At 96 MHz with typical BLE ADV intervals, 400_000 ≈ ~4 ms scan window.
const RX_WINDOW_TIMEOUT: u32 = 400_000;

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

// ── RX arm / GO ───────────────────────────────────────────────────────────────

/// Arm the BLE receiver for one scan window on `logical_ch` / `freq_khz`.
///
/// Sequence mirrors LL_ReceiverTest() from libwchble.a V1.40 as decoded by Lucy
/// (2026-05-01 asm audit, ctl_input.o + ip.o + rf_fh.o).
///
/// IMPORTANT: call `RX_BUF.fill(0)` before arming so a stale `buf[1] != 0` from a
/// previous receive is not misidentified as a new packet.
unsafe fn rx_arm(logical_ch: u8, freq_khz: u32) {
    // 1. BB_CFG (+0x2C): DO NOT TOUCH.
    //    EVT Observer diff confirms: WCH Observer leaves BB+0x2C at bb_dev_init value
    //    (0x92010EC8, rf_flag=9, bit0=0) and relies solely on RFEND PLL for channel select.
    //    Previous write of bits[30:25]=logical_ch + bit0=1 was WRONG for RX path:
    //      - bit0=1 is the DTM TX mode arm bit (ll_hw_api_tx_direct_test) — Lucy confirmed
    //      - bits[30:25] channel LUT override not used in Observer (EVT doesn't do this)
    //      - Channel frequency is set exclusively by RFEND PLL (step 2), not BB+0x2C

    // 2. Program PLL to channel frequency; sets RFEND+0x2C bit1=1 (channel lock).
    set_channel_freq(freq_khz);

    // 3. BB+0x00 bits[8:7] = 10b (BB enable; same for TX and RX per Lucy asm L0xca-0xda).
    //    "a5 &= ~0x180; a5 |= 0x100; a4[0] = a5" (LL_ReceiverTest asm)
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x180) | 0x100);

    // 4. PA / RF path enable: RFEND+0x08 |= 0x330000 (bits 16,17,20,21).
    //    Enables PLL output and RX path power; mirrored from LL_ReceiverTest asm.
    let ana = rfend_read(0x08);
    rfend_write(0x08, ana | 0x0033_0000);

    // 5. Pre-delay timer: LLE+0x50 = 89 (RX; TX = 90).
    bb_write(0x50, 89);

    // 6. Release channel lock: RFEND+0x2C bit1 = 0 (PLL free-runs at programmed freq).
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !(1 << 1));

    // 7. BB+0x00 bits[5:0] = logical_ch | 0x40 (channel index + whitening enable).
    //    bit6=1 is the "test/whitening marker" — same as in TX.
    //    NOTE (EVT diff 2026-05-01): EVT writes just logical_ch (no 0x40); bit6=0 in EVT.
    //    If patch #1 (BB+0x2C removal) is insufficient, remove | 0x40 here as patch #2.
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | ((logical_ch as u32 & 0x3F) | 0x40));

    // 8. ACCESS_ADDR = BLE ADV access address.
    lle_write(0x08, ADV_AA);

    // 9. CRC init value.
    lle_write(0x04, ADV_CRC_INIT);

    // 10. RX DMA buffer pointer: LLE+0x74 = &RX_BUF (MEMAddr).
    //
    // LLE+0x70 is the TX-specific buffer slot (used by ble_dtm_tx.rs, works for TX).
    // LLE+0x74 is MEMAddr — the general DMA landing zone for received packets.
    // WCH rf_rx_basic_rxProcess reads: s1 = gBleIPPara[36] = MEMAddr; len = *(s1+1).
    // lle_dev_init() already wrote LLE+0x74 = LLE_DMA_BUF; overwrite with RX_BUF here.
    bb_write(0x74, addr_of!(RX_BUF) as u32);

    // 11. LLE+0x04 bit0 clear (LL_ReceiverTest asm L_c8: "lle_modify(0x04, 0x1, 0)").
    let v = bb_read(0x04);
    bb_write(0x04, v & !0x1);

    // 12. Window timeout: LLE+0x64 = countdown initial value.
    //     Hardware decrements this; == 0 means window expired (no packet received).
    bb_write(0x64, RX_WINDOW_TIMEOUT);

    // 13. RX GO: LLE+0x00 = 1 — triggers the RX state machine.
    //     (TX uses bit11 edge strobe; RX uses direct write of 1 to LLE+0x00.)
    bb_write(0x00, 1);
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
    hal::println!("Rotating ADV ch37/38/39, window={}  Looking for any advertiser...", RX_WINDOW_TIMEOUT);

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

        loop {
            let (logical_ch, freq_khz, ch_label) = ADV_CHANNELS[ch_idx];

            // Clear RX buffer, pre-clear BB IRQ (W1C), then arm the receiver.
            core::ptr::write_bytes(RX_BUF.as_mut_ptr(), 0, RX_BUF.len());
            lle_write(0x38, 0xFFFF_FFFF); // clear stale BB IRQ events before new window
            rx_arm(logical_ch, freq_khz);
            scan_count += 1;

            // Poll until packet received or window expires.
            // DMA writes 32-bit words as bits arrive at 1 Mbps; buf[1] != 0 fires when
            // the first word (bytes 0-3) lands — the rest of the payload is still in flight.
            let mut got_frame = false;
            for _ in 0..2_000_000u32 {
                if read_volatile(RX_BUF.as_ptr().add(1)) != 0 {
                    got_frame = true;
                    break;
                }
                if bb_read(0x64) == 0 { break; } // LLE window expired
                core::hint::spin_loop();
            }

            if got_frame {
                // Settle: wait for DMA to finish writing the rest of the packet.
                // At 1 Mbps a max ADV (2+37B PDU + 3B CRC = 42B) takes 336 µs on-air.
                // 40_000 nops @ 96 MHz ≈ 417 µs — covers full packet + margin.
                for _ in 0..40_000u32 { core::hint::spin_loop(); }

                rx_count += 1;

                // Read BB IRQ; W1C clear for next window.
                // bit21 = CRC-OK gate (environment-dependent; confirmed ~9.7% in busy environment).
                let bb_irq = lle_read(0x38);
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

                // ── Decode: always run, gate on structural PDU validity ───────────────────
                //
                // bb_irq bit21 gate was environment-dependent (0 in current run) and is
                // dropped as the primary gate. Structural validity (type + RFU + len) is
                // self-validating and robust across environments.
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
                let valid_frame = matches!(pdu_type, 0 | 2 | 4 | 5 | 6)
                    && rfu_ok && real_len >= 8;

                if valid_frame {
                    // AdvA: pdu[2..8] little-endian (on-air LSB first → print MSB first).
                    hal::print!("RX#{rx_count} {ch_label} type={pdu_type} len={real_len} \
                        tx={tx_add} rx={rx_add} bb={bb_irq:#010x} AdvA=");
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

                    // Raw dewhitened PDU dump: hdr(2) + AdvA(6) + AD section.
                    // pdu[8]/pdu[9] = first AD length + type bytes; if pdu[8]>31 or 0 →
                    // parser bails immediately (explains 0 Name/Mfr). Shown separately for easy grep.
                    let dump_end = (2 + real_len).min(pdu.len());
                    hal::println!(" ad0={:02x} ad1={:02x} pdu={:02x?}",
                        pdu[8], pdu[9], &pdu[..dump_end]);
                } else {
                    // Failed structural gate — suppress most; keep first 5 + every 200.
                    if rx_count <= 5 || rx_count % 200 == 0 {
                        hal::println!("  disc#{rx_count} {ch_label} type={pdu_type} \
                            len={real_len} rfu={} bb={bb_irq:#010x}",
                            rfu_ok as u8);
                    }
                }

            } else if scan_count % 30 == 0 {
                // Keepalive: print scan stats + register snapshot.
                let bb38 = lle_read(0x38);  // BB IRQ status (real CRC/event register)
                let bb08 = bb_read(0x08);   // LLE IRQ status (for comparison)
                let bb64 = bb_read(0x64);   // LLE countdown (0 = window expired)
                let bb1c = bb_read(0x1C);   // LLE state machine
                hal::println!("  scan#{scan_count} rx={rx_count} {ch_label}: no frame | \
                    bb_irq={bb38:#010x} lle_irq={bb08:#010x} lcount={bb64} state={bb1c:#04x}");
            }

            ch_idx = (ch_idx + 1) % ADV_CHANNELS.len();
        }
    }
}
