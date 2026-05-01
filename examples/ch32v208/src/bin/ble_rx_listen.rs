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
    // 1. BB_CFG (+0x2C) bits[30:25] = logical_ch; bits[1:0] = 01 (RX mode arm).
    //    bits[30:25]: HW whitening/CRC LUT channel selector.
    //    From adv.rs: "without this, all ADV channels use channel-9 LUT (rf_flag=9
    //    from bb_dev_init) → wrong whitening seed → CRC fail → no DMA write."
    //    Write logical_ch (37/38/39) to override rf_flag=9. bit31,bit24:0 preserved.
    let cfg = lle_read(0x2C);
    lle_write(0x2C, (cfg & 0x81FF_FFFF) | ((logical_ch as u32 & 0x3F) << 25) | 0x1);

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

            // Clear RX buffer and arm the receiver.
            core::ptr::write_bytes(RX_BUF.as_mut_ptr(), 0, RX_BUF.len());
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
                // We detected after the first 32-bit word (~32 µs), so need ~304 µs more.
                // 40_000 nops @ 96 MHz ≈ 417 µs — covers full packet + margin.
                for _ in 0..40_000u32 { core::hint::spin_loop(); }

                rx_count += 1;

                // Snapshot raw bytes (HW-dewhitened by BB before DMA write) for diagnostics.
                // 50 bytes: enough for max ADV PDU (2+37) + trailer + safety margin.
                let mut raw50 = [0u8; 50];
                for (i, r) in raw50.iter_mut().enumerate() {
                    *r = read_volatile(addr_of!(RX_BUF).cast::<u8>().add(i));
                }

                // HW BB dewhitens before DMA write (confirmed by Lucy asm decode 2026-05-01):
                // - WCH BLE_LIB has zero software dewhiten calls in all .o files.
                // - BB+0x00 bits[6:0] = channel|0x40 IS the hardware LFSR seed register.
                // - rf_rx_basic_rxProcess reads s1[1] as length with no per-byte LFSR loop.
                // Therefore raw50 IS the already-dewhitened PDU — read fields directly.
                let pdu_type = raw50[0] & 0x0F;
                // BLE PDU length field is 6 bits [5:0]; bits[7:6] are TxAdd/RxAdd flags
                // or RFU — mask to 6 bits for the payload byte count.
                let real_len = (raw50[1] as usize & 0x3F).min(37); // ADV max 37B payload
                let frame_end = 2 + real_len; // index of hardware status byte

                // PDU header byte 0 flag bits: TxAdd (bit6) = sender random addr; RxAdd (bit7).
                let tx_add = (raw50[0] >> 6) & 1;
                let rx_add = (raw50[0] >> 7) & 1;

                // Read LLE IRQ status (bb_read(0x08) = gptrLLEReg+0x08) — may carry CRC bits.
                let lirq = bb_read(0x08);

                // CRC status: hardware writes 0x80 at raw50[frame_end] on CRC pass.
                // (Lucy: rf_rx_basic_rxProcess writes buf[buf[1]] = 0x80 on CRC OK; with
                //  HW dewhiten, buf[1] = real payload len, so trailer at 2+len = frame_end.)
                let crc_ok = frame_end < raw50.len() && raw50[frame_end] == 0x80;

                // First 0x80 position in the dewhitened raw50 — should cluster at frame_end
                // for CRC-OK packets (trailer byte) after this fix.
                let raw_trailer_idx = raw50.iter().position(|&b| b == 0x80);

                hal::print!(
                    "RX#{rx_count} {ch_label} type={pdu_type} len={real_len} \
                     tx={tx_add} rx={rx_add} crc={} lirq={lirq:#010x}",
                    if crc_ok { "OK" } else { "?" });

                if let Some(ti) = raw_trailer_idx {
                    // at_fe=true → trailer aligns with BLE spec (2+real_len) — expected for CRC-OK.
                    // at_len=true → trailer aligns with raw50[1] raw value (WCH asm alternate).
                    let at_fe  = ti == frame_end;
                    let at_len = ti == (raw50[1] as usize);
                    hal::print!(" t80={ti}(fe={at_fe},l={at_len})");
                }

                if real_len >= 6 && frame_end >= 8 {
                    hal::print!(" AdvA=");
                    for i in (2..8).rev() {
                        hal::print!("{:02x}", raw50[i]);
                        if i > 2 { hal::print!(":"); }
                    }
                }
                // raw50: first 50 dewhitened bytes (BB HW dewhitens before DMA write).
                hal::println!(" raw50={:02x?}", &raw50);

            } else if scan_count % 30 == 0 {
                // Keepalive: print scan stats + register snapshot.
                let bb08 = bb_read(0x08);   // LLE IRQ status
                let bb64 = bb_read(0x64);   // LLE countdown (0 = window expired)
                let bb1c = bb_read(0x1C);   // LLE state machine
                hal::println!("  scan#{scan_count} rx={rx_count} {ch_label}: no frame | \
                    lirq=0x{bb08:08x} lcount={bb64} state=0x{bb1c:02x}");
            }

            ch_idx = (ch_idx + 1) % ADV_CHANNELS.len();
        }
    }
}
