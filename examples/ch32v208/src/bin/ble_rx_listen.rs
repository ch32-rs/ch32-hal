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

// ── RX arm / GO ───────────────────────────────────────────────────────────────

/// Arm the BLE receiver for one scan window on `logical_ch` / `freq_khz`.
///
/// Sequence mirrors LL_ReceiverTest() from libwchble.a V1.40 as decoded by Lucy
/// (2026-05-01 asm audit, ctl_input.o + ip.o + rf_fh.o).
///
/// IMPORTANT: call `RX_BUF.fill(0)` before arming so a stale `buf[1] != 0` from a
/// previous receive is not misidentified as a new packet.
unsafe fn rx_arm(logical_ch: u8, freq_khz: u32) {
    // 1. BB_CFG (+0x2C) bits[1:0] = 01 (RX/TX mode arm). Preserve bits[30:25] (rf_flag=0x09).
    let cfg = lle_read(0x2C);
    lle_write(0x2C, (cfg & !0x3) | 0x1);

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

    // 10. RX DMA buffer pointer: LLE+0x70 = &RX_BUF.
    bb_write(0x70, addr_of!(RX_BUF) as u32);

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

// ── BLE de-whitening ──────────────────────────────────────────────────────────

/// BLE LFSR de-whitening: x^7 + x^4 + 1, seed = {1, ch\[5:0\]}.
fn dewhiten_inplace(data: &mut [u8], logical_ch: u8) {
    let mut lfsr = (logical_ch & 0x3F) | 0x40;
    for byte in data.iter_mut() {
        let mut wbyte = 0u8;
        for bit in 0..8u8 {
            let wbit = lfsr & 1;
            wbyte |= wbit << bit;
            let fb = ((lfsr >> 6) ^ (lfsr >> 3)) & 1;
            lfsr = ((lfsr >> 1) | (fb << 6)) & 0x7F;
        }
        *byte ^= wbyte;
    }
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
            // buf[1] = PDU payload-length byte (DMA writes ≠ 0 when packet lands).
            // LLE+0x64 counts down from RX_WINDOW_TIMEOUT; reaching 0 = no packet this window.
            let mut got_frame = false;
            // Outer loop: hardware window + software watchdog for safety.
            for _ in 0..2_000_000u32 {
                let len_byte = read_volatile(RX_BUF.as_ptr().add(1));
                if len_byte != 0 {
                    got_frame = true;
                    break;
                }
                // Hardware window expired?
                if bb_read(0x64) == 0 {
                    break;
                }
                core::hint::spin_loop();
            }

            if got_frame {
                rx_count += 1;
                let hdr0    = RX_BUF[0];
                let raw_len = (RX_BUF[1] & 0x3F) as usize;
                let frame_end = (2 + raw_len).min(RX_BUF.len());

                // Snapshot raw bytes before dewhitening.
                let raw_snap: [u8; 10] = {
                    let mut s = [0u8; 10];
                    for (i, b) in s.iter_mut().enumerate() {
                        if i < frame_end { *b = RX_BUF[i]; }
                    }
                    s
                };

                // Dewhiten header + payload in-place.
                dewhiten_inplace(&mut RX_BUF[..frame_end], logical_ch);

                let pdu_type = RX_BUF[0] & 0x0F;
                let length   = RX_BUF[1] & 0x3F;

                hal::print!("RX#{rx_count} {ch_label}: raw_hdr=0x{hdr0:02x} type={pdu_type} len={length}");

                if length >= 6 && frame_end >= 8 {
                    hal::print!(" AdvA=");
                    for i in (2..8).rev() {
                        hal::print!("{:02x}", RX_BUF[i]);
                        if i > 2 { hal::print!(":"); }
                    }
                }
                hal::println!(" raw=[{:02x?}]", &raw_snap[..frame_end.min(10)]);

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
