//! BLE Direct Test Mode (DTM) continuous TX example for CH32V208.
//!
//! Transmits a BLE test packet continuously on each advertising channel
//! (2402 / 2426 / 2480 MHz) in round-robin rotation. Verifiable with any
//! SDR (RTL-SDR, HackRF, etc.) or spectrum analyzer.
//!
//! # How to verify with SDR
//!
//! 1. Flash this binary: `cargo run --bin ble_dtm_tx --release`
//! 2. Open SDR# / GNU Radio / gqrx, tune to 2402 MHz, ~2 MHz BW
//! 3. You should see a GFSK burst every few ms; repeat for 2426 / 2480 MHz
//!
//! # Packet format on air (BLE 1M PHY)
//!
//! Preamble (1B, hw) | Access Address 0x71764129 (4B) |
//! PDU: type(1B) + len(1B) + payload(37B) | CRC-24 (3B, hw)
//!
//! # Register block mapping (confirmed from dtm.elf BLE_IPCoreInit)
//!
//! WCH name        | Address     | Our accessors | Contents
//! gptrBBReg       | 0x40024100  | lle_*         | CTRL/GO, TX mode, ACCESS_ADDR, CRC_INIT
//! gptrLLEReg      | 0x40024200  | bb_*          | Timing, IRQ status, TX timer, TX buf ptr
//! gptrAESReg      | 0x40024300  | (rfend init)  | RF analog config (init only)
//! gptrRFENDReg    | 0x40025000  | rfend_*       | PA bias, PLL, channel lock
//!
//! Note: WCH's "BB" and "LLE" labels are counterintuitive — gptrBBReg holds the
//! link-layer engine control registers while gptrLLEReg holds timing/scheduling.
//!
//! # TX sequence (from ll_hw_api_tx_direct_test + RF_DevSetChannel in libwchble V1.40)
//!
//! 1.  gptrBBReg+0x2C  bits[1:0]=01  TX mode arm
//! 2.  gptrLLEReg+0x64 = 160         event timeout
//! 3.  gptrRFENDReg+0x44             PLL int+frac dividers (RF_DevSetChannel)
//!     gptrRFENDReg+0x2C bit1=1      channel lock set
//! 4.  gptrBBReg+0x00  bits[8:7]=0, bit8=1  TX path select
//! 5.  gptrRFENDReg+0x08 |= 0x330000  PA bias enable
//! 6.  gptrLLEReg+0x50 = 90           TX pre-delay timer
//! 7.  gptrRFENDReg+0x2C &= ~2        channel lock release
//! 8.  gptrBBReg+0x00  bits[6:0] = channel index
//! 9.  gptrBBReg+0x08  = ACCESS_ADDR (0x71764129)
//! 10. gptrBBReg+0x04  = CRC_INIT (0x555555)
//! 11. gptrLLEReg+0x70 = TX buffer ptr
//! 12. gptrBBReg+0x00  &=~0x800, |=0x800  GO bit (clear then set for 0→1 edge)
//! 13. gptrBBReg+0x2C  bits[1:0]=00   clear TX arm

#![no_std]
#![no_main]

use core::ptr::{addr_of, read_volatile, write_volatile};
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

// ── Register base addresses ──────────────────────────────────────────────────
// Confirmed from BLE_IPCoreInit in dtm.elf (WCH BLE SDK V1.40).

/// gptrBBReg: link-layer CTRL/GO, TX mode, ACCESS_ADDR, CRC_INIT.
const LLE_BASE: usize = 0x40024100;
/// gptrLLEReg: timing params, IRQ status/mask, TX timer, TX buffer pointer.
const BB_BASE: usize = 0x40024200;
/// gptrRFENDReg: PA bias, PLL int/frac dividers, channel lock.
const RFEND_BASE: usize = 0x40025000;

// ── DTM constants (BLE Core Spec Vol 6 Part F) ───────────────────────────────

const DTM_ACCESS_ADDR: u32 = 0x71764129;
const DTM_CRC_INIT: u32 = 0x555555;

// ── Register accessors ───────────────────────────────────────────────────────

#[inline(always)]
unsafe fn lle_read(off: usize) -> u32 {
    read_volatile((LLE_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn lle_write(off: usize, val: u32) {
    write_volatile((LLE_BASE + off) as *mut u32, val);
}

#[inline(always)]
unsafe fn bb_read(off: usize) -> u32 {
    read_volatile((BB_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn bb_write(off: usize, val: u32) {
    write_volatile((BB_BASE + off) as *mut u32, val);
}

#[inline(always)]
unsafe fn rfend_read(off: usize) -> u32 {
    read_volatile((RFEND_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn rfend_write(off: usize, val: u32) {
    write_volatile((RFEND_BASE + off) as *mut u32, val);
}

// ── PLL / channel programming ─────────────────────────────────────────────────

/// Program PLL for the given frequency (kHz) and set channel-lock bit.
///
/// From RF_DevSetChannel() in libwchble.a V1.40 using gptrRFENDReg (0x40025000):
///   int_div  = (freq_khz / 64000) & 0x1F       → RFEND+0x44 bits[24:20]
///   frac_div = (freq_khz % 64000) * 1024 / 250  → RFEND+0x44 bits[13:0]
///   channel-lock bit1 set in RFEND+0x2C after PLL write.
unsafe fn set_channel_freq(freq_khz: u32) {
    let int_div = (freq_khz / 64000) & 0x1F;
    let frac_div = ((freq_khz % 64000) << 10) / 250;

    let pll = rfend_read(0x44);
    let pll = (pll & 0xFE0F_C000) | (int_div << 20) | (frac_div & 0x3FFF);
    rfend_write(0x44, pll);

    let v = rfend_read(0x2C);
    rfend_write(0x2C, v | (1 << 1));
}

// ── TX buffer (static, in .bss) ──────────────────────────────────────────────

/// Raw PDU buffer: [0]=header0, [1]=header1, [2..38]=payload.
static mut TX_BUF: [u8; 39] = [0u8; 39];

/// Fill TX_BUF with a DTM test PDU.
/// packet_type: 0=PRBS9, 1=0x0F repeated, 2=0x55 repeated, 3=vendor
unsafe fn build_dtm_pdu(packet_type: u8, payload_len: u8) {
    let len = payload_len.min(37);
    TX_BUF[0] = ((packet_type & 0x3) << 6) | (len & 0x3F);
    TX_BUF[1] = 0x00;
    let fill = match packet_type {
        1 => 0x0F,
        2 => 0x55,
        _ => 0xAA,
    };
    for i in 0..len as usize {
        TX_BUF[2 + i] = fill;
    }
}

// ── DTM TX trigger ───────────────────────────────────────────────────────────

/// Trigger one DTM TX burst on the given frequency (kHz).
unsafe fn dtm_tx_burst(freq_khz: u32) {
    // 1. Arm TX mode: gptrBBReg+0x2C bits[1:0] = 01.
    let lle_cfg = lle_read(0x2C);
    lle_write(0x2C, (lle_cfg & !0x3) | 0x1);

    // 2. Set LLE event timeout: gptrLLEReg+0x64 = 160.
    bb_write(0x64, 160);

    // 3. Program PLL and set channel-lock (gptrRFENDReg+0x44, +0x2C).
    set_channel_freq(freq_khz);

    // 4. TX path select: gptrBBReg+0x00 clear bits[8:7], set bit8.
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl & !0x180);
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x180) | 0x100);

    // 5. Enable PA bias: gptrRFENDReg+0x08 |= 0x330000.
    let ana = rfend_read(0x08);
    rfend_write(0x08, ana | 0x0033_0000);

    // 6. TX pre-delay timer: gptrLLEReg+0x50 = 90.
    bb_write(0x50, 90);

    // 7. Release channel lock: gptrRFENDReg+0x2C bit1 = 0.
    let v = rfend_read(0x2C);
    rfend_write(0x2C, v & !(1 << 1));

    // 8. Channel index: gptrBBReg+0x00 bits[6:0] = (freq_MHz - 2402) / 2.
    let channel = (freq_khz / 1000 - 2402) / 2;
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | (channel & 0x7F));

    // 9. Access Address: gptrBBReg+0x08.
    lle_write(0x08, DTM_ACCESS_ADDR);

    // 10. CRC init: gptrBBReg+0x04.
    lle_write(0x04, DTM_CRC_INIT);

    // 11. TX buffer pointer: gptrLLEReg+0x70.
    bb_write(0x70, addr_of!(TX_BUF) as u32);

    // 11.5. Write 2 to gptrLLEReg+0x00 (0x40024200) to arm the LLE in TX mode.
    // From ll_tx_wait_finish in dtm.elf: writes value 2 to *(gptrLLEReg) immediately
    // before the GO strobe. LLE+0x00 is a command/status register: bit1=TX active,
    // bit3=shutdown handshake. Without this write the LLE ignores the GO pulse.
    bb_write(0x00, 2);

    // 12. GO bit: gptrBBReg+0x00 — clear then set to guarantee 0→1 edge trigger.
    // bb_dev_init sets bit11=1; if GO is edge-triggered, that initial set consumes
    // the edge and subsequent |=0x800 is a no-op. Clearing first ensures a real edge.
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl & !0x800); // clear GO
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl | 0x800);  // set GO (0→1 edge)

    // 13. Clear TX arm: gptrBBReg+0x2C bits[1:0] = 00.
    let lle_cfg = lle_read(0x2C);
    lle_write(0x2C, lle_cfg & !0x3);
}

/// Poll for TX done at gptrLLEReg+0x08 (0x40024208, W1C IRQ status).
/// Hardware fires IRQ bits 29+25 (0x22000000) on TX completion — docs showed bit2
/// but live observation shows upper bits. Write all-ones to clear (W1C).
unsafe fn wait_tx_done() -> bool {
    for _ in 0..10_000u32 {
        if bb_read(0x08) != 0 {
            bb_write(0x08, 0xFFFF_FFFF);
            return true;
        }
        riscv::asm::nop();
    }
    bb_write(0x08, 0xFFFF_FFFF);
    false
}

// ── Entry point ──────────────────────────────────────────────────────────────

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let _p = hal::init(Default::default());

    hal::println!("BLE DTM TX — CH32V208");
    hal::println!("channels: 2406 / 2440 / 2472 MHz (DTM ch2/19/35)");

    unsafe {
        hal::ble::ble_phy_init();

        build_dtm_pdu(2, 37);

        hal::println!("PHY init done, starting TX loop");

        // Dump initial register state once to confirm init values.
        // ctrl bit28 must be 1 (0x10000000) — BB hardware enable from BB_DevInit.
        hal::println!(
            "INIT ctrl={:#010x} state={:#04x} irq={:#010x} cfg={:#010x} rfend2c={:#010x}",
            lle_read(0x00), bb_read(0x1C), bb_read(0x08),
            lle_read(0x2C), rfend_read(0x2C)
        );
        hal::println!("ctrl_bit28={} (expect 1)", (lle_read(0x00) >> 28) & 1);
        hal::println!("cfg={:#010x} (expect 0x92010ec8)", lle_read(0x2C));

        // DTM ch2/19/35 = 2406/2440/2472 MHz — avoid busy BLE adv channels.
        const CHANNELS: [u32; 3] = [2_406_000, 2_440_000, 2_472_000];
        let mut idx = 0usize;
        let mut ok_count = 0u32;
        let mut timeout_count = 0u32;
        let mut total = 0u32;

        loop {
            let ctrl_pre = lle_read(0x00);

            dtm_tx_burst(CHANNELS[idx]);

            // Tight read immediately after GO — catches transient state transitions.
            let ctrl_go = lle_read(0x00);   // CTRL after GO (bit11 may have auto-cleared)
            let irq_go  = bb_read(0x08);    // IRQ status right after GO
            let state_go = bb_read(0x1C);   // LLE state right after GO

            let done = wait_tx_done();
            let irq_after = bb_read(0x08);
            let state_after = bb_read(0x1C);

            if done { ok_count += 1; } else { timeout_count += 1; }
            total += 1;

            // Full register dump for first 5 bursts and every 300 thereafter.
            if total <= 5 || total % 300 == 0 {
                hal::println!(
                    "#{} ok={} to={} ctrl:{:#010x}->{:#010x} irq_go={:#06x} state:{:#04x}->{:#04x} irq={:#010x}",
                    total, ok_count, timeout_count,
                    ctrl_pre, ctrl_go, irq_go, state_go, state_after, irq_after
                );
            }

            idx = (idx + 1) % CHANNELS.len();

            // ~625 µs at 32 MHz (1 BLE slot).
            riscv::asm::delay(20_000);
        }
    }
}
