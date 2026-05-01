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
///
/// Header layout per BLE Core Spec Vol 6 Part F §3.1.2:
///   Byte 0: bits[3:0]=PDU_Type, bits[5:4]=RFU, bits[7:6]=Length[1:0]
///   Byte 1: bits[5:0]=Length[7:2], bits[7:6]=RFU
///
/// packet_type: 0=PRBS9, 1=0x0F repeated, 2=0x55 repeated, 3=vendor
unsafe fn build_dtm_pdu(packet_type: u8, payload_len: u8) {
    let len = payload_len.min(37);
    // BLE Core Spec Vol 6 Part F §3.1.2 (fixed 2026-05-01; was wrong: had type in bits[7:6]
    // and length in bits[5:0], which produced 0xA5 for type=2/len=37 → sniffer saw CONNECT_IND)
    TX_BUF[0] = (packet_type & 0x0F) | ((len & 0x03) << 6);
    TX_BUF[1] = (len >> 2) & 0x3F;
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

    // 8. Channel index: gptrBBReg+0x00 bits[6:0] = (freq_MHz - 2402) / 2 | 0x40.
    // WCH LL_TransmitterTest asm: `s0 |= 64` before writing bits[6:0].
    // bit6 is the "test mode marker" — required for correct whitening seed in DTM.
    let channel = (freq_khz / 1000 - 2402) / 2;
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | ((channel | 0x40) & 0x7F));

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

/// Poll for TX done — delegates to the shared `ble_tx_wait_done` primitive.
///
/// DTM packets are shorter than ADV (37-byte payload @ 1 Mbps ≈ 380µs + pre-delay).
/// settle_loops = 60_000 ≈ 420µs at 144 MHz is conservative enough for any DTM PDU.
///
/// Note: BB+0x08 bits 29+25 are sticky (non-W1C) — the `bb_write(0x08, 0xFFFF_FFFF)`
/// call in the old implementation was effectively a no-op for those bits.
unsafe fn wait_tx_done() -> bool {
    hal::ble::ble_tx_wait_done(10_000, 60_000)
}

// ── Entry point ──────────────────────────────────────────────────────────────

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    // Configure 96 MHz from the 32 MHz external crystal (HSE/4 × 12).
    //
    // WCH MCU.c / SetSysClockTo96_HSE always runs at 96 MHz before BLE_IPCoreInit.
    // Running at 8 MHz HSI (Default::default()) leaves the RFEND calibration PLL
    // unable to complete (rfend_wait_tune times out, CO=0, all calibration tables zero).
    // HSE 32 MHz → prediv=DIV4 → 8 MHz VCO input → mul=MUL12 → 96 MHz SYSCLK.
    //
    // Note: ble_phy_init() internally enables HSE too — with this config HSE is
    // already running and HSERDY=1 when ble_phy_init() runs; it will skip its own
    // HSE startup and proceed straight to dev_init / calibration at full speed.
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

    hal::println!("BLE DTM TX — CH32V208");
    hal::println!("channels: 2406 / 2440 / 2472 MHz (DTM ch2/19/35)");

    unsafe {
        // ── RCC / clock sanity check (before BLE init) ───────────────────────
        // CTLR(+0x00): bit1=HSIRDY, bit17=HSERDY, bit25=PLLRDY
        // CFGR0(+0x04): bits[3:2]=SWS (00=HSI, 01=HSE, 10=PLL)
        let rcc_ctlr  = core::ptr::read_volatile(0x4002_1000 as *const u32);
        let rcc_cfgr0 = core::ptr::read_volatile(0x4002_1004 as *const u32);
        hal::println!("RCC: ctlr={:#010x} cfgr0={:#010x} hsirdy={} hserdy={} pllrdy={} sws={}",
            rcc_ctlr, rcc_cfgr0,
            (rcc_ctlr >> 1) & 1, (rcc_ctlr >> 17) & 1,
            (rcc_ctlr >> 25) & 1, (rcc_cfgr0 >> 2) & 3);
        hal::println!("  (hserdy must=1, pllrdy must=1, sws must=2 for 96MHz PLL)");

        // ── Pre-init RFEND state (confirms power-on default of +0x90 bits[26:25]) ──
        // Lucy confirmed: WCH doesn't pre-clear +0x90 before calibration; power-on should be 0.
        // Reading here (before ble_phy_init) tells us if bits[26:25] are set by h/w reset or
        // by something in our init sequence (rfend_dev_init / lle_dev_init).
        let rfend90_pre = read_volatile(0x4002_5090 as *const u32);
        hal::println!("pre-init: rfend90=0x{rfend90_pre:08x} bits26_25={} (expect 0=power-on-fresh)",
            (rfend90_pre >> 25) & 3);

        hal::ble::ble_phy_init();

        // Post-init HSE diagnostic — same format as ble_adv.rs.
        // Expected: hseon=1, hserdy=1, hsebyp=0.
        let rcc_ctlr2 = core::ptr::read_volatile(0x4002_1000 as *const u32);
        let osc_cal   = core::ptr::read_volatile(0x4002_202C as *const u32);
        hal::println!("post: ctlr=0x{rcc_ctlr2:08x} hseon={} hserdy={} hsebyp={} osc_cal=0x{osc_cal:08x}",
            (rcc_ctlr2 >> 16) & 1, (rcc_ctlr2 >> 17) & 1, (rcc_ctlr2 >> 18) & 1);

        // ── RFEND calibration table dump ─────────────────────────────────────
        // RFEND_CAL_BASE = 0x40025000. After ble_reg_init(), CO tables should be non-zero
        // if HSE is running and rfend_tx_ctune() completed valid measurements.
        // Also dump bits[26:25] again to confirm they were 0 before and 1 after first tx_tune_measure.
        let rfend90 = read_volatile(0x4002_5090 as *const u32);
        let rfend38 = read_volatile(0x4002_5038 as *const u32);
        let co_result = rfend90 & 0x3F;
        let nco2440   = rfend38 & 0x3F;
        let nga2440   = (rfend38 >> 24) & 0x7F;
        // CO table 1 first word (should be non-zero if calibration worked)
        let co_t1_0 = read_volatile(0x4002_50A0 as *const u32);
        let co_t1_4 = read_volatile(0x4002_50B0 as *const u32);
        hal::println!("RFEND cal: rfend90=0x{rfend90:08x} co={co_result} rfend38=0x{rfend38:08x} nco2440={nco2440} nga2440={nga2440}");
        hal::println!("CO1 table: [0..7]=0x{co_t1_0:08x} [32..39]=0x{co_t1_4:08x}  (non-zero = cal OK)");

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

            // ~208 µs at 96 MHz (sufficient inter-burst gap for DTM).
            riscv::asm::delay(20_000);
        }
    }
}
