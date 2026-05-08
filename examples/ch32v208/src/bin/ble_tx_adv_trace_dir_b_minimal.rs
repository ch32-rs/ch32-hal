//! Direction B trace binary — minimal (74e8d67 baseline).
//!
//! Outputs SDI TR-tagged register snapshots at T0 (after init) and per-step
//! during the FIRST ch37 burst. No P1/P2.5/P3 modifications applied.
//! Baseline reflects the state that produced cba=0 after fix #4+#5 (commit 74e8d67).
//!
//! Compare against `ble_tx_adv_trace_dir_b_frozen.rs` to find hidden deltas.
//!
//! Output: SDIPrint `TR <phase> reg=val ...` one line per checkpoint.
//! After first-burst trace the device loops silently with 100ms gap.

#![no_std]
#![no_main]

use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};
use {ch32_hal as hal, panic_halt as _};

// ── BSS contract (74e8d67 baseline, same as ble_tx_adv_ch37_minimal.rs) ──────

#[no_mangle] pub static mut gptrBBReg:    u32 = 0x4002_4100;
#[no_mangle] pub static mut gptrLLEReg:   u32 = 0x4002_4200;
#[no_mangle] pub static mut gptrAESReg:   u32 = 0x4002_4300;
#[no_mangle] pub static mut gptrRFENDReg: u32 = 0x4002_5000;

#[no_mangle] #[link_section = ".bss.ble"] #[used]
pub static mut ble: [u32; 16] = [0u32; 16];
#[no_mangle] #[link_section = ".bss.zz_gpa"]
pub static mut gPaControl: u32 = 0;
#[no_mangle] #[link_section = ".bss.zz_dtm"]
pub static mut dtmFlag: u8 = 0;
#[no_mangle] #[link_section = ".bss.gBleLlPara"] #[used]
pub static mut gBleLlPara: [u32; 74] = [0u32; 74];
#[no_mangle] #[link_section = ".bss.gBleIPPara"]
pub static mut gBleIPPara: [u32; 10] = [0u32; 10];
// Phase 2c: natural BSS placement; `ble_ip_core_init()` writes `fallback_clock` fn ptr.
#[no_mangle]
pub static mut fnGetClockCBs: u32 = 0;

const ADDR: [u8; 6] = [0x12, 0x87, 0x65, 0x43, 0x21, 0xC2];

#[link_section = ".bss"]
static mut MINIMAL_ADV_CTX: [u8; 192] = [0u8; 192];

// ── TX buffer (16B-aligned) ───────────────────────────────────────────────────

/// Flat TX buffer for trace binary — built inline, not via adv.rs.
#[link_section = ".tx_buf_aligned"]
static mut TX_BUF: [u8; 39] = [0u8; 39];

unsafe fn fill_tx_buf(addr: &[u8; 6]) {
    // ADV_NONCONN_IND header
    TX_BUF[0] = 0b0000_0010 | (1 << 6); // type=2, TxAdd=1 (random)
    TX_BUF[1] = 14;                      // payload_len = AdvA(6) + Flags(3) + Name(5)
    TX_BUF[2..8].copy_from_slice(addr);
    // Flags AD: len=2, type=0x01, value=0x06
    TX_BUF[8] = 2; TX_BUF[9] = 0x01; TX_BUF[10] = 0x06;
    // Complete Local Name "cba": len=4, type=0x09
    TX_BUF[11] = 4; TX_BUF[12] = 0x09;
    TX_BUF[13] = b'c'; TX_BUF[14] = b'b'; TX_BUF[15] = b'a';
}

// ── Register helpers ──────────────────────────────────────────────────────────

const LLE_BASE:   usize = 0x40024100;
const BB_BASE:    usize = 0x40024200;
const RFEND_BASE: usize = 0x40025000;

#[inline(always)] unsafe fn lr(o: usize) -> u32 { read_volatile((LLE_BASE  + o) as *const u32) }
#[inline(always)] unsafe fn lw(o: usize, v: u32) { write_volatile((LLE_BASE + o) as *mut u32, v); }
#[inline(always)] unsafe fn br(o: usize) -> u32 { read_volatile((BB_BASE   + o) as *const u32) }
#[inline(always)] unsafe fn bw(o: usize, v: u32) { write_volatile((BB_BASE  + o) as *mut u32, v); }
#[inline(always)] unsafe fn rr(o: usize) -> u32 { read_volatile((RFEND_BASE + o) as *const u32) }
#[inline(always)] unsafe fn rw(o: usize, v: u32) { write_volatile((RFEND_BASE + o) as *mut u32, v); }

// ── ll_gblellpara_init (mirror of frozen ll_init_safe_prefix + seed_ble_bd_addr) ──

#[inline(always)] unsafe fn p32(p: *mut u8, o: usize, v: u32) { write_volatile(p.add(o) as *mut u32, v); }
#[inline(always)] unsafe fn p16(p: *mut u8, o: usize, v: u16) { write_volatile(p.add(o) as *mut u16, v); }
#[inline(always)] unsafe fn p8(p: *mut u8,  o: usize, v: u8)  { write_volatile(p.add(o), v); }

unsafe fn ll_gblellpara_init(addr: &[u8; 6]) {
    let p = addr_of_mut!(gBleLlPara) as *mut u8;
    p32(p, 0x00, 0x0000_0004); p32(p, 0x14, 0x07d7_000d); p32(p, 0x18, 0x0d0d_b140);
    p32(p, 0x1c, 0x07d7_b140); p16(p, 0x20, 0xb140);      p16(p, 0x22, 27);
    p32(p, 0x24, 0x0148_0528); p32(p, 0x28, 0x001b_0528); p32(p, 0x2c, 0x001b_0528);
    p32(p, 0x30, 0x0000_0528); p32(p, 0x34, 0x0015_f900); p32(p, 0xe0, 31);
    p32(p, 0xe4, 0);           p32(p, 0xd0, 0x072d_79ff); p32(p, 0xd4, 0x0000_1b9e);
    p32(p, 0xd8, 0x072d_79ff); p32(p, 0xdc, 0x0000_1b9e); p8(p, 0xc1, 0);
    p8(p, 0xc9, 0);            p16(p, 0x106, 0xdfff);     p16(p, 0x108, 0xdfff);
    p8(p, 0x10a, 31);          p8(p, 0x39, 0);             p16(p, 0x3a, 0x0f0f);
    p32(p, 0x3c, 0x0101_0f0f); p32(p, 0x40, 0x0003_01cc); p8(p, 0x03, 0);
    p16(p, 0x7e, 460);         p8(p, 0x3f, 1);             p8(p, 0x89, 0);
    p16(p, 0x42, 3);           p32(p, 0x44, 0);             p32(p, 0x48, 0x0607_1440);
    let ctx = addr_of_mut!(MINIMAL_ADV_CTX) as u32;
    p32(p, 0x58, ctx); p32(p, 0x5c, ctx); p32(p, 0x60, ctx); p32(p, 0x64, ctx);
    p32(p, 0x7c, 0x01cc_0001); p32(p, 0x88, 0x0000_0700);
    p32(p, 0xc0, 0x0000_0300); p32(p, 0xc8, 0x0000_0300);
    for (i, b) in addr.iter().enumerate() { write_volatile(p.add(0xe8 + i), *b); }
}

// ── ISR ───────────────────────────────────────────────────────────────────────

#[ch32_hal::interrupt]
fn BB() { unsafe { hal::ble::bb_irq_lib_handler(); } }

#[ch32_hal::interrupt]
fn LLE() {}

// ── Main ──────────────────────────────────────────────────────────────────────

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

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
            ls: hal::rcc::LsConfig::default_lsi(),
            hspll_src: hal::rcc::HsPllSource::HSE,
            hspll: Some(hal::rcc::HsPll { pre: hal::rcc::HsPllPrescaler::DIV2 }),
        },
        ..Default::default()
    });

    hal::println!("DIR-B-MIN: trace binary (74e8d67 baseline, no P1/P2.5/P3)");

    unsafe {
        // ── Init (74e8d67 baseline) ───────────────────────────────────────────
        hal::ble::ble_hw_preamble();
        hal::ble::ble_ip_core_init();
        let r = read_volatile(0x4002_5004 as *const u32);
        write_volatile(0x4002_5004 as *mut u32, r | 0x0000_1100);
        ll_gblellpara_init(&ADDR);

        let ip = addr_of_mut!(gBleIPPara) as *mut u8;
        write_volatile(ip,              0x60u8);   // ip[0] = 0x60
        write_volatile(ip.add(4),       0x80u8);   // ip[4] = 0x80
        write_volatile(ip.add(16).cast::<u32>(), 776u32); // ip[16..19] = 776

        write_volatile(0x4002_4138 as *mut u32, 0xFF); // W1C stale IRQ flags
        qingke::riscv::asm::delay(72_000);

        // BASELINE: only BB IRQ 63. LLE IRQ 64 NOT enabled (P5 UNTESTED DELTA vs frozen).
        qingke::pfic::enable_interrupt(63);

        // ── T0 snapshot ───────────────────────────────────────────────────────
        hal::println!("TR init_done ll00={:#010x} ll08={:#010x} ll2c={:#010x} ll34={:#010x} ll38={:#010x} ll70={:#010x}",
            lr(0x00), lr(0x08), lr(0x2C), lr(0x34), lr(0x38), lr(0x70));
        hal::println!("TR init_done bb00={:#010x} bb04={:#010x} bb08={:#010x} bb0c={:#010x} bb64={:#010x} bb70={:#010x}",
            br(0x00), br(0x04), br(0x08), br(0x0C), br(0x64), br(0x70));
        hal::println!("TR init_done rf04={:#010x} rf08={:#010x} rf44={:#010x} rf9c={:#010x}",
            rr(0x04), rr(0x08), rr(0x44), rr(0x9C));
        hal::println!("TR init_done ip0={:#04x} ip4={:#04x} ip16={:#010x} txbuf={:#010x}",
            read_volatile(ip) as u32, read_volatile(ip.add(4)) as u32,
            read_volatile(ip.add(16).cast::<u32>()),
            addr_of!(TX_BUF) as u32);

        // ── Build PDU ─────────────────────────────────────────────────────────
        fill_tx_buf(&ADDR);
        hal::println!("TR pdu_built txbuf01={:#010x}", read_volatile(addr_of!(TX_BUF).cast::<u32>()));

        // ── Traced burst: ch37, BASELINE behavior ─────────────────────────────
        // Step 1: TX arm (BB_RF_FLAG_1M=9 in bits[30:25], TX arm bit0=1)
        let cfg = lr(0x2C);
        lw(0x2C, (cfg & 0x81FF_FFFF) | (9u32 << 25) | 0x1);
        hal::println!("TR b0_s1_post ll2c={:#010x}", lr(0x2C));

        // Step 2: BB+0x64 = 160 (baseline, no P1 formula)
        bw(0x64, 160);

        // Steps 3 (PLL): SKIPPED (fix #2)

        // Step 4: TX path select bits[8:7] = 10b
        lw(0x00, lr(0x00) & !0x180);
        lw(0x00, (lr(0x00) & !0x180) | 0x100);

        // Step 5: PA bias
        rw(0x08, rr(0x08) | 0x0033_0000);

        // Step 6: pre-delay
        bw(0x50, 90);

        // Step 7: PLL lock release
        rw(0x2C, rr(0x2C) & !(1 << 1));

        // Step 8: channel + whitening
        // BASELINE: bit6=1 (whitening ON) — DELTA vs frozen (bit6=0)
        lw(0x00, (lr(0x00) & !0x7F) | (37u32 | 0x40)); // 0x65 = ch37|bit6
        hal::println!("TR b0_s8_post ll00={:#010x} [DELTA:bit6=1_vs_frozen_bit6=0]", lr(0x00));

        // Step 9: ADV AA
        lw(0x08, 0x8E89_BED6);

        // Step 10: CRC init
        lw(0x04, 0x555555);

        // Step 11: TX buffer pointer
        bw(0x70, addr_of!(TX_BUF) as u32);
        hal::println!("TR b0_s11_post bb70_written={:#010x} bb70_readback={:#010x}",
            addr_of!(TX_BUF) as u32, br(0x70));

        // Pre-GO state
        let state_pre = lr(0x1C);
        hal::println!("TR b0_s12_pre ll1c={:#010x} bb08={:#010x}", state_pre, br(0x08));

        // Step 12: ADV-mode flag
        bw(0x04, br(0x04) | 0x1);

        // Step 12.5 BASELINE: PHY rate + lock only (no ip4 re-arm, no BB+0x64 formula)
        // DELTAS vs frozen: missing ip4=0x80 write, missing BB+0x0C save/restore, missing formula
        lw(0x00, lr(0x00) & !0x3000); // bits[13:12]=0 → 1Mbps
        bw(0x08, 0x2000);             // PHY rate lock
        hal::println!("TR b0_s125_post ll00={:#010x} bb08={:#010x} bb64={:#010x} ip4={:#04x} [DELTA:no_ip4_no_formula]",
            lr(0x00), br(0x08), br(0x64), read_volatile(ip.add(4)) as u32);

        // Step 13 BASELINE: GO bit23 only (no bit12)
        // DELTA vs frozen: frozen uses 0x0080_1000 (bit23+bit12)
        lw(0x00, lr(0x00) | 0x0080_0000);
        let state_post = lr(0x1C);
        let irq_post = br(0x08);
        hal::println!("TR b0_s13_post ll00={:#010x} ll1c={:#010x} bb08={:#010x} [DELTA:no_bit12]",
            lr(0x00), state_post, irq_post);

        // Step 14: clear TX arm
        lw(0x2C, lr(0x2C) & !0x3);

        // NO RFEND+0x9C clear — DELTA vs frozen (P4 untested)
        hal::println!("TR b0_s14_post ll2c={:#010x} rf9c={:#010x} [DELTA:no_P4_rf9c_clear]",
            lr(0x2C), rr(0x9C));

        // Step 15: TX trigger
        bw(0x00, 2);
        hal::println!("TR b0_s15_post bb08={:#010x} ll1c={:#010x}",
            br(0x08), lr(0x1C));

        // Wait for TX done
        let done = hal::ble::ble_tx_wait_done(10_000, 50_000);
        hal::println!("TR b0_wait done={} bb64={:#010x} bb08={:#010x} ll1c={:#010x}",
            done as u8, br(0x64), br(0x08), lr(0x1C));

        hal::println!("TR trace_end [minimal_baseline_74e8d67]");

        // Continue silently
        let mut tx_n = 0u32;
        let mut ok_total = 0u32;
        let adv_data = [2u8, 0x01, 0x06, 4, 0x09, b'c', b'b', b'a'];
        loop {
            let (ok, _) = hal::ble::adv::adv_event_verbose(&ADDR, true, &adv_data);
            tx_n += 1; ok_total += ok as u32;
            if tx_n <= 3 || tx_n % 100 == 0 {
                hal::println!("adv#{tx_n}: ok={ok}/3 total_ok={ok_total}");
            }
            qingke::riscv::asm::delay(2_400_000);
        }
    }
}
