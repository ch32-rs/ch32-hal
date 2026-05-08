//! BLE ADV_NONCONN_IND minimal example — CH32V208, T44.E.
//!
//! Standalone BLE advertising using the `hal::ble::adv` library API (no inline
//! 15-step register sequence). Transmits ADV_NONCONN_IND on all 3 advertising
//! channels (ch37/ch38/ch39) using `adv_event_verbose`.
//!
//! For the frozen reference binary with full diagnostic history and dead-code
//! archaeology, see `ble_tx_adv_ch37.rs` (T8 baseline, commit f27c394).
//!
//! Validation: nRF Connect Scanner → AdvA = C2:21:43:65:87:12, name = "cba".

#![no_std]
#![no_main]

use core::ptr::{addr_of_mut, write_volatile};
use {ch32_hal as hal, panic_halt as _};

// ── BSS contract symbols ──────────────────────────────────────────────────────
//
// Iron Law #34 v5 final (2026-05-08, locked by ROM hex disassembly + Cindy 06:28
// RAM dump + frozen drift cba=72/61 PASS): ROM is RAM-layout-agnostic for the 6
// BSS-contract symbols. Phase 2a (this commit) drops `#[link_section]` pins on
// the 5 non-fnGetClockCBs symbols; LLVM natural BSS ordering applies. `#[no_mangle]`
// + `#[used]` retain the symbols against --gc-sections (independent of pinning).
//
// fnGetClockCBs keeps `#[link_section = ".fnGetClockCBs"]` as a 4B zero-init
// placeholder at 0x20001c78 (task #56, 2026-05-08). Runtime indirection retired:
// bb_irq_lib_handler calls crate::ble::fallback_clock() directly. The byte stays
// at 0x20001c78 INSIDE [_sbss, _ebss) so chip silicon ROM (if hardcoded read) sees
// NULL on every reset and triggers its auto-install fallback path.
//
// T1: MMIO register pointer cache (DATA, initialized).
#[no_mangle] pub static mut gptrBBReg:    u32 = 0x4002_4100; // WCH "BB"  = lle_* range
#[no_mangle] pub static mut gptrLLEReg:   u32 = 0x4002_4200; // WCH "LLE" = bb_*  range
#[no_mangle] pub static mut gptrAESReg:   u32 = 0x4002_4300;
#[no_mangle] pub static mut gptrRFENDReg: u32 = 0x4002_5000;

// T2: BSS globals — sizes match historical lib COMMON. `#[used]` keeps `ble` (no live
// Rust refs in this binary) against --gc-sections.
#[no_mangle] #[used] pub static mut ble: [u32; 16] = [0u32; 16]; // 64B

// T3: scalars (GlobalMerge isolation no longer needed under v5).
#[no_mangle] pub static mut gPaControl: u32 = 0; // 4B
#[no_mangle] pub static mut dtmFlag:    u8  = 0; // 1B

// gBleLlPara — 296B LL param block. `#[used]` retains it (live caller exists in
// `ll_gblellpara_init` below, but `#[used]` is harmless and documents intent).
#[no_mangle] #[used] pub static mut gBleLlPara: [u32; 74] = [0u32; 74]; // 296B

// gBleIPPara — 40B IP param block. Hot in BB ISR; live caller present.
#[no_mangle] pub static mut gBleIPPara: [u32; 10] = [0u32; 10]; // 40B

// fnGetClockCBs @ 0x20001c78 — 4B zero-init placeholder (task #56, 2026-05-08).
// INSIDE [_sbss, _ebss) so startup zero-init covers it. Slot is no longer read
// at runtime — bb_irq_lib_handler calls crate::ble::fallback_clock() directly.
#[no_mangle] #[link_section = ".fnGetClockCBs"] pub static mut fnGetClockCBs: u32 = 0;

// ── Device address ────────────────────────────────────────────────────────────
/// AdvA: 6-byte BD address, LE order. On-air displays as C2:21:43:65:87:12 (random static).
const ADDR: [u8; 6] = [0x12, 0x87, 0x65, 0x43, 0x21, 0xC2];

// ── gBleLlPara init context ──────────────────────────────────────────────────
//
// Fix #4 (T44.E): mirror frozen binary's ll_init_safe_prefix() + seed_ble_bd_addr().
// gBleLlPara is 296B (74×u32) at 0x20000508. ISR (bb_irq_lib_handler) does NOT directly
// read gBleLlPara, but some ROM/lib paths during init may. Frozen binary populates
// ~30 fields before the TX loop; minimal binary leaves all as BSS zero.
//
// MINIMAL_ADV_CTX: 192B scratch buffer equivalent to RUST_ADV_CTX in frozen binary.
// Written to gBleLlPara+0x58..0x64 (pAdvCtx pointer slots) by ll_gblellpara_init.
// D-1a.0b forensic (2026-05-04): vtable slots 0x68..0x74 are never dereferenced in Path C;
// 0x58..0x64 (adv_ctx) slots included for parity with frozen binary's init sequence.
#[link_section = ".bss"]
static mut MINIMAL_ADV_CTX: [u8; 192] = [0u8; 192];

#[inline(always)]
unsafe fn wll_u32(p: *mut u8, off: usize, v: u32) {
    core::ptr::write_volatile(p.add(off) as *mut u32, v);
}
#[inline(always)]
unsafe fn wll_u16(p: *mut u8, off: usize, v: u16) {
    core::ptr::write_volatile(p.add(off) as *mut u16, v);
}
#[inline(always)]
unsafe fn wll_u8(p: *mut u8, off: usize, v: u8) {
    core::ptr::write_volatile(p.add(off), v);
}

/// Mirror of frozen binary's ll_init_safe_prefix() + seed_ble_bd_addr().
///
/// Populates gBleLlPara with the timing/frequency/vtable fields that WCH LL_Init
/// writes. The frozen binary (ble_tx_adv_ch37.rs) calls this before the TX loop and
/// achieves cba=72. Minimal binary leaves gBleLlPara as BSS zeros; this fixes that gap.
///
/// Field values derived directly from frozen binary ll_init_safe_prefix() (line 292).
unsafe fn ll_gblellpara_init(addr: &[u8; 6]) {
    let p = core::ptr::addr_of_mut!(gBleLlPara) as *mut u8;

    // ── LL timing / frequency parameters (from WCH LL_Init safe prefix) ─────
    wll_u32(p, 0x00, 0x0000_0004);
    wll_u32(p, 0x14, 0x07d7_000d);
    wll_u32(p, 0x18, 0x0d0d_b140);
    wll_u32(p, 0x1c, 0x07d7_b140);
    wll_u16(p, 0x20, 0xb140);
    wll_u16(p, 0x22, 27);
    wll_u32(p, 0x24, 0x0148_0528);
    wll_u32(p, 0x28, 0x001b_0528);
    wll_u32(p, 0x2c, 0x001b_0528);
    wll_u32(p, 0x30, 0x0000_0528);
    wll_u32(p, 0x34, 0x0015_f900);
    wll_u32(p, 0xe0, 31);
    wll_u32(p, 0xe4, 0);
    wll_u32(p, 0xd0, 0x072d_79ff);
    wll_u32(p, 0xd4, 0x0000_1b9e);
    wll_u32(p, 0xd8, 0x072d_79ff);
    wll_u32(p, 0xdc, 0x0000_1b9e);
    wll_u8(p, 0xc1, 0);
    wll_u8(p, 0xc9, 0);
    wll_u16(p, 0x106, 0xdfff);
    wll_u16(p, 0x108, 0xdfff);
    wll_u8(p, 0x10a, 31);
    wll_u8(p, 0x39, 0);
    wll_u16(p, 0x3a, 0x0f0f);
    wll_u32(p, 0x3c, 0x0101_0f0f);
    wll_u32(p, 0x40, 0x0003_01cc);
    wll_u8(p, 0x03, 0);
    wll_u16(p, 0x7e, 460);
    wll_u8(p, 0x3f, 1);
    wll_u8(p, 0x89, 0);
    wll_u16(p, 0x42, 3);
    wll_u32(p, 0x44, 0);
    wll_u32(p, 0x48, 0x0607_1440);

    // ── pAdvCtx pointer slots (gBleLlPara+0x58..0x64) ────────────────────────
    // Frozen binary writes RUST_ADV_CTX address here. Slots 0x68..0x74 (llAdvertise*
    // dispatch) omitted — D-1a.0b proved they are never dereferenced in Path C.
    let adv_ctx = core::ptr::addr_of_mut!(MINIMAL_ADV_CTX) as u32;
    wll_u32(p, 0x58, adv_ctx);
    wll_u32(p, 0x5c, adv_ctx);
    wll_u32(p, 0x60, adv_ctx);
    wll_u32(p, 0x64, adv_ctx);

    // ── Other config/vtable slots ─────────────────────────────────────────────
    wll_u32(p, 0x7c, 0x01cc_0001);
    wll_u32(p, 0x88, 0x0000_0700);
    wll_u32(p, 0xc0, 0x0000_0300);
    wll_u32(p, 0xc8, 0x0000_0300);

    // ── BD address → gBleLlPara+0xE8 (seed_ble_bd_addr equivalent) ──────────
    // Frozen binary: LL_AddrInit copies ble[0x18..0x1d] → gBleLlPara+0xE8 via tmos_memcpy.
    // In minimal path (no LL_AddrInit): write directly.
    for (i, b) in addr.iter().enumerate() {
        core::ptr::write_volatile(p.add(0xe8 + i), *b);
    }
}

// ── ISR handlers ─────────────────────────────────────────────────────────────
/// BB IRQ (IRQn 63): drives .L6 TX-advance path (gBleIPPara[4] state machine).
/// Without this handler the PHY stays at warmup (0x33) and no packet is emitted.
#[ch32_hal::interrupt]
fn BB() {
    unsafe { hal::ble::bb_irq_lib_handler(); }
}

/// LLE IRQ (IRQn 64): not used in standalone ADV TX path — empty stub.
#[ch32_hal::interrupt]
fn LLE() {}

// ── Entry point ──────────────────────────────────────────────────────────────
#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();

    // 96 MHz from HSE 32 MHz crystal (required for RFEND calibration).
    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config {
            hse: Some(hal::rcc::Hse {
                freq: hal::time::Hertz(32_000_000),
                mode: hal::rcc::HseMode::Oscillator,
            }),
            sys:      hal::rcc::Sysclk::PLL,
            pll_src:  hal::rcc::PllSource::HSE,
            pll: Some(hal::rcc::Pll {
                prediv: hal::rcc::PllPreDiv::DIV4,
                mul:    hal::rcc::PllMul::MUL12,
            }),
            pllx:     None,
            ahb_pre:  hal::rcc::AHBPrescaler::DIV1,
            apb1_pre: hal::rcc::APBPrescaler::DIV1,
            apb2_pre: hal::rcc::APBPrescaler::DIV1,
            ls:       hal::rcc::LsConfig::default_lsi(),
            hspll_src: hal::rcc::HsPllSource::HSE,
            hspll: Some(hal::rcc::HsPll {
                pre: hal::rcc::HsPllPrescaler::DIV2,
            }),
        },
        ..Default::default()
    });

    hal::println!("BLE ADV minimal (T44.E) — AdvA=C2:21:43:65:87:12 name=cba");

    unsafe {
        // BLE hardware init: HSE/clocks, then LLE → RFEND → BB → ble_reg_init.
        hal::ble::ble_hw_preamble();
        hal::ble::ble_ip_core_init();

        // H19: RFEND+0x04 bits[12:8] = TX-path analog enable (EVT post-init confirmed).
        let r = core::ptr::read_volatile(0x4002_5004 as *const u32);
        core::ptr::write_volatile(0x4002_5004 as *mut u32, r | 0x0000_1100);

        // Fix #4 (T44.E): populate gBleLlPara and gBleLlPara+0xE8 BD address.
        // Frozen binary calls ll_init_safe_prefix() + seed_ble_bd_addr() before TX loop.
        // Minimal binary was leaving gBleLlPara as BSS zeros — fixed here.
        ll_gblellpara_init(&ADDR);

        // gBleIPPara[0] = 0x60 (bit5+bit6): arms scan-mode ISR path (Iron Law #27).
        // Without this, BB ISR bit5 path never fires → .L6 never triggers → cba=0.
        let ip = addr_of_mut!(gBleIPPara) as *mut u8;
        write_volatile(ip, 0x60u8);

        // gBleIPPara[4] = 0x80: pre-arm .L6 for the first burst (bit7=1, bit6=0).
        // Subsequent bursts are re-armed by .L4/.L8 cleanup paths (set ip[4]=1, bit6=0).
        write_volatile(ip.add(4), 0x80u8);

        // gBleIPPara[16..19] = 776: timer written by ISR .L6 to BB+0x64 per burst.
        // Value confirmed from EVT pre-GO state (BB+0x64=0x308=776, T8 forensic).
        write_volatile(ip.add(16).cast::<u32>(), 776u32);

        // W1C-clear BB+0x38 (gptrBBReg+0x38) stale IRQ flags before enabling BB IRQ.
        // Without this, stale bit6 triggers an immediate ISR on enable_interrupt(63),
        // causing an IRQ storm before the GO strobe fires (v7-probe.3-fix2 root cause).
        core::ptr::write_volatile(0x4002_4138 as *mut u32, 0xFF);

        // Pre-GO settle: ≈500µs @144MHz/4cyc per iter. Lets any residual ISR activity
        // settle after enable before the first GO strobe.
        qingke::riscv::asm::delay(72_000);

        // Enable BB IRQ (IRQn 63). LLE (64) stays masked — not needed for ADV TX path.
        qingke::pfic::enable_interrupt(63);

        // Build AD payload once: Flags(3B) + Complete Local Name "cba"(5B) = 8B total.
        let mut ad_buf = [0u8; 31];
        let mut ad_pos = 0usize;
        ad_pos += hal::ble::adv::ad_flags(&mut ad_buf[ad_pos..], 0x06);
        ad_pos += hal::ble::adv::ad_complete_name(&mut ad_buf[ad_pos..], b"cba");
        let adv_data = &ad_buf[..ad_pos];

        let mut tx_n: u32 = 0;
        let mut ok_total: u32 = 0;

        loop {
            let (ok, _stats) = hal::ble::adv::adv_event_verbose(&ADDR, true, adv_data);
            tx_n += 1;
            ok_total += ok as u32;

            // Print first 5 events, then every 100th.
            if tx_n <= 5 || tx_n % 100 == 0 {
                hal::println!("adv#{tx_n}: ok={ok}/3 total_ok={ok_total}");
            }

            // ~100 ms inter-event gap (2_400_000 iters @96MHz/4cyc ≈ 100ms).
            qingke::riscv::asm::delay(2_400_000);
        }
    }
}
