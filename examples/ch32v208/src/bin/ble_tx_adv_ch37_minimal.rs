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
// These Rust strong symbols override lib COMMON BSS. Layout enforced by Iron Law
// #37 ASSERT pins in examples/ch32v208/build.rs. Addresses:
//   dtmFlag    @ 0x20000750   gPaControl @ 0x20000754   gBleIPPara @ 0x20000758
//
// T1: MMIO register pointer cache (DATA, initialized).
#[no_mangle] pub static mut gptrBBReg:    u32 = 0x4002_4100; // WCH "BB"  = lle_* range
#[no_mangle] pub static mut gptrLLEReg:   u32 = 0x4002_4200; // WCH "LLE" = bb_*  range
#[no_mangle] pub static mut gptrAESReg:   u32 = 0x4002_4300;
#[no_mangle] pub static mut gptrRFENDReg: u32 = 0x4002_5000;

// T2: BSS globals — sizes must match lib COMMON (ble=64B, gBleLlPara=296B, gBleIPPara=40B).
#[no_mangle] pub static mut ble: [u32; 16] = [0u32; 16]; // 64B

// T3: scalars with GlobalMerge isolation (zz_* sub-section suffix, placed after main .bss).
#[no_mangle] #[link_section = ".bss.zz_gpa"] pub static mut gPaControl: u32 = 0; // 4B
#[no_mangle] #[link_section = ".bss.zz_dtm"] pub static mut dtmFlag:    u8  = 0; // 1B

// #35: gBleLlPara — GlobalMerge isolation via named sub-section.
#[no_mangle] #[link_section = ".bss.gBleLlPara"] pub static mut gBleLlPara: [u32; 74] = [0u32; 74]; // 296B

// #34: gBleIPPara @ 0x20000758 — Iron Law #37. GlobalMerge isolation required (Iron Law #34).
#[no_mangle] #[link_section = ".bss.gBleIPPara"] pub static mut gBleIPPara: [u32; 10] = [0u32; 10]; // 40B

// T8: fnGetClockCBs @ 0x20001c78 — outside _ebss boundary.
// Iron Law #36: _ebss = 0x20001c78 (exclusive). ROM unconditionally installs 0x420B000A.
#[no_mangle] #[link_section = ".fnGetClockCBs"] pub static mut fnGetClockCBs: u32 = 0;

// ── Device address ────────────────────────────────────────────────────────────
/// AdvA: 6-byte BD address, LE order. On-air displays as C2:21:43:65:87:12 (random static).
const ADDR: [u8; 6] = [0x12, 0x87, 0x65, 0x43, 0x21, 0xC2];

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
