// BLE PHY initialization for CH32V208.
//
// Phase 1: Hardware init sequence to bring RFEND/BB/LLE into operational state.
//
// Init order (from WCHBLE_Init → BLE_IPCoreInit in libwchble.a V1.40):
//   1. OSC HSE calibration (32 MHz crystal, done by caller before BLE init)
//   2. rfend_dev_init()  — RF frontend reset + analog config
//   3. bb_dev_init()     — Baseband processor config
//   4. lle_dev_init()    — Link layer engine timing + IRQ setup
//   5. Enable CRC peripheral clock (RCC AHB)
//   6. Enable NVIC interrupts: BB (IRQn 63), LLE (IRQn 64)
//
// Verification (Phase 1 milestone): DTM single-tone TX at 2402/2426/2480 MHz
// visible on spectrum analyzer or SDR confirms RF path is operational.

pub mod bb;
pub mod lle;
pub mod rfend;

use bb::bb_dev_init;
use lle::lle_dev_init;
use rfend::rfend_dev_init;

/// OSC base address: AHBPERIPH_BASE (0x40020000) + 0x202C = 0x4002202C
/// Contains HSE_CAL_CTRL at offset +0x00.
const OSC_HSE_CAL_CTRL: *mut u32 = 0x4002_202C as *mut u32;

/// RCC AHB peripheral enable register (AHBPCENR).
/// CH32V208: CRC clock bit is bit 6.
const RCC_AHBPCENR: *mut u32 = 0x4002_1014 as *mut u32;
const RCC_AHBPCENR_CRC_EN: u32 = 1 << 6;

/// Initialize the BLE PHY hardware layer.
///
/// Must be called after the system clock (HSE 32 MHz) is running and stable.
/// Enables CRC clock and NVIC interrupts BB/LLE.
///
/// # Safety
///
/// Must be called exactly once at startup, before any BLE operation.
/// Interrupts BB (63) and LLE (64) must have handlers registered before calling.
pub unsafe fn ble_phy_init() {
    // Configure 32 MHz crystal oscillator calibration.
    // From WCHBLE_Init (MCU.c): clear bits[30:28], set to 0x3 << 28, set 3 << 24.
    let osc = core::ptr::read_volatile(OSC_HSE_CAL_CTRL);
    let osc = (osc & !(0x07 << 28)) | (0x03 << 28);
    let osc = osc | (3 << 24);
    core::ptr::write_volatile(OSC_HSE_CAL_CTRL, osc);

    // RF frontend: reset then configure analog front end.
    rfend_dev_init();

    // Baseband processor: configure mode and timing.
    bb_dev_init(bb::BB_RF_FLAG_1M);

    // Link layer engine: configure timing params and IRQ mask.
    lle_dev_init();

    // Enable CRC peripheral clock (required by BLE stack).
    let ahb = core::ptr::read_volatile(RCC_AHBPCENR);
    core::ptr::write_volatile(RCC_AHBPCENR, ahb | RCC_AHBPCENR_CRC_EN);

    // Enable BB and LLE interrupts at the NVIC level.
    // CH32V208: BB=IRQn 63, LLE=IRQn 64.
    // PFIC IENR registers: IENR1 covers IRQ32-63, IENR2 covers IRQ64-95.
    const PFIC_IENR1: *mut u32 = 0xE000_E100 as *mut u32; // IRQ 32-63
    const PFIC_IENR2: *mut u32 = 0xE000_E104 as *mut u32; // IRQ 64-95
    // BB IRQn 63 → bit 31 of IENR1
    core::ptr::write_volatile(PFIC_IENR1, 1 << 31);
    // LLE IRQn 64 → bit 0 of IENR2
    core::ptr::write_volatile(PFIC_IENR2, 1 << 0);
}
