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
//   7. ble_reg_init()    — RF calibration (CO/GA tables; tail-called from BLE_IPCoreInit)
//
// Verification (Phase 1 milestone): DTM single-tone TX at 2402/2426/2480 MHz
// visible on spectrum analyzer or SDR confirms RF path is operational.

pub mod adv;
pub mod bb;
pub mod lle;
pub mod rfend;
pub mod regint;

pub use regint::ble_reg_init;

use bb::bb_dev_init;
use lle::lle_dev_init;
use rfend::rfend_dev_init;

// ── Shared BLE TX completion primitive ──────────────────────────────────────

/// gptrLLEReg base address — IRQ status at offset +0x08.
const GPTRLLE_BASE: usize = 0x40024200;

/// Wait for a BLE TX burst to fire and settle, using the DTM-style completion signal.
///
/// **Completion signal**: `BB+0x08` bits 29+25 (`0x2200_0000`) are set by the GO
/// strobe when a TX burst fires. These bits are sticky (not W1C-clearable): they
/// transition 0 → non-zero at the first TX after reset and remain set permanently.
///
/// **Timing contract**: this function MUST be called after the GO strobe has already
/// been issued. For the first TX, it waits for the 0→non-zero transition; for all
/// subsequent TXs, the bits are already set so it enters `settle_loops` immediately.
/// In both cases `settle_loops` provides the real timing guarantee — it is the
/// caller's responsibility to size it to cover the full over-the-air packet duration.
///
/// Per-burst fire detection: to observe whether each individual GO causes a new TX
/// (e.g. for diagnostics), write `bb_write(0x08, 0xFFFF_FFFF)` before each GO.
/// On verified hardware bits 29+25 are NOT cleared by this write, but the write does
/// clear bits[15:0] and is correct on other platforms where the sticky bits reset.
///
/// **BB+0x64 must NOT be used**: `gptrLLEReg+0x64` is event-driven and only
/// decrements during active hardware connection events (`RF_Tx` path, asm line 72938).
/// It never decrements for standalone DTM/ADV TX bursts — this function is correct.
///
/// # Parameters
///
/// * `timeout_loops` — spin_loop iterations to wait for fire detection (first TX
///   only; subsequent TXs detect immediately). Use `10_000` for ADV/DTM.
/// * `settle_loops` — spin_loop iterations after detection for on-air completion.
///   At 144 MHz, `50_000` ≈ 350 µs (covers 19-byte ADV_NONCONN_IND @ 1 Mbps:
///   90 µs pre-delay + 240 µs on-air = 330 µs).
///
/// # Safety
///
/// Must be called after `ble_phy_init()` and after the GO strobe has been written
/// to the BLE link-layer engine. Caller is responsible for TX mode setup.
pub unsafe fn ble_tx_wait_done(timeout_loops: u32, settle_loops: u32) -> bool {
    let irq_reg = (GPTRLLE_BASE + 0x08) as *const u32;
    for _ in 0..timeout_loops {
        if core::ptr::read_volatile(irq_reg) & 0x2200_0000 != 0 {
            // TX fired. Spin for packet-on-air completion before returning.
            for _ in 0..settle_loops {
                core::hint::spin_loop();
            }
            return true;
        }
        core::hint::spin_loop();
    }
    false
}

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

    // Init order from BLE_IPCoreInit in dtm.elf: LLE first, then RFEND, then BB.
    // LLE must be initialized before BB_DevInit fires the GO strobe.
    lle_dev_init();
    rfend_dev_init();
    bb_dev_init(bb::BB_RF_FLAG_1M);

    // Enable CRC peripheral clock (required by BLE stack).
    let ahb = core::ptr::read_volatile(RCC_AHBPCENR);
    core::ptr::write_volatile(RCC_AHBPCENR, ahb | RCC_AHBPCENR_CRC_EN);

    // NOTE: BB (IRQn 63) and LLE (IRQn 64) interrupts are NOT enabled here.
    // The DTM polling TX path reads IRQ status directly via register polling.
    // Enabling IRQs without registered handlers causes unhandled exception faults.
    // Applications that use interrupt-driven BLE (TMOS/BLE stack) must enable
    // them separately after installing BB_IRQHandler / LLE_IRQHandler.

    // RF calibration: CO/GA lookup tables (mirrors BLE_RegInit tail-call in BLE_IPCoreInit).
    ble_reg_init();
}
