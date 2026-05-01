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
/// CH32V208: CRC clock bit is bit 6; BLE controller clocks are bits 16+17.
/// From ch32v20x_rcc.h L134: RCC_AHBPeriph_BLE_CRC = 0x00030040
///   bit  6 = CRC peripheral clock
///   bit 16 = BLEC (BLE controller clock, also gates BB register writes)
///   bit 17 = BLES (BLE schedule clock)
/// ALL three must be enabled BEFORE bb_dev_init — BB registers (including the
/// bit23 analog-enable strobe) are non-retentive without the BLE controller clock.
const RCC_AHBPCENR: *mut u32 = 0x4002_1014 as *mut u32;
const RCC_AHBPCENR_BLE_CRC_EN: u32 = 0x0003_0040; // bits 6 + 16 + 17

/// RCC CTLR register address.
/// bit16=HSEON, bit17=HSERDY (32 MHz external crystal).
const RCC_CTLR: *mut u32 = 0x4002_1000 as *mut u32;

/// Initialize the BLE PHY hardware layer.
///
/// Enables the HSE 32 MHz external crystal (required by the BLE RF frontend as its
/// reference oscillator), then runs rfend/bb/lle init and RF calibration.
///
/// **Clock note**: the CH32V208WBU6 BLE RF frontend requires HSE (the 32 MHz external
/// crystal) to be oscillating regardless of the system clock source. If `hal::init` was
/// called with `Default::default()` (which leaves HSEON=0), this function enables HSE
/// before proceeding.  The system CPU clock is left unchanged — only HSE is enabled.
///
/// # Safety
///
/// Must be called exactly once at startup, before any BLE operation.
/// Interrupts BB (63) and LLE (64) must have handlers registered before calling.
pub unsafe fn ble_phy_init() {
    // ── Step 0: 32 MHz crystal oscillator calibration ────────────────────────
    // Configure load capacitance (bits[30:28]) and drive current (bits[25:24])
    // BEFORE enabling HSE, so the analog parameters are settled when oscillation starts.
    // Values from WCHBLE_Init (MCU.c): load = 0x3 << 28, drive = 3 << 24.
    // NOTE: If HSE fails to start, try commenting out this block to use reset-default
    // values — crystal-specific parameters may differ from the WCH reference board.
    let osc = core::ptr::read_volatile(OSC_HSE_CAL_CTRL);
    let osc = (osc & !(0x07 << 28)) | (0x03 << 28);
    let osc = osc | (3 << 24);
    core::ptr::write_volatile(OSC_HSE_CAL_CTRL, osc);

    // ── Step 1: ensure HSE (32 MHz external crystal) is running ──────────────
    // The BLE RF frontend uses HSE as its reference oscillator.
    // WCH official SDK relies on SystemInit() to enable HSE before BLE init;
    // our Rust hal::init(Default::default()) leaves HSEON=0 (HSI only).
    // We enable HSE here if not already running.
    let ctlr = core::ptr::read_volatile(RCC_CTLR);
    if ctlr & (1 << 17) == 0 {
        // HSERDY=0 — enable HSE in oscillator mode (HSEBYP=0, crystal on OSC_IN/OUT).
        // Explicitly clear HSEBYP (bit 18) to guarantee oscillator mode regardless of
        // any previous state, then set HSEON (bit 16).
        core::ptr::write_volatile(RCC_CTLR, (ctlr & !(1 << 18)) | (1 << 16));
        let mut i = 0u32;
        while core::ptr::read_volatile(RCC_CTLR) & (1 << 17) == 0 {
            i += 1;
            if i > 500_000 {
                // HSE did not start within ~300 ms. Break and let the caller's
                // post-init diagnostic print the ctlr/hserdy values — panic_halt
                // would swallow them. Downstream RF will not function, but the
                // "post: hseon=? hserdy=? hsebyp=? osc_cal=?" line will still appear.
                break;
            }
            core::hint::spin_loop();
        }
    }

    // ── Step 2: Enable BLE controller clocks BEFORE any dev_init ────────────
    // RCC_AHBPeriph_BLE_CRC (0x00030040) = bits 6 (CRC) + 16 (BLEC) + 17 (BLES).
    // BB register writes (especially the bit23 analog-enable strobe) are
    // non-retentive without BLEC/BLES running — bit23 self-clears immediately if
    // the BLE controller clock is gated. Hardware confirmed: with only bit6 (CRC)
    // enabled, BB+0x00 bit23 latches momentarily but is cleared by the next write.
    // Must be done BEFORE lle_dev_init / rfend_dev_init / bb_dev_init.
    let ahb = core::ptr::read_volatile(RCC_AHBPCENR);
    core::ptr::write_volatile(RCC_AHBPCENR, ahb | RCC_AHBPCENR_BLE_CRC_EN);

    // Init order: LLE → RFEND → BB → ble_reg_init (RF calibration, always last).
    //
    // CRITICAL ORDER: ble_reg_init() MUST run AFTER all three dev_inits.
    // WCH BLE_IPCoreInit (asm L71209-71248) tail-calls ble_reg_init as the final
    // step after all three dev_inits complete. bb_dev_init() enables the BB analog
    // domain (bit23 strobe) and timing registers needed by the calibration PLL.
    //
    // Note: the exact relative order of rfend/bb/lle dev_inits is uncertain from asm
    // (call 1/2/3 at L71239-71243 not decoded individually). Current order below
    // matches WCH SDK MCU.c (RFEND → BB → LLE) and hardware-validated.
    lle_dev_init();
    rfend_dev_init();
    bb_dev_init(bb::BB_RF_FLAG_1M);

    // RF calibration last — always, unconditionally.
    ble_reg_init();

    // NOTE: BB (IRQn 63) and LLE (IRQn 64) interrupts are NOT enabled here.
    // The DTM polling TX path reads IRQ status directly via register polling.
    // Enabling IRQs without registered handlers causes unhandled exception faults.
    // Applications that use interrupt-driven BLE (TMOS/BLE stack) must enable
    // them separately after installing BB_IRQHandler / LLE_IRQHandler.
}
