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
pub mod listener;
pub mod rfend;
pub mod regint;
pub mod tracer;
pub mod types;

pub use regint::ble_reg_init;
// Re-export individual init steps so examples can call them with dump points between
// stages (Phase B multi-stage diff, task #16).
pub use bb::{bb_dev_init, bb_irq_lib_handler, BB_RF_FLAG_1M};
pub use lle::lle_dev_init;
pub use rfend::rfend_dev_init;

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

/// RCC RSTSCKR register address.
/// bit0=LSION, bit1=LSIRDY on CH32V20x. ch32-hal's `LsConfig::init()` is not
/// implemented yet, so BLE examples must force-start LSI here for the BLE slow
/// scheduling clock domain.
const RCC_RSTSCKR: *mut u32 = 0x4002_1024 as *mut u32;

/// Enable the 32 MHz HSE crystal oscillator and BLE/CRC peripheral clocks.
///
/// This is the clock-setup preamble shared by [`ble_phy_init`] and the Phase B
/// staged-init path. Extracted so examples can insert `tracer::dump_regs` calls
/// between individual hardware-init stages without duplicating clock-setup code.
///
/// **Idempotent**: the HSE-ready check and the RCC OR-mask are both safe to call
/// multiple times (HSERDY poll exits immediately if HSE is already running; the
/// bit-OR of already-set clock bits is a no-op).
///
/// # Safety
///
/// Must be called before any of `lle_dev_init`, `rfend_dev_init`, `bb_dev_init`,
/// or `ble_reg_init`. Safe to call before `hal::init` returns, but must be called
/// from machine-mode context.
pub unsafe fn ble_hw_preamble() {
    // ── Step 0: 32 MHz crystal oscillator calibration ────────────────────────
    // Configure load capacitance (bits[30:28]) and drive current (bits[25:24])
    // BEFORE enabling HSE so the analog parameters are settled when oscillation
    // starts. Values from WCHBLE_Init (MCU.c): load = 0x3 << 28, drive = 3 << 24.
    let osc = core::ptr::read_volatile(OSC_HSE_CAL_CTRL);
    let osc = (osc & !(0x07 << 28)) | (0x03 << 28);
    let osc = osc | (3 << 24);
    core::ptr::write_volatile(OSC_HSE_CAL_CTRL, osc);

    // ── Step 1: ensure HSE (32 MHz external crystal) is running ──────────────
    // The BLE RF frontend uses HSE as its reference oscillator.
    // WCH official SDK relies on SystemInit() to enable HSE before BLE init;
    // our Rust hal::init(Default::default()) leaves HSEON=0 (HSI only).
    let ctlr = core::ptr::read_volatile(RCC_CTLR);
    if ctlr & (1 << 17) == 0 {
        // HSERDY=0 — enable in oscillator mode (HSEBYP=0, crystal on OSC_IN/OUT).
        // Explicitly clear HSEBYP (bit 18) then set HSEON (bit 16).
        core::ptr::write_volatile(RCC_CTLR, (ctlr & !(1 << 18)) | (1 << 16));
        let mut i = 0u32;
        while core::ptr::read_volatile(RCC_CTLR) & (1 << 17) == 0 {
            i += 1;
            if i > 500_000 {
                // HSE did not start within ~300 ms. Break so the caller's
                // post-init diagnostic can still print CTLR values.
                break;
            }
            core::hint::spin_loop();
        }
    }

    // ── Step 2: Enable BLE controller clocks BEFORE any dev_init ────────────
    // RCC_AHBPeriph_BLE_CRC (0x00030040) = bits 6 (CRC) + 16 (BLEC) + 17 (BLES).
    // BB register writes (bit23 analog-enable strobe) are non-retentive without
    // BLEC/BLES running. Must be set before lle/rfend/bb dev_init calls.
    let ahb = core::ptr::read_volatile(RCC_AHBPCENR);
    core::ptr::write_volatile(RCC_AHBPCENR, ahb | RCC_AHBPCENR_BLE_CRC_EN);

    // ── Step 3: Enable LSI slow clock for BLE scheduling ───────────────────
    // The HAL stores an `LsConfig`, but the v3 RCC init path does not yet
    // consume it. WCH BLE scheduling depends on a running low-speed clock.
    let rstsckr = core::ptr::read_volatile(RCC_RSTSCKR);
    core::ptr::write_volatile(RCC_RSTSCKR, rstsckr | 0x1);
    let mut i = 0u32;
    while core::ptr::read_volatile(RCC_RSTSCKR) & 0x2 == 0 {
        i += 1;
        if i > 1_000_000 {
            break;
        }
        core::hint::spin_loop();
    }
}

// ── WCH lib global MMIO pointer cache ────────────────────────────────────────
//
// BLE_IPCoreInit writes these BSS variables so BB_IRQLibHandler / LLE_IRQSubHandler
// can find the hardware base addresses at runtime.  They must be populated before
// any lib IRQ handler fires — even when we replace BLE_IPCoreInit with Rust code.
extern "C" {
    static mut gptrLLEReg:   u32; // set to 0x40024200 (WCH "LLE" = our bb_* range)
    static mut gptrBBReg:    u32; // set to 0x40024100 (WCH "BB"  = our lle_* range)
    static mut gptrRFENDReg: u32; // set to 0x40025000
    static mut gptrAESReg:   u32; // set to 0x40024300
    static mut gPaControl:   u32; // ROM-expected 0x20000754; Iron Law #37 (ASSERT-pinned, build.rs)
    static mut dtmFlag:      u8;  // ROM-expected 0x20000750; Iron Law #37 (ASSERT-pinned, build.rs)
    static mut gBleIPPara:   u8;  // 40-byte array; access as *mut u8 + byte offset
                                  // ROM-expected 0x20000758; Iron Law #37 (ASSERT-pinned, build.rs)
    static mut fnGetClockCBs: u32; // linker-placed BSS (Phase 2c: no address pin).
                                   // ble_ip_core_init writes 0x420B000A (ROM_RTC_TICK_FN)
                                   // explicitly after startup zeroing (= TMOS_TimerInit(0) equiv).
}

/// Pure-Rust replacement for WCH `BLE_IPCoreInit`.
///
/// Sets the lib global MMIO pointer cache (required by `BB_IRQLibHandler` and
/// `LLE_IRQSubHandler` at runtime), then calls the four sub-init functions in
/// WCH order: `LLE_DevInit → RFEND_DevInit → BB_DevInit → BLE_RegInit`.
///
/// Once this function succeeds, the `extern "C" fn BLE_IPCoreInit` FFI symbol
/// is no longer needed and can be removed from the example's extern block.
///
/// # WCH PDF HAL init flow vs. this implementation (WCH BLE manual §4.4.2)
///
/// | WCH PDF step                      | This implementation            | Notes |
/// |-----------------------------------|--------------------------------|-------|
/// | `HAL_Init()` → SysClock + NVIC   | `hal::init()` by caller        | Done before `ble_hw_preamble` |
/// | `HAL_TimeInit()` / RTC init       | **Explicit write** (Step 1)    | `TMOS_TimerInit(0)` = use RTC (PDF §8.1.1); we write `0x420B000A` directly (ROM's built-in RTC tick fn) since we don't call `TMOS_TimerInit`. ROM does NOT auto-install (baf71de gate). `fnGetClockCBs` is a COMMON/PCREL symbol — linker-placed, no address pin (Phase 2c; C ground truth 2026-05-09). |
/// | `WCHBLE_Init()` / `BLE_LibInit()` | **Skipped** (no libwchble)     | Replaced by `ble_ip_core_init` + Rust sub-inits |
/// | `BLE_IPCoreInit()`                | This function                  | MMIO cache + 4 sub-inits |
/// | `TMOS_TimerInit()` / clock config | **Skipped** (no TMOS)          | Not needed for ADV-only TX path |
/// | `GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN/MAX, 160)` | BB+0x64=160 in adv_event | PDF §8.2.2: 160 × 0.625 ms = 100 ms |
/// | `GAPRole_PeripheralInit()`        | **Not applicable**             | We use non-connectable ADV TX only |
/// | Periodic `tmos_systemProcess()`   | **Skipped** (no TMOS)          | Caller drives TX loop directly |
/// | Sleep / LSI calibration callbacks | **Skipped** (no power mgmt)    | `PfnIdleCb` / `PfnLsiCalibrationCb` unused |
///
/// # Safety
///
/// Must be called after [`ble_hw_preamble`] (HSE + BLE clocks enabled) and
/// before any BLE TX/RX operation or IRQ delivery.
pub unsafe fn ble_ip_core_init() {
    use core::ptr::{addr_of_mut, write_volatile};

    // ── Glue: mirror BLE_IPCoreInit steps 1-7 ────────────────────────────────
    // ── Step 0: install ROM's built-in RTC tick fn into fnGetClockCBs slot ──────
    // PDF §8.1.1: `TMOS_TimerInit(0)` = "选择 RTC 作为系统时钟" — internally the
    // library writes the ROM's RTC tick fn address into this slot. Since we never
    // call `TMOS_TimerInit`, we must do this explicitly.
    //
    // 0x420B000A: ROM-internal LSI/RTC-derived tick fn on CH32V208WBU6 B1 silicon.
    //   Returns a u32 counter at 1600 Hz / 625 µs per tick (PDF §3.3, Iron Law #35).
    //   The bb_irq_lib_handler .L7 path reads this slot and stores the return value
    //   at gBleIPPara[0x1c] for use by the BLE scheduler.
    //
    // Hardware-verified (2026-05-08):
    //   - ROM does NOT auto-install: baf71de (NULL slot) → cba=0, slot stayed 0x00000000
    //   - This write = equivalent of calling `TMOS_TimerInit(NULL)` per PDF
    //   - B1-only: 0x420B000A is the ROM fn address for CH32V208WBU6 B1 silicon.
    //     Other silicon revisions may use a different address (not yet verified).
    const ROM_RTC_TICK_FN: u32 = 0x420B_000A; // B1 silicon ROM internal tick fn
    write_volatile(addr_of_mut!(fnGetClockCBs), ROM_RTC_TICK_FN);
    debug_assert_eq!(
        core::ptr::read_volatile(core::ptr::addr_of!(fnGetClockCBs)),
        ROM_RTC_TICK_FN,
        "fnGetClockCBs write verify failed"
    );

    // Steps 1-2: clear dtmFlag + gPaControl (BSS already 0; explicit write for compat)
    write_volatile(addr_of_mut!(dtmFlag),    0u8);
    write_volatile(addr_of_mut!(gPaControl), 0u32);
    // Steps 3-6: MMIO base pointer cache
    //   BB_IRQLibHandler reads gptrBBReg → 0x40024100 (our lle_* base)
    //   LLE_IRQSubHandler reads gptrLLEReg → 0x40024200 (our bb_* base)
    write_volatile(addr_of_mut!(gptrAESReg),   0x4002_4300u32);
    write_volatile(addr_of_mut!(gptrLLEReg),   0x4002_4200u32);
    write_volatile(addr_of_mut!(gptrRFENDReg), 0x4002_5000u32);
    write_volatile(addr_of_mut!(gptrBBReg),    0x4002_4100u32);
    // Step 7: gBleIPPara[7] = 1 — WCH BLE_RegInit uses this as a run-once guard.
    //   Our Rust ble_reg_init() does not check it, but write for future-compat.
    let ip = addr_of_mut!(gBleIPPara) as *mut u8;
    write_volatile(ip.add(7), 1u8);
    // Steps 8-9: gBleIPPara[8/9] = ble_ptr+272 / ble_ptr
    //   WCH LLE_DevInit reads gBleIPPara[9] for its DMA buffer address.
    //   Our Rust lle_dev_init() uses a local LLE_DMA_BUF static instead.
    //   `ble` is NULL in our path (BLE_LibInit never called), so these would
    //   write 0 / 272 — skip to avoid a NULL DMA pointer in gBleIPPara[9].

    // ── Sub-inits in WCH order ────────────────────────────────────────────────
    lle_dev_init();
    rfend_dev_init();
    bb_dev_init(bb::BB_RF_FLAG_1M); // rf_flag = 0x09 = EVT ble[0x14] confirmed
    ble_reg_init(false);
}

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
/// For Phase B staged-init debugging (insert `tracer::dump_regs` between stages),
/// call [`ble_hw_preamble`] followed by [`lle_dev_init`], [`rfend_dev_init`],
/// [`bb_dev_init`], and [`ble_reg_init`] individually instead of this function.
///
/// # Safety
///
/// Must be called exactly once at startup, before any BLE operation.
/// Interrupts BB (63) and LLE (64) must have handlers registered before calling.
pub unsafe fn ble_phy_init() {
    ble_hw_preamble();

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
    // skip_phase_b_rx_clears=false: keep Bug #2/#3 clears for Phase B RX listener path.
    ble_reg_init(false);

    // NOTE: BB (IRQn 63) and LLE (IRQn 64) interrupts are NOT enabled here.
    // The DTM polling TX path reads IRQ status directly via register polling.
    // Enabling IRQs without registered handlers causes unhandled exception faults.
    // Applications that use interrupt-driven BLE (TMOS/BLE stack) must enable
    // them separately after installing BB_IRQHandler / LLE_IRQHandler.
}
