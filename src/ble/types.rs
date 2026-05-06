//! BLE C type mirrors — aligned to `wchble.h` / `wchble_rom.h` public API.
//!
//! # Sources
//!
//! All types in this module mirror definitions from:
//! - `wchble.h` (CH32V20xEVT-2.31/EXAM/BLE/LIB/wchble.h)
//! - `wchble_rom.h` (same dir)
//! - `HAL/RTC.c` L59-100 — concrete `bleClockConfig_t` init example
//!
//! # ABI Contract
//!
//! All `#[repr(C)]` structs include compile-time size/alignment assertions using
//! `core::mem::{size_of, align_of}`. These validate against the WCH C compiler's
//! layout on RISC-V 32-bit (pointer = 4B, max field align = 4B).
//!
//! # Iron Law #35 — fnGetClockCBs / PfnGetSysClock protocol
//!
//! `PfnGetSysClock` (= `fnGetClockCBs` runtime storage) returns a **monotonically
//! increasing tick counter** (e.g. `RTC_GetCounter`), NOT Hz. The ROM default at
//! `0x420B000A` is an execute-only function that reads an internal BLE controller
//! timing counter; it cannot be reverse-engineered via data reads (triple-path probe
//! 2026-05-06: wlink SBA + CPU lw clock-off + CPU lw clock-on all return all-zero).
//!
//! Returning a constant Hz value (e.g. 96_000_000) as attempted in T8 attempt-14
//! causes BLE timing failure (cba=0). The ROM treats the return as a counter delta;
//! a constant returns delta=0, stalling the BLE scheduler.
//!
//! **Recommendation**: leave `fnGetClockCBs = NULL` (boundary mode, startup-zero
//! excluded) so ROM installs its own default. See `t8-final-strip-plan.md` §12.

// ── TMOS primitive type aliases ───────────────────────────────────────────────
// wchble.h L42-56

/// TMOS task identifier. Maps to `tmosTaskID = uint8_t`.
pub type TmosTaskId = u8;

/// TMOS event bitmask. Maps to `tmosEvents = uint16_t`.
pub type TmosEvents = u16;

/// TMOS timer value. Maps to `tmosTimer = uint32_t`.
pub type TmosTimer = u32;

/// SNV storage ID. Maps to `tmosSnvId_t = uint16_t`.
pub type TmosSnvId = u16;

/// SNV storage length. Maps to `tmosSnvLen_t = uint16_t`.
pub type TmosSnvLen = u16;

/// BLE status code. Maps to `bStatus_t = uint8_t`.
pub type BleStatus = u8;

// ── Function pointer typedefs ─────────────────────────────────────────────────
// wchble.h L58-77

/// Random seed generator callback. Maps to `pfnSrandCB = uint32_t(*)(void)`.
///
/// Returns a random seed value for BLE stack initialization.
pub type PfnSrandCb = unsafe extern "C" fn() -> u32;

/// Idle-mode entry callback. Maps to `pfnIdleCB = uint32_t(*)(uint32_t)`.
///
/// Called by BLE scheduler to enter low-power idle. `delta` is the sleep
/// duration in TMOS timer ticks. Returns actual sleep ticks consumed.
pub type PfnIdleCb = unsafe extern "C" fn(delta: u32) -> u32;

/// LSI clock calibration callback. Maps to `pfnLSICalibrationCB = void(*)(void)`.
///
/// Called periodically to calibrate the internal 32 kHz LSI oscillator.
pub type PfnLsiCalibrationCb = unsafe extern "C" fn();

/// Temperature sample callback. Maps to `pfnTempSampleCB = uint16_t(*)(void)`.
///
/// Returns current chip temperature in tenths of a degree (WCH convention).
/// Used by the BLE RF calibration path to compensate for thermal drift.
pub type PfnTempSampleCb = unsafe extern "C" fn() -> u16;

/// BLE event timing callback. Maps to `pfnEventCB = void(*)(uint32_t timeUs)`.
///
/// Called on connect/advertise event completion with the event duration in µs.
pub type PfnEventCb = unsafe extern "C" fn(time_us: u32);

/// Library status/error callback. Maps to `pfnLibStatusErrorCB = void(*)(uint8_t code, uint32_t status)`.
///
/// Called by the BLE library on internal status transitions or errors.
/// `code` identifies the event class; `status` is an event-specific payload.
pub type PfnLibStatusErrorCb = unsafe extern "C" fn(code: u8, status: u32);

/// TMOS task event handler. Maps to `pTaskEventHandlerFn = tmosEvents(*)(tmosTaskID, tmosEvents)`.
///
/// Called by the TMOS scheduler to dispatch events for a registered task.
/// Returns the bitmask of events NOT handled (remaining events).
pub type PfnTaskEventHandler = unsafe extern "C" fn(task_id: TmosTaskId, events: TmosEvents) -> TmosEvents;

/// Flash read callback. Maps to `pfnFlashReadCB = uint32_t(*)(uint32_t addr, uint32_t num, uint32_t *pBuf)`.
///
/// Reads `num` words from flash at `addr` into `p_buf`. Returns 0 on success.
pub type PfnFlashReadCb = unsafe extern "C" fn(addr: u32, num: u32, p_buf: *mut u32) -> u32;

/// Flash write callback. Maps to `pfnFlashWriteCB = uint32_t(*)(uint32_t addr, uint32_t num, uint32_t *pBuf)`.
///
/// Writes `num` words from `p_buf` to flash at `addr`. Returns 0 on success.
pub type PfnFlashWriteCb = unsafe extern "C" fn(addr: u32, num: u32, p_buf: *const u32) -> u32;

/// System clock / tick counter callback. Maps to `pfnGetSysClock = uint32_t(*)(void)`.
///
/// Returns a **monotonically increasing tick counter** suitable for BLE timing
/// delta calculations (NOT Hz — see Iron Law #35 above). The WCH SDK example
/// uses `RTC_GetCounter` (32-bit RTC counter @ LSI/2 ≈ 16 kHz, wraps at 2³²).
///
/// This type is the ABI of `fnGetClockCBs` (lib COMMON BSS at 0x20001c78).
/// ROM default (`0x420B000A`) reads an internal BLE controller counter; it is
/// installed by ROM during BLE init when `fnGetClockCBs` is non-NULL on entry.
pub type PfnGetSysClock = unsafe extern "C" fn() -> u32;

// ── BleClockConfig ────────────────────────────────────────────────────────────
// wchble.h L112-119

/// BLE TMOS timer clock configuration. Mirrors `bleClockConfig_t`.
///
/// Passed to `TMOS_TimerInit()` to configure the tick source used by the BLE
/// TMOS scheduler. The WCH HAL example (`HAL/RTC.c` L59-100) uses:
///
/// ```c
/// conf.getClockValue  = RTC_GetCounter;   // monotonic tick counter fn
/// conf.ClockMaxCount  = 0xFFFFFFFF;       // 32-bit wrap-around
/// conf.ClockFrequency = CAB_LSIFQ / 2;   // ≈ 16_000 Hz (LSI/2)
/// conf.ClockAccuracy  = 1000;             // ppm
/// ```
///
/// # ABI
///
/// `#[repr(C)]`, size = 16 bytes, align = 4 bytes (verified by const asserts below).
///
/// | Offset | Field            | C type          | Size |
/// |--------|------------------|-----------------|------|
/// | 0      | get_clock_value  | pfnGetSysClock  | 4    |
/// | 4      | clock_max_count  | uint32_t        | 4    |
/// | 8      | clock_frequency  | uint16_t        | 2    |
/// | 10     | clock_accuracy   | uint16_t        | 2    |
/// | 12     | irq_enable       | uint8_t         | 1    |
/// | 13     | (padding)        | —               | 3    |
/// | 16     | (end)            |                 |      |
#[repr(C)]
pub struct BleClockConfig {
    /// Tick counter callback. `None` (NULL) → ROM selects HSE as clock source
    /// and installs its own default (`0x420B000A`) during BLE init.
    pub get_clock_value: Option<PfnGetSysClock>,
    /// Maximum counter value before wrap-around (typically `0xFFFF_FFFF`).
    pub clock_max_count: u32,
    /// Tick source frequency in Hz (e.g. `CAB_LSIFQ / 2` ≈ 16_000).
    pub clock_frequency: u16,
    /// Tick source accuracy in ppm (100 for LSE, 1000 for LSI).
    pub clock_accuracy: u16,
    /// Reserved / IRQ enable flag (currently unused by WCH stack).
    pub irq_enable: u8,
    // 3 bytes implicit C padding to pad struct to align-4 boundary
}

// Compile-time ABI assertions — must match WCH C compiler output on RV32.
const _: () = assert!(
    core::mem::size_of::<BleClockConfig>() == 16,
    "BleClockConfig size mismatch vs bleClockConfig_t (expected 16)"
);
const _: () = assert!(
    core::mem::align_of::<BleClockConfig>() == 4,
    "BleClockConfig align mismatch vs bleClockConfig_t (expected 4)"
);

// ── BleConfig ─────────────────────────────────────────────────────────────────
// wchble.h L80-109

/// BLE library initialization configuration. Mirrors `bleConfig_t`.
///
/// Passed to `BLE_LibInit()` (libwchble.a entry point) to configure the full
/// BLE stack. In T8+ (post -lwchble removal) this struct is not directly used
/// by the Rust PHY layer, but it is included here for:
/// 1. Documentation of the WCH BLE C ABI
/// 2. Future TMOS/connection-mode work (post-T8)
/// 3. Type-safe construction of `gBleIPPara` field mappings
///
/// # ABI
///
/// `#[repr(C)]`, size = 64 bytes, align = 4 bytes (verified by const asserts below).
///
/// Notable padding:
/// - 2B after `mem_len` (u16 @ offset 4) → `snv_addr` (u32 @ offset 8)
/// - 2B after `clock_accuracy` (u16 @ offset 32) → `srand_cb` (fn ptr @ offset 36)
#[repr(C)]
pub struct BleConfig {
    /// Library heap memory start address (must be 4-byte aligned).
    pub mem_addr: u32,
    /// Library heap memory size in bytes (minimum 4 KiB recommended).
    pub mem_len: u16,
    // 2 bytes C padding (u16 → u32 alignment)
    _pad0: [u8; 2],
    /// SNV (non-volatile) flash start address. `0` = bonding info not saved.
    pub snv_addr: u32,
    /// SNV flash block size in bytes (default 256).
    pub snv_block: u16,
    /// Number of SNV flash blocks (default 1).
    pub snv_num: u8,
    /// Max TX/RX packet buffer count (default 5; must exceed connection count).
    pub buf_number: u8,
    /// Max HCI data payload length in bytes (default 27; SC mode needs ≥ 69).
    /// `ATT_MTU = buf_max_len - 4`, range [23, ATT_MAX_MTU_SIZE].
    pub buf_max_len: u16,
    /// Max TX data packets per connection event (default 1).
    pub tx_num_event: u8,
    /// Max RX data packets per connection event (default = `buf_number`).
    pub rx_num_event: u8,
    /// Transmit power level (default `LL_TX_POWER_0_DBM`).
    pub tx_power: u8,
    /// Connection count: bits[1:0] = peripheral count, bits[3:2] = central count.
    pub connect_number: u8,
    /// RF start window widening in µs.
    pub window_widening: u8,
    /// Event arrival wait window in system clock units.
    pub wait_window: u8,
    /// Device MAC address (little-endian, 6 bytes).
    pub mac_addr: [u8; 6],
    /// Timing clock frequency in Hz (e.g. `CAB_LSIFQ / 2` ≈ 16_000).
    pub clock_frequency: u16,
    /// Timing clock accuracy in ppm (50 for LSE, 1000 for LSI).
    pub clock_accuracy: u16,
    // 2 bytes C padding (u16 → fn-ptr alignment)
    _pad1: [u8; 2],
    /// Random seed generator callback.
    pub srand_cb: Option<PfnSrandCb>,
    /// Idle-mode entry callback.
    pub idle_cb: Option<PfnIdleCb>,
    /// Temperature sample callback (for RF thermal calibration).
    pub ts_cb: Option<PfnTempSampleCb>,
    /// LSI clock calibration callback.
    pub rc_cb: Option<PfnLsiCalibrationCb>,
    /// Library status/error callback.
    pub sta_cb: Option<PfnLibStatusErrorCb>,
    /// Flash read callback (for SNV bonding info persistence).
    pub read_flash_cb: Option<PfnFlashReadCb>,
    /// Flash write callback (for SNV bonding info persistence).
    pub write_flash_cb: Option<PfnFlashWriteCb>,
}

// Compile-time ABI assertions.
const _: () = assert!(
    core::mem::size_of::<BleConfig>() == 64,
    "BleConfig size mismatch vs bleConfig_t (expected 64)"
);
const _: () = assert!(
    core::mem::align_of::<BleConfig>() == 4,
    "BleConfig align mismatch vs bleConfig_t (expected 4)"
);
