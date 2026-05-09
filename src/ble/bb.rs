// BB (Baseband) initialization and IRQ handler for CH32V208 BLE.
//
// Register block: `BLE_BB` (gptrBBReg) at 0x40024100 — link-layer CTRL/GO,
// MODE, CFG, TIMING, IRQ status (STATR / +0x38).
//
// 2026-05-09: migrated from raw `read_volatile/write_volatile` against the
// hard-coded 0x40024100 / 0x40024200 / 0x40024300 bases to typed
// `BLE_BB.{ctrl,mode,cfg,timing,statr}()` / `BLE_LLE.{access_addr,scan_offset,
// timer}()` / `BLE_AES.statr()` accessors against the new ch32-metapac BLE
// blocks. Bit semantics + write order preserved exactly. Iron Law #38
// hardware-gated.
//
// WCH naming caveat (preserved here for archaeology): WCH's `gptrBBReg` is
// what we now expose as `BLE_BB` (CTRL/MODE/CFG/TIMING/STATR), and WCH's
// `gptrLLEReg` is `BLE_LLE` (TIMING0/STATE_MACHINE/ACCESS_ADDR/SCAN_OFFSET/
// TIMER/DMA_BUF). The BB IRQ handler legitimately reads STATR off `BLE_BB`
// and then drives ACCESS_ADDR / SCAN_OFFSET / TIMER on `BLE_LLE`; the typed
// API makes the cross-block hand-off obvious.

use crate::ble::types::PfnGetSysClock;
use crate::pac::{BLE_AES, BLE_BB, BLE_LLE};

// ── WCH lib BSS globals used by BB_IRQLibHandler ─────────────────────────────
extern "C" {
    /// Tick-counter callback pointer. ABI type: `PfnGetSysClock = unsafe extern "C" fn() -> u32`.
    ///
    /// Declared as `u32` (not `Option<PfnGetSysClock>`) to preserve LLVM GlobalMerge
    /// clustering — changing the LLVM type shifts gBleIPPara off its ROM-expected
    /// address 0x20000758 (Iron Law #34; Iron Law #37 ASSERT-pins this address).
    /// Read as u32, transmute before calling.
    ///
    /// Installed by BLE_LibInit → bleClock_RegisterCB; `0` (NULL) in our standalone
    /// ADV TX path since BLE_LibInit is never called (Path C / boundary mode).
    static mut fnGetClockCBs: u32;
}

/// BB RF flag — PHY mode selection bits for BB_CFG bits[30:25] (= rf_flag << 25).
///
/// Value 0x09 confirmed from live dtm.elf hardware dump:
/// BB+0x2C (CFG) = 0x92010EC8 = 0x80010EC8 | (0x09 << 25).
/// bit0=1M PHY enabled; bit3=additional PHY/feature (exact meaning TBD).
/// Using 0x00 produces CFG=0x80010EC8, which differs from the reference firmware.
pub const BB_RF_FLAG_1M: u8 = 0x09;

/// CFG register base value (bits[24:0] = 0x00010EC8, bit31 = hw_reserved).
const BB_CFG_BASE_LOW: u32 = 0x0001_0EC8;

/// Initialize the BB baseband processor.
///
/// Configures CTRL (bb_en/hw_en), TIMING, CFG, and MODE registers.
///
/// `rf_flag`: PHY mode selection. Use `BB_RF_FLAG_1M` for standard BLE 1M.
pub unsafe fn bb_dev_init(rf_flag: u8) {
    // CTRL (+0x00): set bit23 (bb_en) and bit28 (hw_en) as permanent enables.
    // Two separate RMW ops match the original sequence; the gap between them
    // must be tiny (no logging / delay) — bit23 is treated as a self-clearing
    // strobe by the analog block, so any println in between would make it
    // appear cleared (timing artifact, not a bug).
    BLE_BB.ctrl().modify(|w| w.set_bb_en(true)); // bit23 = 1
    BLE_BB.ctrl().modify(|w| w.set_hw_en(true)); // bit28 = 1

    // TIMING (+0x34): default 464 (0x1D0).
    BLE_BB.timing().write_value(0x1D0);

    // CFG (+0x2C): base 0x80010EC8 ORed with rf_flag<<25.
    // hw_reserved bit31 = 1, base bits[24:0] = 0x00010EC8, rf_flag bits[30:25].
    BLE_BB.cfg().write(|w| {
        w.set_hw_reserved(true);
        w.set_base(BB_CFG_BASE_LOW);
        w.set_rf_flag(rf_flag & 0x3F);
    });

    // MODE (+0x20): 0x90083.
    BLE_BB.mode().write_value(0x9_0083);
}

/// Pure-Rust replacement for WCH `BB_IRQLibHandler`.
///
/// # Control flow
///
/// ```text
/// statr = BLE_BB.statr()                          [WCH gptrBBReg+0x38]
/// if statr.pll_ready (bit6):
///   W1C: BLE_BB.statr() = 0x60                    (clear bits 5+6)
///   if gBleIPPara[0] bit6 SET:
///     ret = (*fnGetClockCBs)()                    (1600 Hz / 625 µs RTC-derived
///                                                  counter; B1 silicon installs
///                                                  ROM clockGetHSEValue at
///                                                  0x420B_000A — Iron Law #38
///                                                  contract per WCH BLE manual.)
///     gBleIPPara[0x1c..0x1f] = ret
///     gBleIPPara[0] &= ~0x40
///   if gBleIPPara[0] bit5 SET:
///     gBleIPPara[0] &= ~0x20
///     BLE_LLE.access_addr() = 0x8000              (active-scan mode write)
///     BLE_LLE.scan_offset() = gBleIPPara[20..23] << 1
///   // .L6: TX advance — only reachable when bit6 was set
///   if gBleIPPara[4] bit6 CLEAR:
///     gBleIPPara[5] = 1
///     BLE_LLE.access_addr() = 0x2000              ← advance PHY 0x33 → 0x37 (TX fire)
///     gBleIPPara[4] = 0xC0                        (= bits 7+6; blocks .L6 re-entry)
///     BLE_LLE.timer() = gBleIPPara[16..19]        (= 776 for ADV TX)
/// // .L4: bit4 of BLE_BB.statr()
/// if statr.bit4: W1C 0x10; gBleIPPara[4] = 1
/// // .L8: bit7 of BLE_BB.statr()
/// if statr.bit7: W1C 0x80; gBleIPPara[4] = 1
/// // .L9: BLE_AES.statr() bits 1+0
/// if aes.statr().phase1: clear phase1 then phase2
/// ```
///
/// # ADV TX path
///
/// * `gBleIPPara[0]` = 0x60 (bit5 + bit6) — written by ADV TX setup
///   (Iron Law #27, v7-probe). Both bit-5 and bit-6 paths fire on every BB IRQ:
///   bit-5 commits the scan-mode TX arm (0x8000 → BLE_LLE.access_addr() etc.); bit-6
///   reads `fnGetClockCBs` (B1 silicon: `ble_ip_core_init` Phase 2c Step 1
///   explicitly writes `0x420B_000A` = ROM `clockGetHSEValue` into this slot;
///   this is the canonical `pfnGetSysClock` of WCH's `TMOS_TimerInit` API per
///   the BLE manual — returns a u32 counter in units of 625 µs, where 1600 = 1 s)
///   and stores the result at `gBleIPPara[0x1c]` for
///   the BLE scheduler timing math.  ⚠ Iron Law #38: any change to the value
///   flowing into ip+0x1c must pass a 30 s air-visible cba ≥ 5 hardware gate
///   before commit AND match the 1600 Hz / 625 µs frequency contract.
///   task #56 v5 regression: substituting RISC-V `cycle` CSR (~96 MHz CPU clock)
///   for the ROM fn put a 60,000× scale error on every IRQ → BLE scheduler
///   timing collapsed → 0 air TX. nm + disasm confirmed v5 was code-form
///   equivalent; only air-visible cba count caught the semantic regression.
/// * `gBleIPPara[4]` = 0x80 after `BLE_SetPHYTxMode` (bit7=1, bit6=0) →
///   `.L6` fires on the first BB IRQ whose `BLE_BB.statr().pll_ready` is set.
/// * `gBleIPPara[16..19]` = 776 (written by ADV TX setup) → BLE_LLE.timer().
///
/// # Safety
///
/// Must be called from the BB ISR context only, after `ble_ip_core_init()` has
/// run (populates `gBleIPPara` and MMIO pointer globals).
pub unsafe fn bb_irq_lib_handler() {
    use core::ptr::{addr_of_mut, read_volatile, write_volatile};

    let ip: *mut u8 = addr_of_mut!(super::gBleIPPara) as *mut u8;

    // ── Step 1: BB.statr() bit6 (PLL-ready / BB-block event) ────────────────
    // CRITICAL: if bit6 is CLEAR, execution jumps to .L4 directly.
    // The .L6 TX-advance path is NEVER reached without bit6 being set.
    if BLE_BB.statr().read().pll_ready() {
        // W1C bits 5+6 of BB.statr() — write a fresh `Statr(0x60)`.
        BLE_BB.statr().write(|w| {
            w.set_bit5(true);
            w.set_pll_ready(true);
        });

        // gBleIPPara[0] bit6: clock callback. SET on every BB IRQ in the ADV TX
        // path — both ble_tx_adv_ch37.rs and ble_tx_adv_ch37_minimal.rs write
        // gBleIPPara[0] = 0x60 at init (Iron Law #27, v7-probe). When set, this
        // reads fnGetClockCBs and stores the tick at gBleIPPara[0x1c] for the
        // BLE scheduler.  The slot's contract (per WCH BLE manual,
        // TMOS_TimerInit / TMOS_GetSystemClock): returns a u32 counter in units
        // of 625 µs, where 1600 = 1 s — i.e. an RTC-derived 1600 Hz counter
        // (Phase 2c Step 1: ble_ip_core_init explicitly writes 0x420B000A which
        // honours this; ROM does NOT auto-install — baf71de gate falsified that).
        // Iron Law #38: do NOT swap the fn for any other clock source unless it
        // matches the 1600 Hz / 625 µs frequency contract — task #56 v5
        // regression proved RISC-V `cycle` CSR (~96 MHz) is 60,000× off → the
        // BLE scheduler reads 60,000 BLE slots per actual slot → 0 air TX.
        let ip0 = read_volatile(ip);
        if ip0 & 0x40 != 0 {
            let fn_addr = read_volatile(addr_of_mut!(fnGetClockCBs));
            if fn_addr != 0 {
                // Safety: fn_addr is a valid PfnGetSysClock installed by lib init or ROM.
                // Declared as u32 to preserve GlobalMerge layout (see extern block above).
                let cb: PfnGetSysClock = core::mem::transmute(fn_addr as usize);
                let ret = cb();
                write_volatile(ip.add(0x1c).cast::<u32>(), ret); // gBleIPPara+28
            }
            write_volatile(ip, read_volatile(ip) & !0x40u8); // clear bit6
        }

        // gBleIPPara[0] bit5: active-scan TX-mode write. SET on every BB IRQ in
        // the ADV TX path (gBleIPPara[0]=0x60 includes bit5). This is the
        // mechanism that arms TX: bit5 path commits 0x8000 → BLE_LLE.access_addr()
        // and (gBleIPPara[20..24] << 1) → BLE_LLE.scan_offset() BEFORE .L6 fires
        // (Iron Law #27, v7-probe forensic proof: probe-A R1=56, 3×60s median=53).
        let ip0 = read_volatile(ip);
        if ip0 & 0x20 != 0 {
            write_volatile(ip, ip0 & !0x20u8);                        // clear bit5
            BLE_LLE.access_addr().write_value(0x8000);                // gptrLLEReg+0x08
            let ip20 = read_volatile(ip.add(20).cast::<u32>());
            BLE_LLE.scan_offset().write_value(ip20 << 1);             // gptrLLEReg+0x6c
        }

        // ── .L6: TX advance path ─────────────────────────────────────────────
        // Fire only when gBleIPPara[4] bit6 is clear (armed but not yet fired).
        // BLE_SetPHYTxMode sets gBleIPPara[4] = 0x80 (bit7=1, bit6=0) to arm .L6.
        //
        // v7-probe (2026-05-05): scan-mode pre-arm writes (0x8000 + ip20<<1) removed
        // from .L6. These writes (v4-v6 explicit MMIO path) all gave cba=0.
        // The active-scan mode arm fires via the ip0 bit5 path BEFORE .L6, triggered
        // by gBleIPPara[0]=0x60 set at init time. Plain .L6 = standard TX advance only.
        let ip4 = read_volatile(ip.add(4));
        if ip4 & 0x40 == 0 {
            write_volatile(ip.add(5), 1u8);                            // gBleIPPara[5] = 1
            BLE_LLE.access_addr().write_value(0x2000);                // advance (W1C bit13)
            write_volatile(ip.add(4), 0xC0u8);  // 0xC0 = bits 7+6; block .L6 re-entry on next IRQ
            let timer = read_volatile(ip.add(16).cast::<u32>());       // gBleIPPara[16..19]
            BLE_LLE.timer().write_value(timer);                       // bb+0x64 timer
        }
    }

    // ── .L4: BB.statr() bit4 cleanup ────────────────────────────────────────
    if BLE_BB.statr().read().bit4() {
        BLE_BB.statr().write(|w| w.set_bit4(true)); // W1C bit4
        write_volatile(ip.add(4), 1u8);             // gBleIPPara[4] = 1 (re-arm)
    }

    // ── .L8: BB.statr() bit7 cleanup ────────────────────────────────────────
    if BLE_BB.statr().read().bit7() {
        BLE_BB.statr().write(|w| w.set_bit7(true)); // W1C bit7
        write_volatile(ip.add(4), 1u8);             // gBleIPPara[4] = 1 (re-arm)
    }

    // ── .L9: BLE_AES.statr() bits 1+0 (AES operation cleanup) ──────────────
    // Two separate RMWs: first clear phase1 (bit1), then re-read and clear
    // phase2 (bit0). Order matters — preserves WCH asm sequence.
    if BLE_AES.statr().read().phase1() {
        BLE_AES.statr().modify(|w| w.set_phase1(false)); // clear bit1
        BLE_AES.statr().modify(|w| w.set_phase2(false)); // then bit0
    }
}
