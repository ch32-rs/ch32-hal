// BB (Baseband) initialization and IRQ handler for CH32V208 BLE.
//
// Register base: 0x40024100 (gptrBBReg — CTRL, GO, ACCESS_ADDR, CRC_INIT, TX mode, CFG, MODE)
// Source: elec-docs/ble-reverse-docs/24-rf-phy-supplement.md
// Derived from BB_DevInit() / BB_IRQLibHandler() in libwchble.a V1.40 (bb.o)

use core::ptr::{read_volatile, write_volatile};
use crate::ble::types::PfnGetSysClock;

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

// gptrBBReg in WCH naming: link-layer CTRL, GO, ACCESS_ADDR, CRC_INIT, TX mode, CFG, MODE.
const BB_BASE: usize = 0x40024100;

#[inline(always)]
unsafe fn bb_read(offset: usize) -> u32 {
    read_volatile((BB_BASE + offset) as *const u32)
}

#[inline(always)]
unsafe fn bb_write(offset: usize, val: u32) {
    write_volatile((BB_BASE + offset) as *mut u32, val);
}

#[inline(always)]
unsafe fn bb_modify(offset: usize, clear: u32, set: u32) {
    let v = bb_read(offset);
    bb_write(offset, (v & !clear) | set);
}

/// BB RF flag — PHY mode selection bits for BB_CFG bits[30:25] (= rf_flag << 25).
///
/// Value 0x09 confirmed from live dtm.elf hardware dump:
/// BB+0x2C (CFG) = 0x92010EC8 = 0x80010EC8 | (0x09 << 25).
/// bit0=1M PHY enabled; bit3=additional PHY/feature (exact meaning TBD).
/// Using 0x00 produces CFG=0x80010EC8, which differs from the reference firmware.
pub const BB_RF_FLAG_1M: u8 = 0x09;

/// Initialize the BB baseband processor.
///
/// Mirrors BB_DevInit() from libwchble.a V1.40 (bb.o, line 71369).
/// Configures CTRL, TIMING, CFG, and MODE registers.
///
/// `rf_flag`: PHY mode selection. Use `BB_RF_FLAG_1M` for standard BLE 1M.
pub unsafe fn bb_dev_init(rf_flag: u8) {
    // CTRL (+0x00): set bit23 and bit28 as permanent enables.
    // WCH BB_DevInit asm L69623: `lui a3,0x800` → a3 = 0x800<<12 = 0x0080_0000 = bit23.
    // Previous code wrongly used 0x0000_0800 (bit11, the TX GO strobe) — lui-shift typo.
    // bit23 is a baseband enable, NOT a GO strobe; WCH leaves it set permanently.
    // bit28 = 0x10000000: hardware enable (`lui a3,0x10000` → or CTRL,CTRL,a3).
    // Neither bit is cleared after init — no "clear strobe" step in WCH asm.
    bb_modify(0x00, 0x0000_0000, 0x0080_0000); // bit23 = 1 (was wrongly bit11/0x0800)
    // bit23 is a self-clearing strobe — write bit28 IMMEDIATELY (no intervening code):
    // WCH bb.o sequence is ~50 ns between bit23 and bit28 writes; any println here would
    // delay hundreds of µs and make bit23 appear cleared (timing artifact, not a bug).
    bb_modify(0x00, 0x0000_0000, 0x1000_0000); // bit28 = 1

    // TIMING (+0x34): default 464 (0x1D0)
    bb_write(0x34, 0x1D0);

    // CFG (+0x2C): base value 0x80010EC8 ORed with rf_flag bits
    // Base from: lui a2,0x80011; addi a2,-312 = 0x80010EC8
    let cfg_base: u32 = 0x80010EC8;
    let cfg_val = cfg_base | ((rf_flag as u32 & 0x3F) << 25);
    bb_write(0x2C, cfg_val);

    // MODE (+0x20): 0x90083
    // From: lui a4,0x90; addi a4,131 = 0x90083
    bb_write(0x20, 0x90083);
}

/// Pure-Rust replacement for WCH `BB_IRQLibHandler`.
///
/// Decoded from libwchble_dr.asm line 91127 (BB_IRQLibHandler, 0x114 bytes, task #22).
///
/// # Register naming note
///
/// WCH's `gptrBBReg` (0x40024100) is the timer/IRQ/DMA base (what this crate calls
/// the `lle_*` register range). WCH's `gptrLLEReg` (0x40024200) is the CTRL/GO base
/// (our `bb_*` range). This function uses WCH-address constants directly to avoid
/// the confusing swapped naming.
///
/// # Control flow (decoded from asm)
///
/// ```text
/// blk = *(0x40024100 + 0x38)   [WCH gptrBBReg+0x38]
/// if blk bit6 SET:
///   W1C: *(0x40024100+0x38) = 0x60   (clear bits 5+6)
///   if gBleIPPara[0] bit6 SET:
///     ret = (*fnGetClockCBs)()        (LIVE in ADV TX path: ble_tx_adv_ch37*.rs
///                                      writes gBleIPPara[0]=0x60 at init, so
///                                      bit6 is set on every BB IRQ. The slot
///                                      holds an RTC-derived 1600 Hz counter fn
///                                      — contract per WCH BLE manual: returns
///                                      tmos system run time, unit = 625µs,
///                                      1600 = 1s.)
///     gBleIPPara[0x1c..0x1f] = ret
///     gBleIPPara[0] &= ~0x40
///   if gBleIPPara[0] bit5 SET:
///     gBleIPPara[0] &= ~0x20
///     *(0x40024200+0x08) = 0x8000    (active-scan mode write)
///     *(0x40024200+0x6c) = gBleIPPara[20..23] << 1
///   // .L6: TX advance — only reachable when bit6 was set
///   if gBleIPPara[4] bit6 CLEAR:
///     gBleIPPara[5] = 1
///     *(0x40024200+0x08) = 0x2000    ← advance PHY 0x33 → 0x37 (TX fire)
///     gBleIPPara[4] = 0xC0           (= bits 7+6; blocks .L6 re-entry on next IRQ)
///     *(0x40024200+0x64) = gBleIPPara[16..19]   (timer; no shift; = 776 for ADV TX)
/// // .L4: bit4 of gptrBBReg+0x38
/// if blk2 bit4 SET: W1C 0x10; gBleIPPara[4] = 1
/// // .L8: bit7 of gptrBBReg+0x38
/// if blk3 bit7 SET: W1C 0x80; gBleIPPara[4] = 1
/// // .L9: gptrAESReg+0x04 bits 1+0
/// if aes4 bit1 SET: clear bit1 then bit0 of gptrAESReg+0x04
/// ```
///
/// # ADV TX path
///
/// * `gBleIPPara[0]` = 0x60 (bit5 + bit6) — written by ADV TX setup
///   (Iron Law #27, v7-probe). Both bit-5 and bit-6 paths fire on every BB IRQ:
///   bit-5 commits the scan-mode TX arm (0x8000 → WCH_LLER+0x08 etc.); bit-6
///   reads `fnGetClockCBs` (B1 silicon: ROM auto-installs an RTC-derived 1600 Hz
///   counter fn at 0x420B000A; this is the canonical `pfnGetSysClock` of WCH's
///   `TMOS_TimerInit` API per the BLE manual — returns a u32 counter in units of
///   625 µs, where 1600 = 1 s) and stores the result at `gBleIPPara[0x1c]` for
///   the BLE scheduler timing math.  ⚠ Iron Law #38: any change to the value
///   flowing into ip+0x1c must pass a 30 s air-visible cba ≥ 5 hardware gate
///   before commit AND match the 1600 Hz / 625 µs frequency contract.
///   task #56 v5 regression: substituting RISC-V `cycle` CSR (~96 MHz CPU clock)
///   for the ROM fn put a 60,000× scale error on every IRQ → BLE scheduler
///   timing collapsed → 0 air TX. nm + disasm confirmed v5 was code-form
///   equivalent; only air-visible cba count caught the semantic regression.
/// * `gBleIPPara[4]` = 0x80 after `BLE_SetPHYTxMode` (bit7=1, bit6=0) →
///   `.L6` fires on the first BB IRQ whose `gptrBBReg+0x38` bit6 is set (PLL ready).
/// * `gBleIPPara[16..19]` = 776 (written by ADV TX setup) → bb+0x64 timer.
///
/// # Safety
///
/// Must be called from the BB ISR context only, after `ble_ip_core_init()` has
/// run (populates `gBleIPPara` and MMIO pointer globals).
pub unsafe fn bb_irq_lib_handler() {
    use core::ptr::addr_of_mut;

    // Use WCH physical addresses directly to avoid the swapped naming.
    const WCH_BBR:  usize = 0x40024100; // gptrBBReg  = lle_* timer/IRQ range
    const WCH_LLER: usize = 0x40024200; // gptrLLEReg = bb_*  CTRL/GO range
    const WCH_AESR: usize = 0x40024300; // gptrAESReg

    let ip: *mut u8 = addr_of_mut!(super::gBleIPPara) as *mut u8;

    // ── Step 1: gptrBBReg+0x38 bit6 (PLL-ready / BB-block event) ────────────
    // CRITICAL: if bit6 is CLEAR, execution jumps to .L4 directly.
    // The .L6 TX-advance path is NEVER reached without bit6 being set.
    let blk = read_volatile((WCH_BBR + 0x38) as *const u32);
    if blk & (1 << 6) != 0 {
        // W1C bits 5+6 of gptrBBReg+0x38
        write_volatile((WCH_BBR + 0x38) as *mut u32, 0x60);

        // ── Experiment B (2026-05-05): 0x8000/0x6c/0x2000 moved into .L6; ──
        // state-machine reads capture WCH_LLER+0x1C before/after each write.
        // v4–v6 all cba=0 with ~110–249 ns natural gap → timing hypothesis exhausted;
        // LLE state-machine context is the new investigation target.

        // gBleIPPara[0] bit6: clock callback. SET on every BB IRQ in the ADV TX
        // path — both ble_tx_adv_ch37.rs and ble_tx_adv_ch37_minimal.rs write
        // gBleIPPara[0] = 0x60 at init (Iron Law #27, v7-probe). When set, this
        // reads fnGetClockCBs and stores the tick at gBleIPPara[0x1c] for the
        // BLE scheduler.  The slot's contract (per WCH BLE manual,
        // TMOS_TimerInit / TMOS_GetSystemClock): returns a u32 counter in units
        // of 625 µs, where 1600 = 1 s — i.e. an RTC-derived 1600 Hz counter
        // (B1 silicon: ROM auto-installs 0x420B000A which honours this).
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
        // mechanism that arms TX: bit5 path commits 0x8000 → WCH_LLER+0x08 and
        // (gBleIPPara[20..24] << 1) → WCH_LLER+0x6c BEFORE .L6 fires (Iron Law
        // #27, v7-probe forensic proof: probe-A R1=56, 3×60s median=53).
        let ip0 = read_volatile(ip);
        if ip0 & 0x20 != 0 {
            write_volatile(ip, ip0 & !0x20u8);                              // clear bit5
            write_volatile((WCH_LLER + 0x08) as *mut u32, 0x8000);          // gptrLLEReg+8
            let ip20 = read_volatile(ip.add(20).cast::<u32>());
            write_volatile((WCH_LLER + 0x6c) as *mut u32, ip20 << 1);       // gptrLLEReg+0x6c
        }

        // ── .L6: TX advance path ─────────────────────────────────────────────
        // Fire only when gBleIPPara[4] bit6 is clear (armed but not yet fired).
        // BLE_SetPHYTxMode sets gBleIPPara[4] = 0x80 (bit7=1, bit6=0) to arm .L6.
        //
        // v7-probe (2026-05-05): scan-mode pre-arm writes (0x8000 + ip20<<1) removed
        // from .L6. These writes (v4-v6 explicit MMIO path) all gave cba=0.
        // The active-scan mode arm fires via the ip0 bit5 path BEFORE .L6, triggered
        // by gBleIPPara[0]=0x60 set at init time. Plain .L6 = standard TX advance only.
        // expB/expC SDI instrumentation removed (production binary).
        let ip4 = read_volatile(ip.add(4));
        if ip4 & 0x40 == 0 {
            write_volatile(ip.add(5), 1u8);                                  // gBleIPPara[5] = 1
            write_volatile((WCH_LLER + 0x08) as *mut u32, 0x2000);          // advance (W1C bit13)
            write_volatile(ip.add(4), 0xC0u8);  // 0xC0 = bits 7+6; block .L6 re-entry on next IRQ
            let timer = read_volatile(ip.add(16).cast::<u32>());             // gBleIPPara[16..19]
            write_volatile((WCH_LLER + 0x64) as *mut u32, timer);           // bb+0x64 timer
        }
    }

    // ── .L4: gptrBBReg+0x38 bit4 cleanup ────────────────────────────────────
    let blk2 = read_volatile((WCH_BBR + 0x38) as *const u32);
    if blk2 & (1 << 4) != 0 {
        write_volatile((WCH_BBR + 0x38) as *mut u32, 0x10); // W1C bit4
        write_volatile(ip.add(4), 1u8);                      // gBleIPPara[4] = 1 (re-arm)
    }

    // ── .L8: gptrBBReg+0x38 bit7 cleanup ────────────────────────────────────
    let blk3 = read_volatile((WCH_BBR + 0x38) as *const u32);
    if blk3 & (1 << 7) != 0 {
        write_volatile((WCH_BBR + 0x38) as *mut u32, 0x80); // W1C bit7
        write_volatile(ip.add(4), 1u8);                      // gBleIPPara[4] = 1 (re-arm)
    }

    // ── .L9: gptrAESReg+0x04 bits 1+0 (AES operation cleanup) ──────────────
    let aes4 = read_volatile((WCH_AESR + 0x04) as *const u32);
    if aes4 & 0x02 != 0 {
        write_volatile((WCH_AESR + 0x04) as *mut u32, aes4 & !0x02u32); // clear bit1
        let aes4b = read_volatile((WCH_AESR + 0x04) as *const u32);
        write_volatile((WCH_AESR + 0x04) as *mut u32, aes4b & !0x01u32); // then bit0
    }
}
