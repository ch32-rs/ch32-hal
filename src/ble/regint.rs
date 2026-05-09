// BLE_RegInit — RF calibration for CH32V208 BLE.
//
// Implements the 4 calibration functions called by BLE_RegInit() in libwchble.a V1.40:
//   1. RFEND_TXCtune  — TX carrier + gain calibration (40-channel lookup table)
//   2. RFEND_TXFtune  — TX filter enable (trivial: set one bit)
//   3. RFEND_RXFilter — RX filter self-calibration with hardware poll
//   4. RFEND_RXAdc    — RX ADC reference configuration
//
// Register block: `BLE_RFEND` (gptrRFENDReg) at 0x40025000 — SAME block used by
// rfend.rs RFEND_DevInit. rfend.rs configures the analog registers (PLL/VCO/loop
// filter/bias offsets +0x28..+0x5C); this file triggers calibration (+0x04) and
// reads calibration results (+0x90..+0xD0). Both files operate on the same
// 0x40025000 hardware block — they are not separate regions.
//
// Confirmed by ip.o BLE_IPCoreInit reloc: `lui a4, 0x40025` → *gptrRFENDReg=0x40025000.
// Also confirmed by wlink dump after running official dtm.elf:
//   0x40025000+0xA0..0xC8 → CO1 table word0=0x45555556 (ch0=6,ch7=4) ✓
//
// Register intent map (from disassembly of libwchble.a V1.40, confirmed):
//   +0x04  bits: bit0=TX_tune_trigger, bit4=TX_cal_mode, bit8=TXF_enable,
//                bit12=RX_filter_mode, bit16=RX_ADC_config            [cal_trig]
//   +0x08  bits: bit16=RX_ADC_path, bit17=TX_cal_path, bit21=RX_filter_path
//                bits[20]=TX_PLL_pre, bits[21]=RX_PLL_pre              [path_en]
//   +0x0C  bits: bit4=RX_filter_strobe, bit8=ADC_ref_strobe            [ctrl]
//   +0x28  bits: bit12=TX_tune_mode_final (set after calibration completes) [cfg0]
//   +0x2C  bits: bit4=post_cal_enable                                 [pll_vco]
//   +0x38  bits[15:8]=freq_code, bits[5:0]=nCO2440, bits[30:24]=nGA2440 [cfg4]
//          (also rfend.rs writes PLL-enable bits[25:19]+bit20+bit31 here — same reg)
//   +0x50  bits: bit16=RX_filter_valid, bits[4:0]=RX_filter_result    [rf2]
//   +0x58  bits: bit16=RX_ADC_ref_enable                              [adc_ref]
//   +0x90  bits: bits[5:0]=CO_result, bit25=tune_done, bit26=tune_active [tune_result]
//   +0x94  bits[16:10]: GA_result (7 bits)                            [ga_result]
//   +0x9C  bits: bit8=RX_filter_done, bits[4:0]=RX_filter_result      [rx_filter_result]
//   +0xA0..0xB0: CO calibration table 1 (nibble-packed, 40 channels)  [co_table1]
//   +0xB4..0xC4: CO calibration table 2 (nibble-packed, 40 channels)  [co_table2]
//   +0xC8..0xD0: GA calibration + CO2 overflow                        [ga_table]
//
// 2026-05-09: migrated from raw `read_volatile/write_volatile` to typed
// `BLE_BB / BLE_LLE / BLE_RFEND` accessors against the new ch32-metapac BLE
// blocks. Bit patterns preserved exactly; semantics unchanged. Iron Law #38
// hardware-gated.
//
// Metapac caveat (filed for future ch32-data MR): the YAML labels +0x38 as
// `cfg4` (PLL enable bits[25:19]) with no fieldset and +0x3C as `cfg5_freq`
// with FREQ_CODE / NCO2440 / NGA2440 fields. Disassembly of the working DTM
// build (tx_tune_measure: `lw 56(a5)` / `sw 56(a5)` where `a5=0x40025000`)
// proves freq_code is actually written to +0x38. Both rfend.rs PLL-enable
// writes AND regint.rs freq_code writes target the same physical register at
// +0x38. We therefore manipulate `cfg4()` as a raw u32 here; the metapac
// `cfg5_freq` named accessors at +0x3C are NOT used (they would touch the
// wrong register). Future MR should rename `cfg4` → e.g. `cfg4_freq` with
// FREQ_CODE/NCO2440/NGA2440 fields, and `cfg5_freq` → `cfg5_strobe` (raw u32).
//
// Sources: static disassembly of dtm.elf (WCH BLE SDK V1.40), live register dump.

use core::arch::asm;

use crate::pac::{BLE_BB, BLE_LLE, BLE_RFEND};

// Frequency selection codes written to BLE_RFEND.cfg4() bits[15:8].
// Confirmed from BB_DevInit / RF_DevSetChannel disassembly.
const FREQ_CODE_2401: u32 = 0xBF00; // one step below BLE ch0 (2402 MHz)
const FREQ_CODE_2480: u32 = 0xE700; // BLE ch39
const FREQ_CODE_2440: u32 = 0xD300; // BLE ch19, band midpoint

/// Run all 4 RF calibration routines in the order used by BLE_RegInit.
///
/// Must be called after `ble_phy_init()` (RFEND_DevInit + BB_DevInit + LLE_DevInit).
/// Calibrates TX carrier, TX filter, RX filter, and RX ADC for the current temperature.
///
/// # Safety
/// Requires the RFEND/LLE/BB peripherals to be initialized and accessible.
///
/// If `skip_phase_b_rx_clears` is `true`, the two extra post-calibration clears that
/// were added for the Phase B RX path (Bug #2: RFEND+0x08 bit16, Bug #3: RFEND+0x04
/// bits 8+12) are omitted, matching the lib BLE_RegInit behavior exactly.
///
/// All current callers pass `false` — the clears are harmless for ADV TX (the bits are
/// re-set by TX-mode setup before GO regardless) and are needed by the RX listener path.
/// The `true` path is available for future lib-faithful testing or caller-specific control.
pub unsafe fn ble_reg_init(skip_phase_b_rx_clears: bool) {
    // ── LLE+0x0C save / clear / fence.i  (WCH BLE_RegInit L71134-71136) ─────────
    // Saves and clears LLE+0x0C (interrupt/DMA mask) for the duration of calibration.
    // "delay(13)" = call to phy_status_clear(13) which is a no-op in standalone DTM
    // (LLE+0x00 & 3 == 0 → immediate return). No actual wait needed.
    let lle_0c_saved = BLE_LLE.irq_mask().read();
    BLE_LLE.irq_mask().write_value(0);
    asm!("fence.i");
    // Brief settle (200k iters) — semantically irrelevant (phy_status_clear is no-op here)
    // but retains ordering margin for any analog settle.
    for _ in 0..200_000u32 {
        core::hint::spin_loop();
    }

    // ── Pre-init: analog enable (WCH BLE_RegInit L71150-71168, ip.o reloc confirmed) ──
    //
    // BB+0x00 writes go to gptrBBReg = 0x40024100 (NOT RFEND_CAL+0x00).
    // Confirmed by ip.o objdump -d -r: s4 register = gptrBBReg, used for +0x00 RMWs.
    // Use raw u32 (`w.0`) on Ctrl fieldset because bit13 is outside the named fields.
    BLE_BB.ctrl().modify(|w| w.0 = (w.0 & !0x3000) | 0x1000); // clear bits[13:12], set bit12
    BLE_BB.ctrl().modify(|w| w.0 &= !0x0180);                  // clear bits[8:7]
    BLE_BB.ctrl().modify(|w| w.0 |= 0x0100);                    // set bit8
    //
    // RFEND+0x08: enable TX/RX PLL paths (bits 21+20+17+16 = 0x0033_0000).
    BLE_RFEND.path_en().modify(|w| {
        w.set_rx_adc_path(true);
        w.set_tx_cal_path(true);
        w.set_tx_pll_pre(true);
        w.set_rx_filter_path(true);
    });
    //
    // LLE+0x50 = 93 (settle timer).
    BLE_LLE.settle().write_value(93);

    // ── 4 calibration routines (WCH BLE_RegInit L71169-71176) ───────────────────
    rfend_tx_ctune();
    rfend_tx_ftune();
    rfend_rx_filter();
    rfend_rx_adc();

    // ── Post-cleanup (WCH BLE_RegInit L71177-71192, ip.o reloc confirmed) ────────
    BLE_BB.ctrl().modify(|w| w.0 = (w.0 & !0x0180) | 0x0080); // clear bits[8:7], set bit7
    BLE_RFEND
        .path_en()
        .modify(|w| w.0 &= !0x0032_0000); // clear bits 21+20+17 (WCH 0xFFCDFFFF)
    // Bug #2 fix: clear bit16 (RX_ADC path enable, set by rfend_rx_adc, not cleared before).
    // Phase B diff: our 0x000119f8 vs EVT 0x000019f8 — bit16 should not remain set post-cal.
    // Bug #3 fix: clear bits 8+12 (TXF_enable + RX_filter_mode, set during cal, never cleared).
    // Phase B diff: our 0x00011100 vs EVT 0x00010000 — bits 8+12 are calibration-only modes.
    //
    // NOTE: lib BLE_RegInit does NOT clear these bits. They are RX-path corrections needed
    // for the Phase B RX listener but HARMFUL for ADV TX (Tier 3 regression confirmed).
    // Skip when skip_phase_b_rx_clears=true (ADV TX / ble_ip_core_init path).
    if !skip_phase_b_rx_clears {
        BLE_RFEND.path_en().modify(|w| w.set_rx_adc_path(false)); // clear bit16
        BLE_RFEND.cal_trig().modify(|w| {
            w.set_txf_enable(false);
            w.set_rx_filter_mode(false);
        });
    }
    BLE_LLE.settle().write_value(0);
    BLE_LLE.irq_mask().write_value(lle_0c_saved); // restore LLE+0x0C
}

/// RFEND_TXCtune: TX carrier oscillator + gain calibration.
///
/// Measures CO (carrier offset, RFEND+0x90 bits[5:0]) and GA (gain adjust,
/// RFEND+0x94 bits[16:10]) at 3 reference frequencies (2401, 2480, 2440 MHz),
/// then linearly interpolates two CO tables and two GA segments into
/// nibble-packed lookup tables spanning RFEND+0xA0..0xD0.
///
/// Delta values (nCOxxx, nGAxxx) are signed 8-bit hardware measurements.
/// In practice delta_low (CO1 span) ≈ 6, delta_high (CO2 span) ≈ 7 per
/// live dump. Both must be in [-8..+7] for the 4-bit nibble encoding to
/// hold without overflow.
///
/// After calibration, RFEND+0x38 bits[5:0] = nCO2440, bits[30:24] = nGA2440.
unsafe fn rfend_tx_ctune() {
    // Setup: clear calibration-mode bits, enable TX cal path.
    BLE_RFEND.cal_trig().modify(|w| w.set_txf_enable(false)); // clear bit8 (TXF off during CO tune)
    BLE_RFEND.cfg0().modify(|w| *w &= !(1 << 12)); // clear bit12 (TX tune mode not yet final)
    BLE_RFEND.pll_vco().modify(|w| *w &= !(1 << 4)); // clear bit4 (post_cal_enable)
                                                     // BUG FIXED: was `0xF` (bits[3:0]) — wrong, matches WCH L75733-36
    BLE_RFEND.path_en().modify(|w| w.set_tx_cal_path(true)); // set bit17 (TX cal path) — Lucy: WCH L75738
    BLE_RFEND.cal_trig().modify(|w| w.set_tx_cal_mode(true)); // set bit4 (TX calibration mode)

    let (nco2401, nga2401, w2401) = tx_tune_measure(FREQ_CODE_2401);
    let (nco2480, nga2480, w2480) = tx_tune_measure(FREQ_CODE_2480);
    let (nco2440, nga2440, w2440) = tx_tune_measure(FREQ_CODE_2440);

    // Calibration result summary (w=u32::MAX means timeout; w=0 may indicate stale state).
    crate::println!(
        "rfend_tune: 2401 co={} ga={} w={} | 2480 co={} ga={} w={} | 2440 co={} ga={} w={}",
        nco2401,
        nga2401,
        w2401,
        nco2480,
        nga2480,
        w2480,
        nco2440,
        nga2440,
        w2440,
    );

    // CO table 1 (RFEND+0xA0..0xB0, 5 words, 40 nibbles for ch0..ch39).
    // CO1[ch] = delta_low * (39-ch) / 39  for ch=0..39
    // where delta_low = nCO2401 - nCO2440 (monotone decreasing: ch0=delta_low → ch39=0).
    // Live dump: delta_low=6, CO1[0]=6, CO1[39]=0 ✓
    let delta_low = nco2401 as i32 - nco2440 as i32;
    debug_assert!(
        (-8..=7).contains(&delta_low),
        "delta_low={} out of 4-bit signed range; CO1 nibbles will wrap",
        delta_low
    );
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = ((delta_low * (39 - ch as i32) / 39) & 0xF) as u32;
            word_val |= nibble << (i * 4);
        }
        BLE_RFEND.co_table1(word).write_value(word_val);
    }

    // CO table 2 (RFEND+0xB4..0xC4, 5 words, 40 nibbles for ch0..ch39).
    // CO2[ch] = delta_high * (ch+1) / 40  for ch=0..39
    // where delta_high = nCO2440 - nCO2480 (monotone increasing: ch0≈0 → ch39=delta_high).
    // Live dump: delta_high=7, CO2[0]=0, CO2[39]=7 ✓
    let delta_high = nco2440 as i32 - nco2480 as i32;
    debug_assert!(
        (-8..=7).contains(&delta_high),
        "delta_high={} out of 4-bit signed range; CO2 nibbles will wrap",
        delta_high
    );
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = ((delta_high * (ch as i32 + 1) / 40) & 0xF) as u32;
            word_val |= nibble << (i * 4);
        }
        BLE_RFEND.co_table2(word).write_value(word_val);
    }

    // CO2 overflow at RFEND+0xC8 (= ga_table[0]) nibbles 0-1 (ch40, ch41 for guard band).
    let co2_ch40 = ((delta_high * 41 / 40) & 0xF) as u32;
    let co2_ch41 = ((delta_high * 42 / 40) & 0xF) as u32;
    let prev_c8 = BLE_RFEND.ga_table(0).read();
    BLE_RFEND
        .ga_table(0)
        .write_value((prev_c8 & !0xFF) | co2_ch40 | (co2_ch41 << 4));

    // GA calibration tables (skipped if either CO delta is zero — degenerate hardware).
    // GA1 (0xC8 nibbles 2-7, 0xCC nibbles 0-3): GA1[n] = (delta_ga*(10-n)/delta_high|8)&0xF
    //   Divisor = delta_high; mults 10→1 descending; |8 minimum bias.
    // GA2 (0xCC nibbles 5-7, 0xD0 nibbles 0-6):  GA2[n] = (delta_ga2*(n+1)/delta_low)&0xF
    //   Divisor = delta_low; mults 1→10 ascending; no |8 bias.
    //   Negative delta_ga2 is valid: RISC-V idiv truncates toward zero, & 0xF gives
    //   4-bit two's-complement, which hardware interprets as a signed gain offset.
    //   This matches dtm.elf exactly — `lbu` loads nGA as [0..127], `sub` may go negative,
    //   then `and s1, s1, 15` is applied with no sign correction (confirmed from asm f73e-f748).
    // 0xCC nibble4 = 0 (separator, confirmed from asm).  0xD0 nibble7 = 0 (confirmed).
    let delta_ga = nga2440 as i32 - nga2480 as i32;
    let delta_ga2 = nga2401 as i32 - nga2440 as i32;

    if delta_high == 0 || delta_low == 0 {
        // Degenerate measurement (cold start, hardware anomaly): CO tables already written.
        // GA tables require a non-zero CO delta for normalization; skip to avoid divide-by-zero.
        #[cfg(feature = "defmt")]
        defmt::warn!(
            "rfend_tx_ctune: skipping GA cal (delta_high={} delta_low={})",
            delta_high,
            delta_low
        );
    } else {
        // 0xC8 (ga_table[0]) nibbles 2-7: GA1[0..5], preserving CO2 overflow in nibbles 0-1.
        {
            let mut v = BLE_RFEND.ga_table(0).read();
            for n in 0..6usize {
                let nib = ((delta_ga * (10 - n as i32) / delta_high | 8) & 0xF) as u32;
                let pos = (n + 2) * 4;
                v = (v & !(0xF << pos)) | (nib << pos);
            }
            BLE_RFEND.ga_table(0).write_value(v);
        }
        // 0xCC (ga_table[1]): GA1[6..9] at nibbles 0-3, separator at nibble4, GA2[0..2] at nibbles 5-7.
        // 0xD0 (ga_table[2]): GA2[3..9] at nibbles 0-6, nibble7 cleared.
        {
            let mut v = BLE_RFEND.ga_table(1).read();
            for n in 0..4usize {
                let nib = ((delta_ga * (4 - n as i32) / delta_high | 8) & 0xF) as u32;
                v = (v & !(0xF << (n * 4))) | (nib << (n * 4));
            }
            v &= !(0xF << 16); // nibble4 = 0 (separator)
            for n in 0..3usize {
                let raw = delta_ga2 * (n as i32 + 1) / delta_low;
                // dtm.elf has no clamping here; hardware expects GA2 nibbles in [0..7].
                debug_assert!((0..=7).contains(&raw), "GA2 nibble out of range: {}", raw);
                let nib = (raw & 0xF) as u32;
                v = (v & !(0xF << ((n + 5) * 4))) | (nib << ((n + 5) * 4));
            }
            BLE_RFEND.ga_table(1).write_value(v);

            let mut v = BLE_RFEND.ga_table(2).read();
            for n in 0..7usize {
                let raw = delta_ga2 * (n as i32 + 4) / delta_low;
                debug_assert!((0..=7).contains(&raw), "GA2 nibble out of range: {}", raw);
                let nib = (raw & 0xF) as u32;
                v = (v & !(0xF << (n * 4))) | (nib << (n * 4));
            }
            v &= !(0xF << 28); // nibble7 = 0
            BLE_RFEND.ga_table(2).write_value(v);
        }
    }

    // Teardown: clear calibration mode, switch to operational state.
    BLE_RFEND.cal_trig().modify(|w| {
        w.set_tx_cal_mode(false); // clear bit4
        w.set_tx_tune_trigger(false); // clear bit0
    });
    BLE_RFEND.cfg0().modify(|w| *w |= 1 << 12); // set bit12 (TX_tune_mode_final)
    BLE_RFEND.pll_vco().modify(|w| *w |= 1 << 4); // set bit4 (post_cal_enable)

    // Store calibration anchors at RFEND+0x38 (`cfg4` per metapac, but actually
    // the FREQ_CODE/NCO/NGA register per disassembly): bits[5:0] = nco2440,
    // bits[30:24] = nga2440. Preserve bit31 (rfend.rs PLL enable).
    // Hardware reads RFEND+0x38 on every TX burst for default channel compensation.
    let v = BLE_RFEND.cfg4().read();
    let v = (v & !0x3F) | (nco2440 as u32 & 0x3F);
    let v = (v & 0x80FF_FFFF) | ((nga2440 as u32 & 0x7F) << 24);
    BLE_RFEND.cfg4().write_value(v);
}

/// Perform one TX tune measurement at the given frequency code.
///
/// Returns `(CO bits[5:0], GA bits[16:10], wait_iters)` where `wait_iters` is the
/// number of spin iterations until bits[26:25] of RFEND+0x90 were both set.
/// `wait_iters == u32::MAX` signals a timeout (bits never became set).
///
/// Trigger: bit0 of RFEND+0x04 (TX_tune_trigger) deassert → assert pulse.
/// Requires the pre-init analog enable in ble_reg_init() to have run first:
/// without `RFEND+0x08 |= 0x0033_0000` the PLL analog path is unpowered
/// and the trigger pulse has no effect (confirmed 2026-05-01).
#[inline]
unsafe fn tx_tune_measure(freq_code: u32) -> (u8, u8, u32) {
    BLE_RFEND.cal_trig().modify(|w| w.set_tx_tune_trigger(false)); // deassert trigger
    // Set freq_code in bits[15:8] of cfg4 (= RFEND+0x38, the FREQ_CODE/NCO/NGA register).
    // Mask 0xFFFE_00FF preserves all bits except bits[16:8] — note the existing
    // pre-migration code also cleared bit16 here (intentional or harmless): we keep
    // the same bit-pattern for semantic equivalence.
    let v = BLE_RFEND.cfg4().read();
    BLE_RFEND.cfg4().write_value((v & 0xFFFE_00FF) | freq_code);
    BLE_RFEND.cal_trig().modify(|w| w.set_tx_tune_trigger(true)); // assert trigger → PLL starts measurement

    let iters = rfend_wait_tune();

    // Reading +0x90 (tune_result) latches GA into +0x94 (ga_result) per WCH read-side-effect.
    // rfend_wait_tune already performed several reads of +0x90 in the polling loop, so the
    // final read here is consistent with the post-lock measurement.
    let co = BLE_RFEND.tune_result().read().co(); // bits[5:0]
    let ga = ((BLE_RFEND.ga_result().read() >> 10) & 0x7F) as u8; // bits[16:10]
    (co, ga, iters)
}

/// Poll for TX tune completion — lib-faithful: LLE+0x64 write + bit-poll on RFEND+0x90.
///
/// Mirrors WCH `RFEND_WaitTune` (decoded 2026-05-01 by Lucy) byte-for-byte:
///
/// ```asm
/// *(LLE+0x64) = 6000           // write countdown (also clears stale bits[26:25])
/// loop:
///   s = *(RFEND+0x90)          // read +0x90 — latches GA into +0x94 (read side-effect)
///   if bit26 set:
///     s = *(RFEND+0x90)        // double-check
///     if bit25 set: return     // both set → PLL locked, CO+GA valid
///   if LLE+0x64 != 0: goto loop
///   return                     // countdown expired (timeout path)
/// ```
///
/// **V_A3 change (2026-05-04)**: Loop control now uses `LLE+0x64 != 0` as the loop condition,
/// matching lib byte-for-byte. Previous version used a SW `for i in 0..30_000` counter with
/// `hint::spin_loop()`, which created non-deterministic per-iteration timing (binary layout
/// sensitive) → 1-LSB ADC sample drift in nGA2440 across builds.
///
/// LLE+0x64 does not count down in standalone calibration (no active BLE events), so the
/// hw countdown path never fires — but the loop structure now matches lib exactly.
/// A hard SW cap of 300_000 iters guards against any hardware anomaly.
///
/// Two key hardware behaviours (unchanged, confirmed 2026-05-01):
/// 1. **LLE+0x64 write clears stale bits[26:25]**: without it, bits left from the previous
///    measurement cause rfend_wait_tune to return immediately (w=0) with old CO/GA.
/// 2. **Reading +0x90 latches GA into +0x94**: fixed-delay paths leave GA=64 (HW default).
///
/// Returns loop count at exit, or `u32::MAX` on SW cap timeout.
#[inline]
unsafe fn rfend_wait_tune() -> u32 {
    // Arm LLE+0x64 countdown = 6000. Matches lib RFEND_WaitTune entry.
    // Side-effect: clears stale bits[26:25] in RFEND+0x90 from the previous measurement.
    BLE_LLE.timer().write_value(6000);

    let mut i = 0u32;
    loop {
        // Reading +0x90 latches GA into +0x94 (WCH read-side-effect).
        let s = BLE_RFEND.tune_result().read();
        if s.tune_active() {
            // WCH double-checks bit26 then bit25.
            let s2 = BLE_RFEND.tune_result().read();
            if s2.tune_done() {
                return i;
            }
        }
        // Lib loop condition: LLE+0x64 hardware countdown (matches RFEND_WaitTune control flow).
        // In standalone mode, LLE+0x64 does not decrement — this path is a safety net.
        if BLE_LLE.timer().read() == 0 {
            return u32::MAX; // hw countdown expired
        }
        i = i.wrapping_add(1);
        if i >= 300_000 {
            return u32::MAX; // SW safety cap
        }
    }
}

/// RFEND_TXFtune: enable TX filter.
///
/// Trivial: set bit8 of RFEND+0x04 (TXF_enable).
/// From RFEND_TXFtune() at 0xf8c2 in dtm.elf.
unsafe fn rfend_tx_ftune() {
    BLE_RFEND.cal_trig().modify(|w| w.set_txf_enable(true));
}

/// RFEND_RXFilter: RX filter self-calibration.
///
/// Hardware self-calibrates the RX filter, polls RFEND+0x9C bit8 for done,
/// then stores result bits[4:0] into RFEND+0x50 bits[4:0].
/// On timeout the filter result register is read as-is (likely 0); RF will
/// still operate but with uncompensated filter offset.
/// From RFEND_RXFilter() at 0xead4 in dtm.elf.
unsafe fn rfend_rx_filter() {
    // RFEND+0x50 = `rf2` per metapac (Analog RF2). bit16 is RX_filter_valid;
    // bits[4:0] is RX_filter_result. Both are folded into the same register.
    BLE_RFEND.rf2().modify(|w| *w &= !(1 << 16)); // clear bit16 (RX_filter_valid)
    BLE_RFEND.path_en().modify(|w| w.set_rx_filter_path(true)); // set bit21 (RX_filter_path)
    // RFEND+0x0C = `ctrl` per metapac. bit4 = RX_filter_strobe.
    BLE_RFEND.ctrl().modify(|w| *w |= 1 << 4); // strobe high
    BLE_RFEND.ctrl().modify(|w| *w &= !(1 << 4)); // strobe low
    for _ in 0..4 {
        asm!("nop"); // 4-cycle settle delay
    }
    BLE_RFEND.ctrl().modify(|w| *w |= 1 << 4); // second strobe = actual calibration trigger
    BLE_RFEND.cal_trig().modify(|w| w.set_rx_filter_mode(true)); // set bit12

    // LLE+0x64 is event-driven (ticks only during active BLE events); use a software counter.
    let mut filter_done = false;
    for _ in 0..200_000u32 {
        if BLE_RFEND.rx_filter_result().read() & (1 << 8) != 0 {
            filter_done = true;
            break;
        }
        core::hint::spin_loop();
    }
    #[cfg(feature = "defmt")]
    if !filter_done {
        defmt::warn!("rfend_rx_filter timeout");
    }

    let result = BLE_RFEND.rx_filter_result().read() & 0x1F;
    let v = BLE_RFEND.rf2().read();
    BLE_RFEND.rf2().write_value((v | (1 << 16)) & !0x1F | result);

    BLE_RFEND.path_en().modify(|w| w.set_rx_filter_path(false)); // clear bit21
}

/// RFEND_RXAdc: configure RX ADC reference.
///
/// Clears and re-sets control bits in RFEND+0x58, 0x08, 0x0C, 0x04 to latch
/// the ADC reference configuration. No wait loop.
/// From RFEND_RXAdc() at 0xeb68 in dtm.elf.
unsafe fn rfend_rx_adc() {
    BLE_RFEND.adc_ref().modify(|w| *w &= !(1 << 16)); // clear bit16 (RX_ADC_ref disable)
    BLE_RFEND.path_en().modify(|w| w.set_rx_adc_path(true)); // set bit16 (RX_ADC path enable)
    BLE_RFEND.ctrl().modify(|w| *w &= !(1 << 8)); // clear bit8 (ADC_ref_strobe low)
    BLE_RFEND.cal_trig().modify(|w| w.set_rx_adc_config(false)); // clear bit16 (RX_ADC_config reset)
    BLE_RFEND.ctrl().modify(|w| *w |= 1 << 8); // set bit8 (ADC_ref_strobe high = latch)
    BLE_RFEND.cal_trig().modify(|w| w.set_rx_adc_config(true)); // set bit16 (RX_ADC_config apply)
}
