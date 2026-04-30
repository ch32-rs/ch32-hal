// BLE_RegInit — RF calibration for CH32V208 BLE.
//
// Implements the 4 calibration functions called by BLE_RegInit() in libwchble.a V1.40:
//   1. RFEND_TXCtune  — TX carrier + gain calibration (40-channel lookup table)
//   2. RFEND_TXFtune  — TX filter enable (trivial: set one bit)
//   3. RFEND_RXFilter — RX filter self-calibration with hardware poll
//   4. RFEND_RXAdc    — RX ADC reference configuration
//
// Register bases (WCH uses two distinct RFEND address regions):
//   RFEND_BASE    (rfend.rs)  = 0x40024300  — gptrAESReg, used by RFEND_DevInit only
//   RFEND_CAL_BASE (this file) = 0x40025000  — gptrRFENDReg, PA bias / PLL / cal tables
//
// The 0x40025000 block was confirmed by dumping memory after running official dtm.elf:
//   wlink dump 0x40025000+0xA0..0xC8 → CO1 table word0=0x45555556 (ch0=6,ch7=4),
//   CO2 table word0=0x11100000 (ch0=0,ch5=1) — matches RFEND_TXCtune algorithm ✓.
//
// Register intent map (from disassembly of libwchble.a V1.40, confirmed):
//   +0x04  bits: bit0=TX_tune_trigger, bit4=TX_cal_mode, bit8=TXF_enable,
//                bit12=RX_filter_mode, bit16=RX_ADC_config
//   +0x08  bits: bit16=RX_ADC_path, bit17=TX_cal_path, bit21=RX_filter_path
//   +0x0C  bits: bit4=RX_filter_strobe, bit8=ADC_ref_strobe
//   +0x28  bits: bit12=TX_tune_mode_final (set after calibration completes)
//   +0x2C  bits: bit4=post_cal_enable
//   +0x38  bits[15:8]=freq_code, bits[5:0]=nCO2440, bits[30:24]=nGA2440
//   +0x50  bits: bit16=RX_filter_valid, bits[4:0]=RX_filter_result
//   +0x58  bits: bit16=RX_ADC_ref_enable
//   +0x90  bits: bits[5:0]=CO_result, bit25=tune_done, bit26=tune_active
//   +0x94  bits[16:10]: GA_result (7 bits)
//   +0x9C  bits: bit8=RX_filter_done, bits[4:0]=RX_filter_result
//   +0xA0..0xB0: CO calibration table 1 (nibble-packed, 40 channels, ch0→ch39)
//   +0xB4..0xC4: CO calibration table 2 (nibble-packed, 40 channels, ch0→ch39)
//   +0xC8..0xD0: GA calibration + CO2 overflow
//
// Sources: static disassembly of dtm.elf (WCH BLE SDK V1.40), live register dump.

use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

// gptrRFENDReg: PA bias, PLL, channel lock, calibration tables.
// Distinct from RFEND_BASE in rfend.rs (0x40024300 = gptrAESReg used by RFEND_DevInit).
const RFEND_CAL_BASE: usize = 0x40025000;
// gptrLLEReg: provides LLE+0x64 countdown register for calibration timeouts.
const LLE_BASE: usize = 0x40024200;

// Frequency selection codes written to RFEND_CAL_BASE+0x38 bits[15:8].
// Confirmed from BB_DevInit / RF_DevSetChannel disassembly.
const FREQ_CODE_2401: u32 = 0xBF00; // one step below BLE ch0 (2402 MHz)
const FREQ_CODE_2480: u32 = 0xE700; // BLE ch39
const FREQ_CODE_2440: u32 = 0xD300; // BLE ch19, band midpoint

#[inline(always)]
unsafe fn r(off: usize) -> u32 {
    read_volatile((RFEND_CAL_BASE + off) as *const u32)
}

#[inline(always)]
unsafe fn w(off: usize, val: u32) {
    write_volatile((RFEND_CAL_BASE + off) as *mut u32, val);
}

#[inline(always)]
unsafe fn rmw(off: usize, clear: u32, set: u32) {
    w(off, (r(off) & !clear) | set);
}

/// Run all 4 RF calibration routines in the order used by BLE_RegInit.
///
/// Must be called after `ble_phy_init()` (RFEND_DevInit + BB_DevInit + LLE_DevInit).
/// Calibrates TX carrier, TX filter, RX filter, and RX ADC for the current temperature.
///
/// # Safety
/// Requires the RFEND/LLE peripherals to be initialized and accessible.
pub unsafe fn ble_reg_init() {
    rfend_tx_ctune();
    rfend_tx_ftune();
    rfend_rx_filter();
    rfend_rx_adc();
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
    rmw(0x04, 1 << 8, 0);  // clear bit8 (TXF enable off during CO tune)
    rmw(0x28, 1 << 12, 0); // clear bit12 (TX tune mode not yet final)
    rmw(0x2C, 0xF, 0);     // clear bits[3:0]
    rmw(0x08, 0, 1 << 17); // set bit17 (TX calibration path enable)
    rmw(0x04, 0, 1 << 4);  // set bit4 (TX calibration mode)

    let (nco2401, nga2401) = tx_tune_measure(FREQ_CODE_2401);
    let (nco2480, nga2480) = tx_tune_measure(FREQ_CODE_2480);
    let (nco2440, nga2440) = tx_tune_measure(FREQ_CODE_2440);

    // CO table 1 (RFEND+0xA0..0xB0, 5 words, 40 nibbles for ch0..ch39).
    // CO1[ch] = delta_low * (39-ch) / 39  for ch=0..39
    // where delta_low = nCO2401 - nCO2440 (monotone decreasing: ch0=delta_low → ch39=0).
    // Live dump: delta_low=6, CO1[0]=6, CO1[39]=0 ✓
    let delta_low = nco2401 as i32 - nco2440 as i32;
    debug_assert!(
        (-8..=7).contains(&delta_low),
        "delta_low={} out of 4-bit signed range; CO1 nibbles will wrap", delta_low
    );
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = ((delta_low * (39 - ch as i32) / 39) & 0xF) as u32;
            word_val |= nibble << (i * 4);
        }
        w(0xA0 + word * 4, word_val);
    }

    // CO table 2 (RFEND+0xB4..0xC4, 5 words, 40 nibbles for ch0..ch39).
    // CO2[ch] = delta_high * (ch+1) / 40  for ch=0..39
    // where delta_high = nCO2440 - nCO2480 (monotone increasing: ch0≈0 → ch39=delta_high).
    // Live dump: delta_high=7, CO2[0]=0, CO2[39]=7 ✓
    let delta_high = nco2440 as i32 - nco2480 as i32;
    debug_assert!(
        (-8..=7).contains(&delta_high),
        "delta_high={} out of 4-bit signed range; CO2 nibbles will wrap", delta_high
    );
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = ((delta_high * (ch as i32 + 1) / 40) & 0xF) as u32;
            word_val |= nibble << (i * 4);
        }
        w(0xB4 + word * 4, word_val);
    }

    // CO2 overflow at RFEND+0xC8 nibbles 0-1 (ch40, ch41 for guard-band use).
    let co2_ch40 = ((delta_high * 41 / 40) & 0xF) as u32;
    let co2_ch41 = ((delta_high * 42 / 40) & 0xF) as u32;
    let prev_c8 = r(0xC8);
    w(0xC8, (prev_c8 & !0xFF) | co2_ch40 | (co2_ch41 << 4));

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
            delta_high, delta_low
        );
    } else {
        // 0xC8 nibbles 2-7: GA1[0..5], preserving CO2 overflow in nibbles 0-1.
        {
            let mut v = r(0xC8);
            for n in 0..6usize {
                let nib = ((delta_ga * (10 - n as i32) / delta_high | 8) & 0xF) as u32;
                let pos = (n + 2) * 4;
                v = (v & !(0xF << pos)) | (nib << pos);
            }
            w(0xC8, v);
        }
        // 0xCC: GA1[6..9] at nibbles 0-3, separator at nibble4, GA2[0..2] at nibbles 5-7.
        // 0xD0: GA2[3..9] at nibbles 0-6, nibble7 cleared.
        {
            let mut v = r(0xCC);
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
            w(0xCC, v);

            let mut v = r(0xD0);
            for n in 0..7usize {
                let raw = delta_ga2 * (n as i32 + 4) / delta_low;
                debug_assert!((0..=7).contains(&raw), "GA2 nibble out of range: {}", raw);
                let nib = (raw & 0xF) as u32;
                v = (v & !(0xF << (n * 4))) | (nib << (n * 4));
            }
            v &= !(0xF << 28); // nibble7 = 0
            w(0xD0, v);
        }
    }

    // Teardown: clear calibration mode, switch to operational state.
    rmw(0x04, (1 << 4) | 1, 0); // clear bit4 (TX_cal_mode) and bit0 (trigger)
    rmw(0x28, 0, 1 << 12);       // set bit12 (TX_tune_mode_final)
    rmw(0x2C, 0, 1 << 4);        // set bit4 (post_cal_enable)

    // Store calibration anchors: nCO2440 in bits[5:0], nGA2440 in bits[30:24].
    // Hardware reads RFEND+0x38 on every TX burst for default channel compensation.
    let v = r(0x38);
    let v = (v & !0x3F) | (nco2440 as u32 & 0x3F);
    let v = (v & 0x80FF_FFFF) | ((nga2440 as u32 & 0x7F) << 24);
    w(0x38, v);
}

/// Perform one TX tune measurement at the given frequency code.
///
/// `freq_code`: value with bits[15:8] = frequency select code (e.g. `FREQ_CODE_2401`).
/// Writes to RFEND+0x38 bits[15:8] (mask 0xFFFE_00FF clears bits[16:8]; bit16 is always 0).
///
/// Returns (CO result bits[5:0], GA result bits[16:10]) after tune completion.
/// On timeout the function returns whatever stale values are in the result registers.
#[inline]
unsafe fn tx_tune_measure(freq_code: u32) -> (u8, u8) {
    rmw(0x04, 1, 0); // deassert previous trigger
    let v = r(0x38);
    w(0x38, (v & 0xFFFE_00FF) | freq_code); // bits[15:8] ← freq code
    rmw(0x04, 0, 1); // assert trigger

    if !rfend_wait_tune() {
        #[cfg(feature = "defmt")]
        defmt::warn!("rfend_wait_tune timeout (freq_code={:#06x})", freq_code);
    }

    let co = (r(0x90) & 0x3F) as u8;        // bits[5:0]
    let ga = ((r(0x94) >> 10) & 0x7F) as u8; // bits[16:10]
    (co, ga)
}

/// Poll for TX tune completion using the LLE countdown timer as timeout.
///
/// Polls RFEND+0x90 bit26 (active phase) + bit25 (done) in a single read per iteration.
/// Assembly reads them separately (two lw instructions) but the combined check is
/// functionally identical since we only proceed when both are set simultaneously.
///
/// Returns `true` if tune completed, `false` on timeout (LLE+0x64 reaches 0).
#[inline]
/// Poll for TX tune completion.
///
/// LLE+0x64 is event-driven (ticks only during active BLE events) so a software
/// iteration counter is used instead of the original LLE countdown timeout.
/// In practice, RFEND+0x90 bits 26+25 are set immediately after trigger assertion.
#[inline]
unsafe fn rfend_wait_tune() -> bool {
    for _ in 0..200_000u32 {
        let status = r(0x90);
        if status & (1 << 26) != 0 && status & (1 << 25) != 0 {
            return true;
        }
        core::hint::spin_loop();
    }
    #[cfg(feature = "defmt")]
    defmt::warn!("rfend_wait_tune timeout");
    false
}

/// RFEND_TXFtune: enable TX filter.
///
/// Trivial: set bit8 of RFEND+0x04 (TXF_enable).
/// From RFEND_TXFtune() at 0xf8c2 in dtm.elf.
unsafe fn rfend_tx_ftune() {
    rmw(0x04, 0, 1 << 8);
}

/// RFEND_RXFilter: RX filter self-calibration.
///
/// Hardware self-calibrates the RX filter, polls RFEND+0x9C bit8 for done,
/// then stores result bits[4:0] into RFEND+0x50 bits[4:0].
/// On timeout the filter result register is read as-is (likely 0); RF will
/// still operate but with uncompensated filter offset.
/// From RFEND_RXFilter() at 0xead4 in dtm.elf.
unsafe fn rfend_rx_filter() {
    rmw(0x50, 1 << 16, 0);          // clear bit16 (RX_filter_valid)
    rmw(0x08, 0, 1 << 21);          // set bit21 (RX_filter_path enable)
    rmw(0x0C, 0, 1 << 4);           // strobe high
    rmw(0x0C, 1 << 4, 0);           // strobe low
    for _ in 0..4 {
        unsafe { asm!("nop") };      // 4-cycle settle delay
    }
    rmw(0x0C, 0, 1 << 4);           // second strobe = actual calibration trigger
    rmw(0x04, 0, 1 << 12);          // set bit12 (RX_filter_mode)

    // LLE+0x64 is event-driven (ticks only during active BLE events); use a software counter.
    let mut filter_done = false;
    for _ in 0..200_000u32 {
        if r(0x9C) & (1 << 8) != 0 {
            filter_done = true;
            break;
        }
        core::hint::spin_loop();
    }
    #[cfg(feature = "defmt")]
    if !filter_done {
        defmt::warn!("rfend_rx_filter timeout");
    }

    let result = r(0x9C) & 0x1F;
    let v = r(0x50);
    w(0x50, (v | (1 << 16)) & !0x1F | result);

    rmw(0x08, 1 << 21, 0);          // clear bit21 (disable RX filter path)
}

/// RFEND_RXAdc: configure RX ADC reference.
///
/// Clears and re-sets control bits in RFEND+0x58, 0x08, 0x0C, 0x04 to latch
/// the ADC reference configuration. No wait loop.
/// From RFEND_RXAdc() at 0xeb68 in dtm.elf.
unsafe fn rfend_rx_adc() {
    rmw(0x58, 1 << 16, 0);          // clear bit16 (RX_ADC_ref disable)
    rmw(0x08, 0, 1 << 16);          // set bit16 (RX_ADC path enable)
    rmw(0x0C, 1 << 8, 0);           // clear bit8 (ADC_ref_strobe low)
    rmw(0x04, 1 << 16, 0);          // clear bit16 (RX_ADC_config reset)
    rmw(0x0C, 0, 1 << 8);           // set bit8 (ADC_ref_strobe high = latch)
    rmw(0x04, 0, 1 << 16);          // set bit16 (RX_ADC_config apply)
}
