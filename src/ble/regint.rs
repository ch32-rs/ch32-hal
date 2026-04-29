// BLE_RegInit — RF calibration for CH32V208 BLE.
//
// Implements the 4 calibration functions called by BLE_RegInit() in libwchble.a V1.40:
//   1. RFEND_TXCtune  — TX carrier + gain calibration (40-channel lookup table)
//   2. RFEND_TXFtune  — TX filter enable (trivial: set one bit)
//   3. RFEND_RXFilter — RX filter self-calibration with hardware poll
//   4. RFEND_RXAdc    — RX ADC reference configuration
//
// All calibration functions use gptrRFENDReg (0x40025000) as the RFEND base.
// This is DISTINCT from the RFEND_DevInit base at 0x40024300 (gptrAESReg).
//
// Register intent map (from disassembly of libwchble.a V1.40, confirmed):
//   RFEND+0x04  bits: bit0=TX_tune_trigger, bit4=TX_cal_mode, bit8=TXF_enable,
//                     bit12=RX_filter_mode, bit16=RX_ADC_config
//   RFEND+0x08  bits: bit16=RX_ADC_path, bit17=TX_cal_path, bit21=RX_filter_path
//   RFEND+0x0C  bits: bit4=RX_filter_strobe, bit8=ADC_ref_strobe
//   RFEND+0x28  bits: bit12=TX_tune_mode_final (set after calibration completes)
//   RFEND+0x2C  bits: bit4=post_cal_enable
//   RFEND+0x38  bits[15:8]=freq_code, bits[5:0]=nCO2440, bits[30:24]=nGA2440
//   RFEND+0x50  bits: bit16=RX_filter_valid, bits[4:0]=RX_filter_result
//   RFEND+0x58  bits: bit16=RX_ADC_ref_enable
//   RFEND+0x90  bits: bits[5:0]=CO_result, bit25=tune_done, bit26=tune_active
//   RFEND+0x94  bits[16:10]: GA_result (7 bits)
//   RFEND+0x9C  bits: bit8=RX_filter_done, bits[4:0]=RX_filter_result
//   RFEND+0xA0..0xB0: CO calibration table 1 (nibble-packed, 40 channels, ch0→ch39)
//   RFEND+0xB4..0xC4: CO calibration table 2 (nibble-packed, 40 channels, ch0→ch39)
//   RFEND+0xC8..0xD0: GA calibration + CO overflow (partially reversed, see TODO)
//
// Sources: static disassembly of dtm.elf (WCH BLE SDK V1.40), live register dump.

use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

// gptrRFENDReg: PA bias, PLL, channel lock, calibration tables.
const RFEND_BASE: usize = 0x40025000;
// gptrLLEReg: provides LLE+0x64 countdown register for calibration timeouts.
const LLE_BASE: usize = 0x40024200;

// Magic cookie: if ftuneFlag matches this value, RFEND_TXCtune skips re-measurement
// and reuses cached nCO/nGA values from the previous calibration run.
// Writing 0 to ftuneFlag forces a fresh calibration on next call.
// WCH stores ftuneFlag at a fixed RAM address; we always run fresh.

#[inline(always)]
unsafe fn r(off: usize) -> u32 {
    read_volatile((RFEND_BASE + off) as *const u32)
}

#[inline(always)]
unsafe fn w(off: usize, val: u32) {
    write_volatile((RFEND_BASE + off) as *mut u32, val);
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
/// Measures CO (carrier offset, bits[5:0] of RFEND+0x90) and GA (gain adjust,
/// bits[16:10] of RFEND+0x94) at 3 reference frequencies (2401, 2480, 2440 MHz),
/// then linearly interpolates two 40-channel nibble-packed lookup tables.
///
/// After calibration, RFEND+0x38 stores nCO2440 in bits[5:0] and nGA2440 in
/// bits[30:24] as the default channel configuration.
unsafe fn rfend_tx_ctune() {
    // Setup: clear calibration-mode bits, enable TX cal path.
    // RFEND+0x04 clear bit8 (TXF enable off during CO tune)
    rmw(0x04, 1 << 8, 0);
    // RFEND+0x28 clear bit12 (TX tune mode not yet final)
    rmw(0x28, 1 << 12, 0);
    // RFEND+0x2C clear bits[3:0]
    rmw(0x2C, 0xF, 0);
    // RFEND+0x08 set bit17 (TX calibration path enable)
    rmw(0x08, 0, 1 << 17);
    // RFEND+0x04 set bit4 (TX calibration mode)
    rmw(0x04, 0, 1 << 4);

    // Measure CO and GA at 3 reference frequencies.
    // Freq codes for RFEND+0x38 bits[15:8]: confirmed from BB_DevInit/RF_DevSetChannel.
    //   0xBF = 191 → 2401 MHz  (one step below BLE ch0 at 2402 MHz)
    //   0xE7 = 231 → 2480 MHz  (= BLE ch39)
    //   0xD3 = 211 → 2440 MHz  (≈ BLE ch19, midpoint)
    let (nco2401, nga2401) = tx_tune_measure(0xBF00);
    let (nco2480, nga2480) = tx_tune_measure(0xE700);
    let (nco2440, nga2440) = tx_tune_measure(0xD300);

    // CO table 1 (RFEND+0xA0..0xB0, 5 words, 40 nibbles for ch0..ch39).
    // Linear interpolation anchored at nCO2401 (ch0) and nCO2440 (ch19):
    //   CO1[0]  = (nco2401 - nco2440) & 0xF          (raw delta, ch0)
    //   CO1[ch] = (nco2401 - nco2440) * (39-ch) / 39  for ch = 1..39
    // Confirmed: live dump CO1[0]=6, CO1[39]=0, monotone decreasing.
    let delta_low = nco2401 as i32 - nco2440 as i32;
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = if ch == 0 {
                (delta_low & 0xF) as u32
            } else {
                ((delta_low * (39 - ch as i32) / 39) & 0xF) as u32
            };
            word_val |= nibble << (i * 4);
        }
        w(0xA0 + word * 4, word_val);
    }

    // CO table 2 (RFEND+0xB4..0xC4, 5 words, 40 nibbles for ch0..ch39).
    // Anchored at nCO2440 and nCO2480:
    //   CO2[ch] = (nco2440 - nco2480) * (ch+1) / 40  for ch = 0..39
    // Confirmed: live dump CO2[0]=0, CO2[39]=7, monotone increasing.
    let delta_high = nco2440 as i32 - nco2480 as i32;
    for word in 0..5usize {
        let mut word_val = 0u32;
        for i in 0..8usize {
            let ch = word * 8 + i;
            let nibble = ((delta_high * (ch as i32 + 1) / 40) & 0xF) as u32;
            word_val |= nibble << (i * 4);
        }
        w(0xB4 + word * 4, word_val);
    }

    // CO2 overflow: 2 extra entries at RFEND+0xC8 nibbles 0-1 (ch40, ch41).
    // The hardware reads up to ch41 for guard-band / out-of-band channels.
    let co2_ch40 = ((delta_high * 41 / 40) & 0xF) as u32;
    let co2_ch41 = ((delta_high * 42 / 40) & 0xF) as u32;
    let prev_c8 = r(0xC8);
    w(0xC8, (prev_c8 & !0xFF) | co2_ch40 | (co2_ch41 << 4));

    // GA table segment 1 (0xC8 nibbles 2-7, 0xCC nibbles 0-3):
    // Multipliers 10→1 descending, divisor=delta_high, |8 minimum bias.
    // GA1[n] = (delta_ga * (10-n) / delta_high | 8) & 0xF, n=0..9
    let delta_ga = nga2440 as i32 - nga2480 as i32;
    {
        // 0xC8 nibbles 2-7 (n=0..5, mult=10..5): preserve CO2 overflow in nibbles 0-1.
        let mut v = r(0xC8);
        for n in 0..6usize {
            let mult = (10 - n) as i32;
            let nib = ((delta_ga * mult / delta_high | 8) & 0xF) as u32;
            let pos = (n + 2) * 4;
            v = (v & !(0xF << pos)) | (nib << pos);
        }
        w(0xC8, v);
    }
    {
        // 0xCC: nibbles 0-3 (GA1 n=6..9, mult=4..1), nibble4=0, nibbles 5-7 (GA2 n=0..2, mult=1..3).
        // GA segment 2: delta_ga2 = nGA2401-nGA2440, divisor=delta_low (=nCO2401-nCO2440).
        // GA2[n] = (delta_ga2 * (n+1) / delta_low) & 0xF  — no |8 bias.
        let delta_ga2 = nga2401 as i32 - nga2440 as i32;
        let mut v = r(0xCC);
        for n in 0..4usize {
            let mult = (4 - n) as i32;
            let nib = ((delta_ga * mult / delta_high | 8) & 0xF) as u32;
            v = (v & !(0xF << (n * 4))) | (nib << (n * 4));
        }
        v &= !(0xF << 16); // nibble 4 = 0 (separator between the two GA segments)
        for n in 0..3usize {
            let mult = (n + 1) as i32;
            let nib = ((delta_ga2 * mult / delta_low) & 0xF) as u32;
            v = (v & !(0xF << ((n + 5) * 4))) | (nib << ((n + 5) * 4));
        }
        w(0xCC, v);
        // 0xD0 nibbles 0-6 (GA2 n=3..9, mult=4..10); nibble7 cleared to 0.
        let mut v = r(0xD0);
        for n in 0..7usize {
            let mult = (n + 4) as i32;
            let nib = ((delta_ga2 * mult / delta_low) & 0xF) as u32;
            v = (v & !(0xF << (n * 4))) | (nib << (n * 4));
        }
        v &= !(0xF << 28);
        w(0xD0, v);
    }

    // Teardown: clear calibration mode, switch to operational state.
    rmw(0x04, (1 << 4) | 1, 0); // clear bit4 (TX_cal_mode) and bit0 (trigger)
    rmw(0x28, 0, 1 << 12);       // set bit12 (TX_tune_mode_final)
    rmw(0x2C, 0, 1 << 4);        // set bit4 (post_cal_enable)

    // Store calibration anchors: nCO2440 in RFEND+0x38 bits[5:0], nGA2440 in bits[30:24].
    // Hardware reads these on every TX burst for default channel compensation.
    let v = r(0x38);
    let v = (v & !0x3F) | (nco2440 as u32 & 0x3F);
    let v = (v & 0x80FF_FFFF) | ((nga2440 as u32 & 0x7F) << 24);
    w(0x38, v);
}

/// Perform one TX tune measurement at the given frequency code.
///
/// `freq_code`: 16-bit value with bits[15:8] = frequency select code.
///   0xBF00 = 2401 MHz, 0xE700 = 2480 MHz, 0xD300 = 2440 MHz.
///
/// Returns (CO result [5:0], GA result [6:0]) after hardware tune completion.
#[inline]
unsafe fn tx_tune_measure(freq_code: u32) -> (u8, u8) {
    // Clear bit0 (deassert previous trigger), set freq code, assert trigger.
    rmw(0x04, 1, 0);
    let v = r(0x38);
    w(0x38, (v & 0xFFFE_00FF) | freq_code); // bits[16:8] → freq code
    rmw(0x04, 0, 1);

    rfend_wait_tune();

    let co = (r(0x90) & 0x3F) as u8;        // bits[5:0] of RFEND+0x90
    let ga = ((r(0x94) >> 10) & 0x7F) as u8; // bits[16:10] of RFEND+0x94
    (co, ga)
}

/// Poll for TX tune completion using the LLE countdown timer as timeout.
///
/// RFEND+0x90 bit26 = tune phase active; bit25 = tune done.
/// LLE+0x64 is used as a software countdown written before polling.
#[inline]
unsafe fn rfend_wait_tune() {
    write_volatile((LLE_BASE + 0x64) as *mut u32, 6000);
    loop {
        let status = r(0x90);
        if status & (1 << 26) != 0 && status & (1 << 25) != 0 {
            return; // active phase + done flag both set
        }
        let countdown = read_volatile((LLE_BASE + 0x64) as *const u32);
        if countdown == 0 {
            return; // timeout
        }
    }
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
/// then stores the result bits[4:0] into RFEND+0x50 bits[4:0].
/// From RFEND_RXFilter() at 0xead4 in dtm.elf.
unsafe fn rfend_rx_filter() {
    // Enable RX filter path and trigger calibration strobe.
    rmw(0x50, 1 << 16, 0);          // RFEND+0x50 clear bit16 (RX_filter_valid)
    rmw(0x08, 0, 1 << 21);          // RFEND+0x08 set bit21 (RX_filter_path enable)
    rmw(0x0C, 0, 1 << 4);           // RFEND+0x0C set bit4 (strobe high)
    rmw(0x0C, 1 << 4, 0);           // RFEND+0x0C clear bit4 (strobe low)
    // 4-cycle delay for strobe settle.
    for _ in 0..4 {
        unsafe { asm!("nop") };
    }
    rmw(0x0C, 0, 1 << 4);           // RFEND+0x0C set bit4 (second strobe = actual trigger)
    rmw(0x04, 0, 1 << 12);          // RFEND+0x04 set bit12 (RX_filter_mode)

    // Poll for done: RFEND+0x9C bit8, countdown from LLE+0x64 = 80.
    write_volatile((LLE_BASE + 0x64) as *mut u32, 80);
    loop {
        if r(0x9C) & (1 << 8) != 0 {
            break; // calibration done
        }
        let countdown = read_volatile((LLE_BASE + 0x64) as *const u32);
        if countdown == 0 {
            break; // timeout
        }
    }

    // Store calibration result: RFEND+0x50 = (prev | bit16) with bits[4:0] = result.
    let result = r(0x9C) & 0x1F;
    let v = r(0x50);
    w(0x50, (v | (1 << 16)) & !0x1F | result);

    rmw(0x08, 1 << 21, 0);          // RFEND+0x08 clear bit21 (disable RX filter path)
}

/// RFEND_RXAdc: configure RX ADC reference.
///
/// Clears and re-sets control bits in RFEND+0x58, 0x08, 0x0C, 0x04 to latch
/// the ADC reference configuration. No wait loop.
/// From RFEND_RXAdc() at 0xeb68 in dtm.elf.
unsafe fn rfend_rx_adc() {
    rmw(0x58, 1 << 16, 0);          // RFEND+0x58 clear bit16 (RX_ADC_ref disable)
    rmw(0x08, 0, 1 << 16);          // RFEND+0x08 set bit16 (RX_ADC path enable)
    rmw(0x0C, 1 << 8, 0);           // RFEND+0x0C clear bit8 (ADC_ref_strobe low)
    rmw(0x04, 1 << 16, 0);          // RFEND+0x04 clear bit16 (RX_ADC_config reset)
    rmw(0x0C, 0, 1 << 8);           // RFEND+0x0C set bit8 (ADC_ref_strobe high = latch)
    rmw(0x04, 0, 1 << 16);          // RFEND+0x04 set bit16 (RX_ADC_config apply)
}
