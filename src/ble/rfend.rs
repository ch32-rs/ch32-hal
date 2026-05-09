// RFEND (RF Frontend) initialization for CH32V208 BLE.
//
// Register block: `BLE_RFEND` (gptrRFENDReg) at 0x40025000 — same block whose
// calibration trigger / result registers are driven from regint.rs.
// Source: elec-docs/ble-reverse-docs/04-rfend-registers.md and 24-rf-phy-supplement.md
//
// BASE ADDRESS HISTORY: Originally coded as 0x40024300 (AES block / gptrAESReg) which
// was WRONG. ip.o BLE_IPCoreInit confirms: `lui a4, 0x40025` → *gptrRFENDReg=0x40025000;
// rfend.o RFEND_DevInit also loads from gptrRFENDReg → all writes target 0x40025000.
// The old 0x40024300 silently accepted writes but didn't configure the RF analog PLL,
// explaining why calibration always timed out (CO=0 for all 3 frequencies).
// Fixed 2026-05-01 (Lucy, ip.o reloc decode).
//
// Bug fixes applied 2026-05-01 from Lucy's full asm audit (L75253-75397):
//   A1: +0x48 clear mask 0x000F→0x0007 (bit19 must NOT be cleared)
//   A2: +0x4C set bit25 not bit21 (WCH t4=0x02000000, not 0x00200000)
//   A3: +0x50 set bit14 not bit12 (WCH a2=0x4000, not 0x1000)
//   A4: three RMW after +0x50[bit14] target +0x54, not +0x50
//   A5: +0x3C clear mask 0xE000_0000 (bit31 also cleared; was 0x6000)
//   B1: +0x2C block (4 RMW) entirely absent — WCH asm L75348-75367
//   B2: +0x30 block (5 RMW) entirely absent — WCH asm L75368-75384
//   B3: +0x38 bit-field write absent (clear bits[23:19], set bit20) — WCH asm L75385-75391
//
// 2026-05-09: migrated from raw `read_volatile/write_volatile` to typed
// `BLE_RFEND.{cfg0,rf0,...}().{read,write,modify}()` against the new
// ch32-metapac BLE_RFEND block. Bit patterns preserved exactly; semantics
// unchanged. Iron Law #38 hardware-gated.

use core::arch::asm;

use crate::pac::BLE_RFEND;

/// Reset the RF frontend.
///
/// Sequence from RFEND_Reset() (rfend.o): write 0x1101 → delay → write 0x0000 → delay → write 0x1101.
/// Hardware-confirmed reset value 0x1101 sets CTRL bits [12, 8, 0].
///
/// Delay: 100 nops ≈ 1 µs at 96 MHz (was 20 nops ≈ 200 ns — too short for analog deassert).
/// If analog reset deasserts too fast, the RFEND PLL domain stays unreachable.
pub unsafe fn rfend_reset() {
    BLE_RFEND.ctrl().write_value(0x1101);
    for _ in 0..100 {
        asm!("nop");
    }
    BLE_RFEND.ctrl().write_value(0x0000);
    for _ in 0..100 {
        asm!("nop");
    }
    BLE_RFEND.ctrl().write_value(0x1101);
}

/// Initialize the RF frontend.
///
/// Mirrors RFEND_DevInit() from libwchble.a V1.40 (rfend.o, line 77124).
/// Configures CFG0, RF0, RF1, RF2/RF2b, CFG5, PLL/VCO (+0x2C), loop filter (+0x30),
/// and CFG4 (+0x38) for normal BLE operation.
pub unsafe fn rfend_dev_init() {
    rfend_reset();

    // CFG0 (+0x28): default value 0x480, confirmed by hardware dump
    BLE_RFEND.cfg0().write_value(0x480);

    // RF0 (+0x48): multi-field analog RF configuration
    BLE_RFEND.rf0().modify(|w| *w = (*w & !0x7000_0000) | 0x2000_0000); // bits[30:28]: set bit29 only
    BLE_RFEND.rf0().modify(|w| *w = (*w & !0x0700_0000) | 0x0400_0000); // bits[26:24] = 0b100
    BLE_RFEND.rf0().modify(|w| *w = (*w & !0x0000_000F) | 0x0000_0009); // bits[3:0] = 9
    BLE_RFEND.rf0().modify(|w| *w &= !0x0007_0000);                    // bits[18:16] = 0 (A1: was 0xF → bit19 wrongly cleared)
    BLE_RFEND.rf0().modify(|w| *w |= 0x8000_0000);                     // bit31 = 1

    // RF1 (+0x4C): bias/filter configuration
    BLE_RFEND.rf1().modify(|w| *w = (*w & !0x07) | 0x03);              // bits[2:0] = 3
    BLE_RFEND.rf1().modify(|w| *w = (*w & !0x70) | 0x30);              // bits[6:4] = 3
    BLE_RFEND.rf1().modify(|w| *w = (*w & !0x700) | 0x300);            // bits[10:8] = 3
    BLE_RFEND.rf1().modify(|w| *w &= !0x0100_0000);                    // bit24 = 0
    BLE_RFEND.rf1().modify(|w| *w |= 0x0200_0000);                     // bit25 = 1 (A2: was 0x0020_0000 = bit21)

    // RF2 (+0x50): one write for bits[15:12], bit14 only
    BLE_RFEND.rf2().modify(|w| *w = (*w & !0x0000_F000) | 0x0000_4000); // bit14 = 1 (A3: was 0x1000 = bit12)

    // RF2b (+0x54): three RMW that were wrongly at +0x50 (A4)
    BLE_RFEND.rf2b().modify(|w| *w = (*w & !0x0000_000F) | 0x0000_000C); // bits[3:0] = 12
    BLE_RFEND.rf2b().modify(|w| *w |= 0x0000_0080);                      // bit7 = 1
    BLE_RFEND.rf2b().modify(|w| *w &= !0x0000_1000);                     // bit12 = 0

    // CFG5 (+0x3C): additional RF configuration. Held as Cfg5Freq fieldset, but
    // the bits touched here don't map to the named freq_code/nco/nga fields,
    // so we operate on the raw underlying u32 (`w.0`).
    BLE_RFEND.cfg5_freq().modify(|w| w.0 = (w.0 & !0x0000_F000) | 0x0000_8000); // bits[15:12] = 8
    BLE_RFEND.cfg5_freq().modify(|w| w.0 = (w.0 & !0x0700_0000) | 0x0200_0000); // bits[26:24]
    BLE_RFEND.cfg5_freq().modify(|w| w.0 = (w.0 & !0xE000_0000) | 0x4000_0000); // bit30=1, bit31=0 (A5: was 0x6000_0000)

    // PLL/VCO (+0x2C): entirely absent from original — WCH asm L75348-75367 (B1)
    BLE_RFEND.pll_vco().modify(|w| *w |= 0x0070_0000);                  // set bits[22:20]
    BLE_RFEND.pll_vco().modify(|w| *w &= !0x0700_0000);                 // clear bits[26:24]
    BLE_RFEND.pll_vco().modify(|w| *w = (*w & !0x0000_3000) | 0x0000_2000); // clear bits[13:12], set bit13
    BLE_RFEND.pll_vco().modify(|w| *w = (*w & !0x0003_0000) | 0x0002_0000); // clear bits[17:16], set bit17

    // Loop filter (+0x30): entirely absent from original — WCH asm L75368-75384 (B2)
    BLE_RFEND.loop_filter().modify(|w| *w &= !0x0000_000F);             // clear bits[3:0]
    BLE_RFEND.loop_filter().modify(|w| *w &= !0x0000_00F0);             // clear bits[7:4]
    BLE_RFEND.loop_filter().modify(|w| *w &= !0x0000_0700);             // clear bits[10:8]
    BLE_RFEND.loop_filter().modify(|w| *w |= 0x0070_0000);              // set bits[22:20]
    BLE_RFEND.loop_filter().modify(|w| *w = (*w & !0x7000_0000) | 0x5000_0000); // clear bits[30:28], set 0b101<<28

    // CFG4 (+0x38): PLL enable — WCH asm L75385-75391 (d_reloc.asm L99556-99563)
    // Bug #1 fix (2026-05-02): clear mask was 0x00F8_0000 (bits[23:19]), but lib uses
    // 0xff080000 - 1 = 0xff07ffff → keeps bits 0-18 + 26-31, clears bits[25:19].
    // Verified by RegTracer diff: post-init 0x38 had bit 25 wrongly set in fix2.
    BLE_RFEND.cfg4().modify(|w| *w = (*w & !0x03F8_0000) | 0x0010_0000); // clear bits[25:19], set bit20
    BLE_RFEND.cfg4().modify(|w| *w |= 0x8000_0000);                      // bit31 = 1
}
