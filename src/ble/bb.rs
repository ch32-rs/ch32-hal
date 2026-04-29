// BB (Baseband) initialization for CH32V208 BLE.
//
// Register base: 0x40024200
// Source: elec-docs/ble-reverse-docs/24-rf-phy-supplement.md
// Derived from BB_DevInit() in libwchble.a V1.40 (bb.o, line 71369)

use core::ptr::{read_volatile, write_volatile};

const BB_BASE: usize = 0x40024200;

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

/// BB RF flag — selects 1M or 2M PHY path in BB_CFG.
///
/// Bit 0: 1M PHY enabled; other bits: extended PHY modes.
/// Default 0 = 1M only. This is read from a global in libwchble; 0 matches
/// standard BLE 1M advertising usage.
pub const BB_RF_FLAG_1M: u8 = 0x00;

/// Initialize the BB baseband processor.
///
/// Mirrors BB_DevInit() from libwchble.a V1.40 (bb.o, line 71369).
/// Configures CTRL, TIMING, CFG, and MODE registers.
///
/// `rf_flag`: PHY mode selection. Use `BB_RF_FLAG_1M` for standard BLE 1M.
pub unsafe fn bb_dev_init(rf_flag: u8) {
    // CTRL (+0x00): set bit11 and bit20
    bb_modify(0x00, 0x0000_0000, 0x0000_0800); // bit11 = 1
    bb_modify(0x00, 0x0000_0000, 0x0010_0000); // bit20 = 1

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
