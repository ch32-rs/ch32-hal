// BB (Baseband) initialization for CH32V208 BLE.
//
// Register base: 0x40024100 (gptrBBReg — CTRL, GO, ACCESS_ADDR, CRC_INIT, TX mode, CFG, MODE)
// Source: elec-docs/ble-reverse-docs/24-rf-phy-supplement.md
// Derived from BB_DevInit() in libwchble.a V1.40 (bb.o, line 71369)

use core::ptr::{read_volatile, write_volatile};

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
    // CTRL (+0x00): set bit11 (GO strobe to clear any stale state) and bit20.
    // bit11 is a GO strobe — the hardware auto-clears it after processing.
    // We set it here to match BB_DevInit in libwchble, then clear it explicitly
    // so it starts at 0 for subsequent TX bursts.
    bb_modify(0x00, 0x0000_0000, 0x0000_0800); // bit11 = GO strobe
    // bit28 = 0x10000000: hardware enable bit (confirmed from BB_DevInit in dtm.elf:
    // `lui a3,0x10000` → or CTRL,CTRL,a3).  Previously wrong as bit20 (0x100000).
    bb_modify(0x00, 0x0000_0000, 0x1000_0000); // bit28 = 1
    bb_modify(0x00, 0x0000_0800, 0x0000_0000); // clear bit11 after strobe

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
