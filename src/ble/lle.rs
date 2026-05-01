// LLE (Link Layer Engine) initialization for CH32V208 BLE.
//
// Register base: 0x40024200 (gptrLLEReg — timing, IRQ, timer, TX buf ptr)
// Source: elec-docs/ble-reverse-docs/05-lle-engine.md and hardware dump
// Hardware-confirmed timing values from live CH32V208WBU6 board dump.

use core::ptr::{read_volatile, write_volatile};

// gptrLLEReg in WCH naming: timing, scheduling, IRQ status, timer, TX buffer.
const LLE_BASE: usize = 0x40024200;

/// Link-layer DMA buffer — maps to gBleIPPara[36] (MEMAddr) in WCH BLE stack.
///
/// WCH's LLE_DevInit writes `LLE+0x74 = MEMAddr` (from pInitConfig->MEMAddr).
/// Without a valid non-zero address here, the LLE state machine may refuse to
/// trigger any burst, which blocks RFEND PLL calibration (rfend_tx_ctune).
///
/// Size: 1 KB (256×u32). WCH uses a larger pool for full TMOS BLE, but for
/// standalone DTM/calibration this size is sufficient.
#[link_section = ".bss"]
static mut LLE_DMA_BUF: [u32; 256] = [0; 256];

#[inline(always)]
unsafe fn lle_read(offset: usize) -> u32 {
    read_volatile((LLE_BASE + offset) as *const u32)
}

#[inline(always)]
unsafe fn lle_write(offset: usize, val: u32) {
    write_volatile((LLE_BASE + offset) as *mut u32, val);
}

/// LLE state machine values (observed from live hardware and assembly).
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum LleState {
    /// No connection, waiting for RX.
    ConnRxWait = 93,
    /// Connection TX prepare.
    ConnTxPrep = 97,
    /// Connection ACK wait.
    ConnAckWait = 101,
    /// Connection event closing.
    ConnEventClosing = 105,
    /// Preparing for sleep.
    SleepPrep = 107,
    /// Sleep state (default after init).
    Sleep = 108,
}

/// Initialize the LLE link layer engine.
///
/// Sets timing parameters from hardware-confirmed defaults, clears all
/// pending IRQ status bits, and enables the standard IRQ mask (0xF00F).
///
/// Hardware-confirmed values from live CH32V208WBU6 board (STATE_MACHINE=108=SLEEP,
/// TIMING0-7 matching values below) via wlink dump at +0x40024100.
pub unsafe fn lle_dev_init() {
    // Timing parameters — all hardware-confirmed from live board dump.
    // WCH LLE_DevInit writes timing registers first, then +0x74/+0x08/+0x0C.
    lle_write(0x14, 140); // TIMING0
    lle_write(0x1C, LleState::Sleep as u32); // STATE_MACHINE = SLEEP (108)
    lle_write(0x24, 140); // TIMING2
    lle_write(0x2C, 60);  // TIMING3
    lle_write(0x34, 140); // TIMING4
    lle_write(0x3C, 60);  // TIMING5
    lle_write(0x44, 140); // TIMING6
    lle_write(0x4C, 108); // TIMING7

    // ★ LLE+0x74 = DMA buffer base address (MEMAddr, gBleIPPara[36] in WCH).
    // WCH's LLE_DevInit writes this from pInitConfig->MEMAddr.
    // Without a valid non-zero address the LLE state machine may refuse to
    // fire any trigger, blocking RFEND PLL calibration (Lucy analysis 2026-05-01).
    let buf_addr = core::ptr::addr_of!(LLE_DMA_BUF) as u32;
    lle_write(0x74, buf_addr);

    // Clear all pending IRQ status (W1C) then set IRQ mask.
    // WCH writes these AFTER timing/+0x74 writes.
    lle_write(0x08, 0xFFFF);  // clear all pending IRQ status
    lle_write(0x0C, 0xF00F);  // IRQ mask: bits [15:12] and [3:0]
}

/// Read the current LLE state machine value.
pub unsafe fn lle_read_state() -> u8 {
    (lle_read(0x1C) & 0xFF) as u8
}

/// Read the current IRQ status (unmasked raw status bits).
///
/// Note: +0x08 is a split register. CPU reads return IRQ_STATUS (live event bits);
/// writes are W1C. Do not interpret read values as ACCESS_ADDR.
/// See wchble disasm notes: real-hardware-verification chapter.
pub unsafe fn lle_read_irq_status() -> u32 {
    lle_read(0x08)
}

/// Clear specific LLE IRQ status bits (W1C).
pub unsafe fn lle_clear_irq(bits: u32) {
    lle_write(0x08, bits);
}
