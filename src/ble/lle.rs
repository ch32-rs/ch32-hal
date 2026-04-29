// LLE (Link Layer Engine) initialization for CH32V208 BLE.
//
// Register base: 0x40024100
// Source: elec-docs/ble-reverse-docs/05-lle-engine.md and hardware dump
// Hardware-confirmed timing values from live CH32V208WBU6 board dump.

use core::ptr::{read_volatile, write_volatile};

const LLE_BASE: usize = 0x40024100;

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
    // Clear all pending IRQ status (W1C) before unmasking.
    // Offset +0x08 is the IRQ_STATUS register (write-only path; read returns IRQ_STATUS,
    // write is W1C). Writing 0xFFFF clears all 16 defined interrupt bits.
    lle_write(0x08, 0xFFFF);

    // Set IRQ mask: 0xF00F enables bits [15:12] and [3:0].
    // Confirmed from libwchble assembly: LLE_IRQSubHandler checks bits 14,3,2,1,0.
    lle_write(0x0C, 0xF00F);

    // Timing parameters — all hardware-confirmed from live board dump.
    lle_write(0x14, 140); // TIMING0
    lle_write(0x24, 140); // TIMING2
    lle_write(0x2C, 60);  // TIMING3
    lle_write(0x34, 140); // TIMING4
    lle_write(0x3C, 60);  // TIMING5
    lle_write(0x44, 140); // TIMING6
    lle_write(0x4C, 108); // TIMING7

    // Initial state: SLEEP.
    lle_write(0x1C, LleState::Sleep as u32);
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
