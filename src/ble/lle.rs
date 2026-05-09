// LLE (Link Layer Engine) initialization for CH32V208 BLE.
//
// Register block: `BLE_LLE` (gptrLLEReg) at 0x40024200 — timing slots,
// state machine, IRQ status / mask, link-layer timer, TX/DMA buffer pointers.
//
// 2026-05-09: migrated from raw `read_volatile/write_volatile` to typed
// `BLE_LLE.{timing0,state_machine,dma_buf,access_addr,irq_mask,...}()
// .{read,write_value,modify}()` against the new ch32-metapac BLE_LLE block.
// Bit patterns + write order preserved exactly; semantics unchanged.
// Iron Law #38 hardware-gated.

use crate::pac::BLE_LLE;

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

/// LLE state machine values (observed from live hardware).
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
/// TIMING0-7 matching values below).
pub unsafe fn lle_dev_init() {
    // Timing parameters — all hardware-confirmed from live board dump.
    // WCH LLE_DevInit writes timing registers first, then DMA_BUF / ACCESS_ADDR / IRQ_MASK.
    BLE_LLE.timing0().write_value(140);
    BLE_LLE.state_machine().write(|w| w.set_state(LleState::Sleep as u8));
    BLE_LLE.timing2().write_value(140);
    BLE_LLE.timing3().write_value(60);
    BLE_LLE.timing4().write_value(140);
    BLE_LLE.timing5().write_value(60);
    BLE_LLE.timing6().write_value(140);
    BLE_LLE.timing7().write_value(108);

    // ★ LLE+0x74 = DMA buffer base address (MEMAddr, gBleIPPara[36] in WCH).
    // WCH's LLE_DevInit writes this from pInitConfig->MEMAddr.
    // Without a valid non-zero address the LLE state machine may refuse to
    // fire any trigger, blocking RFEND PLL calibration.
    let buf_addr = core::ptr::addr_of!(LLE_DMA_BUF) as u32;
    BLE_LLE.dma_buf().write_value(buf_addr);

    // Clear all pending IRQ status (W1C) then set IRQ mask.
    // WCH writes these AFTER timing/DMA_BUF writes.
    BLE_LLE.access_addr().write_value(0xFFFF); // clear all pending IRQ status
    BLE_LLE.irq_mask().write_value(0xF00F);    // IRQ mask: bits [15:12] and [3:0]
}

/// Read the current LLE state machine value.
pub unsafe fn lle_read_state() -> u8 {
    BLE_LLE.state_machine().read().state()
}

/// Read the current IRQ status (unmasked raw status bits).
///
/// Note: ACCESS_ADDR (+0x08) is a split register. CPU reads return IRQ_STATUS
/// (live event bits); writes are W1C. Do not interpret read values as the
/// access-address constant.
pub unsafe fn lle_read_irq_status() -> u32 {
    BLE_LLE.access_addr().read()
}

/// Clear specific LLE IRQ status bits (W1C).
pub unsafe fn lle_clear_irq(bits: u32) {
    BLE_LLE.access_addr().write_value(bits);
}
