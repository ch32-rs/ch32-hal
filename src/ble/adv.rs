// BLE advertising for CH32V208 — ADV_NONCONN_IND (non-connectable undirected).
//
// Spec ref: BLE Core Spec Vol 6 Part B §2.3 (PDU), Vol 6 Part B §4.4.2 (advertising channels).
//
// PDU layout (over the air, LSB first):
//   Preamble (1B, hw) | AA 0x8E89BED6 (4B) | Header (2B) | AdvA (6B) | AdvData (0-31B) | CRC-24 (3B, hw)
//
// BB hardware handles: preamble, CRC-24 (init 0x555555), and whitening.
// SW provides: header, AdvA, AdvData in TX buffer.
//
// Whitening: enabled by setting bit6 of LLE+0x00 alongside the channel index.
// BB hardware seeds its whitening LFSR from LLE+0x00 bits[6:0] = {1, ch[5:0]},
// which matches BLE spec Vol 6 Part B §3.2 (LFSR position 6 = 1, positions[5:0] = channel).
// Source: RF_DevSetChannel in d.asm line 71418 (Lucy's Phase 2 cheat sheet 2026-04-30).
// DTM leaves bit6=0 (no whitening), per DTM spec Vol 6 Part F.

use core::ptr::{read_volatile, write_volatile};

// ── Register bases (same as dtm example, confirmed from BLE_IPCoreInit in dtm.elf) ──────────

/// gptrBBReg: link-layer CTRL/GO, TX mode, ACCESS_ADDR, CRC_INIT.
const LLE_BASE: usize = 0x40024100;
/// gptrLLEReg: timing params, IRQ status/mask, TX timer, TX buffer pointer.
const BB_BASE: usize = 0x40024200;
/// gptrRFENDReg: PLL dividers, channel lock, PA bias. Distinct from rfend.rs (0x40024300).
const RFEND_CAL_BASE: usize = 0x40025000;

#[inline(always)]
unsafe fn lle_read(off: usize) -> u32 {
    read_volatile((LLE_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn lle_write(off: usize, val: u32) {
    write_volatile((LLE_BASE + off) as *mut u32, val);
}
#[inline(always)]
unsafe fn bb_read(off: usize) -> u32 {
    read_volatile((BB_BASE + off) as *const u32)
}
#[inline(always)]
unsafe fn bb_write(off: usize, val: u32) {
    write_volatile((BB_BASE + off) as *mut u32, val);
}

// ── BLE advertising constants ─────────────────────────────────────────────────────────────────

/// Advertising access address (fixed by BLE spec for all advertising channels).
const ADV_ACCESS_ADDR: u32 = 0x8E89_BED6;

/// CRC initial value for advertising (BLE spec Vol 6 Part B §3.1.1).
const ADV_CRC_INIT: u32 = 0x55_5555;

/// PDU type: ADV_NONCONN_IND (non-connectable undirected, type=0b0010).
const PDU_TYPE_ADV_NONCONN_IND: u8 = 0b0010;

/// Advertising channels: (index, freq_kHz). BLE spec Vol 6 §4.4.2.
const ADV_CHANNELS: [(u8, u32); 3] = [(37, 2_402_000), (38, 2_426_000), (39, 2_480_000)];

// ── TX buffer ─────────────────────────────────────────────────────────────────────────────────

/// Maximum AdvData length (BLE spec Vol 6 Part B §2.3.1.4).
pub const ADV_DATA_MAX: usize = 31;

/// TX buffer: header (2B) + AdvA (6B) + AdvData (up to 31B).
static mut ADV_TX_BUF: [u8; 2 + 6 + ADV_DATA_MAX] = [0u8; 2 + 6 + ADV_DATA_MAX];

// ── PDU builder ──────────────────────────────────────────────────────────────────────────────

/// Build an ADV_NONCONN_IND PDU into `ADV_TX_BUF`.
///
/// `addr`: 6-byte device address (LE order, as stored in hardware address registers).
/// `addr_is_random`: true = random address (TxAdd=1), false = public (TxAdd=0).
/// `adv_data`: AD structure bytes (max 31B). Must be pre-formatted AD structures.
///
/// Returns the total PDU length in bytes (header + AdvA + adv_data).
pub unsafe fn build_adv_pdu(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> usize {
    let adv_len = adv_data.len().min(ADV_DATA_MAX);
    let payload_len = 6 + adv_len; // AdvA(6) + AdvData

    // Header byte 0: PDU type | RFU(2b) | ChSel(1b) | TxAdd | RxAdd
    // ADV_NONCONN_IND: ChSel=0, RxAdd=0, TxAdd from addr_is_random.
    let txadd: u8 = if addr_is_random { 1 << 6 } else { 0 };
    ADV_TX_BUF[0] = PDU_TYPE_ADV_NONCONN_IND | txadd;
    // Header byte 1: Length field (6-bit, value = payload_len).
    ADV_TX_BUF[1] = payload_len as u8 & 0x3F;

    // AdvA: device address, 6 bytes LE.
    ADV_TX_BUF[2..8].copy_from_slice(addr);

    // AdvData: AD structures.
    ADV_TX_BUF[8..8 + adv_len].copy_from_slice(&adv_data[..adv_len]);

    2 + payload_len // total bytes used in TX buf
}

// ── AD structure helpers ─────────────────────────────────────────────────────────────────────

/// Write Flags AD structure (type 0x01) into `buf`.
/// Returns number of bytes written.
/// Standard value: LE General Discoverable | BR/EDR Not Supported = 0x06.
pub fn ad_flags(buf: &mut [u8], flags: u8) -> usize {
    if buf.len() < 3 {
        return 0;
    }
    buf[0] = 2; // length
    buf[1] = 0x01; // type: Flags
    buf[2] = flags;
    3
}

/// Write Complete Local Name AD structure (type 0x09) into `buf`.
/// Returns number of bytes written, truncated to fit.
pub fn ad_complete_name<'a>(buf: &'a mut [u8], name: &[u8]) -> usize {
    let max_name = buf.len().saturating_sub(2);
    let name_len = name.len().min(max_name);
    if name_len == 0 {
        return 0;
    }
    buf[0] = (name_len + 1) as u8; // length
    buf[1] = 0x09; // type: Complete Local Name
    buf[2..2 + name_len].copy_from_slice(&name[..name_len]);
    2 + name_len
}

// ── TX trigger ───────────────────────────────────────────────────────────────────────────────

/// Trigger one ADV_NONCONN_IND burst on the given advertising channel.
///
/// Returns (state_pre_go, state_post_go, irq_post_go) — snapshot from immediately
/// around the GO strobe for state-machine diagnostics.
///
/// state_pre/post_go: BB+0x1C (gptrLLEReg+0x1C) LLE state machine value.
///   108 = Sleep (default), non-108 = hardware accepted GO and is processing.
/// irq_post_go: BB+0x08 bits[15:0] immediately after GO; non-zero = TX IRQ fired
///   (bits 29+25 = 0x22000000 are always-set hardware constants, masked out here).
unsafe fn adv_tx_burst(ch_idx: u8, freq_khz: u32) -> (u32, u32, u32) {
    // 1. TX arm: LLE+0x2C bits[1:0] = 01.
    let cfg = lle_read(0x2C);
    lle_write(0x2C, (cfg & !0x3) | 0x1);

    // 2. Event timeout counter: BB+0x64 = 160.
    bb_write(0x64, 160);

    // 3. PLL program for ADV channel (DTM-style: RFEND_CAL+0x44).
    // Lucy confirmed ll_advertise_tx doesn't call this, but the calling context might expect
    // PLL to be pre-programmed. We use the ADV non-DTM formula: freq_offset = freq-1000.
    {
        let freq_offset = freq_khz - 1_000;
        let int_div = (freq_offset / 64_000) & 0x1F;
        let frac_div = ((freq_offset % 64_000) << 10) / 250;
        let pll = read_volatile((RFEND_CAL_BASE + 0x44) as *const u32);
        let pll = (pll & 0xFE0C_0000) | (int_div << 20) | (frac_div & 0x3_FFFF);
        write_volatile((RFEND_CAL_BASE + 0x44) as *mut u32, pll);
        let v = read_volatile((RFEND_CAL_BASE + 0x2C) as *const u32);
        write_volatile((RFEND_CAL_BASE + 0x2C) as *mut u32, v | (1 << 1)); // set lock
    }

    // 4. TX path select: LLE+0x00 clear bits[8:7] then set bit8.
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl & !0x180);
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x180) | 0x100);

    // 5. PA bias: RFEND_CAL+0x08 |= 0x330000.
    let ana = read_volatile((RFEND_CAL_BASE + 0x08) as *const u32);
    write_volatile((RFEND_CAL_BASE + 0x08) as *mut u32, ana | 0x0033_0000);

    // 6. TX pre-delay: BB+0x50 = 90.
    bb_write(0x50, 90);

    // 7. Release channel lock: RFEND_CAL+0x2C bit1 = 0.
    let v = read_volatile((RFEND_CAL_BASE + 0x2C) as *const u32);
    write_volatile((RFEND_CAL_BASE + 0x2C) as *mut u32, v & !(1 << 1));

    // 8. Channel index + whitening enable: LLE+0x00 bits[6:0].
    // bit6=1 enables BB hardware whitener (BLE spec Vol 6 §3.2, seed={1, ch[5:0]}).
    // HW uses a LUT from this field for PLL freq select (confirmed Phase 1 SDR: HW output
    // matched LLE+0x00 channel despite 14-bit PLL formula producing garbage RFEND values).
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | ((ch_idx as u32 & 0x3F) | 0x40));

    // 9. Access address: LLE+0x08 = ADV AA.
    lle_write(0x08, ADV_ACCESS_ADDR);

    // 10. CRC init: LLE+0x04 = 0x555555.
    lle_write(0x04, ADV_CRC_INIT);

    // 11. TX buffer pointer: BB+0x70.
    bb_write(0x70, core::ptr::addr_of!(ADV_TX_BUF) as u32);

    // 12. LLE arm: BB+0x00 = 2 — MUST come before GO bit.
    // DTM comment: "Without this write the LLE ignores the GO pulse."
    bb_write(0x00, 2);

    // Pre-GO: clear all BB+0x08 IRQ bits (full 32-bit W1C, same as DTM code).
    // Bits 29+25 (0x22000000) are set by GO when TX fires; clearing here lets irq_post
    // detect TX fire on a per-channel basis (if they are W1C-able at 32-bit width).
    bb_write(0x08, 0xFFFF_FFFF);
    let state_pre = bb_read(0x1C); // LLE state before GO; should be 108=Sleep

    // 13. GO bit: LLE+0x00 |= 0x800. Clear first to guarantee 0→1 edge.
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl & !0x800);
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl | 0x800);

    // Snapshot immediately after GO — before step 14 clears the arm.
    // If state_post != state_pre: HW state machine accepted the GO command.
    // If irq_post & 0xFFFF != 0: TX completion IRQ already fired.
    let state_post = bb_read(0x1C);
    let irq_post = bb_read(0x08);

    // 14. Clear TX arm: LLE+0x2C bits[1:0] = 00.
    let cfg = lle_read(0x2C);
    lle_write(0x2C, cfg & !0x3);

    (state_pre, state_post, irq_post)
}

/// Poll for TX completion — DTM-style.
///
/// BB+0x64 is event-driven and does not decrement in our TX mode (stays at 160).
/// BB+0x08 bits 29+25 (0x22000000) are SET by the GO strobe when TX fires, matching
/// the DTM TX behavior (confirmed: DTM INIT irq=0x00000000, after first GO irq=0x22000000).
/// These bits are not clearable via W1C, so they remain set permanently.
///
/// After detecting TX fire via BB+0x08 != 0, we spin for ~350µs to allow the over-the-air
/// packet to fully transmit before moving to the next advertising channel.
///
/// Returns (completed: bool, bb64_initial: u32, bb64_final: u32, bb08_final: u32).
unsafe fn wait_adv_tx_done() -> (bool, u32, u32, u32) {
    let initial = bb_read(0x64);
    // Wait for TX fire indicator: BB+0x08 != 0 (bits 29+25 set by GO strobe).
    for _ in 0..10_000u32 {
        if bb_read(0x08) != 0 {
            // TX fired. Spin ~350µs at 144 MHz for over-the-air packet completion.
            // ADV_NONCONN_IND packet @ 1 Mbps ≈ 240µs + pre-delay 90µs = 330µs.
            for _ in 0..50_000u32 {
                core::hint::spin_loop();
            }
            return (true, initial, bb_read(0x64), bb_read(0x08));
        }
        core::hint::spin_loop();
    }
    (false, initial, bb_read(0x64), bb_read(0x08))
}

// ── Diagnostic helpers ────────────────────────────────────────────────────────────────────────

/// Read key registers for diagnostic logging.
/// Returns (lle_2c, bb04_armed, lle_ctrl).
/// Call immediately after `adv_tx_burst` / `adv_event` for a snapshot.
pub unsafe fn diag_read() -> (u32, u32, u32) {
    let lle_2c = lle_read(0x2C);      // channel field bits[30:25], TX arm bits[1:0]
    let bb04 = bb_read(0x04);         // TX armed flag bit0
    let ctrl = lle_read(0x00);        // channel bits[6:0] + whitening bit6 + GO bit11
    (lle_2c, bb04, ctrl)
}

// ── Public API ───────────────────────────────────────────────────────────────────────────────

/// Send one complete advertising event: transmit ADV_NONCONN_IND on all 3 advertising channels.
///
/// Must call `ble_phy_init()` before this.
/// `addr`, `addr_is_random`, `adv_data`: forwarded to `build_adv_pdu`.
///
/// Returns:
///   ok_count: number of channels where TX completed (0-3).
///   stats: per-channel tuple (bb64_init, bb64_final, irq_final, state_pre, state_post, irq_post_go).
///     state_pre/post: LLE state machine (108=Sleep); non-108 post means HW accepted GO.
///     irq_post_go: BB+0x08 bits[15:0] immediately after GO; non-zero = IRQ fired fast.
pub unsafe fn adv_event_verbose(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8])
    -> (u8, [(u32, u32, u32, u32, u32, u32); 3])
{
    build_adv_pdu(addr, addr_is_random, adv_data);
    let mut ok = 0u8;
    let mut stats = [(0u32, 0u32, 0u32, 0u32, 0u32, 0u32); 3];
    for (i, (ch_idx, freq_khz)) in ADV_CHANNELS.iter().enumerate() {
        let (state_pre, state_post, irq_post) = adv_tx_burst(*ch_idx, *freq_khz);
        let (done, init, fin, irq_final) = wait_adv_tx_done();
        stats[i] = (init, fin, irq_final, state_pre, state_post, irq_post);
        if done { ok += 1; }
    }
    (ok, stats)
}

pub unsafe fn adv_event(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> u8 {
    let (ok, _) = adv_event_verbose(addr, addr_is_random, adv_data);
    ok
}
