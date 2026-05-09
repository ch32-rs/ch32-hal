// BLE advertising for CH32V208 — legacy advertising PDU TX helpers.
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

/// PDU type: ADV_IND (connectable undirected, type=0b0000).
pub const PDU_TYPE_ADV_IND: u8 = 0b0000;

/// PDU type: ADV_NONCONN_IND (non-connectable undirected, type=0b0010).
pub const PDU_TYPE_ADV_NONCONN_IND: u8 = 0b0010;

/// PDU type: SCAN_RSP (scan response, type=0b0100).
pub const PDU_TYPE_SCAN_RSP: u8 = 0b0100;

/// Advertising channels: (index, freq_kHz). BLE spec Vol 6 §4.4.2.
const ADV_CHANNELS: [(u8, u32); 3] = [(37, 2_402_000), (38, 2_426_000), (39, 2_480_000)];

// ── TX buffer ─────────────────────────────────────────────────────────────────────────────────

/// Maximum AdvData length (BLE spec Vol 6 Part B §2.3.1.4).
pub const ADV_DATA_MAX: usize = 31;

/// TX buffer: header (2B) + AdvA (6B) + AdvData (up to 31B).
/// Must be 16B-aligned: BLE DMA (BB+0x70) requires 16B alignment (Iron Law T3/T4.5 #36).
/// mod16≠0 → DMA reads wrong PDU bytes → cba=0. `.tx_buf_aligned` section enforces ALIGN(16).
#[link_section = ".tx_buf_aligned"]
static mut ADV_TX_BUF: [u8; 2 + 6 + ADV_DATA_MAX] = [0u8; 2 + 6 + ADV_DATA_MAX];

// ── PDU builder ──────────────────────────────────────────────────────────────────────────────

/// Build a legacy advertising-channel PDU into `ADV_TX_BUF`.
///
/// `addr`: 6-byte device address (LE order, as stored in hardware address registers).
/// `addr_is_random`: true = random address (TxAdd=1), false = public (TxAdd=0).
/// `adv_data`: AD structure bytes (max 31B). Must be pre-formatted AD structures.
/// `pdu_type`: one of [`PDU_TYPE_ADV_IND`], [`PDU_TYPE_ADV_NONCONN_IND`], or
/// [`PDU_TYPE_SCAN_RSP`].
///
/// Returns the total PDU length in bytes (header + AdvA + adv_data).
pub unsafe fn build_legacy_adv_pdu(pdu_type: u8, addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> usize {
    let adv_len = adv_data.len().min(ADV_DATA_MAX);
    let payload_len = 6 + adv_len; // AdvA(6) + AdvData

    // Header byte 0: PDU type | RFU(2b) | ChSel(1b) | TxAdd | RxAdd
    // Legacy advertising PDUs: ChSel=0, RxAdd=0, TxAdd from addr_is_random.
    let txadd: u8 = if addr_is_random { 1 << 6 } else { 0 };
    ADV_TX_BUF[0] = (pdu_type & 0x0F) | txadd;
    // Header byte 1: Length field (6-bit, value = payload_len).
    ADV_TX_BUF[1] = payload_len as u8 & 0x3F;

    // AdvA: device address, 6 bytes LE.
    ADV_TX_BUF[2..8].copy_from_slice(addr);

    // AdvData: AD structures.
    ADV_TX_BUF[8..8 + adv_len].copy_from_slice(&adv_data[..adv_len]);

    2 + payload_len // total bytes used in TX buf
}

/// Build an ADV_NONCONN_IND PDU into `ADV_TX_BUF`.
pub unsafe fn build_adv_pdu(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> usize {
    build_legacy_adv_pdu(PDU_TYPE_ADV_NONCONN_IND, addr, addr_is_random, adv_data)
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

/// Write Complete List of 16-bit Service UUIDs AD structure (type 0x03) into `buf`.
/// Returns number of bytes written.
pub fn ad_uuid16_complete(buf: &mut [u8], uuid: u16) -> usize {
    if buf.len() < 4 {
        return 0;
    }
    let [lo, hi] = uuid.to_le_bytes();
    buf[0] = 3; // length: type + UUID16
    buf[1] = 0x03; // type: Complete List of 16-bit Service UUIDs
    buf[2] = lo;
    buf[3] = hi;
    4
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
    // 1. TX arm: LLE+0x2C bit0 = 1 (TX mode arm).
    //    LLE+0x2C bits[30:25] = BB_RF_FLAG_1M (0x09) — PHY mode configuration written by
    //    bb_dev_init. These bits are rf_flag (PHY mode = 1Mbps), NOT the BLE channel for
    //    whitening. Writing ch_idx (37/38/39) here CORRUPTS the PHY mode bits → TX fails.
    //    Frozen binary hardcodes `9u32 << 25` (= BB_RF_FLAG_1M, confirmed from adv_tx_burst_ch37
    //    "ble[0x14] state (EVT=9)" which equals rf_flag). (T44.E root-cause fix, fix #5).
    //    Whitening channel is set separately in step 8 via LLE+0x00 bits[5:0].
    const BB_RF_FLAG_1M_SHIFT: u32 = 9u32 << 25; // rf_flag=0x09 in bits[30:25]
    let cfg = lle_read(0x2C);
    let cfg = (cfg & 0x81FF_FFFF) | BB_RF_FLAG_1M_SHIFT | 0x1;
    lle_write(0x2C, cfg);

    // 2. Event timeout counter: BB+0x64 = 160.
    bb_write(0x64, 160);

    // 3. PLL channel — NOT reprogrammed per-burst.
    // EVT does NOT rewrite RFEND_CAL+0x44 per-burst. Per-burst reprogram sets
    // RFEND+0x44 to wrong calibration anchor and forces a PLL re-lock cycle
    // that cannot settle before TX fires → RF failure (cba=0).
    // Channel tracking is done via LLE+0x00 bits[5:0] at step 8.
    let _ = freq_khz;

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
    // bits[5:0] = LOGICAL BLE channel number (37/38/39 for ADV).
    //   confirmed by Lucy's RE of ll_advertise_process (asm line 40091):
    //     li a1, 37   ; hardcoded logical ch37, not phys_idx=0
    //   and LL_ReceiverTest / LL_TransmitterTest: map phys_idx → logical, then write logical.
    // bit6=0: whitening flag cleared. Frozen binary hardcodes `37u32` (bit6=0) and passes
    // cba=61. Setting bit6=1 was adv.rs baseline; frozen uses bit6=0. Direction B delta.
    // (T44.E fix#10, P6: single-variable test, 74e8d67 baseline + this one change.)
    let ctrl = lle_read(0x00);
    lle_write(0x00, (ctrl & !0x7F) | (ch_idx as u32 & 0x3F));

    // 9. Access address: LLE+0x08 = ADV AA.
    lle_write(0x08, ADV_ACCESS_ADDR);

    // 10. CRC init: LLE+0x04 = 0x555555.
    lle_write(0x04, ADV_CRC_INIT);

    // 11. TX buffer pointer: BB+0x70.
    bb_write(0x70, core::ptr::addr_of!(ADV_TX_BUF) as u32);

    // Pre-trigger snapshot.
    let state_pre = bb_read(0x1C); // LLE state; should be 108=Sleep

    // 12. ADV-mode flag: BB+0x04 bit0 = 1.
    // Confirmed from lib `ll_advertise_tx` L39233 (Lucy 2026-05-02).
    // Lib DTM (`ll_hw_api_tx_direct_test`) clears this bit. Mode selector for HW TX path.
    bb_write(0x04, bb_read(0x04) | 0x1);

    // 12.5. PHY rate select + lock: BLE_SetPHYTxMode step1 + step4 (d.asm L91784, T44.E fix #3).
    //
    // Step 1: LLE+0x00 bits[13:12] = 00 → select 1 Mbps PHY rate.
    //   Without this, hardware may warmup in wrong PHY mode → TX sends at wrong rate → nRF
    //   cannot decode. ble_ip_core_init may leave bits[13:12] non-zero (2M or Coded PHY default).
    //
    // Step 4: BB+0x08 = 0x2000 → PHY rate lock (W1C-clears bit13 timer IRQ).
    //   Critical step (Lucy d.asm L91836 comment): "without it nRF will not decode the packet".
    //   ISR .L6 repeats this write post-GO, but the pre-GO rate select ensures hardware warmup
    //   starts in the correct mode. ADV_NONCONN_IND burst cba=0 without this despite fix #1+#2.
    let ctrl = lle_read(0x00);
    lle_write(0x00, ctrl & !0x3000); // bits[13:12]=0 → 1Mbps PHY rate
    bb_write(0x08, 0x2000); // PHY rate lock (BLE_SetPHYTxMode step4)

    // 13. ADV GO: LLE+0x00 |= 0x800000 (bit 23 — NOT bit 11).
    // Confirmed from lib `ll_advertise_tx` L39276-39280 (`lui a3,0x800` → a3=0x00800000).
    // Lib DTM does NOT write bit 23; both bit 11 and bit 23 were wrong in old code.
    lle_write(0x00, lle_read(0x00) | 0x0080_0000);

    // 14. Clear TX arm: LLE+0x2C bits[1:0] = 00 (commit all config before trigger).
    let cfg = lle_read(0x2C);
    lle_write(0x2C, cfg & !0x3);

    // 15. TX trigger — FINAL write: BB+0x00 = 2 (lib L39288-39290).
    // Must be last; old code had this mid-sequence before GO — incorrect for ADV mode.
    bb_write(0x00, 2);

    // Post-trigger snapshot.
    let state_post = bb_read(0x1C);
    let irq_post = bb_read(0x08);

    (state_pre, state_post, irq_post)
}

/// Poll for ADV TX completion.
///
/// Delegates to `ble_tx_wait_done` — the shared BLE TX completion primitive.
/// See `crate::ble::ble_tx_wait_done` for the full signal description.
///
/// settle_loops ≈ 350µs at 144 MHz (50_000 spin_loop iterations):
///   ADV_NONCONN_IND @ 1 Mbps ≈ 240µs + BB+0x50 pre-delay 90µs = 330µs.
///
/// Returns (completed: bool, bb64_initial: u32, bb64_final: u32, bb08_final: u32).
unsafe fn wait_adv_tx_done() -> (bool, u32, u32, u32) {
    let initial = bb_read(0x64);
    let done = super::ble_tx_wait_done(10_000, 50_000);
    (done, initial, bb_read(0x64), bb_read(0x08))
}

// ── Diagnostic helpers ────────────────────────────────────────────────────────────────────────

/// Read key registers for diagnostic logging.
/// Returns (lle_2c, bb04_armed, lle_ctrl).
/// Call immediately after `adv_tx_burst` / `adv_event` for a snapshot.
pub unsafe fn diag_read() -> (u32, u32, u32) {
    let lle_2c = lle_read(0x2C); // channel field bits[30:25], TX arm bits[1:0]
    let bb04 = bb_read(0x04); // TX armed flag bit0
    let ctrl = lle_read(0x00); // channel bits[6:0] + bit6=whitening + bit23=BLE enable
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
pub unsafe fn adv_event_verbose(
    addr: &[u8; 6],
    addr_is_random: bool,
    adv_data: &[u8],
) -> (u8, [(u32, u32, u32, u32, u32, u32); 3]) {
    build_adv_pdu(addr, addr_is_random, adv_data);
    legacy_adv_event_from_current_buf()
}

/// Send one complete connectable advertising event: transmit ADV_IND on all 3
/// advertising channels.
///
/// This is Phase 1 plumbing for Rust-only PeripheralRole.  It only transmits
/// connectable advertising PDUs; the CONNECT_IND RX turnaround is implemented
/// as a separate validation step through the listener capture path.
pub unsafe fn connectable_adv_event_verbose(
    addr: &[u8; 6],
    addr_is_random: bool,
    adv_data: &[u8],
) -> (u8, [(u32, u32, u32, u32, u32, u32); 3]) {
    build_legacy_adv_pdu(PDU_TYPE_ADV_IND, addr, addr_is_random, adv_data);
    legacy_adv_event_from_current_buf()
}

/// Send one complete scan-response event: transmit SCAN_RSP on all 3 advertising channels.
///
/// This is a Phase 1 compatibility helper for active scanners.  The strict
/// SCAN_REQ→SCAN_RSP turnaround is handled by the future connection/scan
/// state machine; this helper validates the PDU builder and TX path.
pub unsafe fn scan_response_event_verbose(
    addr: &[u8; 6],
    addr_is_random: bool,
    scan_rsp_data: &[u8],
) -> (u8, [(u32, u32, u32, u32, u32, u32); 3]) {
    build_legacy_adv_pdu(PDU_TYPE_SCAN_RSP, addr, addr_is_random, scan_rsp_data);
    legacy_adv_event_from_current_buf()
}

/// Send one connectable advertising event and return the number of channels
/// that completed TX.
pub unsafe fn connectable_adv_event(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> u8 {
    let (ok, _) = connectable_adv_event_verbose(addr, addr_is_random, adv_data);
    ok
}

/// Send one scan-response event and return the number of channels that
/// completed TX.
pub unsafe fn scan_response_event(addr: &[u8; 6], addr_is_random: bool, scan_rsp_data: &[u8]) -> u8 {
    let (ok, _) = scan_response_event_verbose(addr, addr_is_random, scan_rsp_data);
    ok
}

unsafe fn legacy_adv_event_from_current_buf() -> (u8, [(u32, u32, u32, u32, u32, u32); 3]) {
    let mut ok = 0u8;
    let mut stats = [(0u32, 0u32, 0u32, 0u32, 0u32, 0u32); 3];
    for (i, (ch_idx, freq_khz)) in ADV_CHANNELS.iter().enumerate() {
        let (state_pre, state_post, irq_post) = adv_tx_burst(*ch_idx, *freq_khz);
        let (done, init, fin, irq_final) = wait_adv_tx_done();
        stats[i] = (init, fin, irq_final, state_pre, state_post, irq_post);
        if done {
            ok += 1;
        }
    }
    (ok, stats)
}

pub unsafe fn adv_event(addr: &[u8; 6], addr_is_random: bool, adv_data: &[u8]) -> u8 {
    let (ok, _) = adv_event_verbose(addr, addr_is_random, adv_data);
    ok
}
