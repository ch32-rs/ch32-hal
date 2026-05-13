# Task #79 attempt log — SCAN_RSP Step 2 baseline control → RX-only → warm-kick smoke

Status: control gate in progress (2026-05-14)
Branch: `feat/ble-scan-rsp-step2` on main repo `/Users/mono/Elec/WCH/ch32-hal`
HEAD: `7f04104` + gate patch (see §3)

---

## §1. Baseline selection history

### Candidate: 7f04104 + ch32-data 1b6b0d4
Selected in spec (scan-rsp-implementation-spec.md §2). Air-validated control claim was based on a
1-frame Segment B capture — insufficient.

**P1 (2026-05-11)**: `7f04104` produces 0 ADV / 65.5s after power cycle. Root cause: working
SysTick timeout (2ms) causes `rx_turnaround_capture_ch37` to run mid-TX → `bb_write(0x08, 0x2000)`
(BLE_LLE.access_addr) clobbers in-progress TX → device silent.

**Sniffer confirmation (2026-05-14)**: Three captures, 20309 total BLE ADV-AA records, target
frames = 0. Ambient BLE captured normally → sniffer is working; device genuinely silent.
Tags: `inconclusive/scan-rsp-step2-control-air-blocked-20260514`,
      `inconclusive/scan-rsp-step2-control-no-air-sniffer-restored-20260514`

### Why rx_turnaround fires: df26837 vs 7f04104

| Commit | mcycle behavior | probe-wait exit | rx_turnaround timing | Air result |
|--------|----------------|-----------------|----------------------|------------|
| df26837 | always 0 → `0 < 192_000` always true → waits until ip4==1 | ip4==1 (TX done) | post-TX | 17.1 Hz ✓ |
| 7f04104 | SysTick CNTL, real 2ms timeout | timeout (ip4≠1) | mid-TX | 0 Hz ✗ |

df26837..7f04104 = **1 commit only** (the SysTick fix). No bisect needed; regression commit is
7f04104 itself.

---

## §2. Options considered

| # | Description | Decision |
|---|-------------|----------|
| v4a | probe-wait gate `ip4==0x80 && dt<240_000`, keep rx_turnaround | FAIL — W1C 0xF00F transient bit13 clear kills TX |
| v4b | same + delete rx_turnaround entirely | FAIL — missing LLE[8]+LLE[100]+gBleIPPara[4] re-arm, 0 ADV |
| α bisect df26837..7f04104 | find regression commit | MOOT — only 1 commit in range |
| C1 fork df26837 | use broken-mcycle baseline | REJECTED — loses SysTick fine timer for T_IFS gate |
| **Option 2 (current)** | gate rx_turnaround on `irq==1` in 7f04104 | **IN PROGRESS** |
| A1 surgical | replace rx_turnaround with libwchble ll_tx_wait_finish mode=1 full sequence | task #80, post-task-79-PASS |
| Path A | HW autonomous pivot | deferred |

---

## §3. Current patch (option 2)

File: `examples/ch32v208/src/bin/ble_peripheral_phase1_adv.rs`
Change: gate `rx_turnaround_capture_ch37` call on `irq == 1` at call site (~line 2074).

`irq` = ip4 as u32 (3rd return value from `adv_tx_burst_ch37`).
`irq == 1` means `gBleIPPara[4]` became 1 within 2ms (TX done path).
`irq != 1` means 2ms SysTick timeout (rx_turnaround skipped).

```rust
// Before (unconditional, caused mid-TX clobber):
let rx_snap = rx_turnaround_capture_ch37(adv_channel);

// After (gated):
let rx_snap = if irq == 1 {
    rx_turnaround_capture_ch37(adv_channel)
} else {
    None
};
```

Note: SDI print removed from skip path — SDI µs overhead in timing-sensitive path violates
Andelf directive (msg 8ab182ec). Use pcap first; add a ring/counter only after the air result needs
local diagnosis.

### Attempt result (2026-05-14 02:07)

- Build: PASS, `/tmp/task79_gate_rx_turnaround_noskipprint_build_20260514_0207.log`
- Flash: PASS, `/tmp/task79_gate_rx_turnaround_noskipprint_flash_20260514_0207.log`
- Pcap: `/tmp/task79_scan_rsp_step2/gate_rx_turnaround_noskipprint_sniff60.pcapng`
- Pcap raw evidence: ambient BLE ADV-AA = 17886, target AdvA bytes = 0, `Simple` name bytes = 0.
- Result: FAIL — target remains air-silent with the gate patch.

Root cause confirmed (Vega review, serial analysis):

- `baseline_7f04104_sdi35.log`: 421/421 samples ip4=0x80, reason=1 (timeout). ip4 NEVER becomes 1.
- ip4=0x80 is the STABLE state set by libwchble mode=1 RX-prep arm (`gBleIPPara[4]=0x80`).
- Probe-wait condition `ip4==1` is WRONG — not the actual TX-done signal.
- Correct TX-done: three-source wait (`gBleIPPara[2].b0 || gBleIPPara[3].b0 || LLE[100]==0`).
- ip4==1 in df26837 was likely an initialization-state accident, not TX-done signaling.
- Timeout extension would NOT help; ip4 will not become 1 regardless of wait duration.

**Decision: A1 pivot (task #80) is the correct next step.** Gate patch is closed.

Tags to apply on result:
- PASS: `probe-control/scan-rsp-step2-gate-patch-pass-YYYYMMDD`
- FAIL: `bad/scan-rsp-step2-gate-patch-fail-{symptom}-YYYYMMDD`

---

## §4. Control gate pass criteria

Per scan-rsp-implementation-spec.md §6.1:
- target AdvA `C2:21:43:65:87:12` visible ≥1 frame in 60s pcap
- serial shows firmware alive
- sniffer used today: `/dev/cu.usbmodem21101`

---

## §5. Register naming reminder (GF-2)

In this example file:
- `bb_write(N, v)` → physical `0x40024200 + N` = WCH gptrLLEReg (BLE_LLE)
- `lle_write(N, v)` → physical `0x40024100 + N` = WCH gptrBBReg (BLE_BB)

**SWAPPED from WCH naming.** See notes/ch32-rs/ground-facts.md.

---

## §6. Pending after control gate PASS

1. RX-only gate: add SCAN_REQ parser, log `SCAN_REQ_ACCEPT`, verify pcap+serial match
2. SCAN_RSP warm-kick gate: enable warm kick, pcap sees target SCAN_RSP
3. OQ3 T_IFS on-chip trace: SysTick CNTL ring at SCAN_REQ_ACCEPT + SCAN_RSP_KICK
4. task #80: A1 libwchble mode=1 alignment (post step 2 PASS)

SDI timing note: any `hal::println!` inside the SCAN_REQ→SCAN_RSP turnaround window (T_IFS
150 µs) will perturb timing. Replace with counter/ring before T_IFS gate runs.

---

## §7. libwchble `ll_tx_wait_finish` disasm anchor (the shape A1 must reproduce)

Source: `ll-tx-completion-disasm.md` (Lucy agent workspace, pending migration to repo per Vega `1c1402c9` hygiene flag).

- **mode 0 (cold ADV TX)**: `LLE[0] = 2` (TX kick). No RX prep.
- **mode 1 (RX-prep / warm turnaround)**:
  1. `LLE[8] = 0x2000` (RX kick)
  2. `LLE[100] = window µs` where window is PHY-dependent (`406` / `1086` / `446` decoded from `BB[0]` bits 12-13)
  3. `gBleIPPara[4] = 0x80` (armed flag)
  4. Wait loop (`.L83`): exits on `gBleIPPara[2] & 1 || gBleIPPara[3] & 1 || LLE[100] == 0`
- **mode 3 (warm turnaround TX)**: `BB[0] |= 0x800000` + `BB[44] &= ~3` (Iron Rule §25 dual-kick warm path)

There is NO unconditional call equivalent to `rx_turnaround_capture_ch37` after the wait. The project's helper is a *partial* mode-1 sequence — it sets the RX kick + the PHY window (via `bb_write(0x08, 0x2000)` and `bb_write(0x64, rx_timer)` under §5 accessor swap) but **does not** write `gBleIPPara[4] = 0x80` and does not gate on `gBleIPPara[2]/[3] & 1 || LLE[100] == 0`. Pruning the partial sequence does not yield the libwchble shape; aligning to it requires writing the missing two pieces, not deleting work. This is why §2 v4b (delete rx_turnaround) FAILed and is why the gate-only patch (Attempt A) was not expected to be sufficient on its own — the partial state still poisons TX.

---

## §8. OPEN items (block task #80 spec freeze)

These are accessor-naming cross-checks needed before A1 implementation can start.

- [x] **LLE-bank vs BB-bank naming in project**: libwchble names the PHY window `LLE[100]`; project writes it as `bb_write(0x64, rx_timer)`. Under §5 accessor swap (`bb_*` ↔ physical `0x40024200 + N` = WCH `gptrLLEReg`), `bb_write(0x64, …)` lands at `0x40024264`. Disasm notes use byte offset form (`LLE | 100 (0x64)`), so A1 writes `bb_write(0x64, window)`.
- [x] **`gBleIPPara[4]` ↔ project `ip4`**: project `ip4` is the same byte slot. `src/ble/bb.rs` uses `ip.add(4)` for `.L6/.L4/.L8`, current example pre-GO arm writes `ip.add(4)=0x80`, and the probe wait reads `ip.add(4)` and returns it as the tuple's third value.

## §10. Attempt B: A1 `ll_tx_wait_finish(mode=1)` RX-prep alignment

Goal: replace the current partial RX-prep helper with the `libwchble` mode=1 shape while keeping the surrounding probe buffer/channel setup intact.

Patch boundary:

- Restore caller to invoke `rx_turnaround_capture_ch37()` for every channel attempt.
- Inside `rx_turnaround_capture_ch37()`:
  - keep existing RX buffer / AA / CRC / channel setup as the project-specific probe harness
  - write `bb_write(0x08, 0x2000)`
  - write `gBleIPPara[4] = 0x80`
  - write `bb_write(0x64, rx_timer)` where `rx_timer` is `406/1086/446` from PHY bits
  - wait on `gBleIPPara[2] & 1 || gBleIPPara[3] & 1 || bb_read(0x64) == 0`, matching libwchble `.L83`

Artifacts:

- Build log: `/tmp/task80_a1_mode1_align_build_20260514_0212.log`
- Flash log: `/tmp/task80_a1_mode1_align_flash_20260514_0212.log`
- Pcap: `/tmp/task79_scan_rsp_step2/task80_a1_mode1_align_sniff60.pcapng`
- Pcap raw evidence: ambient BLE ADV-AA = 16489, target AdvA bytes = 0, `Simple` name bytes = 0.
- Result: FAIL — target remains air-silent after A1 mode=1 alignment.

Candidate causes to prove next:

1. The pre-GO/IRQ path suppresses TX before RX-prep begins. A1 changes RX-prep, while the zero-air symptom may be set up earlier.
2. Existing `adv_tx_burst_ch37()` probe-wait still uses `ip4 == 1 || SysTick 2ms timeout`; Vega's 421-sample evidence shows `ip4` stays `0x80`, so that wait condition is no longer meaningful.
3. SDI prints around ADV loop may still perturb timing. A minimal no-SDI build variant should be the next low-cost isolation if Andelf approves.
4. The proper fix may require moving the whole post-TX wait to libwchble `.L83` three-source condition before any RX-prep decision, instead of only aligning the RX helper.

---

## §9. Communication protocol (Andelf `8ab182ec` 2026-05-14)

- 清晰简明有效, 一件事一人讲一次, 不来回补充.
- SDI print 可能干扰时序 — 加新 print 前问"它在什么相位执行 + 是否在 timing-critical 路径上".
- 可信时间源 = SysTick CNTL `0xE000_F008` @ 12 MHz + `bleClock_t` 625 µs. `mcycle` = 0 (QingKe V4F 未实现, only entropy use).
- 硬件验证: pcap-smoke 协议正确性 → on-chip trace ring + SDI 限路径 timing.
- 没搞定不要过早放弃, 列出 candidate 原因 (本 doc §3 "Candidate causes to prove next" 已列 4 项).
- 所有尝试路径记录到本 doc, 不重复叙述到 chat.

---

## Canonical-log statement

This file is the single consolidated attempt log (canonical 2026-05-14 ~02:09 after Cindy `02:07` initial + Vega `02:08` option-matrix + Lucy `02:08` libwchble-anchor + OPEN-items were merged). The parallel-created `scan-rsp-attempt-log.md` and `scan-rsp-step2-attempt-log.md` are removed; do not recreate them. Append new attempts as new subsections under §3 (with build/flash/pcap artifact paths + tags).
