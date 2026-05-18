# SCAN_RSP implementation spec — first step

Status: review-ready for task #78.

Goal: pick the first implementation path for a protocol-correct legacy `SCAN_REQ -> SCAN_RSP` response and lock the development / flashing / verification mode before code changes.

## 0. Correction Lock: What Existing Broadcaster Evidence Proves

Rust broadcaster / EVT Broadcaster evidence proves cold ADV TX. SCAN_RSP uses a distinct warm RX->TX turnaround path.

Do not use broadcaster air success as SCAN_RSP evidence. Use it as the control baseline for ADV visibility only.

SCAN_RSP reference path comes from EVT Peripheral and `libwchble.a`:

```text
ll_advertise_legacy_rx
  -> ll_advertise_generated_scan_rsp
  -> BLE_SetPHYTxMode
  -> ll_tx_wait_finish(mode=3)
  -> ble_ll_hw_api_shut
```

Critical split:

- Cold ADV TX: `ll_advertise_tx` / `ll_tx_wait_finish(mode=0)` eventually writes `LLE[0] = 2`.
- Warm SCAN_RSP TX: `ll_tx_wait_finish(mode=3)` writes `BB[0] |= 0x800000` and `BB[44] &= ~3`.

This section is the guardrail for Step 2. Future attempts must preserve this split in code, logs, and review wording.

## 1. Inputs and constraints

### 1.1 Canonical inputs

| Input | What it contributes |
| --- | --- |
| `ble-followup-roadmap.md` | Step 2 scope and air gate: minimal protocol-correct SCAN_RSP, existing broadcaster path as control, task #75 skeleton as type/signature input. |
| `time-event-model.md` | ADV-only event model, Step 2 PASS gate = OQ3 RX->TX turnaround probe. In this spec OQ3 is implemented with on-chip trace + pcap, not external GPIO. |
| task #75 branch `/tmp/ch32-task75-known` commits `bb00c8f` + `b230a88` | `src/ble/ll/{consts,types,signatures}.rs` skeleton and neutral types. |
| `ll-rx-ingress-disasm.md` | SCAN_REQ vs CONNECT_IND split, `ll_advertise_generated_scan_rsp`, `byte_11=0x93`, SCAN_RSP chain. |
| `ll-phy-preflight-disasm.md` | SCAN_RSP warm kick: `ll_tx_wait_finish(mode=3)` writes `BB[0] |= 0x800000` and `BB[44] &= ~3`; `BLE_SetPHYTxMode` configures PHY/RF and does not kick TX. |
| `ll-advertise-tx-disasm.md` | Cold ADV TX register sequence and PDU buffer model. |
| `ll-tx-completion-disasm.md` | TX/RX completion three-source wait model and `ll_advertise_process` sequencing. |
| `evt-broadcaster-ti-flow.md` | EVT Broadcaster cold ADV reference, EVT Peripheral SCAN_RSP reference, TI GAP/HCI comparison. |

### 1.2 Hard constraints

- Pure Rust long-term implementation. WCH ROM/lib hooks are reference/probe inputs only.
- Preserve the existing broadcaster/control path until Rust SCAN_RSP earns air PASS.
- First-air register sequences are full-write / WCH-faithful. Register delta optimization starts after air PASS.
- Step 2 touches the legacy ADV path only: ADV_IND + SCAN_REQ -> SCAN_RSP. CONNECT_IND handling remains reference-only.
- BL-5 (`ch32-data 2d9eef8` codegen audit) is a final `feat/ble` merge gate, not the first implementation blocker.

## 2. Baseline and branch choice

### 2.1 Baseline matrix

| Candidate | Pros | Risks | Decision |
| --- | --- | --- | --- |
| `feat/ble` | Long-term target branch. | Branch was not an air-validated baseline; current line is entangled with typed-fieldset/codegen state. | Do not use for first SCAN_RSP implementation. |
| `feat/ble-on-known-good @ a8e39bb` | Contains more recent BLE work and task #75 ancestry context. | Air failed with current typed-fieldset combination; depends on the unresolved `2d9eef8` path. | Keep as reference for later branch replay. |
| `7f04104 + ch32-data 1b6b0d4` | Buildable control combination; prior control air hit; current `ble_peripheral_phase1_adv` exists; avoids BL-5. | Older code shape; final `feat/ble` replay needed after air PASS. | Use for first implementation and first hardware gate. |
| task #75 branch `/tmp/ch32-task75-known` | Contains LL skeleton types and signatures. | It is a staging branch, not the air-control path itself. | Cherry-pick/replay the skeleton into the implementation worktree only when needed by code. |

### 2.2 Development baseline

Use a new worktree from the buildable control combo:

```bash
cd /Users/mono/Elec/WCH/ch32-hal
git worktree add /tmp/ch32-scan-rsp-step2 7f04104
cd /tmp/ch32-scan-rsp-step2
git switch -c feat/ble-scan-rsp-step2
```

ch32-data/metapac side:

```bash
cd /Users/mono/Elec/WCH/ch32-data
git checkout 1b6b0d4
./d gen
```

The implementation worktree remains separate from the existing repo worktree so task #75 review artifacts and untracked notes stay undisturbed.

## 3. Candidate implementation approaches

### Option A — surgical control-path swap

Start from `ble_peripheral_phase1_adv.rs` on `7f04104`, keep the known ADV broadcaster and replace the old SCAN_RSP block with a WCH-faithful warm-turnaround chain:

1. existing ADV_IND TX stays intact.
2. RX ingress captures SCAN_REQ after ADV TX.
3. SCAN_RSP PDU is generated into the existing TX buffer.
4. TX PHY is configured with the decoded `BLE_SetPHYTxMode` equivalent.
5. warm kick writes `BB[0] |= 0x800000` and `BB[44] &= ~3`.
6. wait loop records `TxDoneSource` / `RxDoneSource`.

Expected benefit: fastest path to air evidence. Existing broadcaster stays as control, so regressions are easy to bisect.

Risk: `ble_peripheral_phase1_adv.rs` is large and historically accumulated probe code. The first patch must be narrow and heavily logged.

### Option B — task #75 skeleton-first fill

Replay task #75 skeleton into the worktree and fill `src/ble/ll/signatures.rs` implementations first, then wire the example to call those functions.

Expected benefit: source layout matches the final Rust LL design.

Risk: more code moved before the first air gate. It can obscure whether failures come from register semantics, module wiring, or baseline drift.

### Option C — throwaway ROM/lib probe

Use WCH callbacks or lib symbols to timestamp event boundaries and compare against the Rust path.

Expected benefit: good learning data if a single unknown blocks implementation.

Risk: conflicts with the roadmap if treated as implementation. It belongs only as a throwaway probe after a concrete measurement question is identified.

### Recommendation

Use **Option A** for the first implementation slice, with task #75 types gradually imported once the air-visible path is stable.

Reasoning: the next useful fact is air-level SCAN_RSP behaviour. Option A minimizes variables and keeps the old broadcaster as a control. Option B becomes the cleanup path after the first WCH-faithful warm turnaround works. Option C remains a scoped measurement tool.

## 4. First implementation slice

### 4.1 Scope

Modify only the example / local LL helper code needed for one SCAN_RSP air gate:

- Add a named helper for SCAN_REQ classification:
  - accept `pdu_type == 3`.
  - require `pdu_len >= 12`.
  - require `AdvA == C2:21:43:65:87:12` (the current target address).
  - log ScanA and header fields.
- Add a named helper for SCAN_RSP PDU generation:
  - header type `0x04 | TxAdd`.
  - payload = current `build_scan_rsp_pdu()` bytes unless changed by spec review.
  - write into the same TX buffer used by ADV.
- Replace the old SCAN_RSP TX block with decoded warm-turnaround shape:
  - `ble_set_phy_tx_mode_1mbps(scan_rsp_payload_len)`.
  - remove the legacy ad-hoc `bb_write(0x08, 0x8000)` from the SCAN_RSP path unless a later disasm-backed need appears.
  - warm kick = WCH `BB[0] |= 0x800000`.
  - commit clear = WCH `BB[44] &= !0x3`.
  - wait for TX completion with a bounded three-source loop.
- Add instrumentation:
  - `SCAN_REQ_ACCEPT`.
  - `SCAN_RSP_PREP`.
  - `SCAN_RSP_KICK`.
  - `SCAN_RSP_DONE`.
  - `TxDoneSource` / timeout reason.
  - on-chip timestamp/ring entries at SCAN_REQ accept and warm kick. Preferred timestamp source is SysTick CNTL `0xE000_F008` (12 MHz in the existing BLE experiments). BLE 625 us ticks are acceptable for event-budget traces and too coarse for T_IFS.

### 4.2 Explicit non-scope

- No CONNECT_IND state transition.
- No whitelist/resolving-list backend beyond a permissive first-air policy.
- No branch replay to final `feat/ble`.
- No ch32-data `2d9eef8` audit.
- No PATHC-per-loop redesign.
- No extension/aux advertising.

### 4.3 Register-accessor caution

The existing example uses historical `lle_*` / `bb_*` helper naming with WCH naming swapped:

- Example `lle_*` base = physical `0x4002_4100` = WCH `gptrBBReg`.
- Example `bb_*` base = physical `0x4002_4200` = WCH `gptrLLEReg`.

Before implementation, write a three-row comment next to the warm kick helper naming the WCH register, physical address, and project accessor. This prevents reintroducing the GF-2 comment drift class of bug.

Warm kick implementation must cite:

- WCH `BB[0] bit 23` = physical `0x4002_4100`, current example accessor `lle_write(0x00, lle_read(0x00) | 0x800000)`.
- WCH `BB[44] bits 1:0` = physical `0x4002_412c`, current example accessor `lle_write(0x2c, lle_read(0x2c) & !0x3)`.

The current old block already writes the first warm-kick register through `lle_write(0x00, ctrl | 0x800000)`. It lacks the WCH-faithful `BB[44] &= !0x3` commit clear and includes an extra `bb_write(0x08, 0x8000)` inherited from older experiments.

## 5. Development mode

### 5.1 Worktree discipline

All implementation happens in `/tmp/ch32-scan-rsp-step2`.

Commit policy:

1. `probe-control`: build/flash unmodified baseline from `7f04104` and collect a fresh control pcap.
2. `probe-rx-only`: SCAN_REQ capture/log only, no SCAN_RSP response.
3. `probe-scan-rsp-warm-kick`: enable SCAN_RSP warm kick.
4. `probe-cleanup`: small cleanup after air PASS only.

Each failed hardware run gets a `bad/scan-rsp-step2-{symptom}-{shortid}` tag and stops for review.

### 5.2 Build

Primary build:

```bash
cd /tmp/ch32-scan-rsp-step2/examples/ch32v208
cargo build --release --bin ble_peripheral_phase1_adv
```

Optional binary extraction:

```bash
cargo objcopy --release --bin ble_peripheral_phase1_adv -- -O binary /tmp/scan_rsp_step2.bin
```

### 5.3 Flash

Preferred one-command run when SDI logs are useful:

```bash
cd /tmp/ch32-scan-rsp-step2/examples/ch32v208
cargo run --release --bin ble_peripheral_phase1_adv
```

Binary flash path:

```bash
wlink flash /tmp/scan_rsp_step2.bin
wlink reset
```

Keep serial/SDI output to artifacts:

```bash
mkdir -p /tmp/task78_scan_rsp_step2
# capture method depends on the current wlink/SDI workflow; log filename convention:
# /tmp/task78_scan_rsp_step2/{rev}_{stage}_serial.log
```

## 6. Verification mode

### 6.1 Control gate

Before any SCAN_RSP code change:

- flash unmodified `7f04104 + 1b6b0d4`.
- run 20s pcap.
- run 10s phone/mac scanner.

Pass bar:

- target AdvA `C2:21:43:65:87:12` visible at least once in pcap or scanner.
- serial shows firmware alive.

This gate proves the board, scanner, phone, and branch combo are healthy.

### 6.2 RX-only gate

After adding SCAN_REQ detection / logging, with SCAN_RSP response disabled:

- pcap captures target ADV.
- central/phone produces at least one SCAN_REQ to target AdvA, or pcap proves no SCAN_REQ occurred.
- firmware logs `SCAN_REQ_ACCEPT` only for matching AdvA.

Pass bar:

- no loss of baseline ADV visibility.
- SCAN_REQ parser logs match pcap header and addresses.

If no SCAN_REQ appears, the test setup is incomplete; use an active scanner app or nRF tooling before debugging firmware.

### 6.3 SCAN_RSP air gate

After enabling warm kick:

- pcap sees target ADV and target SCAN_RSP for the same AdvA.
- SCAN_RSP payload bytes match the expected PDU.
- phone scanner displays the scan-response payload (`Simple` / UUID as configured).
- firmware logs `SCAN_RSP_PREP`, `SCAN_RSP_KICK`, and `SCAN_RSP_DONE` for the same event.

Scanner hygiene: between runs, force the phone scanner to re-scan or restart the app instance so cached advertising data cannot produce a false positive.

Recommended capture:

```text
/tmp/task78_scan_rsp_step2/{rev}_scan_rsp_60s.pcapng
/tmp/task78_scan_rsp_step2/{rev}_scan_rsp_serial.log
/tmp/task78_scan_rsp_step2/{rev}_scan_rsp_summary.txt
```

Minimum PASS:

- `target_scan_rsp_frames >= 1`.
- `target_adv_frames >= 1`.
- SCAN_RSP payload matches expected bytes.

Sustained PASS target:

- 60s run with repeated ADV and repeated SCAN_RSP when active scan requests arrive.
- no PATHC per-loop reset signature in serial counters.

### 6.4 T_IFS / OQ3 on-chip trace gate

The OQ3 hard gate from `time-event-model.md`:

- Add a small in-SRAM ring buffer with one record per SCAN_REQ/SCAN_RSP attempt.
- Record A: `SCAN_REQ_ACCEPT` immediately after the RX parser accepts the SCAN_REQ and AdvA matches.
- Record B: `SCAN_RSP_KICK` immediately before the warm kick (`BB[0] |= 0x800000`, `BB[44] &= !0x3`).
- Each record stores:
  - timestamp from SysTick CNTL `0xE000_F008` for RX->TX turnaround timing.
  - optional BLE 625 us tick for event-budget correlation.
  - channel.
  - `BB[0]`, `BB[44]`, `LLE[100]`.
  - gBleIPPara done/error flag bytes used by the TX/RX wait loops.
  - compact PDU header fields (`pdu_type`, `pdu_len`, AdvA match bit).
- Dump the ring over serial after the run, or via halt-read if the serial path perturbs timing.

Pass bar:

- pcap shows SCAN_RSP for the same AdvA after SCAN_REQ.
- trace shows each `SCAN_RSP_KICK` follows a matching `SCAN_REQ_ACCEPT` with the expected register state:
  - `BB[0]` bit 23 set at kick.
  - `BB[44]` bits `[1:0]` cleared at kick.
  - RX/TX done source recorded, not inferred from pcap alone.
- timing delta `SCAN_RSP_KICK - SCAN_REQ_ACCEPT` is reported. It is advisory until we calibrate the chosen timestamp source against over-the-air packet boundaries; pcap visibility is the protocol-level pass.
- pcap corroborates SCAN_RSP for same AdvA.

This keeps the formal gate inside the CH32V208 BLE system: BB/LLE register trace + IRQ/wait-source trace + pcap. External GPIO is an optional later measurement aid, not a prerequisite and not a lock question.

## 7. Failure triage

| Symptom | Likely class | First action |
| --- | --- | --- |
| Baseline ADV disappears before code changes | board/scanner/baseline drift | stop; re-run known control; do not edit. |
| ADV visible, no SCAN_REQ in pcap | test setup | use active scanner / nRF; do not debug firmware yet. |
| SCAN_REQ in pcap, firmware logs no accept | RX parser / buffer timing | inspect captured RX buffer and AdvA offsets. |
| Firmware accepts SCAN_REQ, no SCAN_RSP pcap | warm kick / PHY config / TX buffer | compare serial `SCAN_RSP_*` counters; inspect BB[0], BB[44], LLE[100]. |
| One SCAN_RSP then ADV stops | state cleanup / event-close damage | tag bad; review warm kick side effects and `ble_ll_hw_api_shut` equivalent. |
| SCAN_RSP visible but wrong payload | PDU build / DMA pointer | dump TX buffer before kick and compare to pcap bytes. |
| SCAN_RSP visible but trace delta is large or inconsistent | timestamp placement / scheduler shape | keep pcap smoke result, inspect ring placement and wait-source fields before changing registers. |

## 8. Decision record

Recommended next task after this spec:

1. Create `/tmp/ch32-scan-rsp-step2` worktree at `7f04104`.
2. Run and record fresh baseline control.
3. Add RX-only SCAN_REQ parser/logging.
4. Gate RX-only with pcap + serial.
5. Enable SCAN_RSP warm kick using the decoded mode=3 path.
6. Gate pcap/phone SCAN_RSP smoke.
7. Add OQ3 on-chip trace ring for formal Step 2 evidence.

Review questions for Andelf/Lucy:

1. Is Option A accepted as the first implementation path?
2. Is pcap-visible SCAN_RSP accepted as the first smoke gate before the full on-chip trace ring is complete?
3. Should task #75 skeleton be replayed into the implementation worktree before RX-only, or after SCAN_RSP smoke PASS?
