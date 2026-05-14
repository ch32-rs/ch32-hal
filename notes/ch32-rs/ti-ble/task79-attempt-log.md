# Task #79 attempt log — SCAN_RSP Step 2 baseline control → RX-only → warm-kick smoke

Status: control gate in progress (2026-05-14)
Branch: `feat/ble-scan-rsp-step2` on main repo `/Users/mono/Elec/WCH/ch32-hal`
HEAD: `7f04104` + gate patch (see §3)

---

## §0. Correction lock: broadcaster evidence vs SCAN_RSP evidence

Rust broadcaster and EVT Broadcaster are cold ADV TX baselines. They prove the device can emit ADV frames and help validate init / channel TX / pcap tooling.

SCAN_RSP requires the EVT Peripheral warm RX->TX path:

```text
ll_advertise_legacy_rx
  -> ll_advertise_generated_scan_rsp
  -> BLE_SetPHYTxMode
  -> ll_tx_wait_finish(mode=3)
  -> ble_ll_hw_api_shut
```

Do not classify broadcaster success as SCAN_RSP progress. In task #79 rows, record cold ADV visibility and warm SCAN_RSP visibility separately.

Reference doc:

- `notes/ch32-rs/ti-ble/evt-broadcaster-ti-flow.md`

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

### A1 result (2026-05-14 02:12, task #80)

- Drop gate, add `gBleIPPara[4]=0x80`, replace inner wait with three-source (`gBleIPPara[2]&1 || gBleIPPara[3]&1 || bb_read(0x64)==0`)
- Build PASS: `/tmp/task80_a1_mode1_align_build_20260514_0212.log`
- Flash PASS: `/tmp/task80_a1_mode1_align_flash_20260514_0212.log`
- Pcap: `/tmp/task79_scan_rsp_step2/task80_a1_mode1_align_sniff60.pcapng`
- Result: ambient 16489, target 0. FAIL. Tag: `bad/scan-rsp-step2-a1-mode1-align-no-air-20260514`
- Serial: empty (SDI not captured) — A1 inner wait outcome unknown

Key observation: option 2 (rx_turnaround never ran) AND A1 (rx_turnaround ran with fixes) both yield 0 Hz.
TX failure is UPSTREAM of rx_turnaround — the OUTER probe-wait block is the suppressor.

### Next attempt: combined (a)+(b) — OUTER wait + minimal SDI

- (a) OUTER probe-wait in `adv_tx_burst_ch37`: replace `ip4==1 || 2ms` with three-source wait
- (b) Minimal SDI: remove per-iteration prints from ADV/RX hot-loop; counters only
- Both axes together isolate SDI interference vs wait-condition independently

**If still 0 Hz after (a)+(b)**: next candidate = PATHC_LIB_IRQ / Delta Y (PHY re-arm `bb_write(0x08, 0x8000)` after W1C — present in `98cd7d7` worktree but may not be in `7f04104`)

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
5. **PATHC_LIB_IRQ / Delta Y (PHY re-arm) path** (Vega `57a9fa39` post-A1 triage candidate): if combined (a)+(b) experiment also FAILs, this is the next hypothesis — TX suppressor located in the PATHC/PHY-rearm sequence before probe-wait, not in the wait condition or RX helper.

**Vega A1 milestone observation (`57a9fa39`)**: in option 2 (gate, rx_turnaround never ran), TX *still* failed → TX suppressor is in the OUTER probe-wait block or earlier, NOT inside rx_turnaround. A1 fixing rx_turnaround internals was necessary-but-not-sufficient. Combined (a)+(b) endorsed as the correct next step.

**Lucy disambiguation note (`80380516`)**: after the combined (a)+(b) pcap, split judgment using counter telemetry — `count(inner_wait_timeout)` vs `count(inner_wait_succeed)` per ADV cycle isolates whether the OUTER three-source wait is producing real completion signals or still timing out the same way as `ip4 == 1`.

---

## §11. Attempt C: OUTER three-source wait + minimal SDI

Goal: test Lucy's combined `(a)+(b)` recommendation:

- `(a)` replace the OUTER `adv_tx_burst_ch37()` wait condition from `ip4 == 1 || 2ms timeout` to the libwchble three-source condition: `gBleIPPara[2]&1 || gBleIPPara[3]&1 || bb_read(0x64)==0`.
- `(b)` remove hot-loop SDI prints from ADV/RX/SCAN_RSP path; keep counters and one periodic summary every 100 bursts.
- Preserve A1 inner RX-prep alignment from §10.

Patch details:

- `TRACE_FIRST_BURST=false`, `TRACE_EVERY_N=0`, `MINIMAL_SDI=true`.
- Added counters:
  - `OUTER_WAIT_IP2_N`
  - `OUTER_WAIT_IP3_N`
  - `OUTER_WAIT_LLE_IDLE_N`
  - `OUTER_WAIT_TIMEOUT_N`
  - `RX_TURNAROUND_DONE_N`
  - `RX_TURNAROUND_TIMEOUT_N`
- OUTER wait returns a bitmask source: bit0=`ip2`, bit1=`ip3`, bit2=`bb64==0`.
- Hot-path `hal::println!` calls are guarded by `!MINIMAL_SDI`; periodic summary still prints `tx#...`, `PATHC_IRQ counters`, `tx_heartbeat`, and `WAIT_SUMMARY`.

Artifacts:

- First build log (pre-final hot-log guard): `/tmp/task80_outer3_min_sdi_build_20260514_0217.log`
- First flash log (pre-final hot-log guard): `/tmp/task80_outer3_min_sdi_flash_20260514_0217.log`
- First pcap (pre-final hot-log guard): `/tmp/task79_scan_rsp_step2/task80_outer3_min_sdi_sniff60.pcapng`
- First pcap raw evidence: ambient BLE ADV-AA = 16148, target AdvA bytes = 0, `Simple` name bytes = 0.
- Serial check revealed a remaining hot-loop line (`# RX_TURNAROUND tx#... pdu_type=0 len=0`), so the build was tightened further before final judgment.

Final artifacts:

- Build log: `/tmp/task80_outer3_min_sdi_build2_20260514_0226.log`
- Flash log: `/tmp/task80_outer3_min_sdi_flash2_20260514_0226.log`
- Pcap: `/tmp/task79_scan_rsp_step2/task80_outer3_min_sdi_v2_sniff60.pcapng`
- Pcap raw evidence: ambient BLE ADV-AA = 15988, target AdvA bytes = 0, `Simple` name bytes = 0, target ADV prefix = 0, target SCAN_RSP prefix = 0.
- Serial watch log: `/tmp/task79_scan_rsp_step2/task80_outer3_min_sdi_v2_flash_watch35.log`
- Serial summary at `tx#100`: `done=true`, `state=108→108`, `irq_post=0x00000004`, `irq_now=0x33002000`, `adv_ok=100/100`, `solicited_scan_rsp=0`, `conn=0`.
- Counter summary at `tx#100`: `outer_ip2=0 outer_ip3=0 outer_idle=300 outer_timeout=0 rx_done=300 rx_timeout=0`.

Result: FAIL — target remains air-silent with OUTER three-source wait and minimal SDI.

Interpretation:

- OUTER wait always exits via `bb_read(0x64)==0` (`outer_idle=300`) with no `gBleIPPara[2]/[3]` completion flags.
- Inner RX helper also always exits as done (`rx_done=300`) with no timeouts, so A1's inner three-source condition fires, but the captured buffer is empty (`pdu_type=0 len=0` in the pre-final serial run).
- `state=108→108` at summary means the sampled post state is Sleep; `irq_now=0x33002000` stays in the same no-air pattern.
- Minimal-SDI did not recover air visibility, so SDI interference is no longer the primary explanation for the zero-air symptom.

Next candidate to prove:

- **PATHC_LIB_IRQ / Delta Y (PHY re-arm)**: focus on the pre-GO and GO-adjacent path, especially `bb_write(0x08, 0x8000)` / W1C / IRQ-enable ordering and `BB+0x64` lifecycle before GO. This matches Vega's post-A1 hypothesis that the suppressor sits before or at the OUTER wait boundary.

**Lucy disasm anchor (`3b96e6a5`, source `ll-tx-completion-disasm.md` §3 line 467-470)**: `gBleIPPara[2]/[3]` bit 0 is set by the **BB/PHY IRQ handler in `ip.o` (not LL.o)**; `LLE[100] == 0` (= `bb_read(0x64) == 0`) is the **hardware self-clear fallback for IRQ-miss recovery**. Mapping to telemetry: `outer_ip2=0 / outer_ip3=0 / outer_idle=300` means every iteration falls through the fallback path — i.e. **the BB/PHY IRQ handler that writes `gBleIPPara[2]/[3]` is never firing**. Same root cause as `ip4 = 0x80` stable (BB ISR not writing `gBleIPPara[4] = 1` on TX done either).

Specific review questions for the §5 candidate triage:
1. Does `src/ble/bb.rs` ISR write `ip.add(2)` / `ip.add(3)` to `gBleIPPara`? Cindy `c00c1a7f` verified `ip.add(4)` is touched (pre-GO arm + probe-wait reads), but did not enumerate `[2]/[3]` writes. If the ISR is missing those writes, A1/A2 wait-condition patches are treating a symptom — root cause is incomplete ISR.
   - **Lucy answer (2026-05-14 02:50)**: `rg "ip\.add\(2\)|ip\.add\(3\)|gBleIPPara\[2\]|gBleIPPara\[3\]" src/` → **zero matches across all of `src/`**. The Rust port of `BB_IRQLibHandler` writes only `ip.add(4)` (`.L6` arm to 0xC0; `.L4` bit4 + `.L8` bit7 → 1). Slots `[2]/[3]` are never touched by any Rust code path. Implication: if Vega's path-B IRQ-reorder gates ip4==1 visibility, project's [4] substitute is the right wait target; if path-B fails, next candidate must include obtaining `ip.o` disasm to find the libwchble BB-ISR sources for `[2]/[3]` writes (currently unknown — disasm we have is LL.o only).
2. PFIC IRQ enable state for BB IRQ — is it actually enabled at the point ADV-cycle TX is kicked? What register is `irq_now = 0x33002000` sampling?
3. Cross-check libwchble `ip.o` disasm for the BB ISR — find the exact instruction that writes `gBleIPPara[2]/[3]` after TX completion, then verify the project ISR has an equivalent.

Preservation:

- Commit/tag this state before the next patch:
  - commit: `bad(ble): archive outer wait minimal sdi failure`
  - tag: `bad/scan-rsp-step2-outer3-min-sdi-no-air-20260514`

---

## §12. Attempt D: OUTER project-ip4 wait, 50 ms timeout, minimal SDI

Goal: apply Vega's milestone review recommendation after Attempt C:

- Keep project-native completion signal `gBleIPPara[4]` (`ip4`), because project `src/ble/bb.rs` writes only this slot.
- Increase OUTER wait timeout from `24_000` SysTick ticks (2 ms) to `600_000` SysTick ticks (50 ms).
- Preserve minimal-SDI and A1 inner helper from §10/§11.

Patch details:

- OUTER wait condition: `while ip4 != 1 && SysTickDelta < 600_000 { spin }`.
- Added `OUTER_WAIT_IP4_N` counter and extended `WAIT_SUMMARY`.
- Removed the incorrect OUTER `ip2/ip3/bb64==0` condition from Attempt C.

Artifacts:

- Build log: `/tmp/task80_ip4_50ms_min_sdi_build_20260514_0241.log`
- Flash log: `/tmp/task80_ip4_50ms_min_sdi_flash_20260514_0241.log`
- Pcap R1: `/tmp/task79_scan_rsp_step2/task80_ip4_50ms_min_sdi_sniff60.pcapng`
- Pcap R1 raw evidence: ambient BLE ADV-AA = 15453, target AdvA bytes = 1, target ADV prefix = 1, `Simple` name bytes = 0.
- Serial watch: `/tmp/task79_scan_rsp_step2/task80_ip4_50ms_min_sdi_flash_watch35.log`
- Serial summary at `tx#100`: `outer_ip2=0 outer_ip3=0 outer_ip4=0 outer_idle=0 outer_timeout=300 rx_done=300 rx_timeout=0`, `BB=4/4`, `irq_now=0x39002000`.
- Pcap R2 after serial-watch reflash: `/tmp/task79_scan_rsp_step2/task80_ip4_50ms_min_sdi_r2_sniff60.pcapng`
- Pcap R2 raw evidence: ambient BLE ADV-AA = 15708, target AdvA bytes = 1, target ADV prefix = 1, `Simple` name bytes = 1.

Result: MARGINAL AIR — two independent 60s pcap runs saw exactly one target ADV each.

Interpretation:

- Air visibility recovered from 0/60s to about 1 frame/60s, still far below the historical `df26837` 17.1 Hz behavior.
- `outer_ip4=0 outer_timeout=300` means `ip4==1` did not fire in the serial-watch run even with 50 ms timeout. The rare air frames appear to be race leakage, not clean project-ISR completion.
- `BB=4/4` shows BB IRQ can enter/exit during the serial-watch run, but not enough to set `ip4==1` on the sampled path.
- This result satisfies the original loose "≥1 target frame" control visibility criterion only in the weakest possible form. It is not a stable baseline for SCAN_RSP implementation.

Next candidate to prove:

- PATHC_LIB_IRQ / Delta Y remains the main path: the pre-GO/GO-adjacent re-arm and W1C ordering can still suppress the TX path most iterations.
- Specifically inspect why BB IRQ entry reaches only `4/4` by `tx#100` while `ip4` completion never fires, and compare `bb.rs` bit4/bit7 conditions against the current `irq_now=0x39002000` state.

Preservation:

- Commit/tag this state before the next patch:
  - commit: `probe(ble): archive ip4 50ms marginal air result`
  - tag: `probe/scan-rsp-step2-ip4-50ms-marginal-air-20260514`

---

## §13. Attempt E: Path B post-GO IRQ enable

Goal: test Vega's Path B hypothesis from ip4-50ms review:

- Move `enable_interrupt(63)` from the PATHC pre-GO block to immediately after `bb_write(0x00, 2)` (GO strobe).
- Keep ip4 50 ms OUTER wait.
- Keep minimal-SDI and A1 inner helper.

Patch details:

- PATHC pre-GO block still clears W1C/status and unpends IRQ 63/64.
- `qingke::pfic::enable_interrupt(63)` executes after the GO strobe.
- LLE IRQ 64 handling remains unchanged.

Artifacts:

- Build log: `/tmp/task80_pathb_postgo_irq_build_20260514_0248.log`
- Flash log: `/tmp/task80_pathb_postgo_irq_flash_20260514_0248.log`
- Pcap: `/tmp/task79_scan_rsp_step2/task80_pathb_postgo_irq_sniff60.pcapng`
- Pcap raw evidence: ambient BLE ADV-AA = 21683, target AdvA bytes = 0, `Simple` name bytes = 0, target ADV prefix = 0, target SCAN_RSP prefix = 0.
- Serial watch: `/tmp/task79_scan_rsp_step2/task80_pathb_postgo_irq_flash_watch35.log`
- Serial summary at `tx#100`: `outer_ip2=0 outer_ip3=0 outer_ip4=0 outer_idle=0 outer_timeout=300 rx_done=300 rx_timeout=0`, `BB=5/5`, `irq_now=0x39002000`.

Result: FAIL — moving IRQ63 enable post-GO did not recover target ADV.

Interpretation:

- Pre-GO IRQ fire was not the sole suppressor.
- `ip4==1` still does not fire in 300 attempts.
- BB IRQ entry count is tiny (`5/5` by `tx#100`) and still does not produce the project completion signal.
- `irq_now=0x39002000` persists, matching the previous stuck/no-air family.

Next candidate to prove:

- PATHC canonical fix likely needs to stop doing per-iteration PATHC W1C/re-arm at all (Path A: move PATHC setup once at init), or directly compare the current per-GO W1C/status-clearing sequence against EVT/libwchble's `ll_tx_wait_finish(mode=0)` cold-TX shape.
- Do not continue changing wait predicates; all wait variants now show the same completion-signal absence.

**Vega milestone review `bde68b52` (02:52)**: Path B regression explains itself — pre-GO ISR fires `.L6` that configures LLE timer registers required by TX (dependency, not a bug). df26837 works because PATHC W1C runs **once** (task hangs after init). Per-iteration PATHC W1C resets LLE timing every iteration and is incompatible with HW-autonomous TX. **Recommendation: Path A — PATHC once at init**. Hot loop = channel + TX buf + bare GO strobe. Example-only scope (no `src/ble/` changes). Architectural change — Andelf confirmation required before Cindy executes.

**Lucy disasm anchor for Path A (`b17c25cb`, source `ll-tx-completion-disasm.md` §1.3 lines 67-82)** — mode 0 cold-TX entry in libwchble is exactly 3 register touches per ADV slot:

```
d4: while LLE[100] != 0 {}      ; pre-wait LLE busy (bb_read(0x64))
e4: call BLE_SetPHYTxMode        ; PHY mode setup
118: LLE[0] = 2                  ; TX kick (= project bb_write(0x00, 2))
fallthrough → .L81 wait
```

No PATHC W1C, no IRQ enable re-order inside the loop. PATHC runs once during `ble_ip_core_init` / PHY bringup, then hot loop re-enters the 3-step kick per ADV slot. This is the literal disasm shape Vega's Path A reconstructs from df26837 behavior.

**Status (02:53)**: Cindy `8fdf4cc8` holds at `c193349`, no further wait/IRQ tweaks. Vega `f0ce7f99` ack. Both wait for Andelf decision.

Preservation:

- Commit/tag this state before the next patch:
  - commit: `bad(ble): archive post-go irq enable failure`
  - tag: `bad/scan-rsp-step2-postgo-irq-no-air-20260514`

---

## §14. Attempt F: Path A once-at-init PATHC block

Goal: execute Andelf-approved Path A after Vega/Lucy convergence:

- Run the PATHC / IRQ setup block once before the ADV hot loop.
- Keep the per-slot PHY/LLE sequence in the hot loop (`ble_set_phy_tx_mode_1mbps` and surrounding writes), matching libwchble cold-TX shape.
- Keep hot loop to channel update + TX buffer update + per-slot PHY config + bare `bb_write(0x00, 2)` GO strobe.
- Remove timing-heavy SDI probes from the judgement run.

Patch details:

- Added `PATHC_ONCE_AT_INIT = true` and `pathc_lib_irq_arm_once()`.
- Moved these once-init effects out of the hot loop: `gBleIPPara[4]=0x80`, `gBleIPPara[5]=0`, `gBleIPPara[16]=776`, PATHC W1C `bb_write(0x08, 0x0000_FFFF)`, `bb_write(0x38, 0xFF)`, `lle_write(0x38, 0xF0)`, Delta Y `bb_write(0x08, 0x8000)`, `unpend_interrupt(63/64)`, `enable_interrupt(63)`.
- Disabled `RX_TURNAROUND_PROBE` for cold-ADV control.
- Disabled `X1_POLLED_W1C` and `BB08_TRACE` for the final judgement run after an initial flash showed `PATHC_ALIVE x1-*` / `BB08_TRACE[*]` SDI noise in the hot path.

Artifacts:

- Build log before SDI cleanup: `/tmp/task80_patha_once_init/build_clean_20260514_0832.log`
- Flash+watch showing SDI noise: `/tmp/task80_patha_once_init/flash_clean_watch_20260514_0833.log`
- Final build log: `/tmp/task80_patha_once_init/build_clean2_20260514_0834.log`
- Final flash log: `/tmp/task80_patha_once_init/flash_clean2_20260514_0835.log`
- Pcap R1: `/tmp/task80_patha_once_init/patha_clean_sniff60.pcapng`
- Pcap R1 raw evidence: ambient BLE ADV-AA = 22268, target ADV_IND prefix = 2, target AdvA bytes = 2, `Simple` bytes = 2.
- Pcap R2: `/tmp/task80_patha_once_init/patha_clean_r2_sniff60.pcapng`
- Pcap R2 raw evidence: ambient BLE ADV-AA = 19519, target ADV_IND prefix = 0, target AdvA bytes = 0, `Simple` bytes = 0.

Result: FAIL / MARGINAL LEAKAGE — Path A once-init produces at most two target ADV_IND frames in a 60s run and zero in the repeat run. This does not restore the historical stable `df26837` air rate.

Interpretation:

- Per-iteration PATHC W1C/IRQ setup is a real suppressor candidate, but moving it once-at-init is not sufficient to restore stable cold ADV in the current `7f04104`-derived code path.
- The first pcap's two adjacent target frames show the RF path can still leak valid `ADV_IND` payloads; the repeat run confirms it is not a stable baseline.
- The remaining suppressor likely sits in the post-init hot-loop delta versus df26837/EVT/libwchble: TX-completion / BB IRQ dependency, RF/LLE timer setup lifetime, or residual diagnostic code still changing timing/state before the GO strobe.

Data-integrity check after Lucy `6968ac4b`:

- R1 target frames have raw header bytes `40 16`: PDU type = `0b0000` (`ADV_IND`), TxAdd=1, len=22.
- This is expected for the current `ble_peripheral_phase1_adv.rs` file: `build_adv_pdu()` now builds `ADV_IND` and the boot log prints `ADV_IND built`. The older "broadcaster-like ADV_NONCONN" wording is stale for this branch.
- #81 still remains the formal minimal connectable-ADV task; this #80 result can be used only as cold-ADV leakage evidence, not as #81 completion.

Cadence check after Vega `3a776c00`:

- `df26837` and current Path A both use the Embassy scheduler loop with `ADV_INTERVAL = 200ms` and one immediate 37/38/39 channel sweep per scheduled event.
- The linked EVT reference enters ADV through TMOS timers: `llAdvertiseStart` arms the timer, and later `LL_ProcessEvent mask 0x0002 / bit 1 -> llAdvTraverseallChannel -> ll_advertise_tx`.
- EVT's hot TX path performs the repeated full per-channel TX register sequence, while event-level cadence is timer-driven. Current Rust already has an event-level `Timer::at(next_adv)` around the channel sweep, so the next delta target should be the per-channel TX register/ISR state, not a generic free-running-loop assumption.

Next candidates to prove:

1. Compare `df26837` hot-loop register writes against this Path A build and isolate the exact additional per-slot writes still present.
2. Confirm whether `ble_set_phy_tx_mode_1mbps` or surrounding per-slot PHY/LLE writes leave `BB+0x08 = 0x39002caa` stuck after GO in the Path A run.
3. Use EVT Peripheral/Broadcaster linked disassembly as the next anchor before changing code again, per Andelf's "EVT + TI + libwchble baseline" directive.

Preservation:

- Commit/tag this state before the next patch:
  - commit: `bad(ble): archive path a once init failure`
  - tag: `bad/scan-rsp-step2-patha-once-init-no-air-20260514`

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
