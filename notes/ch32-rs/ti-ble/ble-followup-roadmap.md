# BLE Follow-up Roadmap: TI/WCH-aligned Rust LL Path

## Purpose

This roadmap replaces the prior retry-heavy BLE flow with a TI/WCH-aligned path toward a pure Rust BLE Link Layer. It treats TI BLE material as the semantic reference, WCH `libwchble.a` disassembly as CH32V208 hardware ground truth, and the current Rust code as the implementation target.

The immediate product goal is a protocol-correct legacy peripheral path with stable ADV + SCAN_RSP, then a Rust-owned event model for connectable peripheral work.

Retrieval entry: `README.md` is the canonical index for plan / research / decision / pitfall notes.

## Current Correction Lock: Broadcaster vs SCAN_RSP

2026-05-14 correction: Rust broadcaster / EVT Broadcaster are cold ADV TX baselines. They prove the advertising TX/init/hot-loop path. They do not prove the SCAN_REQ -> SCAN_RSP warm turnaround path.

Canonical SCAN_RSP reference is EVT Peripheral plus WCH `libwchble.a` disassembly:

```text
ll_advertise_legacy_rx
  -> ll_advertise_generated_scan_rsp
  -> BLE_SetPHYTxMode
  -> ll_tx_wait_finish(mode=3)
  -> ble_ll_hw_api_shut
```

Implementation implication: Step 2 keeps the working broadcaster/cold ADV path as control, then implements the Peripheral warm RX->TX path. Any future plan or backlog item that uses broadcaster evidence for SCAN_RSP must cite this section and split cold ADV TX from warm SCAN_RSP turnaround.

## Operating Rules

1. **Protocol first**: every Rust LL slice must map to a named BLE/WCH semantic boundary: ADV event start, per-channel TX, RX ingress, SCAN_RSP turnaround, event closed, connect transition.
2. **Full-write first-air baseline**: first working Rust TX path writes the WCH register sequence verbatim per channel. Register delta optimization starts after air PASS.
3. **Single source of timing truth**: use TI/WCH event-budget concepts and WCH 1600 Hz / 625 us scheduler ticks when designing time slices. SysTick, Embassy timers, and GPIO probes are measurement tools around that model.
4. **Air gate drives progress**: the next implementation step must have a concrete observable gate: compile, smoke, pcap, IRQ trace, or GPIO timing trace.
5. **Reference-only use of WCH ROM/lib hooks**: ROM callback APIs such as `LL_AdvertiseEventRegister` are design/probe references for our Rust LL. Production builds use Rust-owned implementations and keep `libwchble.a` symbols out of long-term runtime linkage.
6. **Existing C-path preserved while Rust LL grows**: `adv.rs`, `listener.rs`, and current examples remain fallback/control paths until Rust LL earns air PASS.

## Retired / Paused Work

These lanes produced useful evidence and should stay archived. They are not the next execution path.

| Lane | Status | Reason to pause |
| --- | --- | --- |
| v4a/v4b/v4c task-driven ADV retry series | Paused | Confirmed that patching around PATHC-per-iteration missed the broader scheduler model. Keep artifacts for BB/LLE behavior evidence. |
| `rx_turnaround_capture_ch37` W1C experiments | Paused | Confirmed W1C can kill in-flight TX. Revisit only inside the new SCAN_RSP turnaround model. |
| `feat/ble` air sanity as standalone baseline | Paused | Branch topology showed the line lacked known-good BLE runtime commits. Use buildable control combo or explicit rebase plan. |
| ch32-data `2d9eef8` typed-fieldset path | Merge-gate only | It is a suspected regression source. Audit it when code must land on the final `feat/ble` branch. |
| PATHC-on-each-loop architecture | Retired for Rust LL | WCH scheduler shows per-channel event dispatch plus state-machine boundaries. Rust LL first-air follows WCH event semantics. |

## Current Baseline Decisions

| Item | Decision |
| --- | --- |
| Buildable control combo | `ch32-hal 7f04104` + `ch32-data 1b6b0d4` |
| Long-term target branch | `feat/ble` after codegen / branch-topology cleanup |
| Current skeleton branch | `/tmp/ch32-task75-known`, `feat/ble-ll-skeleton-first-slice`, commits `bb00c8f` + `b230a88` |
| First Rust LL module shape | `src/ble/ll/{consts,types,signatures}.rs` |
| First on-device smoke | Existing `ble_peripheral_phase1_adv` compile/build gate; hardware writes begin in later slices |

## Source Inputs

| Input | Role |
| --- | --- |
| `peripheral-flow-deep-dive.md` | Full app/GAP/HCI/LL flow map against TI/WCH |
| `ll-advertise-tx-disasm.md` | Per-channel ADV TX MMIO sequence and PDU type map |
| `ll-tx-completion-disasm.md` | TX done boundary, event closed, status close |
| `ll-adv-scheduler-disasm.md` | ADV scheduler, channel hop, interval/random delay, vtable dispatch |
| `ll-process-event-minipass.md` | `LL_ProcessEvent` event-mask dispatch table |
| `ll-rx-ingress-disasm.md` | SCAN_REQ / CONNECT_IND split and SCAN_RSP prep |
| `ll-phy-preflight-disasm.md` | PHY setup, dual TX kick, RF magic, irq-mask-like MMIO |
| `ll-slave-init-disasm.md` | CONNECT_IND first-hop, slave core init, first timer |
| `ll-advertise-filter-disasm.md` | Filter policy, resolving-list mutation, return provenance |
| `evt-broadcaster-ti-flow.md` | EVT Broadcaster cold ADV path, EVT Peripheral SCAN_RSP warm path, TI GAP/HCI comparison |

## Roadmap

### Step 1 — Time and Event Model Alignment

Goal: align our WCH model with TI BLE structure and close the known timing/scheduler misconceptions for the ADV path.

Deliverables:

- `notes/ch32-rs/ti-ble/time-event-model.md`.
- ADV-only timing map:
  - TMOS event dispatch vs `ll_advertise_process` synchronous inner loop.
  - `LL_ProcessEvent` mask `0x0001` / `0x0002` dispatch and event bit-position notation.
  - `bleClock_t` + `fnGetClockCBs` 1600 Hz / 625 us time base.
  - TI §7.7.2 event budget (`ADV interval - controller margin`) applied to WCH ADV.
  - `adv_event_closed(timeUs)` where `timeUs` is remaining scheduler budget.
- Misconception ledger:
  - `timeUs` in event callbacks = remaining scheduler budget.
  - `gBleLlPara[0x7c]` = LL op-mode byte, with distinct TX-busy fields elsewhere.
  - SCAN_RSP warm kick = `BB[0] |= 0x800000`, while cold ADV TX kick = `LLE[0] = 2`.
  - `0x4002_420c` = irq-mask-like AHB MMIO, kept separate from PFIC `0xe000_e000`.
  - `ll_advertise_filter` mutates `adv_state` on resolve success.
- Scope boundary: connection timing is recorded as a follow-up input for Step 4.

Gate:

- Cindy/Lucy review pass on the ADV-only time/event model.

### Step 1.5 — Rust LL Skeleton and Contracts

Goal: encode reverse-engineered facts into Rust types, constants, and function boundaries.

Current status:

- Task #75 first slice implemented on `feat/ble-ll-skeleton-first-slice`.
- Types captured: `AdvState`, `BleLlPara`, `PduType`, `PhyMode`, `AdvStateStamp`, `TxDoneSource`, `RxDoneSource`, `FilterOutcome`, `CoreInitOutcome`.
- Function boundary stubs captured: ADV TX, SCAN_RSP prep, TX/RX wait, filter, legacy RX ingress, connect transition, slave init, PHY setup.

Remaining Phase 1 work:

1. Decide merge staging:
   - Stage current skeleton branch as a review artifact.
   - Replay or merge into final branch after Step 2 proves the path and after BL-5 becomes a merge blocker.
2. Add a feature-gated smoke example:
   - Imports `ble::ll`.
   - Constructs `AdvState`.
   - Prints constants/type decisions.
   - Exits before BB/LLE/RFEND writes.
3. Expand BL-9 / BL-10 maps only when the next implementation slice needs a field.

Gate:

- `cargo check --release --bin ble_peripheral_phase1_adv` remains green.
- Standalone LL test harness stays green until an embedded-friendly crate test path exists.

### Step 2 — Minimal Protocol-Correct SCAN_RSP

Goal: implement the smallest Rust-controlled SCAN_RSP path that preserves BLE timing and WCH semantics.

Scope lock:

- Broadcaster evidence is the cold ADV TX control.
- EVT Peripheral is the SCAN_REQ -> SCAN_RSP implementation reference.
- The first SCAN_RSP patch targets the warm path `ll_advertise_legacy_rx -> ll_advertise_generated_scan_rsp -> BLE_SetPHYTxMode -> ll_tx_wait_finish(mode=3)`.

Execution model:

1. Start from the current air-working broadcaster/control path and swap in Rust-owned pieces one boundary at a time.
2. Use task #75 LL skeleton as the type/contract layer.
3. Implement only the legacy RX ingress subset:
   - accept PDU types `{SCAN_REQ, CONNECT_IND}`.
   - parse SCAN_REQ AdvA/ScanA fields.
   - run a first-cut filter policy with provenance logging.
   - generate SCAN_RSP PDU.
4. Use the verified warm turnaround chain:
   - `ll_advertise_generated_scan_rsp()` stamps/preps state.
   - `BLE_SetPHYTxMode` equivalent configures PHY and RF state.
   - `ll_tx_wait_finish(mode=3)` equivalent performs `BB[0] |= 0x800000` + `BB[44] &= ~3`.
5. Keep deterministic `[37, 38, 39]` channel order for first air.
6. Log `TxDoneSource` and `RxDoneSource` per packet.

First implementation slice:

- Replace the relevant task #75 stubs in `src/ble/ll/signatures.rs` as each SCAN_RSP boundary lands, keeping one Rust LL source of truth.
- Keep whitelist/resolving-list backend as a trait with a permissive stub for first-air.
- Preserve WCH-tight connection interval bounds while CONNECT_IND handling is still reference-only.

Gate:

- pcap sees target ADV and target SCAN_RSP for the same AdvA.
- SCAN_RSP payload matches expected bytes.
- A phone scanner displays the scan-response payload.
- RX→TX turnaround latency is measured around T_IFS 150 us with a GPIO or IRQ timestamp probe.
- A 60s air run shows PATHC setup count bounded to the intended event boundary, with no per-loop reset pattern.

### Step 3 — Event Model and Scheduler

Goal: convert the successful SCAN_RSP path into a Rust-owned event model.

Model to derive:

- `AdvEvent` owns channel sequence, active flag, current channel, interval deadline, PDU pointers, and event-close budget.
- ADV scheduler entry mirrors `LL_ProcessEvent` mask `0x0001` and `0x0002` concepts.
- Per-channel work calls a full register-write `tx_adv_pdu()` first.
- `adv_event_closed(remaining_us)` is the single convergence point for event cleanup.
- Embassy integration occurs at event boundaries, with ISR paths limited to capture/signal work.

Work items:

1. Implement `AdvEvent` scheduler with fixed `[37,38,39]`.
2. Implement event-close and interval arming with 1600 Hz / 625 us tick accounting.
3. Add GPIO timing trace for:
   - event start,
   - per-channel TX start/done,
   - RX window start/done,
   - SCAN_RSP warm kick,
   - event closed.
4. Compare timing against TI/WCH budget: `adv_interval - controller_margin`.

Gate:

- Sustained ADV + SCAN_RSP over 60s.
- PATHC setup count stays bounded to the intended event boundary.
- Timing trace shows event windows and task/Embassy work fit within the budget.

### Step 4 — Connection First-Hop and Other BLE Features

Goal: implement the smallest connectable peripheral transition after SCAN_RSP is stable.

Scope:

- Parse CONNECT_IND.
- Run `ll_advertise_to_connection_state` equivalent synchronously in the RX ingress path.
- Implement `CoreInitOutcome` with conservative paths:
  - `ProceedStandby(handle)`,
  - `RejectKeepAdv`,
  - `Fatal`.
- Create a Rust connection state mirror only for fields required by the first-hop path.
- Defer full `ll_slave_process` decode until connection event scheduling needs it.

Gate:

- pcap shows CONNECT_IND accepted.
- Firmware logs connection state allocation and first timer publish.
- HCI/app layer receives a connection-complete equivalent event.

### Step 5 — Abstraction Cleanup

Goal: turn the proven slices into stable public/internal Rust APIs.

Cleanup decisions:

- Freeze `ble::ll` public surface only after first air PASS.
- Move hardware register sequences behind typed traits or concrete register writers.
- Replace raw `u8` selectors with enums after their hardware meaning is validated.
- Decide between sync state-machine calls and Embassy signals at each boundary.
- Add a minimal integration example per stable feature:
  - `ble_ll_skeleton_smoke`,
  - `ble_ll_adv_scanrsp`,
  - `ble_ll_connectable_phase1`.

Gate:

- APIs have one working example and one pcap-backed validation log each.
- Existing C-path examples still compile until replacement examples pass their gates.

## Backlog Disposition

| BL | Action |
| --- | --- |
| BL-1 embassy协作机制 | Phase 3 after SCAN_RSP first-air |
| BL-2 SCAN_RSP kick | Closed by Phase 3 PHY preflight; implement in Phase 2 |
| BL-3 slave init | Closed by A2; implement in Phase 4 |
| BL-4 filter | Closed by A3; implement first-cut in Phase 2 |
| BL-5 ch32-data `2d9eef8` codegen audit | Merge gate for final `feat/ble`; execution waits until the skeleton/SCAN_RSP path needs that branch |
| BL-6 host max-conn gate decode | Phase 4 when CONNECT_IND handling starts |
| BL-7 EXT_ADV / aux paths | Step 6+ |
| BL-8 TI §7.7.2 time-budget GPIO probe | Phase 3 scheduler timing gate |
| BL-9 `gBleLlPara` map | Expand on demand in Phase 2/3/4 |
| BL-10 `adv_state` map | Expand on demand in Phase 2/3/4 |

## Next Concrete Tasks

1. **Step 1 time-event model doc**: write ADV-only `time-event-model.md` from TI/WCH timing and scheduler evidence.
2. **Review task #75 branch**: keep `bb00c8f+b230a88` staged as the Step 2 type input.
3. **Step 2 SCAN_RSP implementation task**: start from the current air-working broadcaster/control path and replace only the SCAN_RSP boundary first.
4. **Step 2 timing probe**: add GPIO/IRQ timestamp probe for RX→TX turnaround and event budget.

## Anti-Drift Checklist

Before starting any new BLE task, answer:

1. Which TI/WCH semantic boundary does this touch?
2. Which disassembly doc anchors the implementation?
3. What is the smallest observable gate?
4. Which archived lane supplies the control or counterexample?
5. Which current path remains as fallback if this slice fails?

If a task cannot answer all five, write the missing answer into the roadmap before coding.
