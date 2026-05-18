# Connectable ADV Baseline Spec

Status: draft for task #81 review, 2026-05-14.

Goal: establish the smallest Rust-side EVT Peripheral baseline that emits legacy connectable `ADV_IND` reliably. This stage proves the Peripheral entry condition before SCAN_REQ parsing or SCAN_RSP warm turnaround work.

Primary cross-reference: `task79-attempt-log.md` §0 records the broadcaster vs Peripheral correction lock. Broadcaster evidence feeds cold ADV control; Peripheral evidence feeds connectable ADV / SCAN_RSP.

## Scope

### In scope

- Start from the existing cold ADV TX path that can be observed on air.
- Change only the advertising PDU mode needed to emit `ADV_IND`.
- Preserve AdvA, payload shape, channel sequence, and TX register bring-up as much as possible.
- Verify with pcap and minimal SDI counters.

### Later stages

- SCAN_REQ parser / RX ingress.
- SCAN_RSP warm RX->TX turnaround.
- CONNECT_IND slave init / connection event.
- Rust LL abstraction cleanup.

## EVT Alignment

| EVT Example | BLE Role | PDU | Use in Rust Plan |
| --- | --- | --- | --- |
| Broadcaster | Non-connectable broadcaster | `ADV_NONCONN_IND` | Cold ADV TX / init / hot-loop reference. |
| Peripheral | Connectable peripheral | `ADV_IND` + optional `SCAN_RSP` + `CONNECT_IND` | Current task #81 reference for connectable ADV baseline. |

Key split:

- Existing Rust Broadcaster evidence proves cold ADV TX.
- task #81 proves the Peripheral advertising entry by switching the legacy PDU to `ADV_IND`.
- Step 2 SCAN_RSP uses the warm path documented in `scan-rsp-implementation-spec.md`.

## Development Mode

| Item | Decision |
| --- | --- |
| Working branch | `feat/ble-scan-rsp-step2`, unless Andelf redirects. |
| Final target | `feat/ble` after Step 2 evidence and BL-5 audit. |
| Branch index | `branch-status.md`. |
| Attempt log | `task79-attempt-log.md` gets a new task #81 row only when hardware is run. |
| Review gate | @Lucy reviews this spec before implementation. |

## Sequencing Lock

task #81 is a spec/review task until Andelf chooses the baseline sequence.

Two valid baselines:

| Option | Meaning | Condition |
| --- | --- | --- |
| Wait for #80 Path A | Restore a reliable cold ADV baseline first, then change only the PDU type to `ADV_IND`. | Preferred serial path. |
| Pivot to an older working cold ADV baseline | Start #81 from a known air-visible cold ADV commit and treat it as replacing the #80 recovery path. | Requires explicit Andelf decision. |

Current `feat/ble-scan-rsp-step2 @ c193349` is a failed Path B state with 0 target cold ADV. It is useful as evidence and attempt history, not as the code base for `ADV_IND` visibility.

## Debug / Verification Plan

### Gate 0 — Build and flash control

Use the current buildable example path and keep artifacts under `/tmp/task81_connectable_adv/`.

Pass criteria:

- Build succeeds.
- Flash succeeds.
- Firmware heartbeat / low-rate SDI counter confirms the loop is alive.

### Gate 1 — Baseline pcap

Before changing code, capture the current ADV output for the selected branch.

Pass criteria:

- pcap contains target AdvA.
- Current PDU type and payload are recorded.
- Result row appended to `task79-attempt-log.md` or a task #81 section.

### Gate 2 — Connectable ADV patch

Patch only the minimum PDU type / state needed for `ADV_IND`.

Expected evidence:

- pcap shows target `ADV_IND` with header type field `0b0000`.
- TxAdd/RxAdd fields match the selected AdvA address mode.
- AdvA matches the baseline AdvA.
- Advertising payload remains parseable.
- Channel rotation remains present across 37/38/39 when observable.

### Gate 3 — Stability window

Run a 60 s pcap.

Pass criteria:

- Target `ADV_IND` appears repeatedly.
- 60 s target `ADV_IND` count meets or exceeds the selected cold ADV baseline rate for the same board/setup.
- No burst of malformed target packets.
- SDI summary counters stay low-rate and do not print inside the per-packet timing path.

## Instrumentation Rules

1. Use pcap as the primary air-visible protocol gate.
2. Use SDI only for low-rate summary counters, for example every 100 loop iterations.
3. Keep SysTick CNTL available for later timing probes; task #81 does not need sub-us T_IFS timing.
4. Preserve failed variants with `bad/*` commits/tags and append rows to the attempt log.

## Candidate Code Touch Points

These are inspection targets before patching:

| Area | Question |
| --- | --- |
| PDU header construction | Which byte currently selects `ADV_NONCONN_IND`, and what is the minimal change to `ADV_IND`? |
| State stamp | Does current code stamp a legacy ADV state that affects downstream RX enablement? |
| Payload length | Does changing PDU type affect header length or AdvA placement? |
| RX setup | Keep RX disabled in this task. If hardware requires post-TX RX arm for legal `ADV_IND`, record that hardware constraint without handling SCAN_REQ. |

## Expected Result

The task is successful when Rust emits stable legacy `ADV_IND` with the current AdvA/payload. That gives SCAN_REQ and CONNECT_IND a valid target in later stages.

## Review Questions

1. Baseline sequencing: wait for #80 Path A to restore cold ADV, or pivot to an older working cold ADV branch?
2. Exact code base after that sequencing decision.
