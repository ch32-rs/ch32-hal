# CH32 BLE Notes Index

Status: canonical retrieval index, 2026-05-14.

This directory is the working knowledge base for the CH32V208 BLE Rust Link Layer effort. Keep existing file paths stable during Step 2 so attempt logs and Slock references stay valid. Add structure through this index first; directory moves can happen after the SCAN_RSP air path is stable.

## Start Here

| Need | Read |
| --- | --- |
| Current route and paused lanes | `ble-followup-roadmap.md` |
| Branches, tags, and worktree status | `branch-status.md` |
| Physical flashing / reset / debug guide | `../physical-debugging-guide.md` |
| Embassy BLE stack logic-port architecture plan | `embedded-c-ble-stack-architecture.md` |
| Step 2 SCAN_RSP implementation contract | `scan-rsp-implementation-spec.md` |
| Connectable ADV baseline plan | `connectable-adv-baseline-spec.md` |
| Current attempts, failures, tags, and next candidate | `task79-attempt-log.md` |
| Timing/event model and measurement rules | `time-event-model.md` |
| EVT/TI source comparison and broadcaster correction | `evt-broadcaster-ti-flow.md` |

## Current Locks

1. **Broadcaster evidence scope**: Rust Broadcaster and EVT Broadcaster prove cold ADV TX / init / hot-loop behavior. SCAN_RSP work uses EVT Peripheral plus `libwchble.a` warm RX->TX disassembly.
2. **SCAN_RSP warm path**: `ll_advertise_legacy_rx -> ll_advertise_generated_scan_rsp -> BLE_SetPHYTxMode -> ll_tx_wait_finish(mode=3) -> ble_ll_hw_api_shut`.
3. **Warm kick register shape**: `ll_tx_wait_finish(mode=3)` sets `BB[0] |= 0x800000` and clears `BB[44] & 0x3`. Cold ADV TX uses the `mode=0` path and `LLE[0] = 2`.
4. **Measurement model**: fine timing uses SysTick CNTL `0xE000_F008` at 12 MHz; coarse LL timing uses `bleClock_t` 625 us ticks; pcap/phone scanner validate protocol visibility.
5. **Production direction**: WCH ROM/lib hooks are research inputs. The implementation target is Rust-owned BLE LL code.
6. **Attempt preservation**: every failed hardware path gets an evidence row in `task79-attempt-log.md` plus a git commit/tag when code changed.

## Taxonomy

### Plan

| Doc | Role |
| --- | --- |
| `ble-followup-roadmap.md` | Overall BLE route: Step 1 timing model, Step 2 SCAN_RSP, later scheduler/connection work. |
| `branch-status.md` | Active branch, final target branch, archived attempt tags, and worktree inventory. |
| `../physical-debugging-guide.md` | Field guide for flashing methods, reset boundaries, Wlink/OpenOCD/GDB/probe-rs commands, and physical evidence rules. |
| `embedded-c-ble-stack-architecture.md` | Architecture plan for porting TI/WCH/TMOS logic into Rust/Embassy-owned modules: ISR boundary, Embassy event engine, global radio scheduler, LL/GAP/GATT state machines, and validation gates. |
| `connectable-adv-baseline-spec.md` | task #81 plan: prove stable legacy `ADV_IND` before SCAN_REQ/SCAN_RSP work. |
| `scan-rsp-implementation-spec.md` | Development mode, baseline, verification gates, and implementation scope for Step 2. |
| `time-event-model.md` | ADV-only time/event contract that feeds Step 2 and later scheduler design. |

### Research

| Doc | Role |
| --- | --- |
| `peripheral-flow-deep-dive.md` | TI/WCH Peripheral example flow and semantic map. |
| `evt-broadcaster-ti-flow.md` | Rebuilt EVT Broadcaster/Peripheral source comparison, TI flow reference, and SCAN_RSP correction. |
| `../libwchble-tmos-ll-disasm.md` | Older TMOS/LL disassembly notes kept as historical reference. |

### Decisions

| Decision | Primary Anchor |
| --- | --- |
| Broadcaster evidence feeds cold ADV control; Peripheral evidence feeds SCAN_RSP | `ble-followup-roadmap.md`, `scan-rsp-implementation-spec.md`, `evt-broadcaster-ti-flow.md` |
| Step 2 starts from the preserved control line and records every hardware attempt | `scan-rsp-implementation-spec.md`, `task79-attempt-log.md` |
| Current Step 2 branch is `feat/ble-scan-rsp-step2`; final target remains `feat/ble` | `branch-status.md` |
| task #81 starts with connectable `ADV_IND` visibility before SCAN_REQ / SCAN_RSP | `connectable-adv-baseline-spec.md` |
| BL-5 `ch32-data 2d9eef8` audit is a final merge gate | `ble-followup-roadmap.md` |
| Task #75 skeleton is staged and absorbed boundary-by-boundary | `ble-followup-roadmap.md`, `scan-rsp-implementation-spec.md` |

### Pitfalls / Attempt Logs

| Doc | Use |
| --- | --- |
| `task79-attempt-log.md` | Canonical Step 2 attempt log: baseline, failed variants, tags, evidence, and current next candidate. |
| `time-event-model.md` §6 | Misconception ledger: `timeUs`, `gBleLlPara[0x7c]`, warm kick, IRQ controller split, filter mutation. |
| `scan-rsp-implementation-spec.md` §0 | Guardrail for cold ADV vs warm SCAN_RSP wording. |
| `evt-broadcaster-ti-flow.md` §7 | Correction record for Broadcaster vs Peripheral evidence. |

## LL Disassembly Notes (in-tree)

Focused LL reverse-engineering notes are now tracked in this directory. They originated in Lucy's agent workspace and were migrated under task #90 (`#ch32-rs:decce4f6`, branch `lucy/disasm-migration`); each file carries a provenance footer recording origin path, migration date, responsible agent, and the behavioral-reference-only boundary. Treat them as observed-behavior source inputs — implementation work may cite them but must not link `libwchble` symbols or call ROM helpers as a production fallback (see `embedded-c-ble-stack-architecture.md` §2.5):

| In-tree Doc | Role |
| --- | --- |
| [`ll-advertise-tx-disasm.md`](./ll-advertise-tx-disasm.md) | Cold ADV TX MMIO and PDU assembly. |
| [`ll-tx-completion-disasm.md`](./ll-tx-completion-disasm.md) | TX/RX wait boundaries and event close. |
| [`ll-adv-scheduler-disasm.md`](./ll-adv-scheduler-disasm.md) | ADV scheduler, channel hop, interval/random delay. |
| [`ll-process-event-minipass.md`](./ll-process-event-minipass.md) | `LL_ProcessEvent` mask dispatch. |
| [`ll-rx-ingress-disasm.md`](./ll-rx-ingress-disasm.md) | SCAN_REQ / CONNECT_IND split and SCAN_RSP prep. |
| [`ll-phy-preflight-disasm.md`](./ll-phy-preflight-disasm.md) | PHY setup, dual TX kick, RF constants. |
| [`ll-slave-init-disasm.md`](./ll-slave-init-disasm.md) | CONNECT_IND first hop and slave init. |
| [`ll-advertise-filter-disasm.md`](./ll-advertise-filter-disasm.md) | Filter policy and resolving-list mutation. |

## Lookup Guide

| Question | Where to Look |
| --- | --- |
| What is the next BLE implementation step? | `ble-followup-roadmap.md`, then `scan-rsp-implementation-spec.md`. |
| How do we prove the Peripheral entry point? | `connectable-adv-baseline-spec.md`. |
| Why did the working broadcaster fail to settle SCAN_RSP? | `evt-broadcaster-ti-flow.md`, then `scan-rsp-implementation-spec.md` §0. |
| Which hardware paths have already failed? | `task79-attempt-log.md`. |
| Which branch should I use? | `branch-status.md`. |
| How should a full Embassy-owned BLE stack be structured? | `embedded-c-ble-stack-architecture.md`. |
| Which time source should probes use? | `time-event-model.md` §8. |
| Which WCH function owns SCAN_RSP warm kick? | [`ll-phy-preflight-disasm.md`](./ll-phy-preflight-disasm.md) and [`ll-rx-ingress-disasm.md`](./ll-rx-ingress-disasm.md). |

## Housekeeping Backlog

1. ~~Migrate Lucy's focused disassembly docs from the agent workspace into this repo after Step 2 stabilizes.~~ Done under task #90, 2026-05-18 (commit on `lucy/disasm-migration`).
2. Later split the directory into `plan/`, `research/`, `decision/`, and `pitfall/` if the note count keeps growing.
3. Keep `README.md` as the single entry point after any future move.
