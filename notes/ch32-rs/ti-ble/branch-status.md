# CH32 BLE Branch Status

Status: working branch index, 2026-05-14.

Purpose: keep branch choice, air evidence, and preserved bad states visible while the BLE work moves through SCAN_RSP Step 2. Update this file whenever a branch becomes a new implementation base, a bad/probe state is archived, or a task changes the recommended next branch.

## Active Branches

| Branch / Ref | Commit | Role | Current Status | Primary Evidence |
| --- | --- | --- | --- | --- |
| `feat/ble-scan-rsp-step2` | `c193349` | Current task #79/#80 implementation branch in the main repo. | In review after Path B post-GO IRQ enable failed; next useful direction is Path A / PATHC once-at-init or direct `ll_tx_wait_finish(mode=0)` cold-TX shape alignment. | `task79-attempt-log.md`; tags `bad/scan-rsp-step2-*`; pcap/serial artifacts under `/tmp/task79_scan_rsp_step2/`. |
| `feat/ble` | `46de89c` | Long-term target branch. | Staging target after Step 2 air evidence and BL-5 audit. | `ble-followup-roadmap.md`; BL-5 typed-fieldset/codegen audit. |
| `feat/ble-ll-skeleton-first-slice` | `b230a88` | Task #75 LL skeleton/types branch. | Staged review artifact; absorb boundary-by-boundary after SCAN_RSP smoke. | task #75; `/private/tmp/ch32-task75-known`. |
| `wip/scan-rsp-mcycle-fix-20260511` | `7f04104` | SysTick timeout baseline that exposed the SCAN_RSP/baseline issue. | Reference point; fresh pcap control showed unreliable/no air. | Tags `bad/scan-rsp-systick-fix-low-adv-rate-20260511-7f04104`, `bad/scan-rsp-step2-baseline-7f04104-unreliable-20260514`. |
| `work/ble-peripheral-phase1-delta-y-explicit` | `98cd7d7` | Known ADV-visible reference branch. | Research/reference branch for prior phase-1 visibility. | Tag `known-good/phase1-delta-y-explicit-adv-visible-20260510`. |

## Target / Staging Decisions

| Decision | Current Value | Reason |
| --- | --- | --- |
| Day-to-day working branch | `feat/ble-scan-rsp-step2` in `/Users/mono/Elec/WCH/ch32-hal` | Main repo branch keeps disk usage contained and preserves current attempts. |
| Final merge target | `feat/ble` | This is the final BLE branch after SCAN_RSP air evidence and BL-5 cleanup. |
| Type skeleton source | `feat/ble-ll-skeleton-first-slice @ b230a88` | It contains `src/ble/ll/{consts,types,signatures}.rs` and stays staged until a boundary needs it. |
| Attempt log source of truth | `task79-attempt-log.md` | Chat threads can compact; this file preserves failed paths, tags, and reasoning. |

## Archived / Evidence Branches

| Branch / Tag Class | Meaning | Current Use |
| --- | --- | --- |
| `bad/scan-rsp-step2-*` | Failed Step 2 hardware variants, each tied to pcap/serial evidence. | Read before proposing a new SCAN_RSP attempt. |
| `probe/scan-rsp-step2-*` | Marginal or diagnostic variants with partial signal. | Use for mechanism evidence; avoid promoting as a baseline. |
| `inconclusive/scan-rsp-step2-*` | Test setup or control-gate uncertainty snapshots. | Preserve environmental context around sniffer/air observability. |
| `bad/phase-c-*` | Earlier task-side RX/SCAN_RSP attempts. | Historical pitfalls for post-GO task starvation and helper timing. |
| `t8-attempt-*` / `bad/t8-*` | Earlier lib dependency and `fnGetClockCBs` removal experiments. | Historical branch hygiene only; current SCAN_RSP route uses newer docs and attempt log. |

## Step 2 Attempt Chain

| Commit / Tag | Variant | Result | Next Lesson |
| --- | --- | --- | --- |
| `7f04104` / `bad/scan-rsp-step2-baseline-7f04104-unreliable-20260514` | Fresh control on SysTick timeout baseline. | Sniffer restored, target air absent. | `7f04104` is an unreliable control for SCAN_RSP work. |
| `df7c31f` / `bad/scan-rsp-step2-gate-patch-no-air-20260514` | Gate `rx_turnaround_capture_ch37()` on `irq == 1`. | 0 target frames. | `ip4 == 1` did not fire in this path. |
| `a2c75f1` / `bad/scan-rsp-step2-a1-mode1-align-no-air-20260514` | A1 mode=1 RX-prep alignment. | 0 target frames. | Inner RX-prep alignment alone did not recover air. |
| `1ddb1f5` / `bad/scan-rsp-step2-outer3-min-sdi-no-air-20260514` | OUTER three-source wait + minimal SDI. | 0 target frames. | `gBleIPPara[2]/[3]` did not fire; `bb64==0` ended the wait immediately. |
| `6e27c3a` / `probe/scan-rsp-step2-ip4-50ms-marginal-air-20260514` | `ip4` wait extended to 50 ms. | Marginal leakage: 1 target ADV in 60 s runs. | Longer wait reduces W1C pressure, but completion signal still fails. |
| `c193349` / `bad/scan-rsp-step2-postgo-irq-no-air-20260514` | IRQ63 enable moved after GO. | 0 target frames. | Pre-GO IRQ setup is part of the working cold-TX shape; moving it after GO breaks the sequence. |

## Worktree Inventory

| Path | Branch / Commit | Status |
| --- | --- | --- |
| `/Users/mono/Elec/WCH/ch32-hal` | `feat/ble-scan-rsp-step2 @ c193349` | Main working tree. |
| `/private/tmp/ch32-task75-known` | `feat/ble-ll-skeleton-first-slice @ b230a88` | Task #75 skeleton staging worktree. |
| `/private/tmp/ch32-deltaY-explicit` | `98cd7d7` detached | Prior ADV-visible reference worktree. |
| `/private/tmp/ch32-scan-rsp-reply` | `wip/scan-rsp-mcycle-fix-20260511 @ 7f04104` | Historical reference worktree. |
| `/private/tmp/ch32-scan-rsp-q2-df26837` | `v4c-tx-complete-wait @ 6e53775` | Historical reference worktree. |
| `/private/tmp/ch32-hal-*`, `/private/tmp/wt_*`, `/private/tmp/ch32_phasec_*` | Detached / archived | Cleanup candidates after confirming no active task uses them. |

## Branch Hygiene Rules

1. Keep `feat/ble-scan-rsp-step2` as the only active Step 2 code branch until Andelf changes direction.
2. Preserve failed code states with `bad/*` commits/tags before changing direction.
3. Record every branch promotion or retirement in this file and `task79-attempt-log.md`.
4. Use `git worktree list --porcelain` before creating a new worktree; prefer the main repo branch for Step 2 to control disk usage.
5. Run `git worktree prune --dry-run` before any cleanup; prune only after the owning task confirms the worktree is historical.
