# ADV-only Time and Event Model — TMOS dispatch, WCH LL inner loop, 625 µs scheduler

This doc consolidates the WCH `libwchble.a` ADV-path time and event model and aligns it
against TI BLE-Stack semantics. It is the model-level companion to the disassembly
notes; it owns the misconception ledger and the boundary that separates "what we have
reverse-engineered" from "what Phase 2 air run still needs to measure".

Scope: **ADV path only** (legacy advertising, ADV_IND + SCAN_REQ/SCAN_RSP turnaround).
Connection-event timing and master/scanner timing are out of scope — they belong in
the Step 4 doc.

Status: **review-ready** — §1-§8 complete; §8 probe DoD finalized; Day 5
cross-check pass log appended at the end of the doc (mask/bit consistency,
TI §7.7.2 inference boundary, §8 → Step 2 probe DoD convertibility). Source
disassembly notes referenced below currently live in the Lucy agent
workspace (`notes/ch32-rs/ti-ble/*.md`); migration into the ch32-hal repo
`notes/` tree is a follow-up administrative item, not a blocker for this
doc.

## Source inputs

| Doc | Used for |
| --- | --- |
| `ll-process-event-minipass.md` (M1+M2) | §1 + §2 dispatch table |
| `ll-tx-completion-disasm.md` (§4 + §5) | §1 inner loop + §5 timeUs semantics |
| `ll-adv-scheduler-disasm.md` (§4 + §10) | §1 state machine + §4 budget |
| `ll-advertise-tx-disasm.md` (§7 + §15) | §1 stamping behavior |
| `ll-rx-ingress-disasm.md` | §6 misconception ledger entries |
| `ti-wch-mapping.md` §2 + §13 | §3 + §5 + §6 |
| `c-ground-truth-fnGetClockCBs.md` | §3 fnGetClockCBs link-time-determined evidence |
| `wch-ble-manual.md` (PDF dump) | §3 + §4 WCH-side timing language |
| TI doc §7.7.2 (event-budget concept) | §4 |
| Iron Rules §0/§14-17/§30/§35-37 | enforcement anchors used inline |

## §1 — TMOS event dispatch vs `ll_advertise_process` synchronous inner loop

### 1.1 The two-layer model

The WCH ADV path has **two distinct execution layers** that are easy to confuse:

1. **Outer layer — TMOS event dispatch (preemptible by other tasks)**:
   - TMOS scheduler polls `LL_ProcessEvent(taskId, events)` when an event bit fires.
   - One bit-per-iteration, return value clears the handled bit (`events ^ bit`).
   - Other tasks (HCI, app, OSAL) interleave between LL event handlings.
   - Anchor: `LL_ProcessEvent` 8-bit dispatch — see §2 below.

2. **Inner layer — `ll_advertise_process` synchronous state machine (runs to completion)**:
   - Once `ll_advertise_tx` is entered (via outer dispatch), it tail-calls
     `ll_advertise_process` which **does not return until the entire ADV event
     completes** (all channels in `[byte_10, byte_25, byte_26]` triple consumed,
     or shutdown gate hits).
   - Inside: busy-wait TX done (3-source: `gBleIPPara[2]&1 || gBleIPPara[3]&1 ||
     LLE[100]==0`), `.L440` 13-entry jump table on `byte_11` (stamp byte),
     RX window setup, SCAN_RSP turnaround, channel hop, then either
     `tail-call ll_advertise_event_closed` (event done, user cb) or
     `tail-call tmos_set_event(task, 2)` (no full close — re-arm next slot).
   - Anchor: `ll-adv-scheduler-disasm.md` §4 + `ll-tx-completion-disasm.md` §2.

### 1.2 Concrete sequence per ADV event

```
TMOS tick: events |= 0x02
   │
   ▼  (TMOS scheduler decides, may run other tasks first)
LL_ProcessEvent(LL_task, events):
   if events & 0x02 && gBleLlPara[0x74] != NULL:
       indirect-call gBleLlPara[0x74]            ; = &llAdvTraverseallChannel
       return events ^ 0x02                       ; clear bit-1
   │
   ▼  (we are now in the "inner" synchronous run)
llAdvTraverseallChannel(advState = gBleLlPara[0x64]):
   if advState.byte_12 != 1: return               ; ADV inactive — clean shutdown
   if gBleLlPara[0x7c] == 4:
       ll_advertise_event_closed(advState)        ; finalize previous event
   tail-call ll_advertise_tx(advState)
   │
   ▼
ll_advertise_tx:
   stamp advState.byte_11 = 0x92/0x93/0x94/... (per PDU type)
   write BB / LLE / RFEND for this channel
   kick TX (LLE[0] = 2)
   tail-call ll_advertise_process(advState)
   │
   ▼  (this loop is the WHOLE rest of the event)
ll_advertise_process:
   loop:
     busy-wait TX done (3-source)
     dispatch on byte_11 via .L440:
        0x91 → shut path
        0x92 → DIRECT done
        0x93 → SCAN_RSP done
        0x94 → ADV_IND done + RX window open
        ... (13 entries indexed (byte_11 + 110) & 0xff)
     either:
       - hop to next channel in triple → re-enter ll_advertise_tx via .L450 jalr
       - or close: tail-call ll_advertise_status_closed
   │
   ▼
ll_advertise_status_closed:
   if byte_10 == byte_26 (last channel done):
       tail-call ll_advertise_event_closed → user cb(timeUs) + arm next event
   else:
       tail-call tmos_set_event(LL_task, 2)        ; re-enter outer dispatch
```

**Key invariant**: TMOS event `mask 0x0002` (bit 1) is the **only event-driven entry** into the
ADV cycle after `llAdvertiseStart` does the initial kick via `mask 0x0001` (bit 0).
`ll_advertise_process` is **not** an event handler — it is reachable only as the
tail of `ll_advertise_tx` (or `ll_advertise_aux_tx` / `ll_advertise_aux_chain_tx`
for ext-adv, out of scope).

### 1.3 Common misconception — `byte_11` is not a "resume point"

A natural reading of "`ll_advertise_process` dispatches on `byte_11` via a jump
table" is that `byte_11` is a saved program counter / resume point for an
interrupted state machine.

**It is not.** `byte_11` is **re-stamped by `ll_advertise_tx` on every channel TX**
to indicate what kind of PDU was just kicked off. `ll_advertise_process` reads it
to know what completion path applies to the TX it just kicked. The state is
*forward-stamped*, not *resume-saved*.

Practical consequence for Rust port: do not model `byte_11` as a "where am I in
the state machine" field. Model it as a *post-TX classification* set by the TX
issuer, consumed by the completion handler.

Anchor: `ll-adv-scheduler-disasm.md` §4 "byte_11 is forward-stamped per channel
TX, not a resume point"; `ll-advertise-tx-disasm.md` §7.

### 1.4 Why this matters for embassy / Rust scheduler choice

The WCH design assumes:

- The outer (TMOS) layer is **cooperative** but driven by a polling event mask.
- The inner (ll_advertise_process) layer is **synchronous** and may busy-wait for
  TX/RX completion across the entire ADV event.

A naive embassy-style port that turns each TX/RX completion into an async point
would break the inner-loop semantics — especially the SCAN_RSP warm turnaround
where the inner loop must hold across an RX window and a follow-up TX without
yielding. Step 3 will lock the Rust event model after Phase 2 air run validates
this constraint experimentally; for now, **first-air must preserve the inner
synchronous loop** (Iron Rule §0 from `ll-adv-scheduler-disasm.md`).

### 1.5 §1 evidence summary

| Claim | Source | Status |
| --- | --- | --- |
| Outer dispatch is TMOS event `mask 0x0002` (bit 1) → `gBleLlPara[0x74]` | `ll-process-event-minipass.md` M1 (LL_ProcessEvent.dump L78-89) | VERIFIED |
| `gBleLlPara[0x74]` is populated only by `LL_AdvertiseEnalbe`, read only by `LL_ProcessEvent` | `ll-process-event-minipass.md` M2 (cross-69-obj xref) | VERIFIED |
| `llAdvTraverseallChannel` guards on `byte_12 == 1` then calls `ll_advertise_event_closed` if `gBleLlPara[0x7c]==4`, then tail-calls `ll_advertise_tx` | `ll-process-event-minipass.md` §M2 (llAdvTraverseallChannel_v2.dump) | VERIFIED |
| `ll_advertise_process` is reachable only from `ll_advertise_tx`, `ll_advertise_aux_tx`, `ll_advertise_aux_chain_tx` (5 sites total incl. periodic/sync) | `ll-process-event-minipass.md` §M2 paradox resolution | VERIFIED |
| `byte_11` is forward-stamped per TX, not resume-saved | `ll-adv-scheduler-disasm.md` §4, `ll-advertise-tx-disasm.md` §7 | VERIFIED |
| Inner loop completes the full ADV event synchronously, returning to outer only via `ll_advertise_event_closed` or `tmos_set_event` re-arm | `ll-tx-completion-disasm.md` §2 + §5 | VERIFIED |

## §2 — `LL_ProcessEvent` event-mask dispatch table

### 2.1 Full table (8-bit dispatch + 2 high-bit specials)

`LL_ProcessEvent(taskId a0, events a1)` reads bits of `events` and for each
set bit dispatches via `gBleLlPara` vtable slots, returning `events ^ <bit>`
so TMOS clears the handled bit.

**Notation lock for this doc**: every event uses **both** `mask 0xNNNN` and
`bit N` notation, where `bit N` is the LSB-0 bit-position of the single set
bit in the mask. This avoids the historical confusion where "event bit 2"
was sometimes used to mean `mask 0x02` (bit 1) and sometimes to mean
`mask 0x04` (bit 2). Always read both labels together.

| Mask | Bit | Vtable read | Target symbol | Subsystem |
| --- | --- | --- | --- | --- |
| `0x0001` | bit 0 | `gBleLlPara[0x70]` (`lw 112`) | `llAdvertiseStart` | ADV — initial kick |
| `0x0002` | bit 1 | `gBleLlPara[0x74]` (`lw 116`) | `llAdvTraverseallChannel` | **ADV — per-event re-entry (the one §1 describes)** |
| `0x0004` | bit 2 | `gBleLlPara[0x80] → +0x6c` (nested vtable) | conn/slave start | CONN — start |
| `0x0008` | bit 3 | `gBleLlPara[0x80] → +0x70` (nested vtable) | conn/slave process | CONN — per-event |
| `0x0010` | bit 4 | `gBleLlPara[0x94] → +0x8c` (nested vtable) | initiate / master | MASTER — start |
| `0x0020` | bit 5 | `gBleLlPara[0x94] → +0x90` (nested vtable) | scan path | SCAN |
| `0x4000` | bit 14 | `LL_TransmitterTest` direct | DTM TX test | DTM |
| `0x8000` | bit 15 | `tmos_msg_receive` (SYS_EVENT_MSG) | OSAL inbox | OSAL |

Anchor: `ll-process-event-minipass.md` M1 (full table verified against
`LL_ProcessEvent.dump` line 17-214).

### 2.2 ADV-relevant rows (Step 1 focus)

For the ADV path, two rows matter:

- **`mask 0x0001` (bit 0) → `llAdvertiseStart`**: One-shot initial kick when the
  host enables ADV. Programs first channel's BB/LLE/RFEND, kicks TX, then enters
  the inner loop. From `ll-adv-scheduler-disasm.md` §5, this is the function that
  arms the very first event but does **not** re-fire on subsequent events.

- **`mask 0x0002` (bit 1) → `llAdvTraverseallChannel`**: Re-entry for every
  subsequent ADV event. Tail-calls `ll_advertise_tx` after the active-flag and
  op-mode guards. See §1.2 sequence.

The split exists because `llAdvertiseStart` does additional one-time work
(populating `gBleLlPara[0x64]` with the advState pointer, arming TMOS timers,
optional `advDelay` randomization) that subsequent events do not repeat.

### 2.3 NULL-guard contract (Iron Rule §16)

Every dispatch site in `LL_ProcessEvent` has the shape:

```
lw   a5, <offset>(gBleLlPara)
beqz a5, .Lnext            ; NULL guard
jalr a5                    ; indirect call
```

Implication for the Rust port: the equivalent Rust scheduler must treat each
vtable slot as `Option<fn>` (or equivalent), skip cleanly when uninitialized,
and not panic. This matters during teardown — e.g. if the host calls
`LL_AdvertiseShut` and the vtable slot is cleared mid-tick, the dispatch must
no-op rather than crash.

### 2.4 Active-flag gate (Iron Rule §17)

Separate from the NULL guard above, `llAdvTraverseallChannel` does a hard
`advState.byte_12 == 1` check at its top, returning a no-op if not set. This
is the de-facto "stop" path when host has called `LL_AdvertiseShut` mid-cycle
(which clears `byte_12`) but the vtable slot has not yet been NULL'd. The
two guards layer:

```
guard 1: gBleLlPara[0x74] != NULL  (slot populated by LL_AdvertiseEnalbe)
guard 2: advState.byte_12 == 1     (ADV currently active)
```

A Rust port that has both guards (slot + active flag) matches the WCH
shutdown sequencing without races.

### 2.5 §2 evidence summary

| Claim | Source | Status |
| --- | --- | --- |
| 8-bit dispatch table (rows 0/1/2/3/4/5/14/15) | `ll-process-event-minipass.md` M1 | VERIFIED |
| `gBleLlPara[0x74]` populated only by `LL_AdvertiseEnalbe` | `ll-process-event-minipass.md` M2 | VERIFIED |
| `gBleLlPara[0x74]` read only by `LL_ProcessEvent` | `ll-process-event-minipass.md` M2 cross-69-obj xref | VERIFIED |
| `gBleLlPara[0x70]` (mask 0x0001) is one-shot initial kick | `ll-adv-scheduler-disasm.md` §5 | VERIFIED |
| Mask 0x0001 path differs from mask 0x0002 path | `ll-adv-scheduler-disasm.md` §5 + minipass M1 | VERIFIED |
| NULL guard + active-flag gate layered | minipass §M2 disasm | VERIFIED |

## §3 — `bleClock_t` + `fnGetClockCBs`: 1600 Hz / 625 µs time base

### 3.1 The single LL time base

The WCH BLE LL has **one canonical time base** for all its scheduler bookkeeping:

- **LSB** = `625 µs` = `0.625 ms`
- **Frequency** = `1600 Hz` (since `1 / 625 µs = 1600 Hz`)
- **Source** = 32768 Hz LSE/RTC ÷ 20.48 ≈ 1600 Hz tick

This base is used by `tmos_start_task`, `tmos_start_reload_task`,
`tmos_get_task_timer`, and the `tmos_get_systemClock()` system-uptime accessor.
WCH BLE manual (`wch-ble-manual.md`) is explicit:

- Line 242: *"TMOS 通过轮询的方式进行调度, 系统时钟一般来源于 RTC, 单位为 625μs"*
- Line 1567: *"延迟 `time*625μs` 后启动 taskID 任务中对应的 event 事件"*
- Line 1627: *"返回 tmos 系统运行时长, 单位为 625μs, **如 1600=1s**"*

### 3.2 TI BLE-Stack heritage, not WCH hardware fixity

The 1600 Hz / 625 µs cadence is **inherited from TI BLE-Stack**, not a property
of CH32V208 hardware:

- TI doc swru271i (TI BLE-Stack 1.x/3.x): all timing constants use `n × 0.625 ms`
  units. `osal_start_timerEx(taskID, evt, ms)` takes ms; internally converts to
  625 µs ticks.
- TGAP_* defaults (TI doc table + WCH manual line 1805-1824): all advertising
  and scan constants are in 0.625 ms units; connection constants in 1.25 ms
  units (2× the LSB, matching BLE Core spec connection-interval encoding).
- WCH replaced TI's heap-allocated OSAL timer storage with `fnGetClockCBs`, a
  COMMON-storage indirection that lets the bring-up code patch the clock-tick
  source without recompiling the LL.

Anchor: `ti-wch-mapping.md` §2 + `c-ground-truth-fnGetClockCBs.md` (2026-05-09
C-side empirical proof of link-time-determined COMMON symbol).

### 3.3 `fnGetClockCBs` — link-time-determined, not hardware-fixed

The `fnGetClockCBs` symbol is treated by some downstream WCH boards as a
"magic ROM address". This is **false**.

Evidence (from `c-ground-truth-fnGetClockCBs.md`):

- `libwchble.a` declares `fnGetClockCBs` as a **COMMON symbol** (4 bytes,
  uninitialized) with PCREL_HI20 relocations from every LL caller.
- The final address is chosen by the linker based on the OSAL heap layout
  (`MEMHEAP` symbol size).
- Empirical: EVT Broadcaster build with `MEMHEAP=6144` → `fnGetClockCBs` at
  `0x20002420`; with `MEMHEAP=7168` → `0x20002820` (delta = +0x400 = exactly
  the MEM_BUF growth).
- The historical `link.x . = 0x20001c78` pin in some WCH examples is
  **archaeological**, not a required hardware fact.

Implication for the Rust port: `fnGetClockCBs` is just a function pointer slot
that the bring-up code populates. Rust port can use any equivalent
indirection (e.g. a `&'static dyn Fn() -> u32` or a typed register accessor)
without simulating the COMMON-symbol address quirk.

### 3.4 Distinct from SysTick and raw RTC

Three time bases co-exist on CH32V208; they must not be confused:

| Time base | Source | Unit | Used by |
| --- | --- | --- | --- |
| **`bleClock_t` (THIS doc's focus)** | 32768 Hz LSE ÷ 20.48 | 625 µs / 1600 Hz | LL scheduler, TGAP_*, ADV/CONN intervals, user cb `timeUs` |
| SysTick | CPU clock-derived | 1 / SysClock | embassy timers, `delay_us`, app-level scheduling |
| Raw RTC | 32768 Hz LSE | 30.5 µs | RTC alarms, very-low-power wake, calendar |

WCH BLE manual line 1535 confirms: *"pfnGetSysClock 0：选择 RTC 作为系统时钟"*
— the BLE stack uses the RTC-derived 1600 Hz tick when configured. Line 1393:
*"在低功耗蓝牙工作期间，需要通过 RTC 定时器计算时间"*.

### 3.5 Rust port anchoring rule

The Rust LL **must anchor "time" to the 1600 Hz / 625 µs base**, not SysTick:

1. All ADV/CONN interval constants stored in 625 µs ticks (matching WCH `n × 0.625 ms`).
2. The LL scheduler's "now" comes from a `bleClock_t` equivalent
   (`fn now_ble_ticks() -> u32` ≡ WCH `tmos_get_systemClock()`), not from
   `embassy_time::Instant`.
3. embassy timers may be used as **wake sources** for the LL scheduler tick,
   but the LL's bookkeeping unit is always `bleClock_t` ticks.
4. User-visible APIs that take/give time (e.g. `event_close_cb(time_remaining)`)
   use 625 µs units for parity with WCH semantics. Conversion to ms or µs
   happens at the boundary, not inside.

This is why Step 1 anchors the model before Step 3's abstraction lock: if the
Rust scheduler picks SysTick as its "now", the LL has to do conversion on
every TGAP read and every `timeUs` computation, accumulating rounding error.

### 3.6 §3 evidence summary

| Claim | Source | Status |
| --- | --- | --- |
| LL time base = 625 µs / 1600 Hz | `wch-ble-manual.md` line 242, 1627 | VERIFIED (vendor doc) |
| Source is RTC-derived (32768 Hz ÷ 20.48) | `wch-ble-manual.md` line 1535 + 1393 | VERIFIED (vendor doc) |
| The 625 µs unit appears identically in both TI swru271i (`n × 0.625 ms` convention) and WCH manual / disasm | `ti-wch-mapping.md` §2 + swru271i `n × 0.625 ms` + WCH manual line 242, 1627 | VERIFIED (both sides) |
| WCH's 625 µs is *historically inherited* from TI BLE-Stack culture (not CH32V208 hardware fixity) | `ti-wch-mapping.md` §2 + swru271i lineage; WCH-side intent not directly stated in any single source | INFERRED (model-level conclusion from convention match + RV32IMC port lineage; no single vendor statement asserts heritage) |
| `fnGetClockCBs` is link-time-determined COMMON symbol | `c-ground-truth-fnGetClockCBs.md` (MEMHEAP=6144 vs 7168 empirical) | VERIFIED |
| All TGAP_* ADV/CONN constants use 0.625 ms / 1.25 ms units | `wch-ble-manual.md` line 1805-1824 + swru271i §TGAP | VERIFIED |
| LL bookkeeping reads time from `bleClock_t` (1600 Hz) — not SysTick — in the disasm sites we have examined | `wch-ble-manual.md` line 1393 + disasm xref (`ll_advertise_*`, `tmos_get_task_timer`) | VERIFIED (within examined disasm scope) |
| 1600 Hz is the **only** LL time base across the whole LL (no co-existing SysTick path anywhere) | analytical extrapolation from the examined sites; full-LL exhaustive search not performed | INFERRED (model-level conclusion; converts to VERIFIED if a future audit scans all LL objects for SysTick reads and finds none) |

## §4 — TI §7.7.2 event budget applied to WCH ADV

This section maps TI's §7.7.2 "controller margin" concept onto the WCH ADV
path. **Cindy review-flag**: this is the section most prone to over-inference.
Each mapping claim below is explicitly tagged `VERIFIED` (both sides have
disasm/doc evidence) or `INFERRED` (TI concept is documented; WCH side
matches by analogy but needs Phase 2 on-chip trace probe to confirm).

### 4.1 TI §7.7.2 verbatim (swru271i, lines 2300-2308)

> "Because of the time-dependent nature of the Bluetooth Low Energy protocol,
> the controller (`LL_ProcessEvent()`) must process before each connection
> event or advertising event. If the controller does not get process,
> advertising restarts or the connection drops. Because OSAL is not
> multithreaded, each task must stop processing to let the controller
> process. The stack layers do not have this issue. Ensure that the
> application processes less than the following:
> **`(connection/advertising interval) – 2 ms`**.
> The 2 ms are added as buffer to account for controller processing time.
> If extensive processing is required in the application task, split it
> up using OSAL events..."

Three operational rules from this paragraph (TI-side, VERIFIED in doc):

1. **R-TI-1**: `LL_ProcessEvent()` must run before each event boundary.
2. **R-TI-2**: OSAL is non-multithreaded; app cannot preempt controller.
3. **R-TI-3**: App task duration < `(event_interval) - 2 ms`; the 2 ms is
   the controller-processing buffer.

### 4.2 WCH-side analog (per disasm)

WCH inherits the OSAL non-multithreaded model (`wch-ble-manual.md` §TMOS).
Concretely:

- **WCH-R-1 (VERIFIED)**: The LL's event-driven entry on the ADV path is
  `LL_ProcessEvent(taskId, events)` dispatched by TMOS. This is the exact
  same function name and same dispatch shape as TI (`ll-process-event-minipass.md`
  M1).
- **WCH-R-2 (VERIFIED)**: TMOS dispatches one event-bit per call, and
  cooperatively yields between tasks. Same non-multithreaded model as TI
  OSAL.
- **WCH-R-3 (INFERRED, Phase 2 confirms)**: The "2 ms buffer" maps to the
  span between `ll_advertise_event_closed` firing user cb and the next
  TMOS dispatch of `mask 0x0002` → `llAdvTraverseallChannel`. This span
  is implicit in WCH's scheduler; it is NOT a documented constant and
  has NOT been verified by disasm.

### 4.3 Where does the "controller_margin" live in WCH code?

This is the crux of §4 and the part Cindy flagged. The honest answer:
**we have not located a single constant in WCH disasm that corresponds to
TI's 2 ms buffer**. What we have:

- `ll_advertise_event_closed` computes `timeUs = tmos_get_task_timer(task, 1) * 625`
  and calls user cb with it (`ll-tx-completion-disasm.md` §4 + §5 of this
  doc when written).
- `tmos_get_task_timer(task, mode=1)` is **inferred** to return "remaining
  ticks to next task event" (not directly disassembled — see §8 OQ4).
- The `advDelay` randomization in `llAdvertiseStart` (0-13 ticks =
  0-8.125 ms, `ll-adv-scheduler-disasm.md` §10) is BLE-spec-required
  jitter, not the controller margin.

**Three possible mappings** (this is where over-inference is dangerous):

- **M-A** (lower bound): WCH does NOT have an explicit controller margin;
  the entire `(interval - elapsed)` budget is exposed to user cb as `timeUs`,
  and it is the **app's responsibility** to leave ≥2 ms (or whatever) for
  the controller to fire the next event. This matches TI's R-TI-3 literally.
- **M-B** (middle): WCH's `tmos_get_task_timer(task, 1)` itself returns
  `(remaining - margin_ticks)` where `margin_ticks = 2_ms / 625 µs = 3.2`,
  so `timeUs` already accounts for the buffer. The app sees a budget
  that's already conservative.
- **M-C** (upper bound): WCH has no margin concept at all; if the app
  uses up all of `timeUs`, the next event simply slips, and the BLE link
  layer drops/resets per `wch-ble-manual.md` line 1393 *"在低功耗蓝牙
  工作期间，需要通过 RTC 定时器计算时间"* warning.

**Current best read**: M-A is most likely (matches TI doctrine, matches
the simplest disasm interpretation, matches BLE-spec jitter handling).
M-B requires extra arithmetic that doesn't show in `tmos_get_task_timer`
disasm. M-C contradicts the user-cb pattern (why provide `timeUs` at all
if it has no margin meaning?).

**Phase 2 on-chip trace probe** (see §8 OQ1) resolves this empirically:
program ADV with interval = 100 ms, drain the on-chip trace ring over
SDI and read both the actual `timeUs` value passed to the user cb and
the surrounding EVT_START / EVT_CLOSE `bleClock_t` records (§8 trace
framework). If `timeUs ≈ (interval - elapsed) - 2 ms`, M-B; if
`timeUs ≈ (interval - elapsed)`, M-A; if next event slips when app
uses all of `timeUs`, M-C.

### 4.4 Rust port — design rule that survives any of M-A/B/C

Regardless of which mapping is true, the Rust port should:

1. **Expose `timeUs` to user cb with the same semantics as WCH** (remaining
   µs to next event, no transformation).
2. **Document** in the `EventCloseCallback` API contract: "leave a safety
   margin (recommended ≥2 ms per TI §7.7.2) before doing app work; the
   exact controller-margin behavior is firmware-version dependent and
   should be characterized on the target hardware via §8 OQ1 probe."
3. **Defer** building a typed `EventBudget` struct until Phase 2 has the
   probe result. Hard-coding a 2 ms constant before measurement would
   bake an over-inferred number into the API.
4. **For first-air SCAN_RSP** (Step 2): the user-cb work fits trivially
   in any of M-A/B/C since SCAN_RSP turnaround is sub-millisecond. Step 2
   does not need the margin question resolved.

### 4.5 §4 evidence summary

| Claim | Source | Status |
| --- | --- | --- |
| TI §7.7.2 specifies `(interval) - 2 ms` rule + 2 ms controller buffer | swru271i lines 2300-2308 | VERIFIED |
| WCH inherits OSAL non-multithreaded model | `wch-ble-manual.md` §TMOS | VERIFIED |
| WCH dispatches via `LL_ProcessEvent` same as TI | `ll-process-event-minipass.md` M1 | VERIFIED |
| `tmos_get_task_timer(task, 1)` returns "remaining ticks" | `ll-tx-completion-disasm.md` §4.2 | INFERRED (not directly disassembled) |
| 2 ms controller buffer maps to a specific WCH location | none found in disasm | **NOT VERIFIED — Phase 2 OQ1** |
| `advDelay` 0-13 ticks IS NOT the controller margin (it is BLE-spec jitter) | `ll-adv-scheduler-disasm.md` §10 | VERIFIED |
| User-cb `timeUs` is "remaining µs to next event" | `ll-tx-completion-disasm.md` §4 | VERIFIED |

### 4.6 Anti-over-inference checklist (for §4 review)

If a future reader asks "what does the WCH equivalent of TI §7.7.2 do
exactly?", the honest answer chain is:

1. "WCH has the same controller-must-run-each-event model" — VERIFIED.
2. "User cb gets remaining µs to next event in `timeUs`" — VERIFIED.
3. "WCH has a 2 ms controller margin like TI" — **WE DO NOT KNOW**. §8 OQ1.
4. "App must leave time for the controller" — TRUE but vague; the exact
   minimum is OQ1.

Anything in §4 phrased more confidently than this chain is over-inference
and should be flagged in review.

## §5 — `adv_event_closed(timeUs)` semantics

### 5.1 What the cb actually receives

The ADV event-close user callback has the C-equivalent signature:

```c
typedef void (*pfnEventCB)(uint32_t timeUs);
```

The single argument `timeUs` is computed in `ll_advertise_event_closed` as:

```
remaining_ticks = tmos_get_task_timer(taskID, mode=1)
timeUs          = remaining_ticks * 625
cb(timeUs)                                   // tail-call (jr t1)
```

Anchor: `ll-tx-completion-disasm.md` §4 (line 416-420 of that doc, with full
disasm of the `2c-44` instruction span in `ll_advertise_event_closed.dump`).

### 5.2 `timeUs` is "remaining µs to next event", NOT a timestamp

This is the most important semantic claim in the whole event model and it
appears in §6 Misconception L1 as well. Stated precisely:

- `timeUs` is the **duration**, in microseconds, from "now" to the next
  scheduled fire of this ADV task's event bit.
- It is NOT a wall-clock timestamp.
- It is NOT the duration of the event that just closed.
- It is NOT the BLE Core spec "advance time" / `instant` value.

Concrete example for verification: with ADV interval = 100 ms and a typical
event taking ~3 ms of inner-loop time, the cb's `timeUs` should be roughly
97_000 (~97 ms). If we saw values near 3_000 it would mean "elapsed time";
if we saw 100_000 it would mean "interval"; the actual ~97_000 confirms
"remaining time".

The `tmos_get_task_timer(taskID, mode=1)` invocation is what supplies the
"remaining" semantic: per `wch-ble-manual.md` §TMOS (and the function's
return-type comment in the WCH BLE manual line 1612-1627), `mode=1` returns
ticks until the next fire of the specified event for the specified task.
Exact disassembly of `tmos_get_task_timer` was not obtained; this is
INFERRED from the function's documented contract and its invocation
pattern. Marked as §8 OQ4 for Phase 2 air-run verification.

### 5.3 The single convergence point

All "ADV event closing" paths funnel through one of two related functions:

```
ll_advertise_status_closed(advState)        ← primary convergence
   ├─ if (byte_10 == byte_26)               ; last channel processed
   │     tail-call ll_advertise_event_closed(advState)
   └─ else
         tail-call tmos_set_event(taskID, 2)  ; re-arm mask 0x0002 (bit 1), not a close

ll_advertise_event_closed(advState)         ← user-cb dispatcher
   ├─ clear gBleLlPara[124] (adv-event state machine reset)
   ├─ compute timeUs = tmos_get_task_timer(taskID, 1) * 625
   └─ tail-call pfnAdvertiseEventCBs(timeUs)
```

Edge case: `.L438` inside `ll_advertise_process` calls `ll_advertise_event_closed`
**directly** (not through `ll_advertise_status_closed`). This is the error
path that fires when the inner loop detects an unrecoverable RX/TX completion
failure. Per `ll-tx-completion-disasm.md` §5.2, this means:

- `ll_advertise_status_closed` is the **decision point** (channel-rotation done? close or continue?).
- `ll_advertise_event_closed` is the **close action** (user cb + state reset).
- The decision point can be bypassed by the error path, but the close action is the same.

### 5.4 `LL_AdvertiseEventRegister` — how the cb gets installed

The user-facing registration function is a 3-instruction stub:

```
LL_AdvertiseEventRegister(pfnEventCB cb):
   auipc a5, %hi(pfnAdvertiseEventCBs)
   sw    a0, %lo(pfnAdvertiseEventCBs)(a5)
   ret
```

- `pfnAdvertiseEventCBs` is a **COMMON 4-byte symbol** (single fn ptr, NOT
  an array despite the trailing "s" in the name).
- The address is link-time-determined, same pattern as `fnGetClockCBs` (§3.3).
- `LL_Init` boot-time clears it via `LL_AdvertiseEventRegister(NULL)`.

Implication for the Rust port: the equivalent API is one slot, not an array.
Replacement semantics, not registration. Match this in the Rust API to avoid
silent UB if a downstream user expects "append".

Anchor: `ti-wch-mapping.md` §13.1 + §13.2.

### 5.5 Misconception warnings (mirrors §6 L1)

The following are **wrong** uses of `timeUs` and must be guarded against in
the Rust API design and in app-side examples:

- ❌ **Wrong**: `if (timeUs - last_event_timeUs > 100_000) { ... }` — `timeUs`
  is not monotonic across events; this comparison is meaningless.
- ❌ **Wrong**: `let event_duration = interval - timeUs;` — only true if the
  app knows it is being called immediately on event close, with no delay
  jitter. Even then, the controller margin (§4) confounds the arithmetic.
- ❌ **Wrong**: scheduling the next user action at an absolute time computed
  from `timeUs`.
- ✅ **Right**: `if (timeUs > MY_WORK_BUDGET) { do_work(); }` — relative
  budget check, decides whether to start work that fits in the remaining
  window.
- ✅ **Right**: split a long task into chunks that each fit in a typical
  `timeUs` budget, dispatched per cb fire.
- ✅ **Right**: signal an embassy task / TMOS event with the residual time
  hint for upper-layer scheduling.

### 5.6 Rust port API contract

The Rust equivalent of `LL_AdvertiseEventRegister` and the event-close cb:

```rust
/// Called from the LL after each ADV event closes.
/// `time_remaining` is the duration until the next ADV event fires, in 625 µs
/// ticks (matching WCH's bleClock_t base — see §3). Not a timestamp.
///
/// The callback runs in the LL task context, after the LL has yielded back
/// to TMOS-equivalent dispatch. It must complete in less than `time_remaining`
/// minus a controller-margin (see §4 §8 OQ1 — defaults to TI's 2 ms suggestion
/// until measured on target).
pub type AdvEventClosedFn = fn(time_remaining: AdvTicks);

impl LinkLayer {
    /// Replaces (not appends) the ADV event-close callback. Matches WCH's
    /// single-slot semantics. Pass None to disable.
    pub fn register_adv_event_cb(&mut self, cb: Option<AdvEventClosedFn>);
}
```

Note the unit on `time_remaining` is `AdvTicks` (625 µs), not µs. This:

- Avoids the `* 625` multiply that WCH does before cb dispatch.
- Keeps the API expressed in the LL's native time base (§3).
- Forces the app to convert at the boundary if it wants ms or µs.

This is a deliberate Rust-port improvement over WCH's `uint32_t timeUs` cb
shape — same semantics, native units. Step 3 abstraction lock revisits.

### 5.7 §5 evidence summary

| Claim | Source | Status |
| --- | --- | --- |
| `timeUs = tmos_get_task_timer(taskID, 1) * 625` | `ll-tx-completion-disasm.md` §4.1 line 416-420 | VERIFIED |
| `timeUs` semantic = "remaining µs to next event" | inferred from `tmos_get_task_timer` contract (WCH manual line 1612-1627) | INFERRED — §8 OQ4 |
| `ll_advertise_status_closed` is the primary convergence | `ll-tx-completion-disasm.md` §2 + §5.2 | VERIFIED |
| `.L438` error path directly calls `ll_advertise_event_closed` | `ll-tx-completion-disasm.md` §5.2 | VERIFIED |
| `ll_advertise_event_closed` clears `gBleLlPara[124]` | `ll-tx-completion-disasm.md` §4 | VERIFIED |
| `pfnAdvertiseEventCBs` is COMMON 4-byte single-slot | `ti-wch-mapping.md` §13.1 disasm | VERIFIED |
| `LL_AdvertiseEventRegister` is 3-instr stub | `ti-wch-mapping.md` §13.1 | VERIFIED |
| `LL_Init` clears the slot at boot via `register(NULL)` | `ti-wch-mapping.md` §13 | VERIFIED |

## §6 — Misconception Ledger

This section enumerates the five concrete misconceptions that the Step 1
research surfaced. Each entry follows the same shape: **(a) the wrong model,
(b) the right model, (c) the disasm anchor that proves it, (d) the consequence
for the Rust port if the wrong model leaks in.**

Where an entry overlaps with content earlier in this doc (§4 / §5), this
section is the canonical short-form callout; the longer reasoning stays in
the earlier section.

---

### L1 — `timeUs` in `adv_event_closed` is **remaining**, not **timestamp** or **elapsed**

**Wrong models** (any one of these will silently mis-time the next event):

- ❌ `timeUs` is a monotonic timestamp (microseconds since boot / since session
  start). Adding to a previous `timeUs` gives "wall-clock time of close".
- ❌ `timeUs` is the elapsed duration of this just-finished event (the time
  the inner loop spent on-air).
- ❌ `timeUs` is fixed per event (e.g. always == ADV interval).

**Right model**:

`timeUs = tmos_get_task_timer(taskID, mode=1) * 625`. The TMOS timer in
mode 1 returns **ticks remaining until the next scheduled fire of this task**.
So `timeUs` is the **µs budget remaining until the next ADV event** at the
moment the close cb is invoked. It is **always shrinking** between consecutive
calls (until the next event re-arms the timer); it is **monotonically decreasing
within one interval** and resets on each event arm.

**Anchor**: `ll-tx-completion-disasm.md` §4.1 line 416 (`tmos_get_task_timer(taskID, 1) → remaining ticks`), §4.2 (`timeUs = ticks * 625 = 距下次 adv event 的剩余 µs`). See §5.2 of this doc for the worked 100 ms example.

**Port consequence**: the registered cb in Rust must be documented as
`fn(time_remaining: AdvTicks)` — passing it as `Instant` or `Duration_elapsed`
will produce a port that compiles cleanly but starves or over-saturates the
next ADV event. See §5.6 of this doc for the locked API contract.

---

### L2 — `gBleLlPara[0x7c]` is the **adv-event state byte**, not the TX-busy guard

**Wrong model** (`ti-wch-mapping.md` §13.2 originally said this — corrected
in `ll-tx-completion-disasm.md` §4.1):

- ❌ `gBleLlPara[0x7c]` (= offset 124) is the "TX-busy guard" that
  `ll_advertise_tx` checks on entry to reject re-entrant TX.

**Right model**:

- `gBleLlPara[0x7c]` (offset **124**) = **adv-event state byte**. Written
  to `0` / `2` / `3` / `4` from multiple sites in the ADV path
  (`ll_advertise_tx`, `llAdvTraverseallChannel`, `ll_advertise_event_closed`).
  The value `4` in this byte is what the dispatch pre-arm uses to decide
  whether to call `ll_advertise_event_closed` *before* arming the next
  channel TX. `ll_advertise_event_closed` clears this byte back to `0` to
  let the next event start cleanly.
- `gBleLlPara[168]` (offset **0xa8**) = **TX-busy guard**, checked by
  `ll_advertise_tx` on entry. Distinct field, distinct lifecycle (cleared by
  the IRQ / hardware completion path, not by `event_closed`).

**Anchor**: `ll-tx-completion-disasm.md` §4.1 lines 426-433 + §4 summary
line 552; row 11 of the §6 corrections table is the canonical pointer.

**Port consequence**: any Rust port that uses one `LlAdvBusy` flag to model
both fields will deadlock the state machine when the busy bit is held by the
hardware path while the state-machine byte expects the user-space writer to
advance. They are two independent state variables — model them as such.

---

### L3 — SCAN_RSP **warm kick** and cold ADV TX kick are **distinct hardware paths**

**Wrong model**:

- ❌ SCAN_RSP is just "another ADV TX" — same code path, same TX-go register
  write at the end. The only difference is the PDU header.

**Right model** — `ll_advertise_tx` writes its **final commit** to one of
two register paths depending on whether this is a cold ADV TX (start of an
event, full PHY/BB/LLE re-config) or a warm turnaround into SCAN_RSP
(PHY/BB already configured by the preceding TX, only re-arm the BB FIFO):

| Path | Final register write | Where it lives |
| --- | --- | --- |
| Cold ADV TX kick | `LLE[0] = 2` | `ll-advertise-tx-disasm.md` §10, line 457: `sw a4, 0(a5) ; LLE[0] = 2 ← final commit, TX go!` |
| SCAN_RSP warm kick | `BB[0] \|= 0x800000` | `ll-advertise-tx-disasm.md` §10, line 448: `or a4, a4, a3 ; BB[0] \|= 0x800000` |

**Anchor**: `ll-advertise-tx-disasm.md` §10 + §6 row 10 (`TX 实际启动 = LLE[0] = 2, 这是最后一笔 store`) — plus §11 open item line 640 flagging that `BB[0]` bit 23 is provisionally "TX-fifo commit" (the warm kick).

**Port consequence**: a Rust port that collapses these into one `kick_tx()`
helper that writes both registers unconditionally **will work** on cold ADV
TX (because BB[0] bit 23 was just configured anyway) but **will fail or
mis-time** on SCAN_RSP warm turnaround if the BB[0] write is omitted or
sequenced wrong. The first-air baseline keeps both writes as distinct named
operations: `kick_cold_adv_tx()` writes `LLE[0] = 2`; `kick_scan_rsp_warm()`
writes `BB[0] |= 0x800000`. Iron Rule §0 (full-write first-air baseline) —
do not delta-optimize until air-validated.

---

### L4 — `0x4002_420c` is an **irq-mask-like AHB MMIO**, **separate** from PFIC at `0xe000_e000`

**Wrong model**:

- ❌ `0x4002_420c` is PFIC's IRQ-mask register (i.e. `0xe000_e000 + 524 * 4`
  by some calculation), or it is just the "global IRQ disable" used during
  PHY reconfig.

**Right model** — Iron Rule §35: there are **two distinct IRQ-related
controllers** referenced in the BLE bringup path:

- **PFIC** (RISC-V Programmable Fast Interrupt Controller), base `0xe000_e000`.
  IENR at `0xe000_e204` enables / disables specific LL-event interrupts.
  This is what `ll-slave-init-disasm.md` writes for connection IRQ enable.
- **`0x4002_420c`** = an AHB-peripheral MMIO. `phy_status_clear` saves its
  current value, zeros it, runs the PHY reconfig sequence, then restores
  the saved value. Provisionally an IRQ-mask register; may be PFIC reg 524,
  may be an LLE-adjacent companion MMIO, may be a peripheral-clock gate.
  **Not corroborated against `ch32-data` PAC defs.**

**Anchor**:

- `ll-phy-preflight-disasm.md` §3 lines 233-254 (the save/zero/restore
  sequence) and §9 line 422-424 (cross-check open items).
- `ll-slave-init-disasm.md` §5 line 343 (Iron Rule §35 formal statement).

**Port consequence**: the Rust port **MUST** keep these as **two distinct typed
register accesses** until ch32-data PAC corroborates `0x4002_420c`'s identity.
A port that collapses both into a single `pfic_mask()` helper will silently
write the wrong register on at least one of the two paths. Naming convention
for this doc and the Rust port: `0x4002_420c` is **"irq-mask-like MMIO at
0x4002_420c"** — provisional wording is intentional.

---

### L5 — `ll_advertise_filter` **mutates `adv_state`** on resolve-list success — it is **not** a `&self` filter

**Wrong model** (Phase 2.5 `ll-rx-ingress-disasm.md` §7 black-box treatment):

- ❌ `ll_advertise_filter` returns `1` for accept, anything else for reject;
  it is a pure-read predicate on the RX PDU and the static filter lists.

**Right model** — `ll_advertise_filter` is a **mutating** filter. On the
resolve-list success paths (RPA-resolve hit, IRK-resolve hit) it writes
**four fields** of `adv_state`:

| Field | Mutation | Meaning |
| --- | --- | --- |
| `s0[48]` (sw, 4 B) | resolving-list entry pointer | Caches the resolved entry so `llSlaveCreateCore` reads peer-identity from one place |
| `s0[68]` (sb, 1 B) | `= 2` | Dest-addr-type tag: "resolved identity" |
| `s0[69]` (sb, 1 B) | resolved-state flag | "this PDU's TargetA was resolved" |
| `s0[70..75]` (memcpy 6 B) | identity address (IDA) | Replaces TargetA payload with the IRK-resolved identity address |

`llSlaveCreateCore` (slave init on CONNECT_IND accept) then **reads these
mutated fields** as the peer identity for the new connection state — so
the mutation is not optional; it is on the CONNECT_IND happy path.

**Anchor**:

- `ll-advertise-filter-disasm.md` §3 lines 22-23 (field layout), §4.4
  lines 105-128 (the four mutation sites), §5.4 line 308 (Iron Rule §37
  formal statement).
- `ll-advertise-filter-disasm.md` §5 line 316 (Phase A2 cross-link: the
  mutated fields feed `llSlaveCreateCore`).

**Port consequence**: Iron Rule §37 — the Rust port signature **MUST** be
`fn filter(&mut self, ...) -> FilterOutcome` (or `fn filter(&self, ...) ->
(FilterOutcome, Option<AdvStatePatch>)` with the caller applying the patch
explicitly). A pure-read `&self` filter is faithless and will break the
CONNECT_IND → slave-init handoff. The current task #75 stub
(`signatures.rs::ll_advertise_filter`) is already `&mut AdvState` — preserve
this when replacing the stub in Step 2+.

---

### §6 cross-reference matrix

| Misconception | Lives in this doc | Disasm anchor | Iron Rule |
| --- | --- | --- | --- |
| L1 `timeUs` semantic | §5.2, §5.5 | `ll-tx-completion-disasm.md` §4 | — (model-level) |
| L2 `gBleLlPara[0x7c]` vs `[168]` | (this section only) | `ll-tx-completion-disasm.md` §4.1 | — (data-layout) |
| L3 SCAN_RSP vs cold ADV kick | (this section only) | `ll-advertise-tx-disasm.md` §10 | §0 |
| L4 `0x4002_420c` vs PFIC | (this section only) | `ll-phy-preflight-disasm.md` §3 + `ll-slave-init-disasm.md` §5 | §35 |
| L5 filter mutation | (this section only) | `ll-advertise-filter-disasm.md` §4.4 | §37 |

L1 is the only L-entry that is fully developed elsewhere in this doc
(§5 is the canonical place). L2-L5 are first-class in §6 because they are
not part of the ADV time/event model proper — they are sibling decoded
artifacts that the model interacts with and must not collide with.

## §7 — Scope Boundary

This doc is the **ADV path time/event model only**. The §7.x sub-sections
state the boundary explicitly so review can flag any drift, and so Step 2
authors know what they cannot inherit from this doc.

### 7.1 In scope (this doc owns the canonical wording)

- TMOS outer dispatch behaviour as it pertains to `LL_ProcessEvent` event
  masks `0x0001` (`llAdvertiseStart`, bit 0) and `0x0002`
  (`llAdvTraverseallChannel`, bit 1). §1, §2.
- `ll_advertise_process` synchronous inner state machine across the
  three-channel ADV sweep, including the `byte_11` post-TX forward-stamp
  and the `ll_advertise_status_closed` / `ll_advertise_event_closed`
  convergence. §1, §5.
- `bleClock_t` LSB = 625 µs / 1600 Hz time base **as used by the ADV path
  scheduler and the `adv_event_closed(timeUs)` cb**. §3.
- The `pfnAdvertiseEventCBs` single-slot registration via
  `LL_AdvertiseEventRegister`, the Rust port API contract for
  `register_adv_event_cb`, and the `AdvTicks` native unit. §5.4, §5.6.
- The ADV-specific application of TI §7.7.2 controller-margin reasoning,
  with VERIFIED/INFERRED tagging and the M-A/B/C mapping candidates. §4.
- The L1-L5 misconception ledger entries — these are ADV-relevant by
  selection (L1 = ADV cb, L2 = ADV state byte, L3 = ADV TX kick vs SCAN_RSP,
  L4 = `phy_status_clear` which is on the ADV preflight, L5 = ADV filter).
  §6.

### 7.2 Out of scope (Step 2+ docs own these)

- **Connection-event timing** (post-CONNECT_IND, slave side and master
  side) — anchor windows, supervision timeout, latency, MD bit. Owned by
  Step 4 connection doc (per roadmap). This doc cites `llSlaveCreateCore`
  only as the consumer of the `adv_state` mutations decoded in L5; it
  does **not** decode the slave-conn timer programming.
- **Scanner / observer timing** — `LL_ProcessEvent` masks `0x0010` /
  `0x0020`, scan window / scan interval, RPA generation cadence for
  active scan. Owned by Step 4+ scanner doc.
- **SMP / pairing / bonding timing** — long-tail concern; ignore until
  Step 5+ work.
- **Periodic ADV / EXT_ADV / aux paths** — Bluetooth 5.x advertising
  extensions. Backlog BL-7 (Step 6+ per roadmap).
- **DTM (direct test mode)** — `LL_ProcessEvent` mask `0x4000` row in §2
  exists for completeness but is not decoded by this doc.
- **`SYS_EVENT_MSG`** — mask `0x8000` row in §2 exists for completeness;
  not decoded here.

### 7.3 Cross-cutting — cited here, decoded elsewhere

- `bleClock_t` / `fnGetClockCBs` are shared across ADV + CONN + scan
  paths. §3 owns the LSB constant and the link-time-determined nature;
  per-subsystem user enumeration is **not** part of this doc.
- TI §7.7.2 controller-margin is a generic event-budget concept shared by
  all event types. §4 applies it specifically to ADV; Step 4 conn doc
  must independently re-apply it for connection events (different
  scheduler constraints, different default values).
- `phy_status_clear` (L4 / `0x4002_420c`) runs on every PHY mode change,
  including connection PHY-update procedure — this doc only covers its
  ADV-side invocation via `ll_advertise_tx` preflight; the conn doc
  inherits the Iron Rule §35 wording.
- `tmos_get_task_timer` mode-1 contract is a TMOS-wide primitive used
  by all subsystems; §5 documents the ADV-cb-side semantic and leaves
  OQ4 to verify the "remaining" wording.

### 7.4 What Step 2 authors **may not** inherit from this doc

Step 2 (SCAN_RSP first-air implementation, per roadmap) is the immediate
follow-on. Two specific items in this doc are **not** Step 2 inputs:

1. The L4 `0x4002_420c` decode (irq-mask-like MMIO). Step 2 SCAN_RSP work
   does not touch `phy_status_clear` — it lives on the warm-turnaround
   path inside an already-running ADV event. Step 2 may cite L3 freely.
2. The §4 controller-margin reasoning. SCAN_RSP turnaround latency is a
   T_IFS = 150 µs problem (BLE spec timing), not a §7.7.2 budget problem.
   Step 2 OQ3 (RX→TX turnaround) measures the spec'd T_IFS path, not the
   per-event scheduler budget. Do not collapse these.

### 7.5 Scope drift detection

If a future revision of this doc adds:

- Any decoded register write that is conn-only or scan-only;
- Any timing wording sourced from a non-ADV `LL_ProcessEvent` mask row;
- Any L-entry that does not touch the ADV path in §1 or §2;

then this doc has drifted from its ADV-only scope. Split the new content
into the appropriate Step 4+ doc instead.

## §8 — Open Questions / Step 2 Probe DoD inputs

Per Cindy `3eac94cb` review concern #3, each OQ in this section is written
in **Step 2 probe DoD shape** — that is: a probe that a Step 2 author can
drop directly into the SCAN_RSP first-air air-run setup, with explicit
signal, capture, pass criterion, and inference-status fields. Step 1 ends
with the model documented and these OQs **enumerated as probes**, not as
generic measurement intentions.

These items are **not blockers for Step 1 doc PASS**. They are the input
backlog for Step 2 measurement work.

### §8 trace framework (片上 / on-chip)

CH32V208 BLE is an **on-chip peripheral** (QingKe V4F core + BLE block),
not an external BLE module. The measurement framework is **on-chip trace
first**, not external GPIO + logic analyzer. All OQ probes below use the
following layered observable set, aligned with Cindy's SCAN_RSP spec
§4.1 / §6.4 wording:

| Layer | Observable | Resolution / cadence | Use |
| --- | --- | --- | --- |
| Fine timing | `SysTick CNTL` at `0xE000_F008` — 12 MHz free-running 32-bit counter, used in v4-series task-layer testing; air-PASS evidence is the control run `7f04104 + ch32-data 1b6b0d4` (Segment B 1-frame PASS) | ~83 ns / cycle | T_IFS, inner-loop tail, sub-µs deltas |
| Coarse timing | `bleClock_t` (§3.6 VERIFIED, 625 µs LSB, RTC-derived) | 625 µs | event-budget, inter-event interval, controller_margin |
| State evidence | direct read of `BB[0]`, `BB[44]`, `LLE[100]`, `gBleIPPara` done flags at instrumented code points + SDI log | per code point | warm-kick reached, RX done, TX done |
| IRQ evidence | BB / LLE / RFEND IRQ-entry ring (§3.5 IRQ table, lock-free SPSC ring of `{irq_id, SysTick CNTL, latched register state}`) | per IRQ | IRQ-entry timestamp, ordering |
| Protocol | sniffer pcap + phone scanner | per air frame | SCAN_RSP / AdvA correctness, NOT internal timing gate |
| Disabled | `mcycle` CSR (RV32 CSR 0xB00) | — | **NOT IMPLEMENTED on QingKe V4F**; reads return 0; do NOT use, even as fine timer (`ble_peripheral_phase1_adv.rs` keeps it only as a non-timing entropy source) |
| Optional aux | GPIO toggle + external logic analyzer | — | **Not in any OQ gate.** May be added later as cross-validation only when the on-chip trace has already converted INFERRED → VERIFIED. |

The "片上 trace ring" used below is the same lock-free SPSC ring schema
Cindy locks in SCAN_RSP spec §6.4: each record is `{slot_id, SysTick CNTL,
bleClock_t low word, BB[0], BB[44], LLE[100], gBleIPPara flag word}`,
written from inside the LL fast paths and drained over SDI by the host.
This doc owns the schema *names*; the spec owns the implementation
layout.

### OQ1 — `controller_margin` empirical value on CH32V208 (mapping candidate selection)

- **Inference status this resolves**: §4 M-A vs M-B vs M-C — which of the
  three mapping candidates carries the TI §7.7.2 ≈ 2 ms margin idea on WCH.
  Currently all three are INFERRED. After this probe, exactly one becomes
  VERIFIED (or all are rejected and we add M-D).
- **Why Step 2 cares**: Step 2 SCAN_RSP is run inside an ADV event; if the
  margin lives in the inner-loop entry deadline (M-A), tight SCAN_RSP
  turnaround can starve the next event. If it lives in TMOS armament (M-B),
  it does not affect inner-loop timing. The probe disambiguates.
- **Probe signal** (on-chip trace, coarse):
  - On entry to `llAdvTraverseallChannel`, push `{slot=EVT_START,
    bleClock_t low word}` to the trace ring.
  - On entry to `ll_advertise_event_closed`, push `{slot=EVT_CLOSE,
    bleClock_t low word}`.
  - ADV interval set to two clean values: 100 ms (= 160 ticks) and
    320 ms (= 512 ticks).
- **Capture**: drain trace ring over SDI for a 5-second window per
  interval value. Decode `bleClock_t` low-word wraparound across the
  window.
- **Pass criterion** (Step 2 air-run report):
  - `t_event_active = (EVT_CLOSE.bleClock_t − EVT_START.bleClock_t) × 625 µs`
    distribution (mean / max).
  - `t_event_idle = (next_EVT_START.bleClock_t − prev_EVT_CLOSE.bleClock_t) × 625 µs`.
  - Report `t_event_idle / configured_interval` ratio. Ratio
    ≈ (1 − 2 ms / interval) ⇒ M-A or M-B. Ratio ≈ 1 (negligible margin)
    ⇒ M-C.
  - 625 µs LSB is fine enough at this scale; SysTick CNTL not needed.
- **Output format**: append to
  `notes/ch32-rs/ti-ble/measurements/oq1-controller-margin.md` with raw
  csv (drained ring records) + the ratio derivation.

### OQ2 — `ll_advertise_process` inner-loop tail latency

- **Inference status this resolves**: §5.3 — confirms that
  `ll_advertise_status_closed` → `ll_advertise_event_closed` convergence is
  immediate (≤ tens of µs) and is **not** the source of any per-event
  budget loss. Currently VERIFIED by disasm structurally; not measured
  in time.
- **Why Step 2 cares**: Step 2 needs to know whether to schedule SCAN_RSP
  preflight earlier in the inner loop or whether the tail is short
  enough to let SCAN_RSP work happen post-`status_closed`. Independent of
  M-A/B/C selection.
- **Probe signal** (on-chip trace, fine):
  - At last-channel TX completion inside `ll_advertise_process` (the
    `ll_tx_wait_finish` return point on the final channel), push
    `{slot=LAST_CH_TX_DONE, SysTick CNTL, BB[0] snapshot}`.
  - At `ll_advertise_event_closed` entry, push `{slot=EVT_CLOSE,
    SysTick CNTL}` (re-used from OQ1).
- **Capture**: drain trace ring over SDI for 5-second window per ADV
  interval value.
- **Pass criterion**: `t_tail = (EVT_CLOSE.SysTick − LAST_CH_TX_DONE.SysTick) / 12`
  µs distribution (handle 32-bit wrap). Report mean / max. Tail < 200 µs
  ⇒ §5.3 convergence-point assumption holds. Tail ≥ 1 ms ⇒ Step 2 author
  must revisit the §5.3 timing diagram.
- **Output format**: same OQ1 file, separate section
  (`measurements/oq1-controller-margin.md` §2 retained for backward ref).

### OQ3 — RX→TX turnaround latency around T_IFS 150 µs (SCAN_REQ → SCAN_RSP)

- **Inference status this resolves**: this is **NOT** an OQ on this doc's
  §4 / §7.7.2 reasoning — per §7.4 it is a separate problem class
  (BLE spec T_IFS, not WCH scheduler budget). §6 L3 (warm vs cold kick)
  is what depends on this probe — the warm kick must complete inside
  T_IFS.
- **Why Step 2 cares**: this is the **gate item** for Step 2 (roadmap
  line 139). Without this probe Step 2 cannot declare PASS.
- **Probe signal** (on-chip trace, fine + state):
  - At `ll_advertise_legacy_rx` SCAN_REQ-accept code point (after filter,
    just before scheduling SCAN_RSP), push `{slot=RX_DONE, SysTick CNTL,
    BB[44] snapshot, LLE[100] snapshot}`.
  - At the SCAN_RSP warm-kick write site (`BB[0] |= 0x800000`, *after*
    `BB[44] &= ~0x3` commit clear), push `{slot=TX_KICK, SysTick CNTL,
    BB[0] post-write snapshot, BB[44] post-clear snapshot}`.
- **Capture**: drain trace ring over SDI across 100 SCAN_REQ → SCAN_RSP
  transactions driven by a phone scanner peer (force re-scan / restart
  app between runs — Cindy spec §5.3 scanner hygiene).
- **Pass criterion**:
  - `t_turnaround = (TX_KICK.SysTick − RX_DONE.SysTick) / 12` µs (handle
    32-bit wrap), mean and max.
  - PASS = max `t_turnaround` < (150 µs − air-frame head margin), so
    SCAN_RSP first bit hits the air at T_IFS = 150 µs measured from
    SCAN_REQ end.
  - State assertions: every TX_KICK record must show `BB[0]` bit 23 set
    after the write and `BB[44] & 0x3 == 0` after the commit clear.
    Failure of either is a structural fault — not just a timing miss.
  - Strict pass also requires pcap from a sniffer showing SCAN_RSP for
    the same AdvA in the same ADV event.
- **Output format**: `notes/ch32-rs/ti-ble/measurements/oq3-tifs-turnaround.md`
  with drained ring csv + sniffer pcap link.

### OQ4 — `tmos_get_task_timer(task, mode=1)` returns "remaining ticks"

- **Inference status this resolves**: §5.2 + §5.7 evidence-table row 2 —
  currently INFERRED. The disasm chain
  `tmos_get_task_timer(taskID, 1) → ticks` is verified; the wording
  "remaining ticks to next event" comes from `ll-tx-completion-disasm.md`
  §4.2 inference, not from TMOS source. This probe converts INFERRED →
  VERIFIED **without** requiring TMOS source access.
- **Why Step 2 cares**: any Rust user of `register_adv_event_cb` (Step 3
  Embassy integration in particular) needs the unit definition locked.
  If the cb actually receives elapsed instead of remaining, every
  embassy timer chained off it will be off by one interval.
- **Probe signal** (on-chip, no GPIO):
  - Install a probe `adv_event_closed` cb that, on every fire, pushes
    `{slot=CB_FIRE, SysTick CNTL, bleClock_t low word, timeUs cb arg}`
    to the trace ring.
  - Set ADV interval to a clean known value (100 ms = 160 ticks).
- **Capture**: drain trace ring over SDI after a 5-second air run.
  Pair each CB_FIRE record with the *next* EVT_START record (OQ1
  slot) within the same drained window.
- **Pass criterion**:
  - Compute, for each CB_FIRE, the gap to the next EVT_START in
    `bleClock_t` ticks × 625 µs. Compare to `timeUs` cb arg.
  - If `timeUs` is **remaining** (inferred semantic):
    `timeUs ≈ (next_EVT_START − CB_FIRE) × 625 µs`, and value will be
    close to but less than the configured interval — roughly
    `100_000 − t_event_active − ε`.
  - If `timeUs` is **elapsed**: value tracks on-air duration (a few
    hundred µs to a few ms) — *much smaller* than 100 ms, and shows
    no correlation with the gap to next EVT_START.
  - PASS = observed `timeUs` ≥ 50 % of configured interval AND
    correlates with `(next_EVT_START − CB_FIRE)` ⇒ remaining semantic
    confirmed. FAIL = `timeUs` < 5 % of configured interval ⇒ elapsed
    semantic; §5 needs rewrite.
- **Output format**: `notes/ch32-rs/ti-ble/measurements/oq4-timeus-semantic.md`
  with drained SDI csv + verdict.

### OQ5 — `advDelay` randomization actual distribution on CH32V208

- **Inference status this resolves**: `ll-adv-scheduler-disasm.md` §10 —
  the **range** 0-13 ticks (0-8.125 ms) is VERIFIED from disasm; the
  **distribution** (uniform? biased toward 0? toward 13?) is INFERRED
  to be uniform from the bit-mask masking pattern, not measured.
- **Why Step 2 cares**: this is **not** a Step 2 gate. It is a Step 3
  jitter-analysis input. Step 2 can pass with `advDelay` distribution
  unknown. Step 3 Embassy scheduler integration needs the worst-case
  arm-to-fire latency, and "uniform 0-13 ticks" is the working
  assumption.
- **Probe signal** (on-chip trace, coarse): re-use EVT_START slot from
  OQ1. ADV interval fixed at 100 ms (= 160 ticks). Observed
  inter-event interval (in `bleClock_t` ticks) = base + advDelay.
- **Capture**: drain trace ring over SDI for a 60-second window
  (≈ 600 EVT_START records at 100 ms interval, sufficient sample for
  distribution shape).
- **Pass criterion**:
  - Compute `(EVT_START[n+1].bleClock_t − EVT_START[n].bleClock_t) − 160`
    ticks for all n. The result should be in `[0, 13]`.
  - Histogram should be approximately uniform over 0-13 ticks. Allowed
    deviation: any single tick bucket within ± 30 % of the mean bucket
    count.
  - Step 2 reports OQ5 result as advisory; does not block Step 2 PASS.
- **Output format**: `notes/ch32-rs/ti-ble/measurements/oq5-advdelay-distribution.md`
  with histogram + drained csv.

### §8 cross-reference

| OQ | Resolves inference in | Step 2 gate? | Primary trace layer | Output file |
| --- | --- | --- | --- | --- |
| OQ1 | §4 (M-A vs M-B vs M-C) | No (advisory for Step 3) | `bleClock_t` (coarse) | `measurements/oq1-controller-margin.md` |
| OQ2 | §5.3 timing diagram | No (advisory for Step 2 layout) | `SysTick CNTL` (fine) | `measurements/oq1-controller-margin.md` §2 |
| OQ3 | §6 L3 warm-kick path | **Yes — gate** | `SysTick CNTL` + `BB[0]`/`BB[44]` state + pcap | `measurements/oq3-tifs-turnaround.md` |
| OQ4 | §5.2 timeUs semantic | No (advisory for Step 3 Embassy) | `SysTick CNTL` + `bleClock_t` correlation | `measurements/oq4-timeus-semantic.md` |
| OQ5 | `ll-adv-scheduler-disasm.md` §10 distribution | No (Step 3 jitter input) | `bleClock_t` (coarse) | `measurements/oq5-advdelay-distribution.md` |

Step 2 PASS requires OQ3 only. OQ1/OQ2/OQ4/OQ5 produce Step 3 inputs.

### §8 honesty bound

Each probe converts an INFERRED claim into VERIFIED (or rejects it).
None of these probes requires more disassembly — they require an
operating board, the on-chip trace ring drained over SDI / RTT, and a
scanner peer (OQ3 only). Specifically:

- **No external logic analyzer in the gate.** GPIO + LA may be added
  later as cross-validation, but is not a Step 2 PASS prerequisite.
- **No `mcycle` CSR anywhere.** QingKe V4F does not implement it
  (returns 0). Fine timing is `SysTick CNTL` at `0xE000_F008`, 12 MHz.
- **Coarse timing uses the LL's own time base** (`bleClock_t`, §3.6),
  which is consistent with the model the doc documents.
- If a probe cannot be executed, the corresponding inference stays
  INFERRED and the Rust port keeps the conservative interpretation
  (largest budget loss / strictest semantic).

## Iron Rule references inline

This doc applies the following Iron Rules from the disasm series; it does not
introduce new ones (this is model-level, not impl-level):

- **§0** (full-write first-air baseline; `ll-adv-scheduler-disasm.md`)
- **§14-17** (single re-entry, NULL guard, active flag; `ll-process-event-minipass.md`)
- **§30** (PFIC vs irq-mask-like MMIO neutrality; `ll-phy-preflight-disasm.md`)
- **§35** (two distinct IRQ controllers; `ll-slave-init-disasm.md`)
- **§36-37** (filter outcome neutrality + `&mut self`; `ll-advertise-filter-disasm.md`)

## Step 2 readiness checklist

After this doc passes review, Step 2 (SCAN_RSP first-air implementation,
per roadmap line 108) should be able to answer:

- [x] Which TMOS event mask drives the per-event ADV re-entry? → §2 mask 0x0002 (bit 1).
- [x] Where does the inner loop start, and what does it depend on? → §1.2, depends on `byte_11` post-TX stamp.
- [x] What unit is `timeUs` in user cb? → §5, "remaining µs to next event" (INFERRED, OQ4 probe converts to VERIFIED).
- [x] What is the time base for the LL? → §3, `bleClock_t` LSB = 625 µs.
- [x] Is SCAN_RSP kick same HW path as cold ADV TX? → §6 L3, no — distinct register (`BB[0] |= 0x800000` warm vs `LLE[0] = 2` cold).
- [x] What's the Step 2 PASS gate? → §8 OQ3 (RX→TX T_IFS turnaround probe).
- [x] What's still TBD that does NOT block Step 2? → §8 OQ1, OQ2, OQ4, OQ5 (advisory probes).

If any of these cannot be answered from this doc alone, the doc has a gap —
flag in review.

## Final cross-check log (Day 5, Cindy `3eac94cb` review concerns)

### Concern #1 — mask/bit-position term consistency

**Pass.** §2.1 introduces the notation lock at lines 161-165: every TMOS
event is labeled with BOTH `mask 0xNNNN` and `bit N` LSB-0. Day 5 audit
swept the whole doc and corrected two violations:

- Old: "TMOS event bit 0x02" (§1.2 invariant statement, §1.5 evidence row)
  — confusing because `bit 0x02` can mean either mask value `0x02` (= bit 1)
  or bit position 2 (= mask 0x04).
- New: "TMOS event `mask 0x0002` (bit 1)".

Inline asm pseudo-code at §5.3 also clarified: `tmos_set_event(taskID, 2)`
now annotated `; re-arm mask 0x0002 (bit 1), not a close`.

No other instances of the bad pattern remain. Register-internal "bit N"
references (e.g. `BB[0]` bit 23, `byte_11`) are not subject to this lock
— they refer to bit positions inside specific registers/bytes, not TMOS
event masks.

### Concern #2 — TI §7.7.2 → WCH ADV over-inference

**Pass.** §4 is the only section that applies §7.7.2; §4.3 + §4.5 + §4.6
collectively make the inference status explicit:

- §4.2 WCH-R-1, WCH-R-2 are VERIFIED. WCH-R-3 (the "2 ms buffer mapping")
  is tagged **INFERRED** with "Phase 2 confirms" rider.
- §4.3 introduces three mapping candidates M-A / M-B / M-C, gives a
  current best-read (M-A) with reasoning, and refuses to assert it.
- §4.5 evidence table has one row explicitly **NOT VERIFIED — Phase 2 OQ1**.
- §4.6 anti-over-inference checklist gives the honest 4-step answer chain;
  step 3 is "WE DO NOT KNOW".
- §5.6 Rust API contract doc comment refers to TI's 2 ms as "TI's 2 ms
  suggestion until measured on target", aligned with §4.4 rule 3 (defer
  typed `EventBudget` until probe result is in).
- §7.4 explicitly forbids carrying the §4 controller-margin reasoning
  into Step 2 T_IFS turnaround (different problem class).

No place in the doc asserts "WCH has a 2 ms margin" without an INFERRED
tag or "NOT VERIFIED" flag. §8 OQ1 is the explicit resolution path.

### Concern #3 — §8 measurement inputs convertibility to Step 2 probe DoD

**Pass (Day 5) with Day 6 measurement-framework correction** — see
"Day 6 amendment" below. §8 was first rewritten Day 5 from OQ outlines
into Step-2-probe DoD shape; the Day 5 version used an external
GPIO + logic analyzer template inherited from TI BLE-Stack eval-board
docs, which is wrong for CH32V208 (片上 BLE peripheral, not an external
BLE module). Day 6 retrofit replaces that with the on-chip trace
framework documented in §8's "trace framework" preamble.

Day-5 / Day-6 invariants (unchanged):

- Inference status this resolves (with which evidence-table row in §1-§7
  it converts INFERRED → VERIFIED).
- Why Step 2 cares (with explicit "Step 2 gate" vs "Step 3 advisory" tag).
- Pass criterion (concrete threshold or distribution test).
- Output file path under `notes/ch32-rs/ti-ble/measurements/`.

Day 6 changes (signal + capture only — gating logic and pass thresholds
unchanged):

- Probe signal: was "named GPIOs"; now layered on-chip trace ring
  (`SysTick CNTL` fine + `bleClock_t` coarse + `BB[0]/BB[44]/LLE[100]/gBleIPPara`
  state samples + IRQ-entry ring) drained over SDI / RTT.
- Capture: was "logic analyzer config"; now SDI drain of the on-chip
  trace ring, optionally with sniffer pcap for protocol cross-check
  (OQ3 only).

§8 cross-reference table makes the Step-2-gate selection explicit:
**only OQ3 is the Step 2 PASS gate** (T_IFS turnaround, matching roadmap
line 139). OQ1 / OQ2 / OQ4 / OQ5 are advisory Step 3 inputs. §8 honesty
bound restates the conservative-interpretation fallback if a probe cannot
be executed.

### Day 6 amendment — measurement-framework correction (Andelf `d9d157c4` callout)

**Trigger.** Andelf msg `d9d157c4` (2026-05-14 01:01) flagged that the
GPIO + logic analyzer framing in §8 (and inherited downstream into
Cindy's SCAN_RSP spec §6.4) treated the CH32V208 BLE block like an
external BLE module. It is not — it is an on-chip peripheral and we
own the firmware + complete CPU + the BLE block's full register and
IRQ surface.

**Root cause (Lucy side).** Day 5 §8 author (this doc) imported the
"signal = GPIO toggle, capture = logic analyzer" template from
TI BLE-Stack eval-board documentation conventions without performing
the "片上 vs 片外 视角自检" step. The Day 5 cross-check pass also
missed this — the audit checked mask/bit-position consistency and TI
§7.7.2 over-inference, but did not check whether the measurement
methodology was consistent with the platform's observability surface.

**Second hallucination caught (Vega + Cindy independently).** A
follow-up Day 6 reframe proposal initially named RV32 `mcycle` CSR as
the fine-timing source. Cindy `d7256a7b` and Vega `7411d028` both
confirmed empirically that QingKe V4F does not implement
CSR 0xB00 — reads return 0 (see `ble_peripheral_phase1_adv.rs`
comment retaining `mcycle()` only as a non-timing entropy source).
The validated on-chip fine-timing source is `SysTick CNTL` at
`0xE000_F008`, 12 MHz, air-PASS validated on v4a / v4b / v4c.

**Resolution.** §8 now writes:

- Fine timing source: `SysTick CNTL` only — `mcycle` explicitly listed
  as DISABLED in the trace-framework table.
- Coarse timing source: `bleClock_t` (consistent with §3.6 VERIFIED
  rows).
- State evidence: direct BB / LLE / RFEND register reads at
  instrumented code points + SDI log.
- IRQ evidence: BB / LLE / RFEND IRQ-entry ring (consistent with §3.5
  IRQ table).
- Protocol: sniffer pcap + phone scanner (unchanged) — but pcap is
  NOT in the internal-timing gate.
- GPIO + LA: removed from every OQ gate; future optional aux only.

**Lucy-side takeaways recorded for next Step 1 / probe DoD writing:**

1. "ISA spec ≠ MCU implementation" — before naming any RV32 CSR as a
   timing source, check the vendor implementation and existing
   air-validated workspace usage (`ble_peripheral_phase1_adv.rs`
   comment was already in repo).
2. Add "片上 vs 片外 视角自检" as a discrete check in any future
   Step 1 doc cross-check pass: for every probe, ask "do we own the
   instrumented surface, or are we treating it as a black-box module?"
3. Generated-question filter: when proposing a "what GPIO pin" lock
   question for a片上 peripheral, the question itself is the smell —
   default answer is "no GPIO needed, use on-chip trace ring".

---

*End of doc. §1-§8 complete. Day 5 cross-check pass complete + Day 6
measurement-framework correction landed (`d9d157c4` callout resolved).
Step 1 deliverable remains the Step 2 implementation entry; SCAN_RSP
spec §6.4 + §4.1 carry the matching片上 trace ring framing. Migration
of source disasm docs from agent workspace to
`ch32-hal/notes/ch32-rs/ti-ble/` is an outstanding admin item, not a
review blocker — disasm anchor citations work today by reading from
the agent workspace.*
