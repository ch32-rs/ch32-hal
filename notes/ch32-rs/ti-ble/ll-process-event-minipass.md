# LL_ProcessEvent + gBleLlPara[0x74] mini-pass

**Scope** — Resolve two open items flagged by Cindy in Phase 2 review (msg `e1b43937`):
- M1: `LL_ProcessEvent` event-bit dispatch table → confirm event bits 1/2 routing
- M2: `gBleLlPara[0x74]` (= `llAdvTraverseallChannel`) call-site xref → classify the "paradox"

**Artifacts** — `/tmp/wchble-disasm/LL_ProcessEvent.dump` (from `ll.o`), `/tmp/wchble-disasm/llAdvTraverseallChannel_v2.dump` (from `ll_advertise.o`), xref via `objdump -dr` across all 69 `.o` files in `libwchble.a`.

**Outcome** — Both items resolved with VERIFIED evidence. Phase 2 doc Q2 inferred → verified. Phase 2 §4 "paradox" resolved: `llAdvTraverseallChannel` IS the event-bit-2 TMOS dispatch target; the tail-call to `ll_advertise_tx` is its normal scheduled work.

---

## M1 — LL_ProcessEvent event-bit dispatch (VERIFIED)

`LL_ProcessEvent(taskId a0, events a1)` reads bits of `events` (16-bit, sign-extended), and for each set bit calls a function pointer from `gBleLlPara`, then returns `events ^ <bit>` so TMOS clears the handled bit.

| Event bit | Mask    | Vtable read | Symbol (Phase 2 §2) | Verified at |
|-----------|---------|-------------|---------------------|-------------|
| 0x0001    | bit 0   | `gBleLlPara[0x70]` (`lw 112`) | `llAdvertiseStart` | LL_ProcessEvent.dump line 56–65 |
| 0x0002    | bit 1   | `gBleLlPara[0x74]` (`lw 116`) | `llAdvTraverseallChannel` | LL_ProcessEvent.dump line 78–89 |
| 0x0004    | bit 2   | `gBleLlPara[0x80] → +0x6c` (nested vtable) | conn/slave start | line 99–115 |
| 0x0008    | bit 3   | `gBleLlPara[0x80] → +0x70` (nested vtable) | conn/slave process | line 117–135 |
| 0x0010    | bit 4   | `gBleLlPara[0x94] → +0x8c` (nested vtable) | initiate/master | line 137–155 |
| 0x0020    | bit 5   | `gBleLlPara[0x94] → +0x90` (nested vtable) | scan path | line 156–177 |
| 0x4000    | bit 14  | `LL_TransmitterTest` direct (DTM) | DTM TX test | line 179–214 |
| 0x8000    | bit 15  | `tmos_msg_receive` (SYS_EVENT_MSG) | OSAL inbox | line 17–48 |

**Key disasm (event-bit-1 routing — line 76–89):**
```
.L11:
  68:  and a5, a1, 2          ; bit 0x0002
  6c:  beqz .L13              ; not set → next test
  6e:  auipc a5, 0; mv a5,a5   ; pc-rel load gBleLlPara base
  76:  lw   a5, 116(a5)       ; a5 = gBleLlPara[0x74]
  78:  beqz a5, .L14          ; NULL guard
  7a:  jalr a5                ; indirect call → llAdvTraverseallChannel
.L14:
  7c:  xor  a0, s0, 2          ; return events ^ 0x02
```

**Conclusion**: TMOS event bit `0x02` for the LL task **deterministically** dispatches to `gBleLlPara[0x74]`, which `LL_AdvertiseEnalbe` populates with `&llAdvTraverseallChannel`. This is the **only** event-driven entry into the ADV TX cycle after `llAdvertiseStart` does the first kick.

---

## M2 — gBleLlPara[0x74] / llAdvTraverseallChannel call-site classification (VERIFIED)

**Xref strategy** — symbol-level scan (`llAdvTraverseallChannel`) across all 69 `.o`, plus `gBleLlPara+0x74` PCREL_HI20 relocations.

**Findings**:
| Site | File | Function | Operation |
|------|------|----------|-----------|
| 1 | `ll_advertise.o` | `LL_AdvertiseEnalbe` @ 0x58 | **WRITE**: `gBleLlPara[0x74] = &llAdvTraverseallChannel` (Phase 2 §2) |
| 2 | `ll.o` | `LL_ProcessEvent` @ 0x76 | **READ + INDIRECT-CALL** via `lw 116(gBleLlPara); jalr` (M1 above) |

**No other call site exists.** No code path direct-calls `llAdvTraverseallChannel` by symbol. The only invocation route is event-bit-2 dispatch through `LL_ProcessEvent`.

**Phase 2 §4 paradox resolved**:
- Phase 2 doc flagged `llAdvTraverseallChannel`'s `tail-call ll_advertise_tx` as paradoxical (hypothesized A=niche-aux, B=fallback-restart, C=unreachable bug-shape).
- **Hypothesis B confirmed**: this function IS the normal per-event ADV-cycle dispatcher. Each time `ll_advertise_event_closed → tmos_set_event(task, 2)` fires, TMOS re-enters via this function on its next scheduler tick and the tail-call to `ll_advertise_tx` is the next channel's TX.

**Function decode (llAdvTraverseallChannel_v2.dump, 88 lines)**:
```
llAdvTraverseallChannel():
  s0 = gBleLlPara[0x64]            ; current advState pointer
  if s0->byte_12 != 1: return       ; guard — only run when ADV is active
  if gBleLlPara[0x7c] == 4:
      ll_advertise_event_closed(s0) ; finalize previous event
  tail-call ll_advertise_tx(s0)     ; issue next channel TX
```

**Newly observed state fields** (deferred for Phase 2.5/3 deep decode):
- `advState.byte_12` — `1` = ADV active (set by `llAdvertiseStart`, cleared by `LL_AdvertiseToStandby`). Distinct from `byte_11` (per-PDU stamp) and `byte_10` (current channel).
- `gBleLlPara[0x7c]` — global "operation mode" byte; `== 4` triggers event-closed cleanup before next TX. Likely matches the BLE LL `ll_state` enum used by other subsystems (init/scan/conn/adv).

---

## Re-derived flow (now fully verified)

```
TMOS task tick: events |= 0x02
       │
       ▼
LL_ProcessEvent(taskId, events):
   if events & 0x02 && gBleLlPara[0x74]: call gBleLlPara[0x74]   (= llAdvTraverseallChannel)
       │
       ▼
llAdvTraverseallChannel(advState):
   if advState.byte_12 != 1: return       # not active → no-op
   if gBleLlPara[0x7c] == 4:
       ll_advertise_event_closed(advState)  # close prior event (clear state, rotate channel?)
   tail-call ll_advertise_tx(advState)
       │
       ▼
ll_advertise_tx:
   stamp byte_11 (0x92/0x93/0x94 depending on PDU type)
   program LLE TX registers + DMA
   call ll_advertise_process(advState)
       │
       ▼
ll_advertise_process:
   busy-wait TX done (3-source: gBleIPPara[2].b0 || gBleIPPara[3].b0 || LLE[100]==0)
   dispatch on byte_11 via .L440 jump table → RX window / SCAN_RSP / event_closed / etc.
       │
       ▼ (eventually)
ll_advertise_event_closed:
   tmos_start_task(self, 2, advDelay_ticks)   # arm next event-bit-2 fire
       │
       ▼
(back to TMOS scheduler — loop)
```

---

## Phase 2 doc updates required

- **§0.Q2** — REMOVE `⚠️ Evidence level: PARTIAL / INFERRED` block. Replace with `✅ VERIFIED via mini-pass M1 (`LL_ProcessEvent` event-bit-1 → `gBleLlPara[0x74]` indirect call → `llAdvTraverseallChannel`)`. Upgrade per-step `_Inferred_:` to `_Verified_:` for steps 1–2.
- **§1 call-graph** — Replace `LL_ProcessEvent (inferred event-bit-1 → ll_advertise_process)` with `LL_ProcessEvent (verified event-bit-1 → llAdvTraverseallChannel; ll_advertise_process is inner state-machine, not event-direct)`.
- **§4 PHASE 2.5 MUST-CHECK #1** — Replace the `🚨` callout with `✅ RESOLVED in mini-pass M2`. Note hypothesis B (fallback-restart / normal dispatcher) confirmed. Drop hypotheses A & C.
- **§5** — Clarify that `ll_advertise_process` is called from `ll_advertise_tx`, `ll_advertise_aux_tx`, `ll_advertise_aux_chain_tx` (3 internal sites + 2 cross-file: `ll_periodic.o`, `ll_sync.o`), NOT directly from TMOS dispatch.
- **§9.1 deferred** — Add: decode `advState.byte_12` (active flag) + `gBleLlPara[0x7c]` (op-mode byte) semantics during Phase 2.5 RX ingress decode.
- **Iron Rule §14** — Replace `out-of-scope until classified` with `Rust ADV scheduler MUST treat event-bit-2 dispatch as the canonical re-entry; do not invent parallel re-entry points`.

## Iron Rules added (carry to Phase 3 / Rust impl)

15. **Single re-entry point for ADV cycle** — In the Rust port, only ONE TMOS event bit (== 0x02 equivalent) drives the per-channel TX cycle after the initial `llAdvertiseStart`-kick. No alternate timer or IRQ shortcut.
16. **NULL-guard the vtable** — `LL_ProcessEvent` checks `if gBleLlPara[0x74] != NULL` before indirect-call. Rust scheduler must use `Option<fn>` (or equivalent) and skip cleanly if the ADV subsystem isn't initialized.
17. **Active-flag gate (byte_12)** — `llAdvTraverseallChannel` does a hard `byte_12 == 1` guard. Rust port must reproduce: `if !adv_state.is_active() { return }` at the top of the equivalent handler. This is the de-facto "stop" path when host has called `LL_AdvertiseShut` mid-cycle.

---

## Verification commands (re-runnable)

```bash
# M1: LL_ProcessEvent dispatch
cd /tmp/wchble-disasm && riscv64-unknown-elf-objdump --disassemble=LL_ProcessEvent -r ll.o

# M2: symbol-level xref
cd /tmp/wchble-disasm/all_obj && \
  for f in *.o; do
    n=$(riscv64-unknown-elf-objdump -dr "$f" 2>/dev/null | grep -c 'llAdvTraverseallChannel')
    [ "$n" -gt 0 ] && echo "$f: $n"
  done

# llAdvTraverseallChannel full decode
cd /tmp/wchble-disasm && riscv64-unknown-elf-objdump -dr all_obj/ll_advertise.o --disassemble=llAdvTraverseallChannel
```

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-process-event-minipass.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
