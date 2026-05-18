# Phase 2 — ADV Scheduler / Channel Hop / Interval (reverse-engineered)

Subject: how WCH `libwchble.a` schedules an advertising **event** (timing + per-channel hop)
Files: `ll_advertise.o` (`llAdvertiseStart`, `llAdvTraverseallChannel`, `ll_advertise_process`, `llAdvertiseTimeout`, `LL_AdvertiseEnalbe`)
Companion: Phase 0 `ll-advertise-tx-disasm.md`, Phase 1 `ll-tx-completion-disasm.md`
Scope: per Cindy locked plan + 2 verification points + 1 Rust scheduler hard constraint.

---

## 0. TL;DR — direct answers to Cindy's two verification points

### Q1. `byte_25 / byte_26` — channel index, channel count, or status helper?

**Answer**: They are the **second and third channel indices** queued for the current adv event.

- `adv_state.byte_10` = current channel being TX'd
- `adv_state.byte_25` = next channel after current
- `adv_state.byte_26` = third (last) channel of the event

`llAdvertiseStart` calls `tmos_rand() & 3` to pick one of four rotations of {37, 38, 39}, then writes the triple into `byte_10/25/26`. `ll_advertise_process .L450` ( the byte_11=0x94 handler ) reads the triple and tail-calls `ll_advertise_tx` for the next index, or exits to `ll_advertise_event_closed` when the triple is exhausted.

Decision tree at `.L470` (top of channel hop):

```
load a4 = byte_25, a5 = byte_26
if byte_25 == 0:               goto .L471  ; bypass / scan_rsp-only
elif byte_25 == byte_10:       goto .L472  ; current already at second
elif byte_10 == byte_26:       goto .L473  ; current is third = end
else:
    byte_10 = byte_25          ; advance to second
    call ll_advertise_tx       ; (.L537)
    fall-through to .L473
.L495 (when byte_10 == byte_25):
    byte_10 = byte_26          ; advance to third
    call ll_advertise_tx       ; (.L537)
```

### Q2. `tmos_set_event(task, 2)` → next ADV TX → how does byte_11 reach `0x94/0x95` to re-trigger TX?

> **✅ VERIFIED via mini-pass M1 + M2** (see `ll-process-event-minipass.md`)
> Both the event-bit dispatch table (M1, `LL_ProcessEvent` in `ll.o`) and the `gBleLlPara[0x74]` call-site xref (M2) are now ground-truth.
> Previous hypothesis "event mask 0x0002 → `ll_advertise_process`" was **wrong**. Event mask `0x0002` (event bit 1) → `llAdvTraverseallChannel`, which **tail-calls `ll_advertise_tx`**, which then calls `ll_advertise_process` as its inner post-TX state machine. The byte_11 stamping path is unchanged.
>
> **Terminology lock (Cindy `9d9ca082`)**: All event-bit references in this doc use **mask value + bit position**, e.g. *"event mask 0x0002 / event bit 1"*. TMOS `tmos_set_event(task, mask)` takes a **mask** argument (e.g. `mask = 2 = 1<<1`), and `LL_ProcessEvent` tests with `and a5, a1, <mask>`. Avoid loose phrasing like "bit 2" that conflates mask value and bit position.

**Answer**: `byte_11` is **not "reset to 0x94"**. It is **stamped fresh by `ll_advertise_tx`** every call.

The verified flow:

1. _Verified (M1)_: **event mask 0x0001 / event bit 0** (timer-armed by `tmos_start_task(task, 1, interval+rand_delay)`): TMOS calls `LL_ProcessEvent`, which on `events & 0x0001` does `lw a5, 112(gBleLlPara); jalr a5` → indirect-call to `gBleLlPara[0x70]` = `llAdvertiseStart`. `llAdvertiseStart` sets `gBleLlPara[0x7c] = 1` (event-start stamp), randomizes channel order, then tail-calls `ll_advertise_tx(adv_state)`. `ll_advertise_tx` stamps `adv_state.byte_11 = 0x94` (or 0x92 / 0x93 by PDU type) and busy-waits TX completion.
2. _Verified (M1+M2)_: **event mask 0x0002 / event bit 1** (`tmos_set_event(task, 2)` from `ll_advertise_event_closed`): TMOS re-invokes `LL_ProcessEvent`, which on `events & 0x0002` does `lw a5, 116(gBleLlPara); jalr a5` → indirect-call to `gBleLlPara[0x74]` = `llAdvTraverseallChannel`. `llAdvTraverseallChannel` guards on `adv_state.byte_12 == 1` (active), optionally calls `ll_advertise_event_closed` when `gBleLlPara[0x7c] == 4`, and **tail-calls `ll_advertise_tx(adv_state)`**. `ll_advertise_tx` stamps `byte_11` again and calls `ll_advertise_process` for post-TX work.
3. _Verified_: Within `ll_advertise_process`, the `.L450` channel-hop path **calls `ll_advertise_tx` directly** (`.L537`, plain `jalr`, not tail-call), busy-waits TX through `ll_tx_wait_finish`, then falls through to `.L473`. `byte_11` was just re-stamped 0x94 inside `ll_advertise_tx`, so a re-loop via `.L437` re-dispatches to `.L450` for the next channel.
4. _Verified_: Loop exits when channel triple is exhausted (`.L541` → `ll_advertise_event_closed`) **or** retry needed (`byte_11 := 0x95` stored at `.L535` → table[3] = `.L449` next iteration).

So the "re-trigger" mechanism is **not byte_11 magic**; it's that `ll_advertise_tx` always stamps byte_11 to the current PDU type's state, and `ll_advertise_process` (called from within `ll_advertise_tx`) re-loops by reading that stamp. The mask-0x0002 re-entry is structured as `llAdvTraverseallChannel → tail-call ll_advertise_tx → ll_advertise_process (inner)`, NOT as a direct event → `ll_advertise_process` call.

### Hard Rust constraint (locked from Cindy `697c9705` follow-up)

> **Iron Rule §0 (Phase 2)**: The first Rust ADV scheduler **MUST** drive the WCH register sequence verbatim, per channel. For each of the three ADV channels (37/38/39 in random rotation), call `tx_adv_pdu()` which writes the **full** BB / LLE / RFEND sequence from Phase 0 (no delta optimization, no shared-prelude assumption). Optimization (delta writes, batched setups) is forbidden until first-air PASS lands.

---

## 1. Function call graph (scheduler-side)

> **Notation** (locked by Cindy `9d9ca082`):
> - `gBleLlPara[0xNN]` uses hex byte-offsets throughout. Where the disasm dump shows decimal `lw a5, NN(...)` we translate to `0xNN` form here. E.g. `gBleLlPara[0x7c]` = the byte at offset 124 (decimal) from base.
> - Event-bit references use **mask + bit-position** form: e.g. *"event mask 0x0002 / event bit 1"*. `tmos_set_event(task, mask)` takes a **mask** value; the `LL_ProcessEvent` `and a5, a1, <mask>` test confirms the mask form.

```
LL_AdvertiseEnalbe                      ← public enable entry
├─ TMOS_ProcessEventRegister(LL_ProcessEvent)
├─ LL_Init  (one-shot init)
└─ gBleLlPara[0x68] = llAdvertiseCreateCore   ← vtable populator
   gBleLlPara[0x6c] = llAdvertiseSet
   gBleLlPara[0x70] = llAdvertiseStart
   gBleLlPara[0x74] = llAdvTraverseallChannel

LL_ProcessEvent (in ll.o, ✅ VERIFIED — mini-pass M1, see ll-process-event-minipass.md)
├─ event bit 0 (mask 0x01) → gBleLlPara[0x70] = llAdvertiseStart           [interval timer fires here]
├─ event bit 1 (mask 0x02) → gBleLlPara[0x74] = llAdvTraverseallChannel    [event_closed re-arms here]
├─ event bit 2 (mask 0x04) → gBleLlPara[0x80]->[0x6c]                       [conn/slave]
├─ event bit 3 (mask 0x08) → gBleLlPara[0x80]->[0x70]                       [conn/slave]
├─ event bit 4 (mask 0x10) → gBleLlPara[0x94]->[0x8c]                       [initiate/master]
├─ event bit 5 (mask 0x20) → gBleLlPara[0x94]->[0x90]                       [scan]
├─ event bit 14 (mask 0x4000) → LL_TransmitterTest                          [DTM]
└─ event bit 15 (mask 0x8000) → tmos_msg_receive                            [SYS_EVENT_MSG]

llAdvertiseStart
├─ gBleIPPara[7] gate (state == 5/3/11 → tmos_set_event(task,1) defer)
├─ TempSample tick (every 1s of adv)
├─ EXT_ADV duration check → HCI terminate cb → LL_AdvertiseToStandby
├─ gBleLlPara[0x7c] := 1                      (LL op-mode byte — event-start stamp; 0x7c = 124)
├─ channel-order randomization (tmos_rand & 3 → byte_10/25/26 triple)
├─ tmos_start_task(task, 1, hword_32 + rand_delay)   (arms NEXT adv interval)
└─ tail-call ll_advertise_tx                  (first channel of event)

ll_advertise_tx  (Phase 0)
├─ stamp byte_11 = 0x92 / 0x93 / 0x94
├─ stamp gBleLlPara[0x7c] = 2 or 3 (LL op-mode byte — TX in progress)
└─ tail-call ll_tx_wait_finish(0, buf, len)

ll_tx_wait_finish  (Phase 1)
├─ mode 0: BB kick (LLE[0]=2), wait for TX done
└─ busy-poll gBleIPPara[2]/[3] bit 0 || LLE[100]==0

ll_advertise_process  (state machine, this doc §3)
├─ .L437 loop top:  byte_11+110 → 13-entry jump table .L440
├─ table dispatches to .L452 (0x92) .. .L439 (0x9e), default .L438
├─ .L450 = channel hop dispatcher (calls ll_advertise_tx per next channel)
├─ .L541 = call ll_advertise_event_closed (user cb)
└─ .L543 = call ll_advertise_status_closed (convergence point)

llAdvTraverseallChannel                       ← vtable[0x74], dispatched by LL_ProcessEvent on event mask 0x0002 (event bit 1)
├─ load s0 = gBleLlPara[0x64] (active adv state ptr; numeric offset 100)
├─ if (s0->byte_12 != 1) return               ; adv disabled
├─ if (gBleLlPara[0x7c] == 4) call ll_advertise_event_closed   ; op-mode byte; 0x7c = 124
└─ tail-call ll_advertise_tx                   ; next channel TX (always, regardless of prior branch)

llAdvertiseTimeout                            ← duration timeout cb (started by llAdvertiseSet via tmos_start_callback_task)
├─ adv_state[22] = -1  ; timeout flag
├─ if (hword_110 != 0): HCI_LE_AdvertisingSetTerminatedEvent(60,...)
│  else if (byte_14&0xf==1 || byte_96&4): HCI_LE_ConnectionCompleteCback(60,...)  ; DIRECT timeout
├─ phy_status_clear(2)
└─ tail-call LL_AdvertiseToStandby
```

---

## 2. `LL_AdvertiseEnalbe` — vtable populator

```asm
LL_AdvertiseEnalbe:
  a5 = gTmosPara[3]                    ; current task id
  if (a5 != 0) skip TMOS register
  a0 = &LL_ProcessEvent
  call TMOS_ProcessEventRegister(a0)
  call LL_Init
.L657:
  gBleLlPara[0x68] = &llAdvertiseCreateCore
  gBleLlPara[0x6c] = &llAdvertiseSet
  gBleLlPara[0x70] = &llAdvertiseStart
  gBleLlPara[0x74] = &llAdvTraverseallChannel
  ret
```

**Implication**: ADV scheduling is **vtable-driven** at the LL boundary. `LL_ProcessEvent` (in `ll.o`) consumes events and dispatches through these slots. The vtable is the protocol-handler hook point.

Vtable map (verified):

| Offset | Symbol | Role |
| ------ | ------ | ---- |
| 0x68 | `llAdvertiseCreateCore` | LL adv-set construction |
| 0x6c | `llAdvertiseSet` | parameter set + timer arm |
| 0x70 | `llAdvertiseStart` | event start (interval-armed) |
| 0x74 | `llAdvTraverseallChannel` | per-channel re-dispatch |

Other gBleLlPara offsets observed in `ll_advertise_process`:
- 0x78 = 120: indirect function ptr called at `.L439` (state 0x9e), purpose TBD (probably scan-rsp completion cb)
- 0x7c = 124: **adv-event progress byte** (value flow: `0 → 1 (Start) → 2/3 (TX in progress) → 4 (channels exhausted) → 0 (cleared by event_closed)`); ALSO bit-flag at bits 0x10/0x20/0x30 used by aux/ext paths
- 0x7d = 125: bit 0 = early-exit flag used by aux paths
- 0x64 = 100: pointer to active adv state struct (write at `llAdvertiseStart .L555: sw s0, 0(a4)`)

---

## 3. `llAdvertiseStart` — event start (the heartbeat)

Entry: TMOS event bit (1) fires after `tmos_start_task(task, 1, interval+rand_delay)`. `LL_ProcessEvent` dispatches to `gBleLlPara[0x70]` = `llAdvertiseStart`.

### 3.1 Gate (early exit)

```
s0 = gBleLlPara[88]                     ; adv state ptr
if s0 == NULL goto .L550 (exit)
if s0->byte_12 != 1 goto .L550          ; adv not enabled
a2 = gBleIPPara[7]                      ; global LL state
if (a2 == 5 || gBleIPPara[7] == 3 || gBleIPPara[7] == 11):
    tail-call tmos_set_event(gTmosPara[3], 1)   ; defer to next round
```

`gBleIPPara[7]` = global LL state machine byte. Values 5/3/11 = unsuitable for ADV (connection in progress / standby transitioning / etc). Defer by re-arming event 1.

### 3.2 Temperature sample tick (every ~1s of adv)

```
.L555:
  a5 = gBleLlPara[0]                    ; temp-sample tick counter (byte)
  a3 = adv_state.hword_32               ; ADV interval (in 0.625ms units)
  a5 = a5 + 1                           ; increment
  gBleLlPara[0] = a5                    ; store
  a4 = 1600 / a3                        ; events per second (1600 ticks/sec)
  if a5 < a4: skip TempSample
  else:
    call TMOS_TempSample()
    gBleLlPara[0] = 0                   ; reset counter
```

**Time base**: 1600 = 1 second / 0.625ms = 1600 ticks. So `1600 / interval_ticks` = events_per_second. Counter ticks once per event; when it reaches events_per_second, that's one second elapsed → sample temperature.

This is a **periodic-housekeeping anchor** independent of adv interval choice. For Rust scheduler: don't try to be clever — just count events, every `1600 / interval_ticks` events do temperature sample.

### 3.3 EXT_ADV duration check (event type 7)

```
.L556:
  if adv_state.byte_14 == 7:            ; EXT_ADV
    if adv_state.byte_105 != 0:         ; duration max set
      if adv_state.hword_112 >= byte_105:    ; counter hit max
        adv_state.byte_23 = 67          ; status code HCI advertising terminated
        call HCI_LE_AdvertisingSetTerminatedEvent(67, addr_type, count, 0)
        tail-call LL_AdvertiseToStandby
      else:
        adv_state.hword_112 += 1        ; increment count
    call ble_ll_common_rand8(1, 35)
    adv_state.byte_98 = a0              ; new random DID for EXT_ADV
```

Status code **67** (0x43) = "HCI_BLE_LIMIT_NUM_REACHED" or similar — duration max reached, terminate adv set.

### 3.4 Event-start stamp + channel randomization

```
.L557:
  gBleLlPara[124] = 1                   ; ← event-start global stamp
  
  if adv_state.byte_24 == 3:            ; standard 3-channel adv
    call tmos_rand()
    a0 &= 3                             ; 0/1/2/3 → 4 rotation choices
    
    rand == 0:                          ; rotation: 39, 37, 38
      byte_10 = 39
      byte_25 = 37
      byte_26 = 38
    rand == 1:                          ; rotation: 38, 37, 39
      byte_10 = 38
      byte_25 = 37
      byte_26 = 39
    rand == 2:                          ; rotation: 37, 39, 38
      byte_10 = 37
      byte_25 = 39
      byte_26 = 38
    rand == 3:                          ; rotation: ?  (.L563)
      byte_10 = 37
      byte_25 = ?  (likely 38)
      byte_26 = ?  (likely 39)
```

(Exact rotation #3 left to verify in a separate full-trace pass.)

For non-3-channel adv (byte_24 != 3, e.g. EXT_ADV with subset), more complex byte_15 bit-flag logic at `.L560 / .L564` picks channels.

### 3.5 Interval arming + first TX

```
.L562 / .L611:
  a4 = adv_state.byte_14                ; event type
  if a4 == 1:                           ; ADV_DIRECT (high-duty-cycle?)
    c = adv_state.hword_32              ; interval, no random delay
  else:
    .L575:
      call ble_ll_common_rand8(1, 14)   ; rand 0-13
      c = adv_state.hword_32 + a0       ; interval + advDelay
  
  call tmos_start_task(gTmosPara[3], 1, c)   ; ARM NEXT ADV EVENT
  
  ; SCAN_RSP filter / RPA prefill (if conditions met):
  if (adv_state[53] & 2) && adv_state[60] && adv_state[48] && adv_state[48]->byte_10:
    adv_state[52] = 2
    tmos_memcpy(adv_state + 54, adv_state[48] + 12, 6)
  
  .L578:
    if adv_state.byte_124 > 2:          ; (per-struct byte, NOT gBleLlPara[124])
      .L579 deep path: bleClock_t + fnGetClockCBs deadline check
    else:
      goto .L585 → tail-call ll_advertise_tx
  
  .L585:                                ; common path → fire first channel
    pop frame
    tail-call ll_advertise_tx(adv_state)
```

**Interval semantics**:
- `adv_state.hword_32` is the advertising interval in 0.625 ms units (BLE std, matches PDF).
- `advDelay` = `ble_ll_common_rand8(1, 14)` returns 0–13 (inclusive of the bounds? probably 0–13). Multiplied by 0.625 ms → 0–8.125 ms. BLE spec says advDelay = 0–10 ms; 13 × 0.625 = 8.125 ms is conservative.
- For `ADV_DIRECT_IND` (high duty cycle), advDelay is skipped — directs need tight back-to-back timing.
- `tmos_start_task(task_id, event_bit=1, ticks)` arms the **next** adv event at `ticks` ticks from now (relative). When timer expires, TMOS sets event bit 1 in the task's event mask → `LL_ProcessEvent` runs → re-calls `llAdvertiseStart`.

**Rust scheduler contract (from Iron Rule §0 + Cindy `e1b43937` amendment #3)**:
```rust
struct AdvEvent {
    ch_seq: [u8; 3],            // (byte_10, byte_25, byte_26) — rotation of [37,38,39]
    current_ch: u8,             // mirrors WCH byte_10 — advances per channel
    interval_ticks: u16,        // mirrors hword_32 — 0.625ms units
    rand_delay_ticks: u8,       // 0–13 for non-DIRECT, 0 for DIRECT
    pdu_buf: *const u8,         // mirrors adv_state[76] → DMA into LLE[112]
    pdu_len: u8,
    // ...
}

/// Three-way TX completion observation — mirrors Phase 1's
/// `gBleIPPara[2]&1 | gBleIPPara[3]&1 | LLE[100]==0` triple poll.
/// First-air Rust MUST preserve all three observations (do not collapse into bool).
struct TxDoneSource {
    ok: bool,             // gBleIPPara[2] bit 0  — TX ISR clean completion
    abort_or_err: bool,   // gBleIPPara[3] bit 0  — TX error / abort path
    lle_idle: bool,       // LLE[100] == 0        — hardware-level idle
}

impl TxDoneSource {
    fn done(&self) -> bool { self.ok || self.abort_or_err || self.lle_idle }
    /// Returns true only if exactly one source confirmed completion.
    /// Diagnostic: multi-source completion may indicate ISR race or stuck state.
    fn unique_source(&self) -> bool { self.ok as u8 + self.abort_or_err as u8 + self.lle_idle as u8 == 1 }
}

// Per channel:
fn tx_adv_pdu(state: &mut AdvState) -> TxDoneSource {
    // Full BB/LLE/RFEND write sequence verbatim (Phase 0 §1-§9)
    // NO delta optimization
    // Returns the three-way completion observation, NOT a bool.
}
```

### 3.6 `llAdvertiseStart` exit paths

- `.L550`: pure early-exit (state == NULL or disabled or LL busy)
- `.L554`: defer via `tmos_set_event(task, 1)`
- `.L585`: tail-call `ll_advertise_tx` (fire first channel)
- Through `.L579` deep path: bleClock-based deadline check, then either `.L585` (fire) or `.L550` (skip this event)

---

## 4. `llAdvTraverseallChannel` — event-bit-1 TMOS dispatcher (RESOLVED)

> **✅ RESOLVED via mini-pass M2** (see `ll-process-event-minipass.md`)
> Xref of `llAdvTraverseallChannel` and `gBleLlPara[0x74]` across all 69 `.o` files in `libwchble.a` shows **exactly two sites**:
> - WRITE: `LL_AdvertiseEnalbe` populates `gBleLlPara[0x74] = &llAdvTraverseallChannel` (Phase 2 §2)
> - READ + INDIRECT-CALL: `LL_ProcessEvent` (`ll.o`) on `events & 0x0002`
>
> Hypothesis **B confirmed** (fallback-restart / normal dispatcher). Hypotheses A & C dropped.
> This function IS the canonical event-bit-1 (mask 0x02) TMOS re-entry point for the ADV cycle after `ll_advertise_event_closed` arms it. The tail-call to `ll_advertise_tx` is its normal scheduled work — not a paradox.

**Compact** (only 36 instructions). Reads global state, decides if more channels remain.

```
llAdvTraverseallChannel:
  a5 = &gBleLlPara
  s0 = gBleLlPara[100]                  ; active adv state ptr (gBleLlPara[0x64])
  if s0->byte_12 != 1: goto .L358 (exit) ; adv not active — silent no-op
  a5 = gBleLlPara[124]                   ; gBleLlPara[0x7c] op-mode byte
  if a5 == 4:                           ; previous event finalized cleanly
    call ll_advertise_event_closed(s0)  ; wrap up state, advance bookkeeping
.L360 (fall-through):
  tail-call ll_advertise_tx(s0)         ; issue next TX (channel determined inside)
.L358:
  pop frame; ret                        ; ADV inactive — bail
```

Re-interpretation post-mini-pass: the `byte_12 != 1` early-return is the **clean shutdown latch** for `LL_AdvertiseShut`. The `gBleLlPara[0x7c] == 4` branch finalizes the previous event; then the tail-call to `ll_advertise_tx` either (a) issues the next channel TX within the same triple, or (b) re-starts a fresh event triple — `ll_advertise_tx` reads `adv_state.byte_10` (current channel) which has been advanced by `ll_advertise_event_closed` or by `ll_advertise_process .L450` channel-hop.

**Rust scheduler implication**: this is THE event-driven entry, not a niche path. Iron Rule §14 below is updated accordingly.

**Newly observed state fields** (deferred for Phase 2.5 RX-ingress decode):
- `adv_state.byte_12` — `1` = ADV active; cleared by `LL_AdvertiseToStandby` / `LL_AdvertiseShut`.
- `gBleLlPara[0x7c]` — global "LL operation mode" byte; `== 4` triggers `event_closed` pre-call.

---

## 5. `ll_advertise_process` — inner post-TX state-machine

> **Note (post-mini-pass)**: `ll_advertise_process` is **NOT** an event-bit handler. It is called from `ll_advertise_tx` (2 sites), `ll_advertise_aux_tx`, `ll_advertise_aux_chain_tx` within `ll_advertise.o`, plus `ll_periodic.o` and `ll_sync.o` cross-file (5 total internal sites verified via mini-pass M2). It runs as the synchronous post-TX state machine that waits for TX-done, handles RX window, and dispatches follow-up TXs.

### 5.1 Jump table `.L440` (verified via `.rodata.ll_advertise_process` relocations)

Each entry stored as `(target - .L440)` 4-byte offset, computed at link time. Table size = 13 entries × 4B = 52B = 0x34.

| Index | byte_11 | Target | Function | State semantics |
| ----: | :------ | :----- | :------- | :--------------- |
| 0 | 0x92 | `.L452` @ 0xea | post-DIRECT_IND TX → maybe RX or close | "DIRECT TX done" |
| 1 | 0x93 | `.L451` @ 0x11e | post-SCAN_RSP TX → user cb via pGapRoles_AppCGs[1] | "SCAN_RSP TX done" |
| 2 | 0x94 | `.L450` @ 0x26a | **post-common-ADV TX → channel hop** | "ADV TX done, next channel" |
| 3 | 0x95 | `.L449` @ 0x316 | aux-tx busy-wait + dispatch `ll_advertise_aux_tx` | "AUX_ADV ready" |
| 4 | 0x96 | `.L448` @ 0x32a | aux-chain handler (bit 0x10/0x20 of gBleLlPara[124]) | "AUX chain stage 1" |
| 5 | 0x97 | `.L447` @ 0x42e | post-AUX TX → close BB | "AUX chain TX done" |
| 6 | 0x98 | `.L446` @ 0x3be | aux post-completion check (bit-flag in gBleLlPara[124]) | "AUX completion" |
| 7 | 0x99 | `.L445` @ 0x474 | scan_rsp completion + user-cb cleanup | "SCAN_RSP cb done" |
| 8 | 0x9a | `.L444` @ 0x52e | connect transition: call `ll_advertise_to_connection_state` + `LL_AdvertiseToStandby` | "CONNECT_IND received → conn" |
| 9 | 0x9b | `.L443` @ 0x66 | post-TX RX: `BLE_SetPHYRxMode` + `ll_tx_wait_finish(mode=1)` + `ble_ll_chkcrc` + `ll_advertise_legacy_rx` | "ADV TX done, listen RX" |
| 10 | 0x9c | `.L442` @ 0x20e | aux scan rx path (`ll_advertise_aux_scan_rx`) | "AUX scan RX" |
| 11 | 0x9d | `.L441` @ 0x1a6 | aux conn rx path (`ll_advertise_aux_conn_rx`) | "AUX conn RX" |
| 12 | 0x9e | `.L439` @ 0x416 | indirect call via `gBleLlPara[120]` + adv_state[124] = 3 | "scan_rsp completion cb" |
| - | 0x91 | `.L438` @ 0x204 (default) | `ble_ll_hw_api_shut` + `ll_advertise_event_closed` | "event closed / idle" |
| - | other | `.L438` (default) | same | "out of range" |

**Index math**: `(byte_11 + 110) & 0xff = index`. So:
- byte_11 = 0x92 (146) + 110 = 256 → wraps to 0 = index 0 → .L452
- byte_11 = 0x9e (158) + 110 = 268 → wraps to 12 = index 12 → .L439
- byte_11 = 0x91 (145) + 110 = 255 → out of range (`12 < 255`) → .L438 default

### 5.2 State transitions

Sources (who writes byte_11):

| Source | Writes | When |
| :----- | :----- | :--- |
| `ll_advertise_event_closed` | 0x91 | every event close |
| `ll_advertise_tx` | 0x92 / 0x93 / 0x94 | pre-TX, per PDU type (Phase 0) |
| `ll_advertise_process .L458` | 0x9b | DIRECT/peer-DIRECT TX done, expects RX |
| `ll_advertise_process .L474` | 0x95 | AUX retry (gBleLlPara[124] = 2/3) |
| `ll_advertise_process .L538` | 0x9e | aux extra completion needed |
| `ll_advertise_process .L539` | 0x9d | aux conn RX needed |
| `ll_advertise_process .L540` | 0x9c | aux scan RX needed |

This is the **state stamping graph**. byte_11 is the LL's "what's next" microstate.

### 5.3 Main path (byte_11 = 0x94, .L450) detailed

```
.L450:                                      ; entry from .L437 jump table
  ; check TX done
  a5 = gBleIPPara[2]
  if (a5 & 1) == 0: goto .L489 (TX still running → close path)
  gBleIPPara[2] = 0                          ; clear TX done flag
  
  call ble_ll_hw_api_shut                   ; release BB/LLE/RFEND
  
  ; retry check
  a5 = gBleLlPara[124] & 0xff
  if a5 == 2: goto .L469 (retry mode 1)
  if a5 == 3: goto .L469 (retry mode 2)
  else: goto .L470 (normal channel hop)

.L469: ; retry — backoff using gBleLlPara[116] (hword) and gptrLLEReg.regs[96]
  a3 = *gptrLLEReg                          ; LLE base
  a4 = adv_state.hword_116 + 300
  a5 = LLE[96] >> 1
  if a5 >= a4: goto .L470 (retry exhausted, fall through)
  byte_11 = 0x95                            ; stamp aux-tx state
  if LLE[96] >> 1 != 0: goto .L437 (re-loop)
  else: goto .L541 (event_closed)

.L470: ; ← CHANNEL HOP CORE
  a4 = adv_state.byte_25
  a5 = adv_state.byte_26
  if a4 == 0: goto .L471 (no second channel)
  a3 = adv_state.byte_10                    ; current
  if a4 == a3: goto .L472 (current already at second)
  if a3 == a5: goto .L473 (current is third = end)
  adv_state.byte_10 = a4                    ; advance current → second
  ; fall through to .L537

.L537: ; ← INVOKE TX FOR NEW CHANNEL
  a0 = s0 = adv_state
  call ll_advertise_tx(adv_state)           ; (jalr, not tail-call)
  ; ll_advertise_tx stamps byte_11 = 0x94 again
  ; ll_advertise_tx → ll_tx_wait_finish busy-waits
  ; returns here
  fall through to .L473

.L473: ; post-channel-hop check
  a5 = gBleIPPara[7] & 0xff
  if a5 != 0: goto .L437 (re-loop top — more state needed)
  a5 = gBleLlPara[124] & 0xff
  if a5 == 2: byte_11 = 0x95; goto .L437 (set aux state and re-loop)
  if a5 == 3: byte_11 = 0x95; goto .L437
  else: goto .L541 (close event)

.L471: ; byte_25 == 0 (only one channel — DIRECT)
  if byte_26 == 0: goto .L473
  if adv_state.byte_10 != 0: goto .L473
  adv_state.byte_10 = byte_26                ; switch to third
  goto .L537                                 ; fire TX

.L472: ; current already == byte_25 (already used)
  if byte_26 == 0: goto .L473 (no third)
  goto .L495 → adv_state.byte_10 = byte_26; goto .L537

.L541: ; close event
  call ll_advertise_event_closed(adv_state)
  return via .L436
```

### 5.4 Why `ll_advertise_process` is "synchronous"

Note `.L537` uses `jalr ra`, NOT `jr t1`. This means `ll_advertise_tx` is a **regular call**, not a tail-call. Inside `ll_advertise_tx`, the final action is `jr t1` to `ll_tx_wait_finish`, which busy-polls until TX completes, then `ret`'s back to `ll_advertise_process .L537+0x4`.

So one invocation of `ll_advertise_process` can:
1. dispatch via byte_11 → land in `.L450`
2. detect TX done
3. hop channel → call `ll_advertise_tx` → wait → return
4. fall through to `.L473`
5. re-loop to `.L437` → re-read byte_11 (now 0x94 again, set by step 3's call) → re-dispatch to `.L450`
6. repeat steps 2–5 for up to 3 channels
7. exit via `.L541` (close event) → `ll_advertise_event_closed` → user cb

**Single TMOS event-2 firing drives one whole multi-channel adv event from TX-done detection to close cb**. Subsequent events come from the timer-armed event-1 path.

---

## 6. `llAdvertiseTimeout` — duration timeout (separate event)

Registered with `tmos_start_callback_task` (see `llAdvertiseSet`'s line 3497 `tmos_start_callback_task` call). Fires after a configured duration to forcibly end the adv set.

```
llAdvertiseTimeout(adv_state):
  adv_state.byte_22 = -1                    ; timeout flag
  if adv_state.hword_110 != 0:              ; EXT_ADV with duration
    adv_state.byte_23 = 60                  ; HCI status: timeout
    call HCI_LE_AdvertisingSetTerminatedEvent(60, byte_8, byte_112, 0)
  elif (adv_state.byte_14 & 0xf) == 1 || (adv_state.byte_96 & 4):
    ; DIRECT_IND or connectable timed out
    if (gBleLlPara[224] & 0x200):           ; feature flag check
      call HCI_LE_EnhancedConnectionCompleteCback(60, 0, 1, ...)
    else:
      call HCI_LE_ConnectionCompleteCback(60, 0, 1, ...)
.L616:
  call phy_status_clear(2)
  tail-call LL_AdvertiseToStandby(adv_state)
```

Status code **60** (0x3C) = "HCI_BLE_ADVERTISING_TIMEOUT" / "directed advertising timeout".

**Implication for Rust**: implement two timers per adv-set:
- inter-event timer (interval + advDelay) → fire next event
- duration timer (per-set max duration) → fire timeout (only some adv types)

---

## 7. Time-base summary

| Concept | Encoding | Source |
| :------ | :------- | :----- |
| TMOS tick | 0.625 ms | PDF / `fnGetClockCBs` contract (1600 Hz) |
| `tmos_start_task(t,e,n)` | arm event `e` after `n` ticks | TMOS API |
| `tmos_set_event(t,e)` | arm event `e` immediately | TMOS API |
| `adv_state.hword_32` | ADV interval in ticks (0.625 ms units) | BLE std |
| `adv_state.hword_110` | duration max in ticks | EXT_ADV |
| `adv_state.byte_105` | duration max count (event count) | EXT_ADV |
| `adv_state.hword_112` | duration current count | EXT_ADV |
| advDelay | `ble_ll_common_rand8(1, 14)` → 0–13 ticks → 0–8.125 ms | BLE std, conservative |
| TempSample period | `1600 / interval_ticks` events = 1 second | implementation choice |

Cross-check with PDF (manual v1.9) **§4.2 Adv parameter ranges**: ADV interval 20 ms–10.24 s → 32–16384 ticks. Matches `u16` hword_32 range.

---

## 8. Channel-rotation table (random advDelay slot)

Verified from `llAdvertiseStart` standard-3-channel path (byte_24 == 3). After `tmos_rand() & 3`:

| rand | byte_10 (1st) | byte_25 (2nd) | byte_26 (3rd) | sequence | status |
| ---: | :------------ | :------------ | :------------ | :------- | :----- |
| 0 | 39 | 37 | 38 | 39 → 37 → 38 | ✅ confirmed |
| 1 | 38 | 37 | 39 | 38 → 37 → 39 | ✅ confirmed |
| 2 | 37 | 39 | 38 | 37 → 39 → 38 | ✅ confirmed |
| 3 | 37 | ? | ? | TBD (`.L563` flow) | ⚠️ unverified |

`rand == 3` case (`.L563`) needs a more careful trace — flagged as Phase 2 follow-up. Other 3 rotations are confirmed.

Note: BLE spec **requires** all 3 primary channels in each event but **does not** prescribe order. WCH's "4-permutation" choice (not all 6) is a simplification.

### 8.1 Rust first-air policy (Cindy `e1b43937` amendment #4)

> **Hard policy**: Until `rand == 3` rotation is fully verified, Rust first-air baseline **MUST** use a **deterministic** channel sequence: `[37, 38, 39]` per event (no rotation, no randomization).
>
> Rationale: an unconfirmed rotation in the first-air variable set adds debugging noise. Deterministic order keeps first-air's failure surface aligned with TX register correctness, not with channel-rotation correctness.
>
> Upgrade path: after first-air PASS, port the verified 3 rotations + complete `rand == 3` verification + add randomized rotation in a separate, gated change.

---

## 9. Open questions (deferred)

### 9.0 Mini-pass before Phase 2.5 — ✅ COMPLETE

Outcome: `notes/ch32-rs/ti-ble/ll-process-event-minipass.md` produced. Both M1 and M2 verified. §0.Q2, §1 call-graph, §4 above all upgraded. Cindy's review pending → Phase 2.5 RX ingress greenlight gate.

| # | Question | Result |
| -: | :------- | :----- |
| M1 | `LL_ProcessEvent` event-bit dispatch | ✅ Verified — full 8-bit dispatch table extracted (see mini-pass doc). Event 0x01 → `gBleLlPara[0x70]` = `llAdvertiseStart`, Event 0x02 → `gBleLlPara[0x74]` = `llAdvTraverseallChannel`. |
| M2 | `gBleLlPara[0x74]` call-site xref | ✅ Verified — exactly 2 sites: 1 write (LL_AdvertiseEnalbe), 1 indirect-call (LL_ProcessEvent event-bit-1). Hypothesis B confirmed. |

### 9.1 Larger follow-ups (deferred to Phase 2.5+)

1. **`gBleLlPara[120]` vtable slot** (callback called at `.L439` in state 0x9e) — what does it point to? Probably an EXT_ADV scan response handler.
2. **`gBleLlPara[124]` bit-flag layout** — bits 0x10 / 0x20 / 0x30 used by AUX paths. Need bit-by-bit decoding.
3. **`adv_state` struct field map** offsets 16, 22, 23, 24, 32, 48, 52-60, 76, 80, 88, 96, 98, 103, 105, 107, 110, 112, 116, 118, 124, 152, 172, 224. Some are documented elsewhere; consolidate into Phase 0 doc §14 or new struct map doc.
4. **`gBleIPPara` field map** — `[2]` TX done, `[3]` TX err, `[7]` LL state — full byte map TBD as cross-phase work.
5. **`rand == 3` channel rotation** in `llAdvertiseStart .L563`. (Rust first-air uses deterministic [37,38,39] until resolved — see §8.1.)

---

## 10. Rust scheduler design rules (locked)

1. **§0 Iron Rule (Cindy hard constraint)**: For each ADV channel in a triple, call a `tx_adv_pdu(state)` function that drives the full Phase 0 register sequence verbatim. No delta optimization until first-air PASS.
2. **Two-timer model**: per adv-set, one **interval timer** (re-fires every `interval + rand[0..13]` ticks) plus one **duration timer** (only for limited-duration adv types).
3. **Time base**: 0.625 ms tick, mirroring WCH's 1600 Hz `fnGetClockCBs` contract — keep this even if our `BleClock` driver is different (RTC-derived per PDF).
4. **Channel triple as state**: a Rust `AdvEvent` owns `[u8; 3]` rotation and a `current_idx: u8`. State machine advances `current_idx` post-TX; event ends when `current_idx >= 3`.
5. **State-stamp re-entrancy**: the WCH byte_11 microstate is **per-PDU-type stamped by TX**. In Rust, model TX completion as an event with a payload that says "what kind of PDU just finished" (DIRECT / SCAN_RSP / common / AUX). Don't try to recover from the state byte alone; carry the PDU-type tag.
6. **Random advDelay scope**: applies to all non-DIRECT adv types. Use a per-event RNG draw, NOT a per-channel draw — the advDelay applies once per event before the first channel, channels within an event are back-to-back.
7. **DIRECT high-duty-cycle bypass**: skip random advDelay for `event_type == 1` (ADV_DIRECT_IND).
8. **TempSample tick**: keep a counter per adv-set; sample temperature every `1600 / interval_ticks` events.
9. **EXT_ADV duration count vs duration timer**: WCH tracks both — `hword_110` = max ticks (timer), `byte_105` = max event count, `hword_112` = current count. Either-or termination.
10. **Convergence on close**: ALL adv-set termination paths funnel through either `ll_advertise_event_closed` (normal close, user cb) or `LL_AdvertiseToStandby` (forced exit). Mirror this in Rust: one `close_event(state, reason)` API + one `to_standby(state)` API.
11. **Iron Rule §11 — Single owner per `AdvEvent`**: WCH's `gBleLlPara[100]` global pointer to the active adv set is **single-active**. Rust scheduler should enforce one-active-adv-set at the type level (e.g., `Option<AdvEvent>` in the BLE driver state).
12. **Iron Rule §12 — Three-source TX completion (Cindy `e1b43937` #3)**: `tx_adv_pdu` returns a `TxDoneSource { ok, abort_or_err, lle_idle }`, never a bool. First-air Rust MUST log all three observations on every channel TX completion. Collapsing into a single `tx_done` bool is forbidden until first-air PASS + diagnostic logs confirm the three sources behave as expected.
13. **Iron Rule §13 — Deterministic channel order for first-air (Cindy `e1b43937` #4)**: until `rand == 3` rotation is verified, Rust first-air baseline uses fixed `[37, 38, 39]` per event. Randomized rotation is a separate post-PASS change.
14. **Iron Rule §14 — `llAdvTraverseallChannel` IS the event-bit-1 re-entry (post-mini-pass)**: Rust ADV scheduler MUST treat TMOS event-bit-1 (mask 0x02) dispatch as the canonical re-entry path for the per-event ADV cycle. Do not invent parallel re-entry timers/IRQs. The Rust equivalent of `llAdvTraverseallChannel` must (a) early-return when `adv_state.is_active() == false`, (b) call `event_close()` cleanup when the previous event indicated finalize-pending, then (c) tail-issue the next channel TX.
15. **Iron Rule §15 — NULL-guard the vtable**: `LL_ProcessEvent` checks `gBleLlPara[0x74] != NULL` before indirect-call. Rust port uses `Option<fn>` (or Cell-of-fn) and silently no-ops if the ADV subsystem isn't initialized — never panic on missing handler.
16. **Iron Rule §16 — Active-flag gate (`byte_12 == 1`)**: `llAdvTraverseallChannel` does a hard active-flag check before any TX. Rust port reproduces: `if !adv_state.is_active() { return; }` at the top of the equivalent handler. This is the de-facto stop path when host calls `LL_AdvertiseShut` mid-cycle.

---

## 11. Cross-reference

- **Phase 0** `ll-advertise-tx-disasm.md` — `ll_advertise_tx` MMIO write sequence (BB/LLE/RFEND), PDU type → byte_11 stamping, channel idx encoding into BB[44] bits 25-30
- **Phase 1** `ll-tx-completion-disasm.md` — `ll_tx_wait_finish` busy-poll + state machine dispatch entry
- **Mini-pass** `ll-process-event-minipass.md` — `LL_ProcessEvent` event-bit dispatch table + `gBleLlPara[0x74]` call-site xref. Resolves Phase 2 §0.Q2 + §4.
- **Phase 2.5** (pending) `ll-rx-ingress-disasm.md` — `ll_advertise_legacy_rx` SCAN_REQ vs CONNECT_IND, `ll_rx_wait_finish`, `ll_advertise_aux_conn_rx`
- **Phase 3** (defer post-air-PASS) `phy_status_clear` + `BLE_SetPHYTxMode` + `BLE_SetPHYRxMode` verbatim black-box
- WCH manual v1.9 §4.2 (ADV parameter ranges) confirms `hword_32` ticks units

---

## 12. Verification artifacts

- `/tmp/wchble-disasm/ll_advertise_full.dump` (5328 lines, full ll_advertise.o)
- `/tmp/wchble-disasm/llAdvTraverseallChannel.dump` (88 lines, focused)
- `/tmp/wchble-disasm/llAdvertiseStart.dump` (488 lines)
- `/tmp/wchble-disasm/llAdvertiseTimeout.dump` (140 lines)
- `/tmp/wchble-disasm/LL_AdvertiseEnalbe.dump` (126 lines)
- `/tmp/wchble-disasm/ll_advertise_process.dump` (830 lines)
- Jump table `.L440` raw relocations: `riscv64-unknown-elf-objdump -r -j .rodata.ll_advertise_process ll_advertise.o`

---

_Last updated: 2026-05-13 00:50 (Phase 2 first pass). Open questions §9 remain for Phase 2.5/3 or full-trace follow-up._

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-adv-scheduler-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
