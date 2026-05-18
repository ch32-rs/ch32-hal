# Phase 2.5 — RX ingress (legacy SCAN_REQ / CONNECT_IND + SCAN_RSP trigger)

**Subject** — how `libwchble.a` processes a received PDU after `ll_advertise_process` post-TX RX window. Resolves Cindy's 3 locked DoD items (msg `9d9ca082`).

**Files** — `ll_advertise.o` (`ll_advertise_legacy_rx`, `ll_advertise_generated_scan_rsp`, `ll_advertise_to_connection_state`, `ll_advertise_filter`, `ll_advertise_aux_conn_rx`), `ip.o` (`ll_rx_wait_finish`).

**Companion** — Phase 0 `ll-advertise-tx-disasm.md`, Phase 1 `ll-tx-completion-disasm.md`, Phase 2 `ll-adv-scheduler-disasm.md`, mini-pass `ll-process-event-minipass.md`.

**Notation** — same lock as Phase 2 §1: `gBleLlPara[0xNN]` hex offsets; event references use mask+bit-pos form.

---

## 0. TL;DR — direct answers to Cindy's 3 DoD items

### DoD-1 — SCAN_REQ vs CONNECT_IND dispatch condition (✅ VERIFIED)

**Answer**: Dispatch happens in **two cuts** inside `ll_advertise_legacy_rx`:

**Cut 1: type-class accept gate** (offset 0x40–0x46):
```
a5 = pdu_type = rx_buf[0] & 0x0f
sb pdu_type → adv_state[19]              ; remember type
a5 = (pdu_type - 3) & 0xfd
if a5 != 0: return 128                    ; reject (not SCAN_REQ or CONNECT_IND)
```

The mask `(pdu_type - 3) & 0xfd == 0` is satisfied **iff** `pdu_type ∈ {3, 5}`:
- `pdu_type == 3` (SCAN_REQ) → `(0) & 0xfd = 0` ✅
- `pdu_type == 5` (CONNECT_IND) → `(2) & 0xfd = 0` ✅
- any other type → non-zero → rejected with return code `128`

This is the legacy-PDU type filter. All other types (ADV_IND, ADV_DIRECT, etc.) silently rejected — they should not reach `ll_advertise_legacy_rx` in the first place (this function is the SCAN/CONNECT request consumer).

**Cut 2: per-type branch** (offset 0x94–0x9a):
```
a4 = adv_state[19]                        ; stored pdu_type
if a4 == 5: fall through to CONNECT_IND path (0x9e..0xe4)
else (pdu_type == 3): jump .L428 → SCAN_REQ path (0xe6..0x142)
```

So the canonical dispatch byte is the PDU header **byte 0, low nibble**: `5` = CONNECT_IND, `3` = SCAN_REQ. **Stored at `adv_state[19]`** for downstream consumers.

### DoD-2 — CONNECT_IND first-hop function (✅ VERIFIED)

**Answer**: `ll_advertise_to_connection_state(adv_state)` — called **synchronously, in-line**, from `ll_advertise_legacy_rx` offset 0xce. NO `tmos_set_event` deferral.

**Call sequence** (offset 0xbc–0xe4 of `ll_advertise_legacy_rx`):
```
1. a5 = fnGetClockCBs(); a0 = a5()        ; snapshot current clock tick
2. adv_state[36] = a0                     ; rx-anchor timestamp for CONN sched
3. a0 = ll_advertise_to_connection_state(adv_state)
4. if a0 == s2 (== filter result, == 1):
       LL_AdvertiseToStandby(adv_state)   ; exit ADV state (clean handoff)
5. return s1 (= 0)
```

**`ll_advertise_to_connection_state` internal first-acts** (synchronous):
- Read `gBleLlPara[0xb8]` (slave-conn init vtable); if NULL → return 1 (caller skips standby)
- `ble_ll_hw_api_shut()` — **immediate** BB/LLE hardware shutdown
- Indirect-call `gBleLlPara[0xb8]()` — slave-conn init handler (populates Core struct)
- `LL_CoreGetCore()` — allocate a Core slot
- Copy CONNECT_IND payload into Core slot (verified fields):
  - `core[136]` = adv_state[36] (rx-anchor)
  - `core[152]` = AccessAddress (4 bytes from rx_buf[14..17])
  - `core[156]` = CRCInit (3 bytes from rx_buf[18..20])
  - `core[55]`  = WinSize (1 byte from rx_buf[21])
  - `core[56]`  = WinOffset (2 bytes from rx_buf[22..23])
  - `core[58]`  = Interval (2 bytes from rx_buf[24..25])
  - `core[60]`  = Latency (2 bytes from rx_buf[26..27])
  - `core[62]`  = Timeout (2 bytes from rx_buf[28..29])
  - `core[320]` = 1 (CONN marker; also set via .L21 ChSel-bit path)
- Validate CONN params (WinOffset ≤ 0x8c0, Interval ≤ 499 0.5ms-units, Timeout bound) — fail path calls `LL_CoreClose(handle)`
- Call `gBleLlPara[0xbc]()` (peripheral start vtable)
- Generate HCI events via `LL_ReportGenerateEvent(...)` with subcode 10 / 20 / 18 depending on Enhanced-Conn-Complete vs ChSel-Algorithm flags
- Return 1 on success, 0 on failure

**Key takeaways**:
- Transition is **synchronous** — no event-bit deferral. The host receives the HCI ConnectionComplete event from inside the RX-ingress callstack.
- **`ble_ll_hw_api_shut` runs early** inside `ll_advertise_to_connection_state` (offset 0x22) — the hardware is taken down BEFORE the Core struct exists. If Core alloc fails (`LL_CoreGetCore` returns NULL → return 1), the caller still calls `LL_AdvertiseToStandby` for the clean exit, but the BB is already off.
- The **rx-anchor timestamp** captured at `adv_state[36]` (and copied to `core[136]`) is the connection event 0 anchor — first slave window will be `WinOffset × 0.625ms` after this.

### DoD-3 — SCAN_RSP trigger point (✅ VERIFIED)

**Answer**: `byte_11 = 0x93` is stamped by `ll_advertise_generated_scan_rsp` at offset 0x6c–0x70, **NOT** by `ll_advertise_tx`. The stamp is fully verified.

**`ll_advertise_generated_scan_rsp` byte_11 stamp** (offset 0x6c–0x70):
```
6c:  li   a5, -109            ; -109 as i8 = 0xFFFFFF93, low byte = 0x93
70:  sb   a5, 11(s0)          ; adv_state.byte_11 = 0x93
```

This stamp happens AFTER the function has:
1. Prepared SCAN_RSP PDU header (offset 0x00–0x14):
   - `adv_state[17] = adv_state[30] + 6`  (PDU length = ScanRspData_len + 6)
   - `adv_state[16] = 4`                  (event_type = 4 = SCAN_RSP)
   - `rx_buf[0] = 4`                      (PDU header type = 4 = SCAN_RSP)
   - `rx_buf[1] = adv_state[17]`          (PDU header length)
2. Conditional `RxAdd` bit 6 set on `rx_buf[0]` if `adv_state[53].bit0` or `adv_state[52] == 2`
3. `tmos_memcpy(rx_buf+2, adv_state+54, 6)` — own address into ScanA position
4. `tmos_memcpy(rx_buf+8, adv_state[44], adv_state[30])` — ScanRspData payload
5. Stamp `byte_11 = 0x93` (the trigger)
6. Return

The stamp 0x93 routes into `ll_advertise_process` jump table index 1 → `.L451` ("post-SCAN_RSP TX done → user cb via pGapRoles_AppCGs[1]") on the next `ll_advertise_process` re-entry.

**Trigger chain (full path SCAN_REQ → SCAN_RSP TX)**:
```
ll_advertise_legacy_rx (RX SCAN_REQ accepted):
  ├─ Cut 1 type filter passes (pdu_type ∈ {3, 5})
  ├─ AdvA match → adv_state[68] = 1 (valid scan/conn received)
  ├─ ScanA → adv_state[70..75] + adv_state[69] (TxAdd)
  ├─ ll_advertise_filter(adv_state) == 1 (accept)
  ├─ Cut 2 branch: pdu_type == 3 → .L428 (SCAN_REQ path)
  ├─ adv_state[16] (event_type) ∈ {0, 6} or reject 128
  ├─ adv_state[20] (signed local filter level) ≥ GAP_GetParamValue(66) (param 0x42)
  ├─ ll_advertise_generated_scan_rsp(adv_state)            ← STAMPS byte_11 = 0x93
  ├─ BLE_SetPHYTxMode((adv_state[100] | adv_state[106]), adv_state[17])  ; PHY back to TX
  ├─ ll_tx_wait_finish(3, 0, 0)                            ; mode=3 BB toggle, wait SCAN_RSP TX done
  └─ ble_ll_hw_api_shut()                                  ; BB shutdown after SCAN_RSP TX
```

**Important nuance**: The `ll_tx_wait_finish(3, 0, 0)` here is the **same** wait primitive as Phase 1 §1, but called with `mode=3` (BB toggle, NOT TX kick). The PDU is already loaded by `ll_advertise_generated_scan_rsp` (it wrote the PDU into the slot at `adv_state[76]` indirectly through `tmos_memcpy`). The `BLE_SetPHYTxMode` reconfigures PHY for TX. Then `ll_tx_wait_finish(3,...)` performs the BB-mode toggle + wait. This implies the actual **TX kick** (`LLE[0] = 2`) is folded into `BLE_SetPHYTxMode` or `ll_tx_wait_finish(3,...)`, NOT in `ll_advertise_generated_scan_rsp` itself.

> **✅ PARTIAL CLOSED in Phase 3** (`ll-phy-preflight-disasm.md` §0 priority 1 + §4.2): SCAN_RSP TX kick is **`BB[0] |= 0x800000`** (set bit 23, "warm RX→TX turnaround fire") + `BB[44] &= ~3`, written inside `ll_tx_wait_finish` **mode=3 fall-through** at offset 0x152–0x15a. This is a DIFFERENT mechanism than the ADV cold-TX kick (`LLE[0] = 2` at `ll_tx_wait_finish` mode=0). `BLE_SetPHYTxMode` writes neither — it's pure PHY/RF config. New Iron Rule §25 records the dual-kick design.

---

## 1. Function call graph (RX-side)

```
ll_advertise_process  .L443 (byte_11 == 0x9b)             ← post-ADV TX, opens RX window
├─ BLE_SetPHYRxMode(...)                                  (Phase 3 black-box)
├─ ll_tx_wait_finish(1, ...)                              (mode=1 RX prep, Phase 1 §1)
├─ ble_ll_chkcrc()                                        (CRC check; if fail, skip ingress)
└─ ll_advertise_legacy_rx(adv_state)                      ← THIS FUNCTION
   ├─ length filter (rx_buf[1] & 0x3f ∈ [6..37]) → adv_state[18]
   ├─ type filter (rx_buf[0] & 0x0f, accept {3, 5}) → adv_state[19]
   ├─ AdvA filter (tmos_memcmp(adv_state[54..59], rx_buf[8..13])); mismatch → ret 4
   ├─ stamp adv_state[68] = 1, adv_state[69] = (rx_buf[0] >> 6) & 1 (TxAdd)
   ├─ tmos_memcpy(adv_state[70..75], rx_buf[2..7]) (ScanA / InitA)
   ├─ ll_advertise_filter(adv_state) → must return 1
   │
   ├─ if pdu_type == 5 (CONNECT_IND):
   │  ├─ slave-count check: ll_connect_get_slave_number() >= (ble[0x15] & 3) → goto .L428 fallback
   │  ├─ ble[0x15] gate (host-side max-conn cap)
   │  ├─ filter-code vs adv_state[16] (event_type) ≤ 1: else reject 128
   │  ├─ a0 = fnGetClockCBs(); adv_state[36] = a0()         (rx-anchor)
   │  ├─ ll_advertise_to_connection_state(adv_state)        ← FIRST-HOP (synchronous)
   │  │  ├─ ble_ll_hw_api_shut()                            (immediate HW shutdown)
   │  │  ├─ gBleLlPara[0xb8]() (slave-conn init vtable)
   │  │  ├─ LL_CoreGetCore() → Core slot
   │  │  ├─ Copy CONNECT_IND fields (AA / CRCInit / WinSize / WinOffset / Interval / Latency / Timeout)
   │  │  ├─ Param validation (WinOffset/Interval/Timeout bounds)
   │  │  ├─ if invalid: LL_CoreClose(handle); return 0
   │  │  ├─ gBleLlPara[0xbc]() (peripheral start vtable)
   │  │  ├─ LL_ReportGenerateEvent(handle, 128, 10|20|18)   (HCI ConnectionComplete)
   │  │  └─ return 1
   │  └─ if return == filter-code (== 1): LL_AdvertiseToStandby(adv_state)
   │
   └─ else (pdu_type == 3, SCAN_REQ) [.L428 path]:
      ├─ adv_state[16] (event_type) check: must be 0 or 6, else reject 128
      ├─ GAP_GetParamValue(66) vs adv_state[20] (signed) — RSSI/filter level gate
      ├─ ll_advertise_generated_scan_rsp(adv_state)         ← STAMPS byte_11 = 0x93
      ├─ BLE_SetPHYTxMode((adv_state[100]|adv_state[106]), adv_state[17])
      ├─ ll_tx_wait_finish(3, 0, 0)                         (mode=3 BB toggle + SCAN_RSP TX wait)
      └─ ble_ll_hw_api_shut()                               (post-SCAN_RSP HW shutdown)

ll_advertise_aux_conn_rx   (EXT_ADV CONNECT_IND analog — out of Phase 2.5 main scope)
└─ at offset 0x126–0x12a: byte_11 = 0x9a (jump table index 8 → .L444 "CONNECT_IND received → conn")
   ; this is the parallel path for EXT_ADV. Deferred to Phase 4 EXT_ADV decode.
```

---

## 2. `ll_advertise_legacy_rx` — full breakdown

### 2.1 ABI

| reg | role |
|:----|:-----|
| a0 (in) | `adv_state` pointer |
| a0 (out) | return code — `128` = reject, `4` = AdvA mismatch, `0` = handled (SCAN_RSP sent or CONN started) |

Internal saved: `s0 = adv_state`, `s1 = scratch / return-code carrier`, `s2 = filter return code`.

### 2.2 Reject codes (caller dispatch)

| return | meaning | trigger |
|:------:|:--------|:--------|
| `128`  | hard reject — wrong PDU type, filter fail, wrong event_type, slave-cap exceeded, filter-level fail | many — see decision tree |
| `4`    | AdvA mismatch — PDU was for a different advertiser | `tmos_memcmp(own_addr, rx_buf[8..13]) != 1` |
| `0`    | handled successfully — SCAN_RSP TX'd, OR CONN started, OR fallback path completed | every success / normal-exit path |

### 2.3 PDU layout assumed

Legacy LE 1M SCAN_REQ / CONNECT_IND PDU (Bluetooth Core 5.x Vol 6 Part B §2.3):
```
byte 0:    header type (bits 0..3) | RFU (bit 4) | ChSel (bit 5) | TxAdd (bit 6) | RxAdd (bit 7)
byte 1:    length (bits 0..5) | RFU (bits 6..7)
byte 2..7: ScanA (SCAN_REQ) or InitA (CONNECT_IND) — 6-byte BD_ADDR
byte 8..13: AdvA — 6-byte BD_ADDR (must match own)
byte 14+:  payload (CONNECT_IND: LLData = 22 bytes; SCAN_REQ: no payload, length = 12)
```

Verified field extraction matches this layout exactly (offsets 8 for AdvA filter, 2 for ScanA/InitA copy, 14+ for CONN LLData in `ll_advertise_to_connection_state`).

### 2.4 State writes (what changes in `adv_state`)

| Offset | Field | Written when | Value source |
|-------:|:------|:-------------|:-------------|
| 18  | rx_pdu_length | always (after length filter pass) | `rx_buf[1] & 0x3f` |
| 19  | rx_pdu_type | always (after length filter pass) | `rx_buf[0] & 0x0f` |
| 68  | rx_valid flag | after AdvA match | `1` |
| 69  | rx_peer_addr_type (TxAdd) | after AdvA match | `(rx_buf[0] >> 6) & 1` |
| 70..75 | rx_peer_addr (ScanA/InitA) | after AdvA match | `rx_buf[2..7]` |
| 36  | rx_anchor_timestamp | CONNECT_IND accept only | `fnGetClockCBs()()` |

These writes mirror what TI BLE5stack does in `MAP_llAdvScheduler` / `linkConn_t` setup — informational, not behavior-critical.

---

## 3. `ll_advertise_to_connection_state` — CONNECT_IND first-hop

### 3.1 Critical sequence

1. **Slave-conn init gate** (offset 0x0c–0x18): read `gBleLlPara[0xb8]`; if NULL, return 1 (no slave subsystem available, abort CONN handoff).
2. **HW shutdown** (offset 0x1e–0x22): `ble_ll_hw_api_shut()` — immediate BB/LLE off. This is the **point of no return** from advertising-state to connection-state. If subsequent steps fail, we cannot return to ADV.
3. **Slave init call** (offset 0x26–0x2a): `gBleLlPara[0xb8]()` — indirect-call slave-conn init.
4. **Core alloc** (offset 0x2c–0x36): `LL_CoreGetCore()`; if NULL → return 1 (`.L45` path) with `a0 = 1`.
5. **Copy rx-anchor + LLData fields** (offset 0x38–0xec): see §3.2 below.
6. **Param validation** (offset 0xf6–0x144): WinOffset, Interval, Timeout bounds; on fail → `LL_CoreClose(handle)`; return 0.
7. **Peripheral start** (offset 0x184–0x18a): `gBleLlPara[0xbc]()` — start the slave role state machine.
8. **HCI events** (offset 0x1a2–0x1ec): `LL_ReportGenerateEvent(handle, 128, code)` with `code = 10` (basic), `+ 20` if Enhanced-Conn-Complete enabled (gBleLlPara[0xd8] bit 17 = 0x20000 — `sll 0x11; bgez`), `+ 18` if certain ChSel flags set.
9. Return 1 on full success.

### 3.2 LLData field extraction (verified offsets)

```
rx_buf base = adv_state[80]                  ; rx buffer pointer (same as legacy_rx used)
LLData starts at rx_buf[14] (after header(2) + InitA(6) + AdvA(6))

core[152]  ← u32  AA          = rx_buf[14..17] little-endian
core[156]  ← u32  CRCInit     = rx_buf[18..20] little-endian (24-bit, top 8 = 0)
core[55]   ← u8   WinSize     = rx_buf[21]
core[56]   ← u16  WinOffset   = rx_buf[22..23] little-endian
core[58]   ← u16  Interval    = rx_buf[24..25] little-endian
core[60]   ← u16  Latency     = rx_buf[26..27] little-endian
core[62]   ← u16  Timeout     = rx_buf[28..29] little-endian
core[312]  ← u32  0           (CONN state init)
core[316]  ← u32  0           (CONN counter init)
core[136]  ← u32  rx_anchor   = adv_state[36] (captured by caller before this fn)
core[320]  ← u8   1           (CONN active marker; also set via .L21 ChSel path)
core[326]  ← u8   adv_state[99] (PHY config)
core[327]  ← u8   adv_state[99] (PHY config, second copy)
```

ChannelMap (5 bytes, rx_buf[30..34]) and Hop/SCA (rx_buf[35]) come via the `tmos_memcpy` at offset 0xf2 and the post-memcpy stamps at offset 0xfa–0x112. `core[53]` = ChannelMap[0] & 0x1f (lower 5 bits = hop), `core[48]` = ChannelMap[0] >> 5 (upper 3 bits = SCA).

### 3.3 Param bounds enforcement (offset 0x116–0x144)

> ⚠️ **Unit watch** (Cindy `c43039d0` #2): CONN params here use BLE-spec **CONN** units (1.25ms / 10ms). Do NOT confuse with **ADV** scheduler ticks (0.625ms, see Phase 2 §6). Mixing the two will silently miscompute timing.

| Param | Raw bound (offset) | Unit | Bound in real time | Spec max | Notes |
|:------|:-------------------|:-----|:-------------------|:---------|:------|
| Hop sanity | `(hop - 5) > 11` (0x118) | — (channel hop integer) | n/a | 5..16 | hop ∈ [5,16] valid |
| WinOffset | `> 0x8c0` = 2240 (0x11c–0x12c) | 1.25ms | > 2800 ms | ConnInterval−1 | WCH hardcoded ceiling (tighter than spec) |
| Interval  | `> 499` (0x130–0x138) | 1.25ms | > 623.75 ms | 3200 (4 s) | **WCH-tight**, see Iron Rule §23 |
| Timeout   | `> 0xc80` = 3200 (0x13c–0x144) | 10ms | > 32 000 ms (32 s) | 3200 (32 s) | matches BLE spec max |

On any fail → fall through to `LL_CoreClose(handle)` (§3.4).

### 3.4 Failure path = LL_CoreClose

Offset 0x148–0x154: on any param-bound failure, `LL_CoreClose(adv_state[8])` is called with the handle, and the function returns 0 (caller does NOT call `LL_AdvertiseToStandby` because `0 != filter-result-1`, so ADV stays in error state — host must issue reset or retry).

---

## 4. `ll_advertise_generated_scan_rsp` — SCAN_RSP PDU prep + state stamp

### 4.1 Body (101 lines, fully decoded)

```
adv_state[17] = adv_state[30] + 6         ; PDU length = ScanRspData_len + (header 2 + ScanA 6 - 2)
                                          ; matches BLE spec: SCAN_RSP = header + AdvA + ScanRspData

rx_buf = adv_state[76]                    ; PDU output buffer (NOTE: same slot as TX buffer)
rx_buf[0] = 4                             ; PDU type = SCAN_RSP (4)
adv_state[16] = 4                         ; event_type = 4 (SCAN_RSP)
rx_buf[1] = adv_state[17]                 ; PDU length byte

if (adv_state[53] & 1) || (adv_state[52] == 2):
    rx_buf[0] |= 0x40                     ; set TxAdd bit 6 (random addr type)

tmos_memcpy(rx_buf+2, adv_state+54, 6)    ; AdvA = own address
tmos_memcpy(rx_buf+8, adv_state[44], adv_state[30])   ; payload = ScanRspData
                                                       ; ScanRspData ptr at adv_state[44]
                                                       ; ScanRspData len at adv_state[30]

adv_state.byte_11 = 0x93                  ← TRIGGER STAMP (DoD-3 verified)
return
```

### 4.2 What it does NOT do

- **No PHY/BB/RFEND register writes** (no LLE writes either). Confirmed: only `tmos_memcpy` and stores into `adv_state[...]` + `rx_buf[...]`.
- **No TX kick** (`LLE[0] = 2`). The kick is issued elsewhere — see §0 DoD-3 note + §5 below.
- **No timer arming**. No `tmos_set_event` or `tmos_start_task` calls.

### 4.3 Why this matters for Rust port

`ll_advertise_generated_scan_rsp` is a **pure data-shaping + state-stamp** function. The Rust equivalent should:
1. Build SCAN_RSP PDU into the same TX buffer slot (the BB hardware reads from a fixed buffer pointed at by `LLE[112]`, which is `adv_state[76]` per Phase 0).
2. Stamp the equivalent of `byte_11 = 0x93` to mark "SCAN_RSP ready" for the next state-machine iteration.
3. Do NOT touch PHY/BB registers — those are owned by the actual TX primitive (`BLE_SetPHYTxMode` + `ll_tx_wait_finish(3, ...)` in WCH).

---

## 5. `ll_rx_wait_finish` (in `ip.o`) — RX-done busy-wait

Distinct from `ll_tx_wait_finish` (Phase 1), this is the RX-side completion wait. Short function:

```
ll_rx_wait_finish():
  a3 = gptrLLEReg
  a5 = &gBleIPPara
  loop .L3:
    if gBleIPPara[0] & 1: break  ; RX done (ok flag)
    if gBleIPPara[1] & 1: break  ; RX err (err flag)
    if LLE[100] != 0:    continue ; timeout/idle counter not yet 0 → keep polling
    break  ; LLE[100] == 0 → idle/timeout reached
  ; on exit:
  gBleIPPara[5] = 0    ; clear RX-active flag
  return
```

**Three-source RX completion observation** (mirrors TX's pattern):
- `gBleIPPara[0] bit 0` = RX clean completion
- `gBleIPPara[1] bit 0` = RX error / abort
- `LLE[100] == 0` = hardware idle / timeout reached

After the busy-wait, **`gBleIPPara[5] = 0`** clears the "RX active" flag (a separate bookkeeping byte, similar to TX-side `gBleIPPara[5]` semantic but distinct slot). This is the RX-side analog of the TX's `gBleIPPara[5]` clear in `ll_tx_wait_finish`.

**Rust port implication**: same `TxDoneSource`-style struct, named `RxDoneSource { ok, err, lle_idle }`. Iron Rule §12 generalizes from TX-only to TX+RX.

---

## 6. CONNECT_IND first-hop in EXT_ADV (`ll_advertise_aux_conn_rx`)

Out of Phase 2.5 main scope (it's the EXT_ADV equivalent), but useful for cross-check:

- Stamps **`byte_11 = 0x9a`** at offset 0x12a (`li a5,-102; sb a5,11(s0)`).
- 0x9a maps to jump table index 8 = `.L444` ("CONNECT_IND received → conn") in `ll_advertise_process` (Phase 2 §5.1).
- So EXT_ADV CONNECT_IND defers via the state-machine route (stamp + return), while legacy CONNECT_IND is synchronous in-line via `ll_advertise_to_connection_state` (DoD-2).

The two CONNECT_IND paths use **different** transition mechanisms — Rust port should not unify them prematurely.

---

## 7. Rust RX-ingress design rules (additions to Phase 2 Iron Rules)

17. **Iron Rule §17 — Legacy RX type dispatch admits {3, 5} only** (relaxed wording per Cindy `c43039d0` #1): Rust port may use the readable form `matches!(pdu_type, 3 | 5)` — the WCH arithmetic gate `(pdu_type - 3) & 0xfd == 0` is preserved here as evidence, not as a required code shape. What MUST be preserved: the **admit set** ({3, 5}), the **reject code** (128), and the **store of `pdu_type` into `adv_state[19]`** before the second cut.

18. **Iron Rule §18 — Return code is the contract**: `ll_advertise_legacy_rx` returns `{128 reject, 4 AdvA mismatch, 0 handled}`. The Rust equivalent must preserve this 3-state return — callers (state machine entry .L443) discriminate on it.

19. **Iron Rule §19 — CONNECT_IND first-hop is synchronous; no event-bit deferral**: Rust port keeps `to_connection_state()` call **in-line** within the RX-ingress callstack. Do NOT defer via async task / TMOS event simulation. The HCI ConnectionComplete event must be observable in the same call from the host's POV.

20. **Iron Rule §20 — `ble_ll_hw_api_shut` runs BEFORE Core alloc**: in `ll_advertise_to_connection_state`, hardware is taken down at offset 0x22 BEFORE `LL_CoreGetCore` (offset 0x2c). Rust port must keep this order — if hardware is left running while Core alloc fails, the next state-machine iteration sees BB in an undefined state.

21. **Iron Rule §21 — SCAN_RSP byte_11 stamp is in the prep function, NOT the TX function**: `ll_advertise_generated_scan_rsp` writes `byte_11 = 0x93`. Rust port keeps the stamp co-located with PDU prep, NOT with the actual TX kick. The state-machine re-entry reads the stamp; the TX primitive is invariant w.r.t. PDU type.

22. **Iron Rule §22 — Three-source RX completion** (generalization of §12): `rx_wait_finish()` returns `RxDoneSource { ok, err, lle_idle }`. First-air Rust MUST log all three observations on every RX window completion.

23. **Iron Rule §23 — WCH bounds are tighter than spec**: `ll_advertise_to_connection_state` rejects `Interval > 499` (623.75ms), spec allows up to 3200 (4s). Rust port reproduces the 499 cap until first-air PASS confirms it's needed (otherwise we relax to spec max).

24. **Iron Rule §24 — Legacy vs EXT_ADV CONNECT_IND are different paths**: legacy = synchronous via `ll_advertise_to_connection_state`; EXT_ADV = deferred via `byte_11 = 0x9a` + state-machine. Do NOT merge them in the first-air Rust port. They will be unified post-PASS only if the synchronous path proves problematic for embassy task scheduling.

---

## 8. Cross-reference

- Phase 0 `ll-advertise-tx-disasm.md` — TX register write sequence, PDU type → byte_11 stamping by `ll_advertise_tx`
- Phase 1 `ll-tx-completion-disasm.md` — `ll_tx_wait_finish` (TX side); §4.1 `gBleIPPara` field semantics
- Phase 2 `ll-adv-scheduler-disasm.md` — `ll_advertise_process` jump table (`.L440` 13 entries, esp. 0x9b post-TX-RX entry and 0x9a CONNECT_IND-conn-transition)
- Mini-pass `ll-process-event-minipass.md` — `LL_ProcessEvent` event-bit dispatch (this doc operates inside event-bit-1 callstack: `llAdvTraverseallChannel → ll_advertise_tx → ll_advertise_process → .L443 → ll_advertise_legacy_rx`)
- Phase 3 (deferred post-air-PASS) — will decode `BLE_SetPHYRxMode` / `BLE_SetPHYTxMode` and identify where SCAN_RSP TX kick (`LLE[0] = 2`) is folded for SCAN_RSP path

---

## 9. Verification artifacts (re-runnable)

```bash
cd /tmp/wchble-disasm
riscv64-unknown-elf-objdump --disassemble=ll_advertise_legacy_rx -r ll_advertise.o          # 209 lines
riscv64-unknown-elf-objdump --disassemble=ll_advertise_generated_scan_rsp -r ll_advertise.o # 101 lines
riscv64-unknown-elf-objdump --disassemble=ll_advertise_to_connection_state -r ll_advertise.o # 256 lines
riscv64-unknown-elf-objdump --disassemble=ll_advertise_aux_conn_rx -r ll_advertise.o        # 204 lines
# ll_rx_wait_finish lives in ip.o, already dumped at /tmp/wchble-disasm/ll_rx_wait_finish.dump (75 lines)

# byte_11 stamp xref (cross-verify trigger points)
grep -n -B1 -A2 'sb.*,11(' ll_advertise_*.dump
```

## 10. Open items (deferred to later phases)

- **SCAN_RSP TX kick site — Phase 3 DoD #1** (promoted per Cindy `c43039d0` #3): Confirmed NOT in `ll_advertise_generated_scan_rsp`; folded into `BLE_SetPHYTxMode` or `ll_tx_wait_finish(3, ...)`. **Phase 3 first DoD must answer**:
  - (a) Does the `ll_tx_wait_finish(mode=3, ...)` path write `LLE[0] = 2` (TX kick)?
  - (b) Does `BLE_SetPHYTxMode` itself also write a TX kick, or is it pure PHY/RF config?
  - This determines the **Rust SCAN_RSP TX primitive** for first-air. Without this answer, the Rust SCAN_RSP path cannot be ported faithfully.
- **`gBleLlPara[0xb8]` / `gBleLlPara[0xbc]` vtable slots** — slave-conn init + peripheral start. Need separate decode pass on `ll_slave.o` / `ll_connect.o`.
- **`ll_connect_get_slave_number()` + `ble.byte[0x15]`** — host-side max-slave gate. Cross-file refs; defer to a Phase that touches `ll.o` host config.
- **`ll_advertise_filter()`** — 223-line function; not in DoD-1 critical path (returns 1=accept / other=reject, treated as black-box here). Full decode deferred.
- **EXT_ADV CONNECT_IND path** (`ll_advertise_aux_conn_rx`) — fully out of legacy scope; Phase 4 if/when EXT_ADV is needed.

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-rx-ingress-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
