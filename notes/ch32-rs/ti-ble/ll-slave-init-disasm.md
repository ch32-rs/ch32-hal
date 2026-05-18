# Phase A2 — slave-conn init + peripheral start vtable (LL_SlaveEnalbe / llSlaveCreateCore / llSlaveSetFirstTimer)

**Subject** — decode the two `gBleLlPara` vtable slots Phase 2.5 black-boxed: `[0xb8]` (slave-conn init) and `[0xbc]` (peripheral start). Resolves Cindy's A2 greenlight focus (msg `fbee9584`): "`gBleLlPara[0xb8]/[0xbc]` vtable 到底来自哪个对象、slave core init 写哪些连接态字段、以及 peripheral start 是否会再碰 BB/LLE/RFEND".

**Files** — `ll_slave.o` (`LL_SlaveEnalbe`, `llSlaveCreateCore`, `llSlaveSetFirstTimer`, `ll_slave_process`); `ll_connect.o` (`ll_set_connect_event`, `ll_connect_flowcontrol_init`, `ll_connect_init_dataLen`, `ll_connect_init_pingOffset`, `ll_connect_set_connect_timeout`); `ll_hop.o` (`LL_HopInit`).

**Companion** — Phase 2.5 §3 `ll-rx-ingress-disasm.md` (where the NULL-guard + indirect call on `gBleLlPara[0xb8]/[0xbc]` lives), Phase 3 `ll-phy-preflight-disasm.md` (PHY/PFIC reg map).

---

## 0. TL;DR — Cindy's 3 A2 priorities answered

### Priority 1 — Vtable populator (✅ ANSWERED)

The populator is **`LL_SlaveEnalbe`** in `ll_slave.o` (yes, with WCH's `Enable→Enalbe` typo). It is a ~64-byte one-shot init that wires three globals:

| Global slot | Value installed | Callee role |
|:------------|:----------------|:------------|
| `gBleLlPara[0xb8]` (= `+184`) | `&llSlaveCreateCore` | slave-conn init (called when CONNECT_IND accepted) |
| `gBleLlPara[0xbc]` (= `+188`) | `&llSlaveSetFirstTimer` | peripheral start / first timer schedule |
| `pfnSlaveHandle` (global fn ptr) | `&ll_slave_process` | slave event-loop handler (called every TMOS tick when slave active) |

**Prerequisite**: `LL_SlaveEnalbe` first calls `LL_AdvertiseEnalbe()` — the advertise vtable populator from A1. So **enable order is fixed**: advertise vtable must be installed before slave vtable. Rust port mirrors this with an explicit dependency.

### Priority 2 — Slave core init conn-state field writes (✅ ANSWERED)

`llSlaveCreateCore` allocates a new conn-handle via `LL_CoreOpen`, retrieves the conn-state struct via `LL_CoreGetCore` (= `s0`), then writes **~30 distinct fields** of the conn-state struct + allocates an RX buffer. Full map in §3. No HW register writes — purely software struct init + memory allocation.

### Priority 3 — Does peripheral start touch BB/LLE/RFEND? (✅ ANSWERED — NO)

**`llSlaveSetFirstTimer` does NOT touch BB / LLE / RFEND.** Pure software:
- Timer math (SCA divisor, conn-interval, `bleClock_t` reads)
- Conn-state field writes (timer scaffold + ll_connect_* sub-inits + LL_HopInit channel-hop table)
- Stores `gBleLlPara[0xa8] = s0` (active slave conn pointer)
- Tail-jumps to `ll_set_connect_event`

**However, `ll_set_connect_event` DOES write PFIC** — but at the **standard PFIC/NVIC base `0xe000e204`**, NOT the `0x4002_420c` from `phy_status_clear`. See §4.4. This is a critical corroboration of Cindy's Phase 3 #2 caution: the `0x4002_420c` MMIO is **almost certainly NOT PFIC** (real PFIC is at `0xe000e000`). Update to Phase 3 open item below.

---

## 1. `LL_SlaveEnalbe` — the populator

ABI: `void LL_SlaveEnalbe(void)` — no args, no return.

### 1.1 Decode (verbatim)

```
@ 0x00:
  sp -= 16; sp[12] = ra
  call LL_AdvertiseEnalbe     ; advertise vtable first (A1 prerequisite)

@ 0x0c (after return):
  a5 = &gBleLlPara             ; via PCREL_HI20+LO12
  a4 = &llSlaveCreateCore      ; via PCREL_HI20+LO12 (function pointer)
  *(a5 + 184) = a4             ; gBleLlPara[0xb8] = llSlaveCreateCore
  ra = sp[12]                  ; (note: epilogue interleaved with vtable writes)

@ 0x22:
  a4 = &llSlaveSetFirstTimer
  *(a5 + 188) = a4             ; gBleLlPara[0xbc] = llSlaveSetFirstTimer

@ 0x2e:
  a5 = &ll_slave_process
  a4 = &pfnSlaveHandle
  *a4 = a5                     ; pfnSlaveHandle = ll_slave_process

  sp += 16
  ret
```

### 1.2 Phase 2.5 cross-reference (vtable indirect call site)

Phase 2.5 §3 `ll-rx-ingress-disasm.md` (CONNECT_IND admit path, `ll_advertise_to_connection_state`):

```c
fn = gBleLlPara[0xb8];          // ← installed here by LL_SlaveEnalbe
if (fn != NULL) {
    handle = fn();              // → llSlaveCreateCore (see §3)
    if (handle) {
        fn2 = gBleLlPara[0xbc]; // ← installed here
        if (fn2 != NULL) {
            fn2(handle, conn_params);  // → llSlaveSetFirstTimer (see §4)
        }
    }
}
```

**Confirmed black-box → decoded transition**: both fn pointers resolved to known `ll_slave.o` symbols. Phase 2.5 §3 indirect-call comment box can now be replaced with direct references.

### 1.3 Iron Rule §33 (new)

**Iron Rule §33 — Slave enable order**: `LL_SlaveEnalbe` is the only legitimate populator of `gBleLlPara[0xb8]/[0xbc]/pfnSlaveHandle`. It calls `LL_AdvertiseEnalbe` first as prerequisite. Rust port enforces this via type-state: `AdvertiseEnabled` must exist before `SlaveEnabled` can be constructed. Single one-shot — re-calling is harmless but does not reset conn-state.

---

## 2. `ll_slave_process` (heads-up only)

Mentioned for completeness: `pfnSlaveHandle = &ll_slave_process` makes `ll_slave_process` the slave event-loop handler. Size: **0xa28 bytes (2600+ lines disasm)** — full decode is **Phase B-or-later** work. Phase A2 does NOT decode it.

For now, the only fact needed is: `ll_slave_process` is what walks the BB/LLE registers each TMOS tick when a slave conn is active — it is the **actual consumer** of the `gBleLlPara[0xa8]` conn-state pointer that `llSlaveSetFirstTimer` writes.

---

## 3. `llSlaveCreateCore` — conn-state struct init (slave side)

ABI: `int llSlaveCreateCore(void *conn_params)` — returns one of three values; see §3.1 ⚠️ table below.

### 3.1 Allocation + early failure paths

```
@ 0x00:  sp -= 16; sp[4]=s1, sp[8]=s0, sp[12]=ra
@ 0x08:  s1 = LL_CoreOpen(conn_params)      ; allocate conn handle
@ 0x12:  if (s1 == NULL): goto .L2 → return s1=NULL  ; epilogue at 0x88

@ 0x14:  s0 = LL_CoreGetCore(s1)            ; resolve handle → struct ptr
@ 0x1e:  if (s0 == NULL): goto .L6 → s1=1, return 1   ; ⚠️ failure path forces s1=1, see table
@ 0x22:  ... main init block ...; on tmos_memory_allocate() failure (.L4_close):
                                  goto LL_CoreClose; s1 retains conn-handle value; return s1

@ 0x22:  s0[10] (sh) = 0x111 (273)          ; static state word
@ 0x2a:  s0[14] (sb) = 0xb0 (signed -80)    ; packed byte / role
@ 0x32:  s0[16] (sb) = 1                    ; flag / slave-role marker

@ 0x38:  rx_size = clamp(min=64, max=260, 9 + ble[0x10])
@ 0x6e:  s0[272] (sw) = tmos_memory_allocate(rx_size, 595)
@ 0x7a:  if (allocation == NULL): goto .L4_close → LL_CoreClose, fall to .L2
@ 0x94:  → .L5 main init block (next subsection)
```

⚠️ **Return-value table (3 distinct paths, ambiguous on the wire)** — per Cindy `ebbd3209` #1, model conservatively until full call-chain confirmed:

| Path | `s1` at return | C-level meaning (decoded) | Conservative Rust model |
|:-----|:---------------|:--------------------------|:------------------------|
| `LL_CoreOpen` returns NULL | `0` (= NULL) | "no free conn slot, reject CONNECT_IND" | `CoreInitOutcome::RejectKeepAdv` |
| `LL_CoreOpen` OK, `LL_CoreGetCore` NULL | `1` | "handle allocated but struct lookup failed — exotic invariant break" | `CoreInitOutcome::Fatal` (or `CoreInitOutcome::RejectKeepAdv` if caller treats same as NULL — verify in Phase B+) |
| `LL_CoreOpen` OK, `LL_CoreGetCore` OK, `tmos_memory_allocate` NULL | retained from `LL_CoreOpen` (≥ 2 typically) | "core registered but no RX buffer; LL_CoreClose called to roll back" | `CoreInitOutcome::Fatal` (rollback already happened in callee) |
| Full success | retained from `LL_CoreOpen` (≥ 2 typically) | "conn-state populated, proceed to peripheral start" | `CoreInitOutcome::Proceed(handle)` |

The collision between path 2 (s1=1, "exotic failure") and path 3+ (s1≥2, success) is **the ambiguity Cindy flagged** — `1` can mean either Fatal-handle-resolve-failure OR a literal handle ID of 1. The C ABI has no way to distinguish; Phase B caller (likely `ll_advertise_to_connection_state`) must be inspected to see whether it treats return == 1 specially. **Rust port: model as enum first, only collapse to bool after full chain audit** (per Cindy `ebbd3209` #1).

```rust
// Phase A2 Rust model — conservative
enum CoreInitOutcome {
    ProceedStandby(ConnHandle),     // s1 ≥ 2, success — start peripheral start phase
    RejectKeepAdv,                  // s1 == 0 — no slot, keep advertising
    Fatal,                          // s1 == 1 — invariant break OR ambiguous; verify caller
}
```

### 3.2 .L5 — main field init block (offsets relative to s0 = conn-state struct)

| s0 offset | size | source / value | observed semantic guess |
|:----------|:-----|:---------------|:------------------------|
| `s0[10]`  | sh   | `0x111`        | header/state flags (from §3.1) |
| `s0[14]`  | sb   | `0xb0`         | packed byte (signed -80) |
| `s0[16]`  | sb   | `1`            | slave-role flag |
| `s0[272]` | sw   | `tmos_memory_allocate(...)` | RX buffer ptr |
| `s0[276]` | sw   | `gBleIPPara[36]` | IP-state copy |
| `s0[326]` | sh   | `0`            | state half-word clear |
| `s0[248]` | sw   | `gBleLlPara[0xd8]` | feature/AA word 0 (also at +264) |
| `s0[252]` | sw   | `gBleLlPara[0xdc]` | feature/AA word 1 (also at +268) |
| `s0[264]` | sw   | `gBleLlPara[0xd8]` | duplicate of [248] (mirror?) |
| `s0[268]` | sw   | `gBleLlPara[0xdc]` | duplicate of [252] |
| `s0[361]` | sb   | `gBleLlPara[0x4a]` | byte config |
| `s0[324]` | sh   | `gBleLlPara[0x3a]` | half-word config |
| `s0[64]`  | sh   | `-1`           | sentinel ("no update pending"?) |
| `s0[38]`  | sh   | `-1`           | sentinel |
| `s0[40]`  | sh   | `-1`           | sentinel |
| `s0[46]`  | sb   | `\|= 0xc0`     | OR-mask bits 6,7 set |
| `s0[74]`  | sh   | `0xbb8 (3000)` | likely supervision/window default |
| → call `ll_connect_flowcontrol_init(s0)` (12-byte stub, no HW) |
| `s0[322]` | sb   | `ble[0x14]`    | TX power byte |
| → call `GetTxPower()` — returns `a0` |
| `s0[391]` | sb   | `20`           | const |
| `s0[389]` | sb   | `40`           | const |
| `s0[376]` | sh   | `1776 (0x6f0)` | const |
| `s0[378]` | sb   | `127 (0x7f)`   | const |
| `s0[380]` | sh   | `0xceba` (low16 of 0xffffceba) | signed RSSI/offset const |
| `s0[323]` | sb   | `ble[0x14]`    | TX power dup |
| `s0[384]` | sb   | `ble[0x14]`    | TX power dup |
| `s0[385]` | sb   | `ble[0x14]`    | TX power dup |
| `s0[386]` | sb   | `ble[0x14]`    | TX power dup |
| `s0[387]` | sb   | `ble[0x14]`    | TX power dup |
| `gBleLlPara[8]` | sh | `0`         | clear global ("active slave count"?) |

**Struct size lower bound**: ≥ 392 bytes (largest offset = 391 + 1). Phase B Rust mirror will need a full struct decode; defer to backlog item BL-10 / BL-9 (now both partially populated by this phase).

### 3.3 Sub-init callees (no HW touches, listed for completeness)

| Function | Size | Role | HW writes? |
|:---------|:-----|:-----|:----------|
| `ll_connect_flowcontrol_init` | 0x0c (12 B) | Init flow-control queue pointers | NO |
| `ll_connect_init_dataLen` | 0xc0 (192 B) | Init data-length params | NO |
| `ll_connect_init_pingOffset` | 0x2e (46 B) | Init LE Ping (Authenticated Payload Timer) | NO |
| `ll_connect_set_connect_timeout` | 0x44 (68 B) | Set supervision timeout | NO |
| `LL_HopInit` | 0x58 (88 B) | Count channel-map bits, reset hop index | NO |

All five are pure software / conn-state struct mutators. Verified via `objdump --disassemble=<sym> -r` + grep for `gptrBBReg|gptrLLEReg|gBleIPPara`.

---

## 4. `llSlaveSetFirstTimer` — peripheral start / first timer (NO HW touch)

ABI: `void llSlaveSetFirstTimer(void *conn_state, ...)` — `s0 = conn_state`.

### 4.1 Decode summary (verbatim path is large; structure given)

```
@ entry:
  a1 = s0[48] (lbu)              ; some byte arg
  s0[47] (sb) = gBleLlPara[3]    ; copy gBleLlPara[3] → s0[47]
  
  ra = sp[12]; s1 = sp[4]
  call ll_get_sca_divisor()      ; a0 = SCA divisor

  a5 = s0[58] (lhu)              ; conn interval
  a6 = bleClock_t[8] (lhu)       ; clock LSU
  s1 = ble[0x16] (lbu)           ; tx-window offset config

  ; --- main timer math ---
  a4 = a6 * a5
  s0[148] (sw) = a0              ; SCA divisor saved
  a2 = a4 / 800
  a4 = a4 % 800
  s0[140] (sw) = a2              ; timer base
  s0[132] (sh) = (s1 + ...)      ; timer offset half-word
  s0[32] (sh) = a4               ; remainder
  s0[130] (sh) = s1              ; secondary timer
  s0[136] (sw) = ... ; s0[144] (sw) = ... — more timer scaffold

  call __udivdi3                 ; 64-bit divide (pure compiler helper)

  s0[136] (sw) re-read           ; later math
  bleClock_t[0] (lw) = a3        ; ?? actually READ for carry, not write
  ; 64-bit add s0[136] + a0 with carry into a1; if overflow, subtract bleClock_t[4]
  s0[144] (sw) = a4

  ; --- sub-init dispatch ---
  call ll_connect_init_dataLen(s0)
  call ll_connect_init_pingOffset(s0)
  call ll_connect_set_connect_timeout(s0, computed_timeout)
  call LL_HopInit(s0)            ; build channel-hop table

  ; --- global pointer install ---
  gBleLlPara[0xa8] (sw) = s0     ; "active slave conn pointer" — ll_slave_process consumes this

  ; --- optional user data callback ---
  a5 = llRecvDataDisable; if (a5 & 2 == 0): skip
    cb = ble[52] (lw)            ; data callback fn ptr
    if (cb != NULL):
      cb(13, s0[152])            ; slot 13: TX buffer?
      cb(14, s0[156])            ; slot 14: TX buffer

  ; --- global state clear ---
  gBleLlPara[0xe] (sh) = 0        ; clear pending flag
  gBleLlPara[0x10] (sw) = 0       ; clear timer-expired flag

  ; --- tail-call to event scheduler ---
  jmp ll_set_connect_event        ; tail-call (jr t1 after auipc)
```

### 4.2 HW touch audit

| Subsystem | Touched? | Evidence |
|:----------|:--------|:---------|
| BB regs (`gptrBBReg`) | NO | `objdump --disassemble=llSlaveSetFirstTimer ll_slave.o \| grep gptrBBReg` returns empty |
| LLE regs (`gptrLLEReg`) | NO | (idem) |
| RFEND regs (BB[16..40]) | NO | not reached without `gptrBBReg` access |
| PFIC (`0xe000e000` block) | indirect via tail-call to `ll_set_connect_event` — see §4.4 |
| `gBleIPPara` | NO direct (read-only via `gBleLlPara`/`bleClock_t`/`ble`) |

**Verbatim grep run for verification**:
```bash
riscv64-unknown-elf-objdump --disassemble=llSlaveSetFirstTimer -r ll_slave.o | grep -E 'gptrBBReg|gptrLLEReg|gBleIPPara'
# (no output)
```

### 4.3 `gBleLlPara[0xa8]` install — the "active slave conn" handoff

`gBleLlPara[0xa8] (sw) = s0` is the **key handoff write** — after this, `ll_slave_process` (the TMOS-scheduled handler installed by `LL_SlaveEnalbe`) starts walking this conn-state pointer every event tick. This is the moment a slave conn becomes "live".

**WCH C-level semantic (observed)**: plain word store. No memory barrier, no atomic intrinsic, no critical section around the write. WCH's correctness comes from the runtime invariant that:
- The writer (`llSlaveSetFirstTimer`, executed from the CONNECT_IND accept path) runs in TMOS task context.
- The reader (`ll_slave_process`, scheduled via the TMOS periodic-callback installed by `ll_set_connect_event` → IRQ #13 fire) runs after the IRQ is enabled.
- The PFIC enable `*(0xe000e204) = 0x2000` happens at the **end** of `ll_set_connect_event`, which is itself reached **only after** the `gBleLlPara[0xa8]` write completes. So the write precedes the first possible read by program order + IRQ-enable ordering — single-core CH32V208 + TMOS scheduling guarantees the publish.

Iron Rule §34 (new): **Slave-conn liveness handoff via `gBleLlPara[0xa8]`**. `gBleLlPara[0xa8]` is the single global pointer that gates `ll_slave_process` enter/exit. WCH C-level uses a plain word store with no synchronization — this is safe given TMOS single-core + IRQ-enable-after-write ordering.

**Rust-specific safety layer (port design choice — NOT mirroring WCH semantics)**: per Cindy `ebbd3209` #2, the Rust port wraps this slot as `static ACTIVE_SLAVE_CONN: AtomicPtr<ConnState>` with explicit Release on publish + Acquire on read, **specifically to make the publish/retract ordering machine-checkable**. This is a Rust-side safety harness, not a claim that WCH's original model is atomic. If a future port targets a multi-core SoC or moves work to a different priority/context, the atomic semantics become necessary; on CH32V208 single-core they are essentially redundant but cheap and discipline-preserving.

Phase B port must not bypass this slot regardless of which safety layer is chosen.

### 4.4 `ll_set_connect_event` — registers TMOS handler, enables IRQ via STANDARD PFIC

Tail-callee of `llSlaveSetFirstTimer`. Decoded:

```
@ entry:
  if (bleClock_t[0xc] == 0): goto .L312    ; first call, schedule first event

@ .L313 (handler register path):
  if (pfnConnectHandle == NULL):
    gBleLlPara[0xb0] (sw) = bleClock_t[4]   ; save clock baseline
    pfnConnectHandle = &llProcessConnectEvent
  gBleLlPara[0xa4] (sw) = 0                 ; clear event flag
  call TMOS_IrqProcessRegister(pfnConnectHandle)
  *(0xe000e204) = 0x2000                    ; ← PFIC IENR: enable IRQ #13
  ret

@ .L312 (no clock yet):
  pfnConnectHandle = NULL
  gBleLlPara[0xa4] = 0
  ; compute next-event time = s0[134] * bleClock_t[8] / (0xf4240 + ...) + ble[0x17]
  ; clamp/adjust against s0[144]
  tail-call tmos_start_periodic_callback_task(ll_process_connect_event, ...)
```

**`*(0xe000e204) = 0x2000`** is the **canonical ARM/RISC-V NVIC/PFIC interrupt-enable register** (`PFIC_BASE = 0xe000e000`, offset `0x204` = `IENR3` or similar — bit 13 set = enable IRQ 13). IRQ 13 on CH32V208 is **almost certainly the BLE LL event IRQ** (TI BLE-Stack convention).

### 4.5 ⚠️ Cross-validation of Cindy Phase 3 #2 caution

| Reference | Base address | Reg semantic |
|:----------|:-------------|:-------------|
| `phy_status_clear` save/zero/restore | `0x4002_420c` | provisional "irq-mask-like MMIO" (Cindy `fbee9584` #2) |
| `ll_set_connect_event` IRQ enable    | `0xe000e204` | **canonical PFIC IENR** (standard RISC-V NVIC base 0xe000e000) |

**Strong evidence that `0x4002_420c` is NOT PFIC** — the canonical PFIC base is `0xe000e000`, and `0x4002_xxxx` is in a completely different MMIO region (AHB peripherals on CH32V208). The `0x4002_420c` MMIO is more likely an **LLE/BB-adjacent companion register** (clock gate, PHY enable, or LLE-side IRQ mask). This corroborates Cindy's `fbee9584` #2 caution; Phase 3 doc open item §9 should be updated to reflect this finding.

Iron Rule §35 (new): **Two distinct IRQ controllers in BLE path**:
- `*(0xe000e204)` = canonical PFIC IENR — used by `ll_set_connect_event` to enable BLE LL event IRQ
- `*(0x4002_420c)` = unidentified irq-mask-like MMIO — used by `phy_status_clear` for PHY transition save/zero/restore

Rust port treats these as separate primitives. Do NOT collapse into single "PFIC" wrapper until ch32-data PAC cross-check resolves both.

---

## 5. Phase A2 Rust port design rules (continuation §33-§35)

33. **Iron Rule §33 — Slave enable order**: `LL_SlaveEnalbe` requires `LL_AdvertiseEnalbe` first. Rust port: `SlaveEnabled = AdvertiseEnabled.enable_slave()` type-state pattern. Single one-shot.

34. **Iron Rule §34 — Slave-conn liveness handoff via `gBleLlPara[0xa8]`**: The pointer install at the end of `llSlaveSetFirstTimer` is the publish point that lets `ll_slave_process` consume the conn-state. **WCH C-level**: plain word store, no synchronization — correctness comes from single-core CH32V208 + TMOS scheduling + IRQ-enable-after-write program order. **Rust port (port design choice, NOT a claim about WCH semantics)**: wrap as `AtomicPtr<ConnState>` with Release-on-publish / Acquire-on-read so publish/retract ordering is machine-checkable. The atomic is a Rust-side safety harness layered onto WCH's single-thread invariant (per Cindy `ebbd3209` #2). Phase B port MUST replicate this exact pointer slot regardless of safety-layer choice.

35. **Iron Rule §35 — Two distinct IRQ controllers**: PFIC IENR at `0xe000e204` (LL event enable) is separate from the `0x4002_420c` MMIO save/zero/restore in `phy_status_clear`. Rust port keeps them as distinct typed accesses until ch32-data PAC corroborates the latter.

---

## 6. Cross-reference + impact on prior phases

- **Phase 2.5 `ll-rx-ingress-disasm.md` §3**: indirect calls on `gBleLlPara[0xb8]/[0xbc]` — now resolved to `llSlaveCreateCore` + `llSlaveSetFirstTimer`. Phase 2.5 §3 "NULL-guard + indirect call but internal black-box" comment can be replaced with direct refs.
- **Phase 3 `ll-phy-preflight-disasm.md` §9 open items**: the "Irq-mask-like MMIO at `0x4002_420c` identification" question gets new evidence — canonical PFIC is `0xe000e000` per `ll_set_connect_event`, so `0x4002_420c` is **almost certainly not PFIC**. Update §9 with this finding.
- **backlog.md BL-3 (slave-conn init + peripheral start vtable)**: ✅ CLOSED by this doc.

---

## 7. Verification artifacts (re-runnable)

```bash
cd /tmp/wchble-disasm/all_obj
# Vtable populator
riscv64-unknown-elf-objdump --disassemble=LL_SlaveEnalbe        -r ll_slave.o
# Conn-state init
riscv64-unknown-elf-objdump --disassemble=llSlaveCreateCore     -r ll_slave.o
# Peripheral start / first timer
riscv64-unknown-elf-objdump --disassemble=llSlaveSetFirstTimer  -r ll_slave.o
# Event scheduler tail-callee
riscv64-unknown-elf-objdump --disassemble=ll_set_connect_event  -r ll_connect.o
# Channel-hop table init
riscv64-unknown-elf-objdump --disassemble=LL_HopInit            -r ll_hop.o

# HW touch audit (should ALL be empty)
for sym in llSlaveSetFirstTimer LL_HopInit ll_connect_flowcontrol_init ll_connect_init_dataLen ll_connect_init_pingOffset ll_connect_set_connect_timeout; do
  echo "=== $sym ==="
  riscv64-unknown-elf-objdump --disassemble=$sym -r ll_slave.o ll_hop.o ll_connect.o 2>/dev/null | grep -E 'gptrBBReg|gptrLLEReg' || echo "(no HW touch)"
done

# Cross-grep for offset 184/188 writes across all .o (sanity that LL_SlaveEnalbe is the only populator)
for f in *.o; do
  out=$(riscv64-unknown-elf-objdump -dr "$f" 2>/dev/null | grep -E 'sw\s+[as][0-9]+,(184|188)\(a5\)')
  [ -n "$out" ] && echo "=== $f ===" && echo "$out"
done
# Expected hits: ll_slave.o (the LL_SlaveEnalbe populator) + a handful of false positives
# in ll_pdu.o / ll_sync.o / rfend.o where 184/188 are offsets in OTHER struct contexts
# (verified manually: only ll_slave.o offsets 0x1c+0x2a are gBleLlPara writes).
```

---

## 8. Open items (post-A2)

- **`ll_slave_process` (0xa28 bytes)** — full decode deferred to Phase B-or-later. This is the consumer of `gBleLlPara[0xa8]` and almost certainly the function that walks BB/LLE registers each TMOS tick.
- **`llProcessConnectEvent` / `ll_process_connect_event`** — referenced from `ll_set_connect_event` as the registered TMOS handler. Decode when porting connection-event arming.
- **`tmos_start_periodic_callback_task`** — TMOS scheduler API. Reference-only; Rust port replaces with embassy timer.
- **`gBleLlPara[0xd8]/[0xdc]` semantic** — read by `llSlaveCreateCore` and copied to conn-state[248..252] AND [264..268] (duplicated). Likely 64-bit feature mask or access-address seed. Backlog BL-9 owns full `gBleLlPara` map.
- **conn-state struct full decode** — partial map in §3.2; ~30 fields populated, struct size ≥ 392 B. Backlog BL-10 owns full map.
- **`0x4002_420c` identification** — new evidence (canonical PFIC is `0xe000e000`) suggests it's NOT PFIC. Update Phase 3 §9 open item; Andelf may have insight from ch32-data PAC.

---

## 9. A3 hand-off

Per Andelf `ccf549f4` order lock (A1 → A2 → A3 → B → D), next is **A3 — `ll_advertise_filter()` 223-line full decode** (backlog BL-4). A2 unblocks A3 because the slave-conn init path is now closed — `ll_advertise_filter` is the AdvA whitelist + RSSI gate + filter policy that runs **before** CONNECT_IND admit can dispatch into `llSlaveCreateCore`.

ETA for A3: 60-75 min (larger function, multiple state-machine branches).

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-slave-init-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
