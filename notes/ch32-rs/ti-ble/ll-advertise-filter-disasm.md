# Phase A3 — ll_advertise_filter() full decode

**Subject** — full decode of `ll_advertise_filter` (223 disasm lines, 0x14e = 334 bytes). Phase 2.5 treated this as a black-box "return 1 = accept / other = reject" gate; A3 opens the black box to map filter policy inputs, return codes, and `adv_state` mutations.

**File** — `ll_advertise.o` (`ll_advertise_filter`).

**Companion** — Phase 2.5 `ll-rx-ingress-disasm.md` §7 (the call site, RX ingress filter dispatch), Phase A2 `ll-slave-init-disasm.md` (downstream consumer when filter returns accept on CONNECT_IND).

**Cindy `ebbd3209` 3-point review focus locked**:
1. Filter policy input fields → §3 input-read map
2. Return code semantics → §4 return-code table
3. `adv_state` / global state mutations → §5 mutation audit

---

## 0. TL;DR — Cindy's 3 A3 priorities answered

### Priority 1 — Filter policy input fields (✅ ANSWERED, §3)

`ll_advertise_filter` reads **one global byte** (`gBleLlPara[1]`) and **~14 fields** of the adv_state struct (`s0 = a0` input arg). No PHY register reads, no other globals. Inputs cluster into three groups:
- **Mode select**: `gBleLlPara[1]` (global privacy/RPA-mode byte), `s0[13]` filter-policy bits, `s0[16]` target-type, `s0[19]` PDU type, `s0[60]` own-addr type bits, `s0[96]` filter-policy bit-2 bypass.
- **Address payloads (compared)**: `s0[61]` AdvA-type byte, `s0[62..67]` AdvA (6 B), `s0[68]` InitA/TargetA-type byte, `s0[69]` resolved-state flag, `s0[70..75]` TargetA / local-identity (6 B).
- **Resolving-list entry pointer**: `s0[48]` → entry struct, dereferenced at `entry[1]`, `entry[3]`, `entry[20..25]`, `entry+42` (IRK key offset for `ll_resolvinglist_checkRPA`).

### Priority 2 — Return code semantics (✅ ANSWERED, §4)

**FOUR distinct return-value sources**, not three:

| Source | Value | Path | Meaning |
|:-------|:------|:-----|:--------|
| Explicit `s1 = 0` (.L55) | `0` | AdvA whitelist mismatch + resolving-list reject | **REJECT** — drop PDU |
| Explicit `s1 = 1` (.L58) | `1` | RPA-resolve success / IRK-resolve success / "no-filter-applicable" fallthrough | **ACCEPT** |
| `s1 = snez(memcmp)` (.L48 fall-through) | `0` or `1` | Normal-mode AdvA compare path | `1` if memcmp returned non-zero (addresses differ), `0` if memcmp returned zero (addresses match). Caller-policy maps `differs`→accept/reject. **Naming kept neutral** (`AddressCompare { differs: bool }`) per Cindy `8a3ad99c` #2 — loopback/self-reject interpretation deferred to air-PASS, see §4.3. |
| Tail-jump to `LL_WhitelistLookup` (.L57) | whatever `LL_WhitelistLookup` returns | SCAN_REQ + filter-policy bit 0, OR CONNECT_IND + filter-policy bit 1 | **DELEGATED** — downstream whitelist function's return value |

**Caller convention (per Cindy `8a3ad99c`)**: at the call site, treating `return == 1` as "accept" is still valid — this is the WCH C-level convention that Phase 2.5 §7 captured. **Rust port internal model**: preserve outcome provenance; don't collapse explicit-0, snez-derived-0, and delegated-whitelist-miss into a single `Reject` reason. The 4 sources carry distinct downstream meaning (resolving-list mismatch vs address compare vs delegated whitelist miss vs explicit reject) that conn-state init + logging need.

### Priority 3 — adv_state / global state mutations (✅ ANSWERED, §5)

**`ll_advertise_filter` IS a mutating filter** — it writes to `adv_state` in two paths (RPA-resolve success and IRK-resolve success):

| Mutation | When | Effect |
|:---------|:-----|:-------|
| `s0[68] (sb) = 2` | RPA/IRK resolve success | Marks dest-addr-type as "resolved identity" |
| `s0[69] (sb) = entry[3] \| 2` | (idem) | Sets identity-type byte with bit 1 = "resolved" |
| `s0[70..75] (memcpy 6 B)` | (idem) | Replaces TargetA payload with IDA (identity address from resolving-list entry) |
| `s0[48] (sw) = LL_ResolvinglistPeerRpaAddressGetIrk()` | IRK lookup success | Caches the resolving-list entry pointer back into adv_state |

**No `gBleLlPara` writes** — only reads at offset `+1`. **No PHY register writes**.

**Rust port API implication (per Cindy's #3 focus)**: this is **NOT a `&self` filter**; must be `&mut self` (or expose a `(Outcome, AdvStatePatch)` tuple that the caller applies). Pure-read API is not faithful.

---

## 1. ABI

`int ll_advertise_filter(adv_state *s)` — `a0` = adv_state pointer (loaded into `s0` for the function body). Returns int (0 = reject, 1 = accept, or delegated `LL_WhitelistLookup` return).

Caller: `ll_advertise_legacy_rx` (Phase 2.5 §7).

---

## 2. Full control-flow tree

```
ENTRY @ 0x00:
  prologue: sp -= 16; save s0/s1/s2/ra
  s0 = a0 (= adv_state)
  a5 = gBleLlPara[1] (lbu)        ; GLOBAL privacy/RPA-mode select byte
  if a5 != 0: goto .L47           ; privacy/RPA path
  else:       fall through to .L53

.L53 @ 0x16 (NORMAL whitelist path, gBleLlPara[1] == 0):
  a4 = s0[16] (lbu)
  if a4 != 1: goto .L74           ; type-1 special handling
  else:       fall through to .L48

.L48 @ 0x20 (AdvA self/whitelist compare):
  a5 = s0[61] (lbu) & 1           ; low bit of AdvA-type byte
  a4 = s0[69] (lbu)               ; resolved-state byte
  if a4 != a5: goto .L55 → return 0   ; type-byte mismatch → REJECT
  call tmos_memcmp(s0+62, s0+70, 6)
  s1 = (memcmp != 0) ? 1 : 0      ; snez s1, a0
  goto .L52 → return s1            ; 1 if differs, 0 if identical

.L47 @ 0x46 (PRIVACY/RPA path, gBleLlPara[1] != 0):
  ; (note: this path reads from a0, NOT s0 — a0 still holds the original arg
  ;  until the explicit `s0 = a0` was done at 0x12; but compiler kept a0 live
  ;  through 0x46-0x5e, equivalent to reading from adv_state)
  a5 = a0[69] (lbu)                ; resolved-state byte
  if a5 == 0: goto .L50            ; not yet resolved → fallthrough tier
  a5 = a0[75] (lbu) & 0xc0
  if a5 != 0x40: goto .L50         ; high bits not 0x40 → fallthrough
  a0 = a0[48] (lw)                 ; resolving-list entry ptr
  if a0 == 0: goto .L51            ; no entry → IRK-lookup path
  
  ; Have entry pointer — try RPA resolve
  s2 = s0+70                       ; TargetA buffer ptr
  a1 = s2
  a0 = entry+42                    ; (entry[42..49] is likely 8-byte IRK key)
  call ll_resolvinglist_checkRPA(entry+42, s0+70) → s1 = a0
  if s1 != 1: goto .L51            ; RPA check failed → IRK fallback
  
  ; RPA resolve success — mutate adv_state
  a0 = s0[48]                      ; entry ptr (re-loaded)
  s0[68] (sb) = 2                  ; ← MUTATE: dest-type = "resolved"
  a5 = entry[3] (lbu) | 2          ; identity flags | bit 1
  s0[69] (sb) = a5                 ; ← MUTATE: resolved-state byte
  a1 = s2 (= s0+70); a2 = 6; a0 = entry+20
  call tmos_memcpy(s0+70, entry+20, 6)   ; ← MUTATE: 6-byte IDA copy
  fallthrough → .L52 → return s1=1

.L51 @ 0xaa (IRK fallback / no entry):
  a4 = s0[16] (lbu)
  if a4 == 1: goto .L50            ; type-1 → fallthrough
  a5 = s0[96] (lbu) & 4
  if a5 != 0: goto .L50            ; filter-policy bit 2 set → fallthrough
  a5 = s0[60] (lbu)
  if a5 != 0: goto .L50            ; own-addr type set → fallthrough
  
  ; Try IRK-based resolve
  a0 = s0+68
  call LL_ResolvinglistPeerRpaAddressGetIrk(s0+68) → a0
  s0[48] (sw) = a0                 ; ← MUTATE: cache entry ptr in adv_state
  if a0 == 0: goto .L50            ; no IRK match → fallthrough
  
  ; IRK match — mutate adv_state (same as RPA path)
  s0[68] (sb) = 2                  ; ← MUTATE
  a5 = a0[3] (lbu) | 2
  s0[69] (sb) = a5                 ; ← MUTATE
  a1 = s0+70; a2 = 6; a0 = entry+20
  call tmos_memcpy(s0+70, entry+20, 6)   ; ← MUTATE
  goto .L58 → s1 = 1 → return 1

.L50 @ 0xf8 (RPA-resolve failed/skipped — final filter tier):
  a5 = s0[60] (lbu) & 2
  if a5 == 0: goto .L53            ; resolving-list not required → normal path
  a5 = s0[48] (lw)
  a5 = a5[1] (lbu)                 ; entry[1] (= entry-valid flag?)
  if a5 != 0: goto .L53            ; entry valid → normal path
  ; entry invalid AND resolving-list required → REJECT
  fallthrough to .L55

.L55 @ 0x108:
  s1 = 0
  goto .L52 → return 0

.L52 @ 0x9c (common epilogue):
  return s1

.L74 @ 0x10c (s0[16] != 1, type-N path):
  a5 = s0[96] (lbu) & 4
  if a5 != 0: goto .L48            ; filter-policy bit 2 → same as type-1 path
  a5 = s0[13] (lbu) & 1
  if a5 == 0: goto .L56
  a3 = s0[19] (lbu)                ; PDU type
  if a3 != 3: goto .L56            ; not SCAN_REQ → check CONNECT_IND
  goto .L57 (tail-jump to LL_WhitelistLookup)

.L56 @ 0x13e:
  a5 = s0[13] (lbu) & 2
  if a5 == 0: goto .L58 → s1=1, return 1   ; no filter applies → ACCEPT
  a4 = s0[19] (lbu)
  if a4 != 5: goto .L58 → s1=1, return 1   ; not CONNECT_IND → ACCEPT
  fallthrough to .L57

.L57 @ 0x128 (delegated whitelist lookup, TAIL-CALL):
  a0 = s0[68] (lw)                 ; pass 4 bytes starting at s0+68 (= dest type + IDA[0..3])
  a1 = s0[72] (lw)                 ; pass next 4 bytes (= IDA[4..5] + padding)
  ; (Note: this is reading 8 contiguous bytes from s0+68..s0+76 as two u32 args —
  ;  effectively packing (type-byte + 6-byte addr + 1 padding) into 2 u32 regs
  ;  per the RV32 ABI convention. LL_WhitelistLookup signature is
  ;  probably u32(u32, u32) reinterpreting as (type, 6-byte-addr) struct.)
  restore registers, deallocate stack
  jr t1 → tail-call LL_WhitelistLookup    ; return value = its return
```

---

## 3. Filter policy input field map (Cindy #1)

### 3.1 Global state (read-only)

| Source | Type | Use | Decoded meaning |
|:-------|:-----|:----|:----------------|
| `gBleLlPara[1]` | u8 | Dispatch root: `== 0` → normal whitelist, `!= 0` → privacy/RPA mode | Likely **LL Address Resolution Enable** bit from `LE_Set_Address_Resolution_Enable` HCI command |

No other globals read; **no PHY/MMIO reads**.

### 3.2 adv_state fields (input)

| Field | Type | Bit/value semantics (decoded) | Used by |
|:------|:-----|:------------------------------|:--------|
| `s0[13]` | u8 | bit 0 = "apply whitelist on SCAN_REQ", bit 1 = "apply whitelist on CONNECT_IND" | .L74 / .L56 (filter-policy mode select) |
| `s0[16]` | u8 | `== 1` → type-1 target dispatch | .L53 / .L74 / .L51 |
| `s0[19]` | u8 | PDU type (3 = SCAN_REQ, 5 = CONNECT_IND per Phase 2.5) | .L74 / .L56 (LL_WhitelistLookup dispatch) |
| `s0[48]` | u32 ptr | Resolving-list entry pointer (cached from prior call); also used as `entry[1]` and `entry[3]` and `entry+20..25` and `entry+42` after deref | .L47 / .L51 (RPA resolve) |
| `s0[60]` | u8 | bit 0 = "own-addr type bit", bit 1 = "resolving-list required" | .L50 / .L51 |
| `s0[61]` | u8 | bit 0 = AdvA-type-low-bit (masked `& 1` for compare with s0[69]) | .L48 |
| `s0[62..67]` | u8[6] | AdvA payload (6 bytes from received PDU) | .L48 (`tmos_memcmp(s0+62, s0+70, 6)`) |
| `s0[68]` | u8 | InitA/TargetA-type byte (pre-resolve); also reinterpreted as low byte of u32 arg to LL_WhitelistLookup | .L47 / .L51 / .L57 — also MUTATED |
| `s0[69]` | u8 | Resolved-state byte; bit 1 = "resolved-success marker" | .L47 / .L48 — also MUTATED |
| `s0[70..75]` | u8[6] | TargetA payload (or local-identity address — semantic ambiguity, see §4.3) | .L48 (memcmp arg2) / .L47 / .L51 — also MUTATED |
| `s0[75]` | u8 | bits 6,7 checked via `& 0xc0 == 0x40` → "RPA-resolve eligible" marker | .L47 |
| `s0[96]` | u8 | bit 2 = "bypass dispatch to .L48 short-path" (filter-policy escape) | .L74 / .L51 |

### 3.3 Resolving-list entry fields (deref via s0[48])

| Field | Type | Use |
|:------|:-----|:----|
| `entry[1]` | u8 | "entry-valid" flag — checked in .L50 final tier |
| `entry[3]` | u8 | Identity-AddrType byte — OR'd with `2` and stored to `s0[69]` |
| `entry[20..25]` | u8[6] | IDA (identity address) — memcpy'd to `s0[70..75]` |
| `entry+42` | u8[?] | Likely IRK key buffer — passed to `ll_resolvinglist_checkRPA` |

(Full entry struct decode deferred — see §9 open items.)

---

## 4. Return code semantics (Cindy #2)

### 4.1 Static return-code table

| Return source | Value | Condition |
|:--------------|:------|:----------|
| .L55 (s1=0) | `0` | (normal-mode AdvA mismatch — but actually .L55 is only reached from .L48 when `s0[69] != (s0[61] & 1)`, OR from .L50 fall-through when resolving-list required + entry invalid) |
| .L58 (s1=1) | `1` | (a) RPA-resolve success (.L47 → .L52), (b) IRK-resolve success (.L51 → .L58), (c) no-filter-applicable fall-through (.L74 → .L56 → .L58, when neither bit 0/1 of `s0[13]` triggers a whitelist dispatch) |
| .L48 snez | `0` or `1` | `s1 = (tmos_memcmp(s0+62, s0+70, 6) != 0)` — interpreted as "1 = AdvA differs from local-identity buffer, 0 = identical" |
| .L57 tail-jump | `LL_WhitelistLookup(s0[68..76] as 2×u32) return` | (a) `s0[13]&1` set AND `s0[19]==3` (SCAN_REQ), (b) `s0[13]&2` set AND `s0[19]==5` (CONNECT_IND), (c) `s0[16]!=1` AND `s0[96]&4` set (filter bypass) routes through .L48 fall-through |

### 4.2 Caller convention vs Rust internal model (per Cindy `8a3ad99c` #1)

**Caller convention (WCH C-level, captured in Phase 2.5 §7)**: at the call site, `return == 1` → accept, anything else → reject. This is valid and remains the public ABI of the function.

**Rust internal model**: preserve outcome provenance — DO NOT collapse all non-1 returns into a single `Reject`. The 4 return sources carry different downstream semantics:

| Source | Caller sees | Rust internal variant | Distinct downstream use |
|:-------|:------------|:----------------------|:------------------------|
| `.L58` (explicit 1) | accept | `AcceptResolved` | RPA/IRK resolved or no-filter applies — conn-state init reads mutated `s0[68..75]` |
| `.L55` (explicit 0) | reject | `Reject` | Drop PDU, log "resolving-list strict mismatch" or "AdvA-type mismatch" |
| `.L48` snez 1 | accept | `AddressCompare { differs: true }` | Address compare disagreed — caller maps to accept/reject per policy |
| `.L48` snez 0 | reject | `AddressCompare { differs: false }` | Address compare agreed — caller maps to accept/reject per policy |
| `.L57` tail-call ret 1 | accept | `DelegateToWhitelist(arg)` then upcall ret | Whitelist hit — distinct from `.L58` accept because whitelist entry index might propagate |
| `.L57` tail-call ret 0 | reject | `DelegateToWhitelist(arg)` then upcall ret | Whitelist miss — distinct from `.L55` reject because no resolving-list state changed |

The Rust port should expose both: a high-level `is_accept()` bool method that matches caller convention, and the structured `FilterOutcome` for logging / state-machine reasoning.

### 4.3 ⚠️ Semantic caveat for .L48 snez path

The interpretation "s0[62..67] = received AdvA, s0[70..75] = local-identity-or-TargetA" yields the result "`1 = differs = accept, 0 = matches = reject as loopback`". **But this is not provable from disasm alone**: alternative interpretations include:
- s0[62..67] = expected (whitelist top entry), s0[70..75] = received → match = whitelist hit, then `snez` returns `0` (treat as "exact match → reject" — counterintuitive, probably wrong)
- s0[62..67] and s0[70..75] are two received-packet fields (e.g. AdvA vs InitA), and matching them is malformed → reject the malformed PDU

The data-flow into this function from `ll_advertise_legacy_rx` (Phase 2.5 §3) populates `s0[60..76]` during PDU ingress; without that field map locked, the .L48 semantic stays ambiguous. **Air-PASS verifies. Per Cindy `8a3ad99c` #2, Rust port keeps variant name neutral as `AddressCompare { differs: bool }` — no `Loopback`/`SelfReject` naming until data-flow confirmed.**

### 4.4 Iron Rule §36 (new) — wording per Cindy `8a3ad99c` #1

**Iron Rule §36 — preserve outcome provenance, don't collapse non-1 returns into one reason**:

- **Caller convention (matches Phase 2.5 §7 black-box)**: caller code MAY treat `return == 1` as "accept" for branch decisions; this is the WCH C-level convention and remains valid.
- **Rust internal model**: Rust port MUST preserve the outcome source (which path produced the return value) — DO NOT collapse explicit-0, snez-derived-0, and `LL_WhitelistLookup`-returned-0 into a single `Reject` variant. Each carries different downstream meaning (resolving-list mismatch vs AdvA address compare vs delegated whitelist miss) that the conn-state initializer and logging need to distinguish.

```rust
enum FilterOutcome {
    Reject,                              // explicit s1=0 (.L55: resolving-list required + entry invalid, or AdvA-type-byte mismatch)
    AcceptResolved,                      // .L58 fall-through (RPA / IRK / no-filter applies)
    AddressCompare { differs: bool },    // .L48 snez — neutral naming pending air-PASS (see §4.3 caveat). bool: true if memcmp returned non-zero
    DelegateToWhitelist(WhitelistArg),   // .L57 — call site provides arg, LL_WhitelistLookup runs and returns 0/1
}
```

Naming note: `.L48` variant is `AddressCompare { differs: bool }` (neutral) — the loopback/self-reject interpretation (per §4.3) is NOT baked into the type until air-PASS or full data-flow confirmation. Caller chooses how to map `differs: true/false` to accept/reject.

Iron Rule §0 (逐 register 全写) still applies to the mutation paths in §5.

---

## 5. adv_state mutations (Cindy #3)

### 5.1 Write map (`ll_advertise_filter` is NOT a pure-read filter)

| Write site | Byte | Value | Trigger path |
|:-----------|:-----|:------|:-------------|
| `s0[48] (sw)` | 4 B | `LL_ResolvinglistPeerRpaAddressGetIrk(s0+68)` return ptr | .L51 (IRK fallback path) |
| `s0[68] (sb)` | 1 B | `2` | .L47 RPA-success path, .L51 IRK-success path |
| `s0[69] (sb)` | 1 B | `entry[3] \| 2` | .L47 RPA-success path, .L51 IRK-success path |
| `s0[70..75]` | 6 B | `entry[20..25]` (via `tmos_memcpy`) | .L47 RPA-success path, .L51 IRK-success path |

**Total mutation surface**: up to 12 bytes of `adv_state` written per call, IF the resolve path triggers. Normal-mode paths (.L48, .L57, .L55, .L74→.L58) leave adv_state untouched.

### 5.2 No global state writes

Cross-verified: `objdump --disassemble=ll_advertise_filter ll_advertise.o | grep -E 'sb|sh|sw' | grep -vE '\([as][0-9]+\)$'` — all stores are to `s0+offset` or `sp+offset` (= stack). No `gBleLlPara` or `gBleIPPara` or PHY-register writes.

### 5.3 External-callee side effects (unknown)

The filter calls 4 external functions that may have their own side effects:
- `tmos_memcmp` — pure (no writes expected)
- `tmos_memcpy` — writes the dest buffer (s0+70..75) — accounted for in §5.1
- `ll_resolvinglist_checkRPA` — likely pure (read IRK + RPA, return match bool), but not yet decoded
- `LL_ResolvinglistPeerRpaAddressGetIrk` — pointer return; might update an internal cursor in the resolving-list (not yet decoded)
- `LL_WhitelistLookup` (tail-call only) — likely pure read; not yet decoded

Backlog: full audit of these 4 callees deferred to "post-air-PASS resolving-list decode".

### 5.4 Iron Rule §37 (new)

**Iron Rule §37 — `ll_advertise_filter` mutates adv_state on resolve success**: Rust port signature MUST be `&mut self` (or `fn filter(&self, ...) -> (Outcome, Option<AdvStatePatch>)` with caller applying). The function modifies 4 fields (s0[48], s0[68], s0[69], s0[70..75]) when the RPA or IRK resolve hits. Pure-read API is faithless — Phase B must not regress.

---

## 6. Cross-reference + impact on prior phases

- **Phase 2.5 `ll-rx-ingress-disasm.md` §7** — filter "return 1 = accept / other = reject" black-box description is structurally correct but **incomplete in 2 dimensions**: (1) misses the .L57 tail-call delegation, (2) misses the adv_state mutations on RPA/IRK success paths. Phase 2.5 §7 should reference this doc for full semantic.
- **Phase 2.5 `ll-rx-ingress-disasm.md` §3** — the `adv_state` byte-13/16/19/60/61/68/69/96 fields used by this filter need to be added to the `adv_state` struct map (backlog BL-10). Field-19 = PDU type is consistent with Phase 0/2.5 (3=SCAN_REQ, 5=CONNECT_IND).
- **Phase A2 `ll-slave-init-disasm.md`** — `llSlaveCreateCore` runs **after** this filter returns accept on CONNECT_IND. The filter's mutations to `s0[68..75]` (dest type = 2, identity address) are what `llSlaveCreateCore` reads as the peer-identity address when populating the new conn-state.
- **backlog.md BL-4 (ll_advertise_filter decode)**: ✅ CLOSED by this doc.

---

## 7. Rust port design rules (continuation §36-§37)

36. **Iron Rule §36 — preserve outcome provenance**: see §4.4. Caller convention (`return == 1` → accept) is preserved as the public ABI. Rust internal MUST keep the 4 outcome sources distinguishable — explicit-0, snez-derived, delegated-whitelist-miss carry different downstream meaning and must not be collapsed into a single `Reject`. `.L48` variant naming stays neutral as `AddressCompare { differs: bool }` until air-PASS or full data-flow confirmation per `8a3ad99c` #2.

37. **Iron Rule §37 — filter mutates adv_state**: see §5.4. `&mut self` API required. Rust port preserves the 4 write sites verbatim on resolve-success paths.

---

## 8. Verification artifacts (re-runnable)

```bash
cd /tmp/wchble-disasm/all_obj
# Function disasm
riscv64-unknown-elf-objdump --disassemble=ll_advertise_filter -r ll_advertise.o

# No-globals-written audit (should show only s0+offset and sp+offset destinations)
riscv64-unknown-elf-objdump --disassemble=ll_advertise_filter -r ll_advertise.o \
  | grep -E '^\s+[0-9a-f]+:.*\bs[bhw]\b' | grep -vE '\(([as]0|sp)\)' || echo "(no non-s0/sp stores)"

# External-callee list
riscv64-unknown-elf-objdump --disassemble=ll_advertise_filter -r ll_advertise.o \
  | grep -E 'R_RISCV_CALL' | awk '{print $NF}' | sort -u
# Expected: tmos_memcmp, tmos_memcpy, ll_resolvinglist_checkRPA,
#           LL_ResolvinglistPeerRpaAddressGetIrk, LL_WhitelistLookup

# Globals-read audit
riscv64-unknown-elf-objdump --disassemble=ll_advertise_filter -r ll_advertise.o \
  | grep -E 'R_RISCV_PCREL_HI20' | awk '{print $NF}' | sort -u
# Expected: gBleLlPara+0x1 (the only global)
```

---

## 9. Open items (post-A3, post-air-PASS)

- **`LL_WhitelistLookup`** — tail-callee from .L57. Its return value bubbles up as the filter's return when whitelist-mode applies. Decode when porting whitelist subsystem.
- **`ll_resolvinglist_checkRPA`** — RPA match-or-no check (probably hash-based using IRK + 24-bit prand). Used by .L47. Decode when porting privacy subsystem.
- **`LL_ResolvinglistPeerRpaAddressGetIrk`** — IRK lookup by peer RPA. Returns resolving-list entry pointer (or NULL). Decode with `ll_resolvinglist_checkRPA`.
- **Resolving-list entry struct** — fields `entry[1]`, `entry[3]`, `entry[20..25]`, `entry+42` (likely IRK). Full layout pending decode.
- **adv_state field-13/16/19/60/61/68/69/75/96 semantic** — bit-by-bit decode needed against air capture / spec; backlog BL-10.
- **.L48 snez semantic** — interpretation of `tmos_memcmp(s0+62, s0+70, 6)` direction (received-vs-local? own-vs-incoming?) needs air-PASS confirmation. Conservative Rust model is `DependsOnAddressMatch(bool)` per Iron Rule §36.

---

## 10. Hand-off

Per Andelf `ccf549f4` order lock, A1 → A2 → A3 → **B** is now unblocked. A3 closes BL-4. Next: **B milestone — Rust ADV/SCAN_RSP skeleton** in `ch32-hal/src/ble/`. ETA TBD after Cindy A3 review.

The B kickoff has all the pieces it needs:
- Phase 0 `ll-advertise-tx-disasm.md` (ADV TX register sequence + 15 Iron Rules)
- Phase 1 `ll-tx-completion-disasm.md` (TX completion + cold-kick)
- Phase 2.5 `ll-rx-ingress-disasm.md` (RX ingress + PDU dispatch + admit set)
- Phase 3 `ll-phy-preflight-disasm.md` (PHY/RFEND/PFIC preflight + RX kick + IRQ-mask gating)
- Phase A2 `ll-slave-init-disasm.md` (slave conn-state init + peripheral start)
- Phase A3 `ll-advertise-filter-disasm.md` (this doc — filter policy + 4-valued outcome + mutation surface)

A3 deliberately does NOT touch `ll_slave_process`, EXT_ADV, or aux paths — those remain post-B (backlog BL-7) or deferred entirely.

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-advertise-filter-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
