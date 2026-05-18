# Phase 3 — PHY preflight (BLE_SetPHYTxMode / BLE_SetPHYRxMode / phy_status_clear / ll_tx_wait_finish mode=3)

**Subject** — how `libwchble.a` wires the BB/LLE PHY registers before TX/RX and where the actual "TX kick" happens for SCAN_RSP. Resolves Cindy's locked Phase 3 DoD #1 (msg `c43039d0`).

**Files** — `ip.o` (`BLE_SetPHYTxMode`, `BLE_SetPHYRxMode`, `phy_status_clear`, `ll_tx_wait_finish`).

**Companion** — Phase 0 `ll-advertise-tx-disasm.md` (TX register sequence), Phase 1 `ll-tx-completion-disasm.md` (`ll_tx_wait_finish` mode=0 KICK), Phase 2.5 `ll-rx-ingress-disasm.md` (SCAN_REQ → SCAN_RSP path, DoD-3 PARTIAL).

**Notation** — same lock as Phase 2/2.5: hex offsets, BB = `gptrBBReg`, LLE = `gptrLLEReg`, RFEND register file is the BB chain `BB[16..40]`.

---

## 0. TL;DR — Cindy's 3 review priorities answered

### Priority 1 — SCAN_RSP TX kick attribution (✅ ANSWERED)

**The TX kick is split across TWO mechanisms depending on path**:

| TX path | Trigger write | Where |
|:--------|:--------------|:------|
| **Primary ADV / SCAN_REQ broadcast** (TX from cold) | **`LLE[0] = 2`** | `ll_tx_wait_finish` **mode = 0** path @ offset 0x11a |
| **SCAN_RSP response** (RX → TX turnaround after SCAN_REQ accepted) | **`BB[0] |= 0x800000`** (set bit 23) + `BB[44] &= ~3` | `ll_tx_wait_finish` **mode = 3** path @ offset 0x15a–0x160 |

**Phase 3 DoD #1 direct answers**:
- **(a) Does `ll_tx_wait_finish(mode=3, ...)` write `LLE[0] = 2`?** → **NO**. mode=3 path writes `BB[0] |= 0x800000` (bit 23) + `BB[44] &= ~3`, then drops into the same .L81 wait loop as mode=0 (no LLE[0] write).
- **(b) Does `BLE_SetPHYTxMode` write a TX kick?** → **NO** in any of its 4 path variants. It writes PHY/RF config (BB[0] mode bits, BB[44] lower 2 bits, LLE[8]=0x2000, LLE[12]=mode_config, LLE[100]=window-timeout, gBleIPPara[4]=0x80) but does NOT write LLE[0] or BB[0] bit 23.

**Interpretation**: BB[0] bit 23 is a **"hardware-armed RX→TX turnaround fire"** trigger. The hardware was placed in an RX-then-respond armed state by the SCAN_REQ ingress; setting BB[0] bit 23 releases the pre-staged SCAN_RSP. This is a different control surface than `LLE[0] = 2` which is the **"cold TX command"** trigger (full PHY setup → start TX state machine).

### Priority 2 — 1M PHY default path verbatim reproducibility (✅ ANSWERED)

**Yes, the 1M PHY mode==1 path in `BLE_SetPHYTxMode` is verbatim-portable to Rust.** It consists of pure register writes with one indirection (PA control optional prologue). See §1.1 for the exact instruction sequence.

The 1M PHY default path writes (in order):
1. *(Optional)* PA Control prologue — only if `gPaControl != NULL`: read/OR two BB pointer-offset registers
2. `BB[0] &= 0xfffd_cfff` (clear bits 12, 13, 17, 18 — PHY mode select region)
3. `LLE[12] &= 0xfffe_dfff` (clear LLE[12] bit 12)
4. `LLE[8] = 0x2000`
5. `gBleIPPara[4] = 0x80` (signed -128, low byte = 0x80, this is the "IRQ enable byte")
6. `LLE[100] = ((channel_idx + 11) << 2 + (gBleIPPara[10] + 158)) << 1` (TX window/timeout)
7. `LLE[12] = original_a0_value` (PHY mode register restored)

**No "magic constants" outside of standard PHY config**. 1M is the WCH default and the simplest path.

### Priority 3 — Which PHY/RFEND bit-segments MUST remain constant (✅ ANSWERED)

The "verbatim must-preserve" RFEND calibration constants live in `BLE_SetPHYRxMode` (writes BB[16..40] with magic values, per-PHY-mode). These are vendor-calibrated values **without datasheet** (GF-1) — Rust port MUST copy verbatim.

**Per-PHY-mode RFEND constants** (`BLE_SetPHYRxMode`):

| PHY mode (`a0`) | BB[16] (variant A, `dtmFlag&1 != 0`) | BB[16] (variant B, `dtmFlag&1 == 0`) | BB[20] | BB[24] | BB[28] | BB[32] | BB[36] | BB[40] |
|:----------------|:--------|:--------|:-------|:-------|:-------|:-------|:-------|:-------|
| 2 / 6+ (Coded / aux) `(a0 & 2)` | `0x3fa_4df` | `0x34a_4df` | `0x8301_ff1` | `0x31_619` | (not set) | `0x9_0086` | `0x1006_310` | `0x3_28de` |
| 1 (1M default, `(a0 & 2)==0`) | `0x3fa_2ce` | `0x322_2d0` | `0x8101_901` | `0x31_624` | (not set) | `0x9_0083` | `0x1006_310` | `0x3_28be` |

**Constant across all PHY modes** (always re-written to same value):
- `BB[36] = 0x1006_310` — RFEND constant
- `LLE[8] = 0x2000` — written by every `BLE_SetPHYTxMode` path and `ll_tx_wait_finish` mode=1
- `gBleIPPara[4] = 0x80` — IRQ enable byte (written by every PHY-set path)

**`dtmFlag` BB[16] swap** (decoded @ offset 0x7a–0x84 of `BLE_SetPHYRxMode`):
```
a3 = dtmFlag & 1
lui   a4, 0x3fa                  ; a4 = 0x3fa_000  ← default ("variant A")
bnez  a3, .L108                  ; if (dtmFlag & 1) keep a4 = 0x3fa_xxx
lui   a4, 0x34a                  ; else overwrite to 0x34a_xxx ("variant B")
.L108:
  a4 += 1247 (= 0x4df)           ; or +718 = 0x2ce on 1M path
  BB[16] = a4
```

⚠️ **Naming caution (Cindy `fbee9584` #1)** — the production-vs-DTM labeling in the original docstring is counterintuitive and not yet EVT-verified. Use neutral `rf_calib_variant_a` (= `0x3fa_xxx`, taken when `dtmFlag & 1 != 0`) and `rf_calib_variant_b` (= `0x34a_xxx` / `0x322_2d0`, taken when `dtmFlag & 1 == 0`) until an EVT capture proves which variant is the production air calibration. Rust port exposes `rf_calib_variant: Variant` selector; air-PASS confirms semantic mapping.

---

## 1. `BLE_SetPHYTxMode` — TX PHY/RF setup, NO kick

ABI: `void BLE_SetPHYTxMode(u8 mode, u32 channel_idx)` — `a0 = mode`, `a1 = channel_idx`.

### 1.1 Mode-1 default path (1M PHY, RIGHT after entry, .L69 fall-through)

Decision: `mode == 1` → fall through to lines 0x42–0x80.

```
0x00: a5 = &gPaControl; if (gPaControl != NULL) → PA prologue (next 4 lines)
       0x0c: a3 = gPaControl[4] (pointer)
       0x0e: a2 = gPaControl[5] (value)
       0x10: *a3 |= a2                                    ; PA-A: OR into MMIO at *gPaControl[4]
       0x16: a3 = gPaControl[0] (pointer)
       0x18: a5 = gPaControl[2] (value)
       0x1a: *a3 |= a5 | (*a3 & ...)                      ; PA-B: OR into MMIO at *gPaControl[0]

.L69 @ 0x20: a5 = &gptrBBReg → BB pointer in a5 (then a5 = *a5 = BB base)
       0x28: a2 = &gptrLLEReg
       0x30: a6 = 1
       0x32: a5 = *a5 = BB base
       0x34: a4 = gBleIPPara[0xa]                         ; lookup byte for channel→freq map
       0x3c: a3 = *a2 = LLE base
       0x3e: bne a0, a6, .L70   ; if mode != 1, go to .L70 path

       0x42: a0 = BB[0]
       0x44: a6 = 0xfffd_cfff   (mask: clear bits 12, 13, 17, 18)
       0x48: a0 &= a6
       0x4c: BB[0] = a0          ; clear PHY-mode bits in BB[0]

       0x4e: a0 = LLE[12]
       0x50: a5 = 0xfffe_dfff   (mask: clear LLE[12] bit 12)
       0x54: a5 &= a0
       0x56: LLE[12] = a5

       0x58: nop ; nop                                     ; pipeline drain

       0x5c: a3 = *a2 = LLE base   (reload — gptrLLEReg may have changed?)
       0x5e: a5 = 0x2000
       0x60: a1 += 11
       0x62: LLE[8] = a5          ; LLE[8] = 0x2000   ← LLE config strobe
       0x64: a5 = -128 (i8)        (= 0x80 low byte)
       0x6c: gBleIPPara[4] = 0x80  ; IRQ enable byte
       0x70: a5 = a1 << 2          ; a5 = (channel_idx + 11) << 2
       0x74: a1 = a4 + 158         ; a1 = gBleIPPara[0xa] + 158
       0x78: a1 += a5              ; a1 = a4 + 158 + ((ch+11) << 2)
       0x7a: a1 <<= 1              ; final shift
       0x7c: LLE[100] = a1         ; TX window/timeout (← THIS is what ll_tx_wait_finish polls)
       0x7e: LLE[12] = a0          ; LLE[12] restored to original

       0x80: ret
```

**Critical**: no `LLE[0] = 2`, no `BB[0] |= 0x800000`. The TX **kick** is the caller's responsibility (via `ll_tx_wait_finish` mode=0 for cold TX, or mode=3 for SCAN_RSP turnaround).

### 1.2 Mode-2/3/other paths (.L70)

`a0 != 1` → `.L70` @ 0x82.

Sub-dispatch: `t1 = a0 & 2`:
- `t1 != 0` (a0 ∈ {2, 3, 6, 7, ...}) → continue at 0x90 (BB[0] = (old & 0xfffd_cfff) | 0x2000), then `bne a0, 3, .L73`
  - **mode == 3** (.L70 + fall-through, Coded S2 or 2M): BB[0] = (old & 0xffff_3fff) | **0x4000**, LLE[12] &= 0xfffe_dfff, LLE[8] = 0x2000, gBleIPPara[4] = 0x80; common tail .L77
  - **else** (mode == 2 or 6/7) (.L73 @ 0xf2): BB[0] &= 0xffff_3fff (no OR), LLE[12] &= 0xfffe_dfff, LLE[8] = 0x2000, gBleIPPara[4] = 0x80; common tail .L77
- `t1 == 0` (a0 ∈ {0, 4, 5, ...}) → .L72 @ 0x122: BB[0] = (old & 0xffff_3fff) | **0x1000** (set bit 12), LLE updates same, common tail .L77

**Common tail .L77 @ 0xe8**:
```
a1 += a4                         ; final addend
a1 <<= 1                          ; shift left 1
LLE[100] = a1                     ; TX window/timeout
LLE[12] = a0_saved                ; PHY mode register
ret
```

### 1.3 BB[0] PHY-mode bit assignment (decoded)

From mask + OR analysis:

| PHY mode (a0) | BB[0] bits set | Likely PHY |
|:--------------|:---------------|:-----------|
| 0 / 4 / 5 (`a0 & 2 == 0`, but != 1) | `0x1000` (bit 12) | (unknown / direct test mode?) |
| 1 (default fall-through) | (none added, mask=0xfffd_cfff only) | **1M LE** (default) |
| 2 / 6 / 7 (`a0 & 2 != 0`, a0 != 3) | `0x2000` (bit 13, intermediate) | (Coded S8? aux variant?) |
| 3 (`a0 & 2 != 0`, a0 == 3) | `0x4000` (bit 14) | **2M LE** or **Coded S2** |

Without datasheet (GF-1), exact PHY assignment is **inferred by call-site context**. Rust port should expose a `BBPhyMode` enum with raw bits as variants and let air-PASS confirm semantic mapping.

---

## 2. `BLE_SetPHYRxMode` — RX PHY/RF setup, NO kick, MUST be verbatim

ABI: `void BLE_SetPHYRxMode(u8 mode, u32 ?, u32 chmask)`.

### 2.1 Structure

```
.L93 @ entry: optional PA prologue (gPaControl[1]/[3] paths, `or` into MMIO)
.L93 cont. @ 0x24:
  a5 = BB base
  a3 = dtmFlag & 1
  a4 = a0 & 2                          ; PHY mode branch selector
  if (a4 != 0) → .L94 path (Coded / 2M / aux)
  else → .L94' fall-through (1M default)

Mode-A path (a0 & 2 != 0, .L94 NOT taken) @ 0x3e (this is the 2M/Coded path):
  BB[0] = (BB[0] & 0xfffd_cfff) | 0x2000     ; PHY mode bit 13 set
  BB[32] = 0x90086                            ; RFEND register
  BB[20] = 0x8301ff1                          ; RFEND
  BB[24] = 0x31619                            ; RFEND
  BB[40] = 0x328de                            ; RFEND
  BB[36] = 0x1006310                          ; RFEND constant
  if (dtmFlag & 1): BB[16] = 0x3fa4df         ; RFEND rf_calib_variant_a
  else:             BB[16] = 0x34a4df         ; RFEND rf_calib_variant_b

Mode-B path (a0 & 2 == 0, .L94 taken) @ 0xae (1M default):
  BB[32] = 0x90083                            ; (note: differs from Mode-A by 3)
  BB[20] = 0x8101901
  BB[24] = 0x31624
  BB[40] = 0x328be
  BB[36] = 0x1006310                          ; same constant
  if (dtmFlag & 1): BB[16] = 0x3fa2ce         ; rf_calib_variant_a
  else:             BB[16] = 0x3222d0         ; rf_calib_variant_b
  BB[0] = old & 0xfffd_cfff (no OR, just clear)
  ; mode == 1 (a0 == 1) → 4-shift, else 5-shift on channel arg

Common tail .L110 @ 0x90:
  gBleIPPara[0x10] = a1                       ; first config word (channel/freq derived)
  if (a2 != 0): a2 <<= 1; gBleIPPara[0x10] = a2  ; override with arg
  gBleIPPara[4] = 0                            ; clear IRQ flag byte (note: TX side sets to 0x80)
  ret
```

### 2.2 What's "verbatim must-preserve"

| Register | Value (1M default mode-B) | Value (Coded/2M mode-A) | Constant? |
|:---------|:--------------------------|:------------------------|:----------|
| BB[16]   | variant A `0x3fa_2ce` / variant B `0x3222d0` | variant A `0x3fa_4df` / variant B `0x34a_4df` | **`dtmFlag & 1`-dependent** (variant A when bit set, variant B when cleared) |
| BB[20]   | 0x8101901                  | 0x8301ff1                | per-PHY |
| BB[24]   | 0x31624                    | 0x31619                  | per-PHY |
| BB[32]   | 0x90083                    | 0x90086                  | per-PHY |
| BB[36]   | 0x1006310                  | 0x1006310                | **constant across PHY** |
| BB[40]   | 0x328be                    | 0x328de                  | per-PHY |

**Rust port mandate**: these values are **vendor-calibrated RFEND magic** (GF-1 no datasheet). They MUST be copied verbatim as PHY-mode lookup tables. Do NOT attempt to compute or derive them from spec values.

---

## 3. `phy_status_clear` — IRQ-mask-like save/restore + conditional shut

ABI: `void phy_status_clear(u8 expected_phy_status)`.

> ⚠️ **Reg-naming caution (Cindy `fbee9584` #2)** — the save/zero/restore target at `*(0x4002_420c)` is **provisionally** PFIC's IRQ-mask register but has NOT been corroborated against `ch32-data` PAC defs. It might be PFIC reg 524, an LLE-adjacent companion MMIO, or a peripheral-clock gate. Treat as `irq-mask-like MMIO at 0x4002_420c` everywhere in the Rust port until cross-checked. See §9 open items.

### 3.1 Structure

```
@ entry:
  s2 = *(0x4002_420c)                        ; save irq-mask-like MMIO (provisional: PFIC[524])
  *(0x4002_420c) = 0                         ; zero the mask (provisional: disable all configured IRQs)
  fence.i

@ 0x1a-0x28:
  a5 = (*gptrLLEReg) & 3                     ; check LLE base low 2 bits — "PHY active?"
  if (a5 == 0): goto .L154 (just restore + ret)

@ 0x2a-0x52:
  a4 = gBleIPPara[7]                         ; PHY active mode byte
  if (a4 == 3): rfConfig[16](17, 0, 0)       ; vtable call into RF subsystem to "freeze 3"

@ 0x54-...:
  s1 = expected_phy_status (saved input)
  ; complex per-status dispatch updating gBleIPPara[8] / [9] (current PHY state bytes)
  ; if expected_phy_status == 6: ...
  ; if expected_phy_status == 7: ...
  ; if expected_phy_status == 8: ... gBleIPPara[8] = 264 (sh, half-word)
  ; default: tmosSign = 0; ble_ll_hw_api_shut()

@ .L154 exit:
  *(0x4002_420c) = s2                        ; restore irq-mask-like MMIO (provisional: PFIC[524])
  ret
```

### 3.2 Interpretation

`phy_status_clear` is the **PHY transition arbiter** — called when LL needs to flip between RX/TX/idle states. It:
1. Saves and zeros the irq-mask-like MMIO at `0x4002_420c` (provisionally global IRQ mask = PFIC reg 524, unverified)
2. Checks whether LLE base reports "PHY active" (low 2 bits of `*LLE` non-zero)
3. If active, dispatches based on expected status byte vs current `gBleIPPara[7]/[8]/[9]` byte:
   - status 6: complex match (only update `gBleIPPara[9]` if `gBleIPPara[8] == expected`)
   - status 7: similar with stricter triple-check
   - status 8: hard-set `gBleIPPara[8..9] = 264` (half-word, 0x108)
   - other: clear `tmosSign`, call `ble_ll_hw_api_shut`
4. Restores the saved value to `*(0x4002_420c)`, returns

**Critical**: this function gates PHY transitions by save/zero/restore on the irq-mask-like MMIO at `0x4002_420c`. The semantics look like a global IRQ mask (likely PFIC reg 524 on CH32V208), but the address has not been corroborated against the `ch32-data` PAC yet — could also be an LLE/BB-adjacent companion register. Either way the **save → zero → do-work → restore** structure is verbatim-mandatory: Rust port wraps the equivalent in `critical_section::with(|cs| { ... })` AND mirrors the explicit `*(0x4002_420c)` save/zero/restore. Do not collapse the two layers — embassy's normal yielding is forbidden inside this window regardless of which interpretation is correct.

---

## 4. `ll_tx_wait_finish` — re-examined for mode-3 SCAN_RSP path

(Phase 1 §1 covered mode=0 KICK. Here we focus on mode=3 verified for Phase 2.5 SCAN_RSP DoD-3 PARTIAL.)

### 4.1 ABI re-derive

`int ll_tx_wait_finish(u32 mode, void *buf, u32 len)` — `a0 = mode`, `a1 = buf`, `a2 = len`.

Mode dispatch at 0xd0 (`bnez a0, .L79`):
- **mode == 0** (.L80 fall-through @ 0xd2): COLD TX KICK — call `BLE_SetPHYTxMode(buf, len)`, then `LLE[0] = 2`. (Phase 1 §1.)
- **mode != 0** → .L79 @ 0x142
  - **mode == 3** (.L79 fall-through @ 0x148): BB toggle — `BB[0] |= 0x800000` + `BB[44] &= ~3`, then drop to .L81 wait.
  - **mode == 1** (.L82 @ 0x164): RX PREP — LLE config updates + PHY-dependent LLE[100] window (406/1086/446 for 1M/Coded/2M).
  - **else** (anything not 0/1/3): drop directly to .L81 wait (no register writes).

### 4.2 mode=3 path verbatim (SCAN_RSP TX kick)

```
.L79 @ 0x142:
  a5 = 3
  bne a0, a5, .L82           ; if mode != 3, fall through to .L82

mode=3 fall-through @ 0x148:
  a5 = gptrBBReg pointer; a5 = *a5 = BB base
  a3 = 0x800000               ; lui 0x800 → 0x800000 (bit 23)
  a4 = BB[0]
  a4 |= a3                    ; SET BIT 23
  BB[0] = a4                  ; ← THE SCAN_RSP TX KICK

  a4 = BB[44]
  a4 &= ~3                    ; clear lower 2 bits
  BB[44] = a4                 ; reset some sub-state

  j .L81                       ; drop into wait loop (same as mode=0)
```

**Wait loop .L81** (three-source completion, same for all modes):
```
.L81 @ 0x11c:
  a3 = LLE base
  a4 = &gBleIPPara

.L83 @ 0x126 (loop):
  a5 = gBleIPPara[2] & 1; if non-zero, break (TX done flag)
  a5 = gBleIPPara[3] & 1; if non-zero, break (TX err flag)
  a5 = LLE[100]; if non-zero, continue (hw still busy)
  break (LLE[100] == 0 = idle/timeout)

exit:
  ret
```

### 4.3 mode=1 (RX prep) — PHY-dependent LLE[100] window

```
.L82 @ 0x164:
  bne a0, 1, .L81             ; if mode != 1, drop to wait

  a3 = LLE[12]
  LLE[12] = a3 & 0xfffe_dfff   ; clear bit 12
  fence.i
  a5 = LLE base (reload)
  LLE[8] = 0x2000              ; same strobe as TX
  gBleIPPara[4] = 0x80         ; IRQ enable
  
  a2 = BB[0]
  a4 = (a2 >> 12) & 3          ; extract PHY mode bits 12-13
  if a4 == 0: LLE[100] = 406   ; 1M PHY RX window
  if a4 == 2: LLE[100] = 1086  ; Coded PHY RX window
  else:        LLE[100] = 446   ; 2M PHY RX window

  LLE[12] = a3                  ; restore
  j .L81                        ; drop to wait
```

**These three RX window magic values (406 / 1086 / 446) must be preserved verbatim** — they're PHY-mode-specific RX timeout counters with no public datasheet.

---

## 5. Full SCAN_RSP TX trigger chain (corrected from Phase 2.5 PARTIAL)

```
ll_advertise_legacy_rx (SCAN_REQ accepted, .L429 path):
├─ ll_advertise_generated_scan_rsp(adv_state)        ; PDU prep + stamp byte_11=0x93 (NO HW write)
├─ BLE_SetPHYTxMode(mode_byte, length)               ; PHY/RF config
│  ; Writes: BB[0] PHY-mode bits, BB[44] (via .L70/.L73 lower bits), 
│  ;         LLE[8]=0x2000, LLE[12]=mode_cfg, LLE[100]=window, gBleIPPara[4]=0x80
│  ; Does NOT write LLE[0]=2 or BB[0] bit 23
├─ ll_tx_wait_finish(3, 0, 0)                        ; SCAN_RSP path
│  ├─ mode=3 fall-through: BB[0] |= 0x800000  ← THE ACTUAL SCAN_RSP TX KICK
│  ├─                       BB[44] &= ~3
│  └─ .L81 wait loop: poll gBleIPPara[2]/[3] / LLE[100] until completion
└─ ble_ll_hw_api_shut()                              ; HW shutdown after TX done
```

**Now fully resolved** — Phase 2.5 §0 DoD-3 PARTIAL flag can be closed.

---

## 6. Rust PHY preflight design rules (additions to §17-§24)

25. **Iron Rule §25 — Two TX kick mechanisms, not one**: ADV/SCAN_REQ broadcast = `LLE[0] = 2` (cold). SCAN_RSP turnaround = `BB[0] |= 0x800000` + `BB[44] &= ~3` (warm RX→TX). Rust port must implement BOTH as distinct primitives — `tx_kick_cold()` and `tx_kick_scanrsp()`. Iron Rule §0 (逐 register 全写) applies to both.

26. **Iron Rule §26 — `BLE_SetPHYTxMode` / `BLE_SetPHYRxMode` are pure config (no kick)**: All 4 TX paths and 2 RX paths write PHY mode bits to BB[0], BB[44], LLE[8/12/100], gBleIPPara[4], plus RFEND chain BB[16..40] (RX side only). NONE issue a TX kick. Rust port replicates them as `phy_tx_setup(mode, channel)` and `phy_rx_setup(mode, ?, chmask)` functions returning nothing — caller is responsible for the kick.

27. **Iron Rule §27 — RFEND magic constants are vendor calibration, NOT spec-derivable** (GF-1 no datasheet): BB[16] / BB[20] / BB[24] / BB[32] / BB[40] in `BLE_SetPHYRxMode` carry per-PHY-mode magic values. Rust port copies them verbatim from `BLE_SetPHYRxMode` decode (§2.2 table). DO NOT derive from BLE spec. BB[36] = 0x1006310 is constant across PHY modes — still copy verbatim.

28. **Iron Rule §28 — dtmFlag swaps BB[16] calibration**: `dtmFlag & 1 != 0` selects `rf_calib_variant_a` (BB[16] = `0x3fa_4df` for Coded/2M / `0x3fa_2ce` for 1M); `== 0` selects `rf_calib_variant_b` (BB[16] = `0x34a_4df` / `0x322_2d0`). Rust port exposes `rf_calib_variant: Variant` selector and switches BB[16] accordingly inside `phy_rx_setup`. **Naming intentionally neutral** until an EVT capture proves which variant is production air vs DTM test (per Cindy `fbee9584` #1).

29. **Iron Rule §29 — RX window magic 406 / 1086 / 446 are PHY-mode counters**: `ll_tx_wait_finish` mode=1 sets `LLE[100]` to one of these based on `BB[0] >> 12 & 3`. Rust port maps {0 → 406, 2 → 1086, default → 446}. Air-PASS verifies semantics; no derivation from spec.

30. **Iron Rule §30 — phy_status_clear save/zero/restore on irq-mask-like MMIO `0x4002_420c`**: function saves `*(0x4002_420c)` → zeros it → does PHY transition work → restores. The MMIO is **provisionally** PFIC IRQ-mask (reg 524) on CH32V208 but NOT yet corroborated against `ch32-data` PAC defs — could alternately be an LLE/BB companion mask (per Cindy `fbee9584` #2). Regardless of which interpretation, Rust port MUST (a) wrap the equivalent transition in `critical_section::with(|cs| { ... })`, AND (b) replicate the explicit save/zero/restore on the typed `0x4002_420c` reference. Cannot use embassy's normal yielding within this window. Reclassify the wrapper macro to a single typed PAC reg once cross-check lands.

31. **Iron Rule §31 — gBleIPPara[4] = 0x80 vs 0x00 is TX-vs-RX directionality**: TX-side PHY-set writes `gBleIPPara[4] = 0x80`. RX-side writes `gBleIPPara[4] = 0`. Rust port preserves this byte as `phy_direction: PhyDir` with `TX_ARMED = 0x80, RX_ARMED = 0`.

32. **Iron Rule §32 — LLE[8] = 0x2000 is always written, never read**: every PHY-set path writes 0x2000 to LLE[8]. This is a write-only strobe — Rust port treats it as a fixed config write with no readback validation.

---

## 7. Cross-reference

- Phase 0 `ll-advertise-tx-disasm.md` §3-§5 — TX register sequence; the ADV `LLE[0] = 2` cold kick (matches §0 priority 1 row 1)
- Phase 1 `ll-tx-completion-disasm.md` §1.5 — `ll_tx_wait_finish` mode=0 ABI (re-derived here in §4)
- Phase 2.5 `ll-rx-ingress-disasm.md` §0 DoD-3 — PARTIAL flag NOW CLOSED (BB[0] bit 23 is SCAN_RSP kick)
- backlog.md BL-2 — closed by this doc

---

## 8. Verification artifacts (re-runnable)

```bash
cd /tmp/wchble-disasm
riscv64-unknown-elf-objdump --disassemble=BLE_SetPHYTxMode    -r ip.o      # 158 lines
riscv64-unknown-elf-objdump --disassemble=BLE_SetPHYRxMode    -r ip.o      # 170 lines
riscv64-unknown-elf-objdump --disassemble=phy_status_clear    -r ip.o      # 213 lines
riscv64-unknown-elf-objdump --disassemble=ll_tx_wait_finish   -r ip.o      # 203 lines (Phase 1, mode=3 path re-examined here)

# Cross-verify BB[0] bit 23 = SCAN_RSP kick (Phase 0 path also writes BB[0] but only bit 19 = channel)
grep -n '800000' /tmp/wchble-disasm/ll_tx_wait_finish.dump   # should match only ll_tx_wait_finish (mode=3) + ll_advertise_tx (different bit pos)
```

---

## 9. Open items (post-Phase 3, post-air-PASS)

- **`BLE_SetCTEMode`** (CTE / Constant Tone Extension for AoA/AoD) — defer to Phase 5 if/when direction-finding needed
- **`BLE_PAControlInit` / `BLE_AccessAddressGenerate` / `BLE_RegInit` / `BLE_IPCoreInit`** — boot-time one-shot, decode when porting BLE init sequence (Phase B Rust skeleton)
- **`rfConfig[16]` vtable** — referenced from `phy_status_clear` for PHY mode==3 freeze. Need separate xref pass.
- **Irq-mask-like MMIO at `0x4002_420c` identification (Cindy `fbee9584` #2)** — `phy_status_clear` save/zeros/restores this address; provisionally PFIC reg 524 (IRQ mask) but unverified.
  - **Phase A2 update (2026-05-13)**: `ll_set_connect_event` in `ll_connect.o` writes `*(0xe000e204) = 0x2000` (canonical PFIC/NVIC base `0xe000e000`, offset `0x204` = IENR, bit 13 = IRQ #13 enable). This is **strong evidence that `0x4002_420c` is NOT PFIC** — real PFIC base on CH32V208 is `0xe000e000`, and `0x4002_xxxx` is the AHB peripheral region. The `0x4002_420c` MMIO is more likely an **LLE/BB-adjacent companion register** (LLE-side IRQ mask, PHY enable gate, or clock control). See Phase A2 doc §4.5 + Iron Rule §35 for cross-validation.
  - Remaining cross-check questions: (i) does `0x4002_4000` correspond to LLE register block in ch32-data PAC? (ii) is it a single mask reg or part of LLE control word? (iii) is the address referenced anywhere else in `libwchble.a` (search `bb.o` / `rfend.o` / `ll.o` for `0x4002_420c` writes/reads)? Until ch32-data corroborates, all docs/Iron Rules use "irq-mask-like MMIO at 0x4002_420c" wording.
- **`BB[0]` full bit map** — bits 12-13 = TX PHY mode, bits 12-13 of `>>12` = RX PHY mode (so same nibble), bit 14 = 2M, bit 19 = channel idx scaling (Phase 0), bit 23 = SCAN_RSP kick. Rust port maintains a single typed `BBControl` register with named fields.

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-phy-preflight-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
