# libwchble TMOS-LL disasm: IRQ-bit RX arm dispatch

Task: #71, unblock task #68 Phase C.

Scope: document the ROM/lib path from legacy advertising TX to the RX window used for SCAN_REQ / CONNECT_IND. This note changes no `src/` or `examples/` code.

## Artifacts and commands

Primary artifact:

```sh
riscv64-unknown-elf-objdump -dr /Users/mono/Elec/WCH/ch32v208_ble/libwchble.a > /tmp/libwchble_v208_dr.asm
```

Cross-check artifact:

```sh
/Users/mono/Elec/WCH/ch32ble/ch32v20x-ble/disasm/d.asm
/Users/mono/Elec/WCH/ch32ble/ch32v20x-ble/disasm/symbols.txt
```

Existing docs checked:

- `/Users/mono/Elec/elec-docs/WCHBLE.md`
- `/Users/mono/Elec/elec-docs/ble-reverse-docs/07-tmos-internals.md`
- `/Users/mono/Elec/elec-docs/ble-reverse-docs/09-ll-layer-complete.md`
- `/Users/mono/Elec/elec-docs/ble-reverse-docs/17-register-reference.md`
- `/Users/mono/Elec/elec-docs/ble-reverse-docs/25-advertising-scanning.md`
- `/Users/mono/Elec/WCH/ch32-hal/notes/ch32-rs/ble-registers.md`
- `/Users/mono/Elec/WCH/ch32-hal/notes/ch32-rs/tmos-embassy-task-migration.md`

Existing docs already identify the high-level path:

```text
ll_advertise_process
  -> ll_advertise_tx
  -> BLE_SetPHYRxMode
  -> ll_tx_wait_finish
  -> ble_ll_chkcrc
  -> ll_advertise_legacy_rx
```

The old docs stop at flow-level detail. This note adds the IRQ state and MMIO write sequence needed by task #68.

## Symbol map

| Symbol | Section / local offset | Size | Evidence | Confidence |
|---|---:|---:|---|---|
| `ll_advertise_legacy_rx` | `.text.ll_advertise_legacy_rx + 0x000` | `0x12c` | `symbols.txt:7129`; `/tmp/libwchble_v208_dr.asm:53243` | high |
| `ll_advertise_process` | `.text.ll_advertise_process + 0x000` | `>= 0x5c2` | `/tmp/libwchble_v208_dr.asm:53431` | high |
| `BB_IRQLibHandler` | `.highcode + 0x000` | `0x114` | `/tmp/libwchble_v208_dr.asm:69270`; `src/ble/bb.rs` Rust port | high |
| `LLE_IRQLibHandler` | `.highcode.LLE_IRQHandler + 0x002` | `0x038` | `/tmp/libwchble_v208_dr.asm:90539` | high |
| `LLE_IRQSubHandler` | `.highcode + 0x000` | `0x1f8` | `/tmp/libwchble_v208_dr.asm:93513` | high |
| `lle_irq_process` | `.highcode + 0x1f8` | `>= 0x13c` | `/tmp/libwchble_v208_dr.asm:93848` | high |
| `ll_tx_wait_finish` | `.highcode + 0x0c0` | `0x0fe` | `/tmp/libwchble_v208_dr.asm:91473`; `symbols.txt:11388` | high |
| `BLE_SetPHYRxMode` | `.text.BLE_SetPHYRxMode + 0x000` | `0x11a` | `/tmp/libwchble_v208_dr.asm:70141`; `symbols.txt:11389` | high |

Register names in this note use WCH lib symbol names plus current Rust aliases:

- `gptrBBReg` -> physical `0x40024100`, current Rust `BLE_BB`.
- `gptrLLEReg` -> physical `0x40024200`, current Rust `BLE_LLE`.
- `gptrRFENDReg` -> physical `0x40025000`, current Rust `BLE_RFEND`.

## IRQ -> RX arm dispatch table

| Stage | Trigger | ip state | BB/LLE writes | RX arm timing | Evidence | Confidence |
|---|---|---|---|---|---|---|
| BB IRQ `.L6` | `BB.statr bit6` with `gBleIPPara[4] bit6 clear` | `ip4: 0x80 -> 0xC0`; `ip5=1` | `BLE_LLE.access_addr(+0x08)=0x2000`; `BLE_LLE.timer(+0x64)=gBleIPPara[16..19]` | TX fire / advance marker, not CONNECT_IND RX arm | `BB_IRQLibHandler +0x8a..0xba` | high |
| BB IRQ `.L4` | `BB.statr bit4` | `ip4=1` | `BB.statr(+0x38)=0x10` W1C | cleanup / re-arm marker | `BB_IRQLibHandler +0xbc..0xd2` | high |
| BB IRQ `.L8` | `BB.statr bit7` | `ip4=1` | `BB.statr(+0x38)=0x80` W1C | cleanup / re-arm marker | `BB_IRQLibHandler +0xd4..0xea` | high |
| Legacy ADV RX setup | `ll_advertise_process` state `.L443` | clears `gBleIPPara[3]` before CRC check | `BLE_SetPHYRxMode(0, 37, 0)` then `ll_tx_wait_finish(1, 0, 37)` | official RX window setup happens in task process, driven by `ll_tx_wait_finish`, then CRC + legacy parser runs if `gBleIPPara[3] bit0` is set | `ll_advertise_process +0x66..0xae` | high |
| `ll_tx_wait_finish(1, ...)` | `a0 == 1` | writes `gBleIPPara[4]=0x80` | saves `BLE_LLE.irq_mask(+0x0c)`; clears bit13 in saved mask; `fence.i`; writes `BLE_LLE.access_addr(+0x08)=0x2000`; sets timer 406/1086/446 based on `BLE_BB.ctrl(+0x00)[13:12]`; restores irq mask | arms the RX wait window and waits for `gBleIPPara[2] bit0` or `gBleIPPara[3] bit0` or timer expiry | `ll_tx_wait_finish +0x164..0x1bc` | high |
| RX completion gate | `ll_tx_wait_finish` wait loop | reads `gBleIPPara[2]` and `gBleIPPara[3]` | no direct MMIO in loop except `BLE_LLE.timer(+0x64)` read | returns when either bit is set or timer reaches zero | `ll_tx_wait_finish +0x11c..0x13a` | high |
| LLE IRQ RX completion | `LLE.statr(+0x08) bit2` | ORs `gBleIPPara[3] bit0`; clears `gBleIPPara[5]` | W1C `BLE_LLE.access_addr(+0x08)=0x04`; state ACK via `BLE_LLE.settle(+0x1c)=2/93/97/101/105/107/108`; updates `BLE_BB.timing(+0x2c)` low bits | this is the lib-side setter that lets `ll_tx_wait_finish` observe legacy ADV RX completion | `LLE_IRQSubHandler +0x84..0x1c8`; `lle_irq_process +0x256..0x334` | high |
| LLE IRQ TX/RX auxiliary completion | `LLE.statr(+0x08) bit3` | ORs `gBleIPPara[2] bit0`; writes `gBleIPPara[4]=0x80` | W1C `BLE_LLE.access_addr(+0x08)=0x08` | secondary wait completion path used by `ll_tx_wait_finish` | `LLE_IRQSubHandler +0x3a..0x64`; `lle_irq_process +0x20c..0x23a` | high |

## LLE IRQ completion path addendum

Task #68 Phase C v1 mirrored `ll_tx_wait_finish(1)` in task context and waited for `gBleIPPara[2]` / `gBleIPPara[3]`. Both bytes stayed zero. The missing setter is in the LLE interrupt path, not in the BB interrupt path.

`LLE_IRQLibHandler` dispatches through two modes:

```asm
LLE_IRQLibHandler +0x0a:
  0a: lb   t1,0(t0)              ; tmosSign
  0e: sb   zero,0(t0)
  12: beqz t1,.L1
  28: jal  lle_irq_process       ; scheduler mode
  32: jal  LLE_IRQSubHandler     ; normal IRQ mode
```

`LLE_IRQSubHandler` and `lle_irq_process` share the same relevant logic. The normal path is shown here.

`LLE.statr(+0x08) bit3` sets `gBleIPPara[2] bit0` and acknowledges bit3:

```asm
LLE_IRQSubHandler +0x3a:
  3a: lw   a4,8(a5)              ; BLE_LLE+0x08 status
  3c: srli a4,a4,0x3
  40: beqz a4,.L3
  42: lbu  a4,gBleIPPara+2
  4a: ori  a4,a4,1
  52: sb   a4,gBleIPPara+2
  5a: sb   -128,gBleIPPara+4     ; ip4 = 0x80
  62: li   a4,8
  64: sw   a4,8(a5)              ; W1C bit3
```

`LLE.statr(+0x08) bit2` sets `gBleIPPara[3] bit0`, then acknowledges bit2:

```asm
LLE_IRQSubHandler +0x94:
  94: sb   zero,gBleIPPara+5
  9c: lw   a4,8(a5)              ; BLE_LLE+0x08 status
  ae: srli a4,a4,0x2
  b6: beqz a4,.L7
  ...
LLE_IRQSubHandler +0x1b6:
  1b6: lbu  a4,gBleIPPara+3
  1ba: ori  a4,a4,1
  1c2: sb   a4,gBleIPPara+3
  1c6: li   a4,2
  1c8: sw   a4,8(a5)             ; W1C bit2
```

The bit2 branch also writes `BLE_LLE.settle(+0x1c)` and `BLE_BB.timing(+0x2c)`:

```asm
LLE_IRQSubHandler +0xba:
  ba: lw   a4,0(a3)              ; BLE_BB+0x00
  bc: srli a4,a4,0xc
  c0: bnez a4,.L8                ; non-idle -> settle 108
  c2: lw   a2,80(a5)             ; BLE_LLE+0x50
  c4: lw   a4,36(a1)             ; gBleIPPara[36] pointer
  ce: beqz a2,.L9
  ...
LLE_IRQSubHandler +0xd2:
  d2: li   a4,93                 ; settle value for common RX path
  d6: sw   a4,28(a5)             ; BLE_LLE+0x1c
  d8: lbu  a4,gBleIPPara
  e0: ori  a4,a4,1
  e4: sb   a4,gBleIPPara
  e8: li   a4,4
  ea: sw   a4,8(a5)              ; W1C bit2 or bit1 group
  ec: lw   a4,44(a3)             ; BLE_BB+0x2c
  ee: and  a4,a4,-4
  f0: ori  a4,a4,1
  f4: sw   a4,44(a3)
```

`lle_irq_process` has the same setters at `+0x20c..0x23a` for bit3 and `+0x256..0x334` for bit2, with an extra snapshot store of `BLE_LLE+0x08` into `gBleIPPara[0x18..0x1b]`.

## Six task questions

### Q1. TX-done IRQ bit for `ip4_post == 1`

Both `BB.statr bit4` and `BB.statr bit7` write `gBleIPPara[4]=1`.

Evidence:

```asm
BB_IRQLibHandler +0xbc:
  be: lw a5,56(a4)        ; BB.statr
  c0: srli a5,a5,0x4      ; bit4
  c6: li a5,16
  c8: sw a5,56(a4)        ; W1C bit4
  cc: li a5,1
  d0: sb a5,gBleIPPara+4

BB_IRQLibHandler +0xd4:
  d4: lw a5,56(a4)        ; BB.statr
  d6: srli a5,a5,0x7      ; bit7
  dc: li a5,128
  e0: sw a5,56(a4)        ; W1C bit7
  e4: li a5,1
  e8: sb a5,gBleIPPara+4
```

Answer: `ip4_post == 1` is a shared cleanup marker for bit4 and bit7. The lib does not select RX arm from bit4 versus bit7. The RX wait path uses `gBleIPPara[2] bit0`, `gBleIPPara[3] bit0`, and the `BLE_LLE.timer(+0x64)` countdown.

Confidence: high.

### Q2. TX teardown sequence before RX arm

The teardown sequence is split between `BLE_SetPHYRxMode(0, 37, 0)` and `ll_tx_wait_finish(1, 0, 37)`.

`ll_advertise_process` legacy path:

```asm
ll_advertise_process +0x66:
  66: li a2,0
  68: li a1,37
  6c: li a0,0
  6e: call BLE_SetPHYRxMode
  76: li a2,37
  7a: li a1,0
  7c: li a0,1
  7e: call ll_tx_wait_finish
  86: lbu a5,gBleIPPara+3
  8a: andi a5,1
  9e: call ble_ll_chkcrc
  aa: call ll_advertise_legacy_rx
```

`ll_tx_wait_finish(1)`:

```asm
ll_tx_wait_finish +0x16a:
  16a: lw a3,12(gptrLLEReg)        ; save BLE_LLE+0x0c irq mask
  170: and a5,a3,0xffffdfff        ; clear bit13 in mask
  172: sw a5,12(gptrLLEReg)
  174: fence.i
  178: lw a5,0(gptrLLEReg)
  17c: sw 0x2000,8(a5)             ; BLE_LLE+0x08 command/status
  182: sb -128,gBleIPPara+4        ; ip4 = 0x80
  194: lw a4,0(gptrBBReg)          ; read BLE_BB+0x00 mode bits
  19c/1b2/1b8: select timer 406/1086/446
  1a0: sw timer,100(gptrLLEReg)    ; BLE_LLE+0x64
  1a2: sw a3,12(gptrLLEReg)        ; restore irq mask
```

Answer: the lib saves/restores `BLE_LLE+0x0c`, writes `BLE_LLE+0x08=0x2000`, sets `gBleIPPara[4]=0x80`, sets `BLE_LLE+0x64` to the RX window value, and then waits. It does not perform an explicit `BB+0x00` GO write in this function. The RX start is driven by the LLE command/timer path.

Confidence: high for the listed writes, medium for the hardware-internal meaning of `BLE_LLE+0x08=0x2000`.

### Q3. RX arm exact writes

`BLE_SetPHYRxMode(0, 37, 0)` writes the RX PHY register block and sets `BLE_LLE.ctrl(+0x00) bit12`.

Relevant RX branch:

```asm
BLE_SetPHYRxMode +0xae:
  ae: li a4,0x90083
  b6: sw a4,32(a5)          ; BLE_LLE+0x20
  b8..c0: sw 0x08101901,20(a5) ; +0x14
  c2..ca: sw 0x00031624,24(a5) ; +0x18
  cc..d2: sw 0x000028be,40(a5) ; +0x28
  d4..dc: sw 0x01006310,36(a5) ; +0x24
  e0..10e: choose +0x10 value
  e8: lw a4,0(a5)           ; ctrl
  f6: and a4,~0x3000        ; clear bits[13:12]
  110: or a4,0x1000         ; for a0 != 1
  114: sw a4,0(a5)          ; set bit12
```

It computes a timing word from `a1=37`, then stores it to an internal global at `+0x90` in the disassembly section. The direct MMIO sequence above is the important task #68 part.

Answer: the exact direct MMIO arm writes are `BLE_LLE+0x20/0x14/0x18/0x28/0x24/0x10`, then `BLE_LLE+0x00 = (ctrl & !0x3000) | 0x1000`. The function clears and sets `bits[13:12]`; it does not clear `bits[8:7]`.

Confidence: high for direct MMIO. Medium for the internal timing global because relocation target is section-local.

### Q4. Call depth: direct ISR call or state jump

`ll_advertise_legacy_rx` is called from `ll_advertise_process` after `ll_tx_wait_finish` returns and `ble_ll_chkcrc` succeeds. It is not called from `BB_IRQLibHandler`.

Evidence:

```asm
ll_advertise_process +0x86:
  86: lbu a5,gBleIPPara+3
  8a: and a5,1
  8c: beqz a5,.L542        ; shut if no RX flag
  96: sb zero,gBleIPPara+3
  9e: call ble_ll_chkcrc
  a6: bnez a0,.L454
  aa: call ll_advertise_legacy_rx
```

Answer: call depth is `LLE IRQ updates gBleIPPara flags` -> TMOS schedules/runs `ll_advertise_process` -> `ll_tx_wait_finish` waits on flags/timer -> `ll_advertise_legacy_rx` parses the received PDU. The relevant setters are `LLE.statr bit2 -> gBleIPPara[3] bit0` and `LLE.statr bit3 -> gBleIPPara[2] bit0`.

Confidence: high.

### Q5. TX teardown completion criterion

The lib uses multiple markers:

- `BB_IRQLibHandler .L6`: `ip4=0xC0` marks TX fire / PHY advance.
- `BB_IRQLibHandler .L4/.L8`: `ip4=1` marks BB status bit4 or bit7 cleanup.
- `ll_tx_wait_finish`: returns when `gBleIPPara[2] bit0`, `gBleIPPara[3] bit0`, or `BLE_LLE.timer(+0x64)==0`.
- `ll_advertise_process`: accepts RX work only when `gBleIPPara[3] bit0` is set and CRC passes.

Evidence:

```asm
ll_tx_wait_finish +0x126:
  126: lbu a5,gBleIPPara+2
  12a: and a5,1
  12c: bnez a5,.L78
  12e: lbu a5,gBleIPPara+3
  132: and a5,1
  134: bnez a5,.L78
  136: lw a5,100(gptrLLEReg) ; BLE_LLE+0x64
  138: bnez a5,.L83
```

Answer: `ip4=1` is a cleanup/re-arm marker. The RX receive completion criterion for legacy ADV is `gBleIPPara[3] bit0` plus CRC success. `gBleIPPara[3] bit0` is set by the LLE IRQ bit2 path.

Confidence: high.

### Q6. RX arm pre-state dependency

Known dependencies from disasm:

1. `BLE_SetPHYRxMode(0, 37, 0)` is called before `ll_tx_wait_finish(1, 0, 37)`.
2. `BLE_SetPHYRxMode` sets `bits[13:12]=01` and leaves `bits[8:7]` as found.
3. `ll_tx_wait_finish(1)` saves/restores `BLE_LLE+0x0c`, writes `BLE_LLE+0x08=0x2000`, sets `gBleIPPara[4]=0x80`, and sets `BLE_LLE+0x64`.
4. Timer selection reads `BLE_BB+0x00 bits[13:12]`:
   - `00` -> 406
   - `10` -> 1086
   - other non-zero -> 446

Evidence:

```asm
ll_tx_wait_finish +0x194:
  194: lw a4,0(gptrBBReg)
  196: srli a4,0xc
  198: andi a4,3
  19a: bnez a4,.L84
  19c: li a4,406
  ...
  1aa: andi a4,3
  1ae: bne a4,2,.L86
  1b2: li a4,1086
  ...
  1b8: li a4,446
```

Answer: RX arm depends on prior RX PHY mode setup, `BLE_LLE+0x0c` mask preservation, `gBleIPPara[4]=0x80`, `BLE_LLE+0x08=0x2000`, and a `BLE_LLE+0x64` watchdog value selected from `BLE_BB+0x00 bits[13:12]`. The disasm does not show explicit `BLE_RFEND` writes in this path; RFEND state is inherited from the surrounding PHY setup.

Confidence: high for listed state, medium for RFEND inheritance.

## Historical doc review

| Doc | Existing claim | Disasm result | Action |
|---|---|---|---|
| `09-ll-layer-complete.md` | `ll_advertise_process -> ll_advertise_tx -> ll_advertise_legacy_rx` | Correct at high-level. The process path includes `BLE_SetPHYRxMode` and `ll_tx_wait_finish` before parser entry. | Keep, add this note for detail. |
| `25-advertising-scanning.md` | `ll_advertise_legacy_rx` is documented as done, with address index `41325`. | Function exists and parses SCAN_REQ / CONNECT_IND after RX completion. The old doc lacks IRQ dispatch detail. | Keep high-level struct, use this note for Phase C. |
| `17-register-reference.md` | BB/LLE base and IRQ status tables describe bit4/5/6/7. | `BB_IRQLibHandler` confirms bit6 -> `.L6`, bit4/bit7 -> `ip4=1`. | Keep. |
| `ble-registers.md` | `BLE_LLE+0x08=0x8000` pre-arm and `+0x08=0x2000` advance are critical. | `BB_IRQLibHandler` and `ll_tx_wait_finish` confirm both writes appear in the lib. | Keep. |

## Phase C implications

1. Phase C should treat `ip4=1` as a cleanup/re-arm marker, not as the RX-arm decision point.
2. Phase C v1 mirrored `ll_tx_wait_finish` in task context, while the Rust example kept IRQ64 masked and its `LLE()` vector as a counter-only stub. The wait flags therefore had no active setter.
3. The closest lib-faithful one-shot path has two required halves:
   - TX/RX wait setup: `BLE_SetPHYRxMode(0, channel, 0)` plus the `ll_tx_wait_finish(1, 0, channel)` command/timer sequence.
   - RX completion setter: an LLE IRQ bit2/bit3 handler that W1Cs `BLE_LLE+0x08` and signals a Rust-owned completion path.
4. A direct BB ISR write at bit4/bit7 time explains the `state=108 -> 108` regression: that point is downstream cleanup, while the lib RX wait window is armed through `ll_tx_wait_finish` and completed by LLE IRQ bits.
5. The `886e8b2` fix v4 mask cleared `bits[8:7]`, set `bits[13:12]=01`, and rewrote the channel field. The disasm confirms `BLE_SetPHYRxMode` sets `bits[13:12]`; it does not confirm a lib-side clear of `bits[8:7]` in that function. Clearing `bits[8:7]` inside the BB ISR remains an experimental deviation from the lib path.
6. `b7b4408` task-side handoff staying ADV-visible matches the disasm: task context is the correct orchestration layer, and LLE IRQ completion is the missing signal source.

## Evidence table

| Claim | Source | Tier | Confidence |
|---|---|---|---|
| `ll_advertise_process` calls `BLE_SetPHYRxMode(0,37,0)` then `ll_tx_wait_finish(1,0,37)` in the legacy RX branch. | `/tmp/libwchble_v208_dr.asm:53466-53483` | objdump | high |
| `ll_advertise_process` gates legacy parser on `gBleIPPara[3] bit0` and CRC success. | `/tmp/libwchble_v208_dr.asm:53484-53508` | objdump | high |
| `ll_advertise_legacy_rx` parses PDU type 3 / 5 after AdvA match and filter pass. | `/tmp/libwchble_v208_dr.asm:53245-53354` | objdump | high |
| `BB_IRQLibHandler` bit6 sets `ip4=0xC0`, writes `BLE_LLE+0x08=0x2000`, and programs `BLE_LLE+0x64`. | `/tmp/libwchble_v208_dr.asm:69270-69340`; `src/ble/bb.rs` | objdump + Rust port | high |
| `BB_IRQLibHandler` bit4 and bit7 both set `ip4=1`. | `/tmp/libwchble_v208_dr.asm:69341-69365`; `src/ble/bb.rs` | objdump + Rust port | high |
| `LLE_IRQSubHandler` bit3 sets `gBleIPPara[2] bit0`, sets `ip4=0x80`, and W1Cs `BLE_LLE+0x08` bit3. | `/tmp/libwchble_v208_dr.asm:93551-93564` | objdump | high |
| `LLE_IRQSubHandler` bit2 sets `gBleIPPara[3] bit0` and W1Cs `BLE_LLE+0x08` bit2. | `/tmp/libwchble_v208_dr.asm:93597-93816` | objdump | high |
| `lle_irq_process` duplicates the bit3 and bit2 completion setters and snapshots `BLE_LLE+0x08` into `gBleIPPara[0x18..0x1b]`. | `/tmp/libwchble_v208_dr.asm:93848-94006` | objdump | high |
| `ll_tx_wait_finish(1)` saves/restores `BLE_LLE+0x0c`, writes `BLE_LLE+0x08=0x2000`, sets `ip4=0x80`, and selects timer 406/1086/446. | `/tmp/libwchble_v208_dr.asm:91518-91636` | objdump | high |
| `BLE_SetPHYRxMode(0,...)` writes the RX PHY register block and sets `BLE_LLE+0x00 bit12`. | `/tmp/libwchble_v208_dr.asm:70141-70260` | objdump | high |
| Existing early docs have correct high-level function order and incomplete IRQ dispatch detail. | `09-ll-layer-complete.md`, `25-advertising-scanning.md`, this objdump | doc + objdump | high |

## Minimal reopen contract for task #68 Phase C

The next implementation should use `441e7b1` as baseline and treat the sequence above as the Phase C reference.

Implementation contract:

- Keep the existing BB IRQ TX path order intact.
- Add a minimal LLE IRQ completion path for task #68 Phase C, preferably Rust-owned completion state (`TaskEvents` / `Signal` / `Mutex<Cell>`), so task code can wait without writing `gBleIPPara`.
- Handle at least `BLE_LLE+0x08` bit2 and bit3:
  - bit2: W1C bit2 and report RX completion for legacy ADV (`gBleIPPara[3]` equivalent).
  - bit3: W1C bit3 and report auxiliary completion (`gBleIPPara[2]` equivalent).
- Keep `BLE_LLE+0x00 bits[8:7]` unchanged by the completion handler.
- Diagnostic output should include `BLE_LLE+0x08`, `BLE_LLE+0x64`, `BLE_BB+0x00`, `BLE_BB+0x38`, and DMA buffer first 40 bytes.

Review gate: ADV still visible, and one of the three DoD states from Phase C applies: parsed CONNECT_IND, classified snapshot, or regression with a bad branch.

## Addendum: Full BB_IRQLibHandler + LLE_IRQSubHandler RX completion path

Added 2026-05-10 as task #68 Phase C v3 prerequisite. Source: `/tmp/libwchble_v208_dr.asm`.

### BB_IRQLibHandler — complete 0x114-byte structure

The handler handles exactly 4 paths. There is **no RX completion handler**:

| Label | Offset | Trigger | Action |
|---|---|---|---|
| entry | +0x00..+0x1a | — | Load gptrBBReg; read BB+0x38; check bit6 |
| bit6 preamble | +0x1c..+0x5a | bit6 SET | W1C 0x60; if ip0.bit6: call fnGetClockCBs→ip0x1c, clear ip0.bit6 |
| .L5 | +0x5c..+0x88 | ip0.bit5 SET | clear ip0.bit5; write LLE+0x08=0x8000; LLE+0x6c=ip20<<1 (scan pre-arm) |
| .L6 | +0x8a..+0xba | ip4.bit6 CLEAR | gBleIPPara[5]=1; LLE+0x08=0x2000; gBleIPPara[4]=0xC0; LLE+0x64=ip16 |
| .L4 | +0xbc..+0xd2 | statr bit4 | W1C 0x10; gBleIPPara[4]=1 |
| .L8 | +0xd4..+0xea | statr bit7 | W1C 0x80; gBleIPPara[4]=1 |
| .L9 | +0xec..+0x108 | AES.statr bit1 | clear bit1, clear bit0 |
| .L2 | +0x10a..+0x112 | — | epilogue / ret |

**Confirmed: BB_IRQLibHandler has NO path that sets gBleIPPara[2] or gBleIPPara[3].**

### LLE_IRQSubHandler — RX completion flag setters

`LLE_IRQSubHandler` is called from `LLE_IRQLibHandler` (LLE IRQ, IRQ64). It dispatches on `LLE+0x08` status bits:

| LLE+0x08 bit | Offset | Action |
|---|---|---|
| bit3 (RX data done) | +0x3a..+0x64 | **`gBleIPPara[2] \|= 1`**; gBleIPPara[4]=0x80; W1C LLE+0x08=8 |
| bit0 (timeout/other) | +0x66..+0x82 | gBleIPPara[1] \|= 1; W1C LLE+0x08=1 |
| bit2 + bit1 | +0x84..+0x92 | routes to .L5 (connection PDU path) |
| — | +0x1b6..+0x1c8 (.L7) | **`gBleIPPara[3] \|= 1`**; W1C LLE+0x08=2 |

Key asm evidence (gBleIPPara[2] bit0):
```asm
LLE_IRQSubHandler +0x3a:
  3a: lw a4, 8(a5)       ; read LLE+0x08
  3c: srl a4, a4, 3      ; shift right 3
  3e: and a4, a4, 1      ; check bit3
  40: beqz a4, .L3       ; skip if bit3 CLEAR
  42: lbu a4, gBleIPPara+2  ; read gBleIPPara[2]
  4a: or a4, a4, 1          ; set bit0
  52: sb a4, gBleIPPara+2   ; write back → gBleIPPara[2] |= 1
  56: li a4, -128 (=0x80)
  5e: sb a4, gBleIPPara+4   ; gBleIPPara[4] = 0x80
  62: li a4, 8
  64: sw a4, 8(a5)           ; LLE+0x08 = 8 (W1C bit3)
```

### Root cause of Phase C v1/v2 failures

IRQ64 (LLE interrupt) is NVIC-disabled in the current Rust example. Therefore:
1. `LLE_IRQSubHandler` never runs
2. `gBleIPPara[2/3]` are never set by LLE bit3 completion event
3. `ll_tx_wait_finish` polling loop exits only via `BLE_LLE+0x64` timer expiry
4. DMA buffer is empty at timeout (0x00 0x00 header)

### Phase C v3 fix path (no LLE IRQ needed)

**Direct polling approach**: Instead of relying on gBleIPPara[2/3], poll `LLE+0x08 bit3` directly in task context. `LLE+0x08 bit3` is a W1C status bit set by hardware when RX data is ready.

```rust
// After BLE_LLE+0x08=0x2000 advance + timer setup:
// Poll LLE+0x08 bit3 with bounded timeout (~600µs)
let deadline = Instant::now() + Duration::from_micros(600);
loop {
    let lle08 = lle_read(0x08);
    if lle08 & (1 << 3) != 0 {
        lle_write(0x08, 8);  // W1C bit3
        // read DMA buffer
        break;
    }
    if Instant::now() >= deadline { break; }  // timeout
}
```

This avoids enabling IRQ64 or implementing LLE_IRQSubHandler. No ISR changes required.

Confidence: high (disasm evidence for bit3 → gBleIPPara[2] write path is unambiguous).
