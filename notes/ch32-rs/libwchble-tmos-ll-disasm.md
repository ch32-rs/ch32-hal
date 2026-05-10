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

Answer: call depth is `BB/LLE IRQ updates gBleIPPara flags` -> TMOS schedules/runs `ll_advertise_process` -> `ll_tx_wait_finish` waits on flags/timer -> `ll_advertise_legacy_rx` parses the received PDU.

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

Answer: `ip4=1` is a cleanup/re-arm marker. The RX receive completion criterion for legacy ADV is `gBleIPPara[3] bit0` plus CRC success.

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
2. The closest lib-faithful one-shot path is:
   - call or mirror `BLE_SetPHYRxMode(0, channel, 0)` after the ADV TX PDU is committed,
   - mirror `ll_tx_wait_finish(1, 0, channel)` setup: save `BLE_LLE+0x0c`, clear bit13 in the mask, write `BLE_LLE+0x08=0x2000`, set `gBleIPPara[4]=0x80`, program `BLE_LLE+0x64`, restore `BLE_LLE+0x0c`,
   - wait on a bounded condition equivalent to `gBleIPPara[3] bit0` or timer expiry,
   - parse `CONNECT_IND` from the DMA buffer in task context.
3. A direct ISR write at bit4/bit7 time explains the `state=108 -> 108` regression: that point is downstream cleanup, while the lib RX wait window is armed through `ll_tx_wait_finish`.
4. The `886e8b2` fix v4 mask cleared `bits[8:7]`, set `bits[13:12]=01`, and rewrote the channel field. The disasm confirms `BLE_SetPHYRxMode` sets `bits[13:12]`; it does not confirm a lib-side clear of `bits[8:7]` in that function. Clearing `bits[8:7]` inside the BB ISR remains an experimental deviation from the lib path.
5. `b7b4408` task-side handoff staying ADV-visible matches the disasm: task context is the correct layer, but the handoff must mirror the `ll_tx_wait_finish` state machine instead of waiting for `adv_tx_burst` to return through the diagnostic path.

## Evidence table

| Claim | Source | Tier | Confidence |
|---|---|---|---|
| `ll_advertise_process` calls `BLE_SetPHYRxMode(0,37,0)` then `ll_tx_wait_finish(1,0,37)` in the legacy RX branch. | `/tmp/libwchble_v208_dr.asm:53466-53483` | objdump | high |
| `ll_advertise_process` gates legacy parser on `gBleIPPara[3] bit0` and CRC success. | `/tmp/libwchble_v208_dr.asm:53484-53508` | objdump | high |
| `ll_advertise_legacy_rx` parses PDU type 3 / 5 after AdvA match and filter pass. | `/tmp/libwchble_v208_dr.asm:53245-53354` | objdump | high |
| `BB_IRQLibHandler` bit6 sets `ip4=0xC0`, writes `BLE_LLE+0x08=0x2000`, and programs `BLE_LLE+0x64`. | `/tmp/libwchble_v208_dr.asm:69270-69340`; `src/ble/bb.rs` | objdump + Rust port | high |
| `BB_IRQLibHandler` bit4 and bit7 both set `ip4=1`. | `/tmp/libwchble_v208_dr.asm:69341-69365`; `src/ble/bb.rs` | objdump + Rust port | high |
| `ll_tx_wait_finish(1)` saves/restores `BLE_LLE+0x0c`, writes `BLE_LLE+0x08=0x2000`, sets `ip4=0x80`, and selects timer 406/1086/446. | `/tmp/libwchble_v208_dr.asm:91518-91636` | objdump | high |
| `BLE_SetPHYRxMode(0,...)` writes the RX PHY register block and sets `BLE_LLE+0x00 bit12`. | `/tmp/libwchble_v208_dr.asm:70141-70260` | objdump | high |
| Existing early docs have correct high-level function order and incomplete IRQ dispatch detail. | `09-ll-layer-complete.md`, `25-advertising-scanning.md`, this objdump | doc + objdump | high |

## Minimal reopen contract for task #68 Phase C

The next implementation should use `441e7b1` as baseline and treat the sequence above as the Phase C reference. The first code experiment should mirror `ll_tx_wait_finish(1,0,ch)` as a bounded diagnostic helper and print:

- `gBleIPPara[2]`, `gBleIPPara[3]`, `gBleIPPara[4]`
- `BLE_LLE+0x08`, `+0x0c`, `+0x64`
- `BLE_BB+0x00`, `+0x38`
- DMA buffer first 40 bytes

Review gate: ADV still visible, and one of the three DoD states from Phase C applies: parsed CONNECT_IND, classified snapshot, or regression with a bad branch.
