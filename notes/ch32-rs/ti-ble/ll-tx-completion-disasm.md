# `ll_tx_wait_finish` + `ll_advertise_process` + `ll_advertise_status_closed` + `ll_advertise_event_closed` 反汇编笔记

**Status**: 2026-05-13 00:xx — Lucy 写, Phase 1 of "周围必备 LL 函数分析计划" (Andelf msg `ee7b4a6b`, Cindy review `f06e3e5d`).
**Companion to**: `ll-advertise-tx-disasm.md` (sibling, covers TX initiation; this doc covers TX completion / next-event trigger / multi-path convergence / user-cb).
**Source**: `libwchble.a`, 提取自 `ip.o` (PHY + wait functions) 和 `ll_advertise.o` (LL adv state machine):
```
riscv64-unknown-elf-ar x libwchble.a ip.o
riscv64-unknown-elf-objdump -dr --disassemble=ll_tx_wait_finish        ip.o > ll_tx_wait_finish.dump        (203 lines)
riscv64-unknown-elf-objdump -dr --disassemble=ll_advertise_process     ll_advertise.o > ll_advertise_process.dump     (830 lines)
riscv64-unknown-elf-objdump -dr --disassemble=ll_advertise_status_closed ll_advertise.o > ll_advertise_status_closed.dump (101 lines)
riscv64-unknown-elf-objdump -dr --disassemble=ll_advertise_event_closed ll_advertise.o > ll_advertise_event_closed.dump (93 lines)
```

**⚠️ Reference-only** — Cindy 三档框架 bucket #1 (参考实现); 不进 ch32-hal::ble 长期 API. 见 `ti-wch-mapping.md` 顶部 ⚠️ STATUS.

---

## 0. TL;DR — Cindy 三问直接答案

| 问 | 答 |
|----|----|
| **TX done 的判定源** | `ll_tx_wait_finish.L83` 轮询 3 个条件 (任一满足即退出): `gBleIPPara[2] & 1` (IRQ/event flag), `gBleIPPara[3] & 1` (second flag), `LLE[100] == 0` (LLE-busy 清空). 这 3 个由硬件 IRQ handler 推动 — IRQ 不在 LL 这层. |
| **event_closed 的汇合路径** | `ll_advertise_status_closed(st)` 是**唯一汇合点**. 它根据 `st->byte_10 / byte_25 / byte_26` 三态决定: (a) tail-call `ll_advertise_event_closed` (event 完全结束, 触发 user cb) 或 (b) tail-call `tmos_set_event(gTmosPara[3], 2)` (没结束, 推进下一个 channel/sub-state). |
| **下一次 `ll_advertise_tx` 的触发点** | `ll_advertise_process.L537` (offset 0x2d8) 直接 `call ll_advertise_tx(st)`. `process` 是 TMOS task dispatcher (由 `TMOS_ProcessRegister` 注册), 在 task event 触发时被 TMOS scheduler 调用; `status_closed` 走 `tmos_set_event(…, 2)` 之后 TMOS scheduler 在下一个 SysTick 调度 `process`, `process` 跳到 .L537 重发. |

**进一步**: ll_tx_wait_finish 不是单纯"等" — mode=0 时它**直接 kick TX** (写 LLE[0]=2), 而 ll_advertise_tx 的 `.L310` slow-path 通过 tail-call 把 kick 交给 ll_tx_wait_finish. 函数名误导.

---

## 1. `ll_tx_wait_finish(mode, arg1, arg2)` — TX 完成 + 模式 dispatch

### 1.1 Signature + 实测 callers

```c
void ll_tx_wait_finish(uint32_t mode, uint32_t arg1, uint32_t arg2);
```

**实测 caller 清单** (`grep R_RISCV_CALL ll_tx_wait_finish full.dump`):

| Caller | 参数 | 含义 |
|--------|------|------|
| `ll_advertise_tx` (offset 0x574) tail-call | `(0, pdu_buf, pdu_len)` | mode=0: kick TX + wait, "slow path" |
| `ll_advertise_process .L443` (offset 0x7e) | `(1, 0, 37)` | mode=1: 等 TX 完成后 RX-prep, channel 37 |
| `ll_advertise_process .L441` (offset 0x1c4) | `(1, 0, 0)` | mode=1: 等 TX 完成后 RX-prep (无 channel arg) |
| `ll_advertise_process .L442` (offset 0x22c) | `(1, 0, 0)` | mode=1: 同上 |
| `ll_advertise_aux_chain_tx` (其他文件 grep) | `(?, ?, ?)` | 未深挖 |
| `ll_initiate.o`, `ll_scan.o` 等 | 多个 | 这是通用 TX 完成函数, 不只 ADV 用 |

**关键发现**:
- mode=0 = TX initiate (写 LLE[0]=2) + wait — ll_advertise_tx slow-path 委托给这里
- mode=1 = pure-wait + RX preparation — ll_advertise_process 在调度 RX 前用
- mode=3 = BB toggle (TX commit) + wait — 未见 ADV 路径直接调用; 可能在 ll_connect 路径
- 其他 mode 值 (2/4/5+) = fallthrough 到 `.L81` 纯等待

### 1.2 Mode dispatch (offsets 0xc0-0xd2)

```asm
c0:  add  sp, sp, -16
c2:  sw   s0, 8(sp)
c4:  sw   ra, 12(sp)
c6:  auipc s0, %hi(gptrLLEReg); addi s0, s0, %lo(gptrLLEReg)  ; s0 = &gptrLLEReg
ce:  lw   a4, 0(s0)                                          ; a4 = gptrLLEReg (saved)
d0:  bnez a0, 142 <.L79>                                     ; if mode != 0: .L79
d2:  mv   a5, a1                                              ; a5 = arg1 (buf)
```

### 1.3 mode=0 路径 (ll_advertise_tx tail-call 入口) — `.L80`

```asm
d4: <.L80>:  while ((*a4)[100] != 0) {}              ; busy-wait LLE[100] (TX-busy?)
d8:          a0 = a5 (= arg1 = buf, but discarded by callee?)
da:          a1 = a2 (= arg2 = len)
dc-e0:       gBleIPPara[2] = 0                       ; clear "TX done flag" pre-shot
e4:          call BLE_SetPHYTxMode                    ; PHY 进入 TX 模式
ec-fe:       a5 = *gptrBBReg
             (*a5)[0]  |= 0x800000                   ; BB[0] bit 23 set (TX commit / FIFO?)
100:         (*a5)[44] &= ~3                         ; BB[44] mode field clear
106-114:     gBleIPPara[5] = 0; gBleIPPara[1] = 0    ; clear other state flags
116:         a5 = *gptrLLEReg
118-11a:     (*a5)[0] = 2                            ; *** LLE[0] = 2 -- TX KICK ***
             fallthrough to .L81 wait loop
```

**关键**: 这里是 `LL_advertise_tx slow path` 真正写 `LLE[0] = 2` 的地方. `ll_advertise_tx .L311` fast-path 自己写; `.L310` slow-path 通过 tail-call 委托给这里.

### 1.4 mode=1 路径 — `.L82` (RX prep + wait)

```asm
142: <.L79>: li a5, 3
144:         bne a0, a5, 164 <.L82>                  ; mode != 3 → .L82
              // mode=3 branch:
148-160:     BB[0] |= 0x800000; BB[44] &= ~3
162:         j .L81  (wait)

164: <.L82>: li a5, 1
166:         bne a0, a5, 11c <.L81>                  ; mode != 1 → pure wait (.L81)
              // mode=1 branch (RX preparation):
16a:         a3 = LLE[12]
16c-172:     LLE[12] &= ~0x2000                      ; clear bit 13 in LLE[12]
174:         fence.i                                  ; instruction barrier
178:         a5 = *gptrLLEReg
17a-17c:     LLE[8] = 0x2000                         ; LLE[8] = 0x2000 (RX kick?)
182-186:     gBleIPPara[4] = 0x80
18a-192:     a2 = *gptrBBReg
194-198:     a4 = (BB[0] >> 12) & 3                  ; sample BB[0] bits 12-13
19a:         if a4 != 0 goto .L84
              // BB[0][12:13] == 0:
19c:         a4 = 406                                ; (0x196)
              fallthrough to .L91

1a0: <.L91>: (*gptrLLEReg)[100] = a4                 ; LLE[100] = 406/1086/446
1a2:         (*gptrLLEReg)[12]  = a3                 ; restore LLE[12] (already masked)
1a4:         j .L81  (wait)

1a6: <.L84>: a4 = (BB[0] >> 12) & 3
1ae:         if (a4 != 2) goto .L86
              // BB[0][12:13] == 2:
1b2:         a4 = 1086 (0x43e); j .L91

1b8: <.L86>: a4 = 446 (0x1be); j .L91                ; BB[0][12:13] == 3
```

**LLE[100] 写值 = RX listen window**:
- BB[0] bits 12-13 = 0 → LLE[100] = 406  (推测 1M PHY)
- BB[0] bits 12-13 = 2 → LLE[100] = 1086 (推测 Coded PHY, 长 RX 窗)
- BB[0] bits 12-13 = 3 → LLE[100] = 446  (推测 2M PHY)
- BB[0] bits 12-13 = 1 → 未处理? (fallthrough? bug? 还是 1 = 不可能值)

**这是关键发现**: BB[0] bits 12-13 = PHY 模式选择 (推测 1M=0, 2M=3, Coded=2). LLE[100] = RX 窗口 µs 数 (推测).

### 1.5 Wait loop — `.L81` / `.L83` (offsets 0x11c-0x13e) — **TX done 判定源**

```asm
11c: <.L81>: a3 = *s0 = *gptrLLEReg                  ; re-load (PHY/BB 写后 invalidated)
11e:         a4 = &gBleIPPara
122:         ...
126: <.L83>: a5 = gBleIPPara[2]                      ; <-- Condition A
12a:         if (a5 & 1) goto .L78                   ; bit 0 → exit
12e:         a5 = gBleIPPara[3]                      ; <-- Condition B
132:         if (a5 & 1) goto .L78                   ; bit 0 → exit
136:         a5 = (*a3)[100] = LLE[100]              ; <-- Condition C
138:         if (a5 != 0) goto .L83                  ; LLE-busy → keep waiting
             ; a5 == 0: fall through to .L78
13a: <.L78>: lw ra, 12(sp); lw s0, 8(sp); addi sp, sp, 16; ret
```

**TX done 判定源**: 函数退出条件 = **任一为真**:
1. `gBleIPPara[2] & 1`  — 由 IRQ handler 置位 (推测: TX-IRQ / RX-IRQ done)
2. `gBleIPPara[3] & 1`  — 同上, 不同 IRQ vector
3. `LLE[100] == 0`      — LLE-busy 自然清零 (硬件 TX/RX 完成自清)

**Rust 设计含义**:
- TX 完成检测是**轮询 polling**, 不是 IRQ-driven 直接唤醒任务
- IRQ handler 把 `gBleIPPara[2]/[3]` 的 bit 0 置位, ll_tx_wait_finish 在主循环 busy-wait 看到就退
- LLE[100] = 0 是硬件 fallback, 用于 IRQ 丢/miss 时仍能退出
- Rust 第一版可以**用同样的 polling**, 后续优化用 IRQ + `wfi` 节能

### 1.6 Caller / Callee ABI 表 (Cindy adjustment #1)

| Field | mode=0 (TX kick) | mode=1 (RX prep) | mode=3 (BB toggle) | 其他 (纯等待) |
|-------|------------------|------------------|--------------------|-------------|
| Caller | `ll_advertise_tx .L310` (tail-call) | `ll_advertise_process` 多处 | `ll_connect_*` (未深挖) | 未见 |
| a0 (mode) | 0 | 1 | 3 | 2/4/5+ |
| a1 (arg1) | `pdu_buf` | 0 | ? | ? |
| a2 (arg2) | `pdu_len` | 0 / 37 (channel) | ? | ? |
| 返回值 | 无 (void) | 无 | 无 | 无 |
| 副作用 | 写 LLE[0]=2 + 清 IPPara 标志 + 等 done | 设 LLE[8]=0x2000 + LLE[100]=window + 等 done | BB[0]\|=0x800000 + BB[44]&=~3 + 等 done | 仅等 done |
| 返回点语义 | TX 完成 + 后续 IPPara/BB 已重置 | RX 完成 (或 TX 完成后) | BB 状态 toggled + done | done |

**Rust 替代设计**:
- mode=0 直接拆成 `tx_kick(buf, len)` + `wait_done()` 两个原语 — 不需要继承 "wait_finish" 误导名
- mode=1 拆成 `rx_prep(channel)` + `wait_done()` 
- mode=3 拆成 `bb_commit()` + `wait_done()`
- `wait_done()` 用 IRQ + signal/future 替代 polling (后续优化)

---

## 2. `ll_advertise_process(void)` — TMOS task dispatcher + state machine

### 2.1 注册 + 入口

```c
// 注册在 ll_advertise_tx 里:
//   .L311 slow:  TMOS_SysRegister(ll_advertise_process)
//   .L310 fast:  TMOS_ProcessRegister(ll_advertise_process, 0)
// 含义: ll_advertise_process 由 TMOS 调度器在每次 task event 时调用,
//       不是 IRQ-driven, 是 cooperative scheduler.
```

入口 (offsets 0-4a) 大量全局加载 + 准备:
```asm
0:    sp -= 64; sw s1..s9, *(sp)
4-8:  s1 = &gBleLlPara
2e-32: s2 = &gBleIPPara
36-3a: s3 = &gptrLLEReg
24-28: s5 = &.L440        ; jump table base
40-44: s9 = &ble          ; 某全局结构
22:    s6 = 12             ; jump table size limit
2c:    s7 = 4              ; (用于其他 cmp)
3e:    s8 = 3              ; (用于其他 cmp)
e:    s0 = gBleLlPara[100] ; *** 当前 adv_state_t * ***
48:    s4 = s0 + 20         ; 某子字段 base
```

`gBleLlPara[100]` 是当前 adv state ptr — 这就是 ll_advertise_tx 拿到的 `a0`. **整个 LL 通过 `gBleLlPara[100]` 共享当前 adv 状态指针**.

### 2.2 主循环 `.L437` — byte_11 状态机分派

```asm
4c: <.L437>:
4c:  a5 = st->byte_11           ; state stamp byte
50:  a5 += 110                  ; offset for table indexing
54:  zext.b a5                  ; unsigned byte
58:  if (a5 > 12) goto .L438    ; out of range → error path
5c:  a5 <<= 2                   ; word offset
5e:  a5 += s5  (= jump table base)
60:  a5 = *a5                   ; load entry
62:  a5 += s5  (= base, entry was PC-relative)
64:  jr  a5                     ; jump
```

**Indexed jump table** 大小 13 (0..12), 索引 = `(byte_11 + 110) & 0xff`. 推测的 byte_11 ↔ handler 映射:

| byte_11 (signed) | byte_11 (hex) | +110 | Handler | 含义 (推测) |
|------------------|---------------|------|---------|------|
| -110 | 0x92 | 0 | (table[0]) | ADV_DIRECT_IND ready (ll_advertise_tx 写) |
| -109 | 0x93 | 1 | (table[1]) | SCAN_RSP ready (ll_advertise_generated_scan_rsp 写) |
| -108 | 0x94 | 2 | (table[2]) | common ADV ready (ll_advertise_tx 写) |
| -107 | 0x95 | 3 | (table[3]) → .L470 | post-TX, decide next channel/event |
| -106 | 0x96 | 4 | (table[4]) | ? (未在 tx/closed 见到设置) |
| -105 | 0x97 | 5 | (table[5]) | post-direct? .L451? |
| -104 | 0x98 | 6 | (table[6]) | ? |
| -103 | 0x99 | 7 | (table[7]) → .L442 | post AUX_SCAN_RX wait |
| -102 | 0x9a | 8 | (table[8]) → .L441 | post AUX_CONN_RX wait |
| -101 | 0x9b | 9 | (table[9]) | post-process? .L452 |
| -100 | 0x9c | 10 | (table[10]) | .L540 sets this |
| -99 | 0x9d | 11 | (table[11]) | .L539 sets this |
| -98 | 0x9e | 12 | (table[12]) → .L538 |   |

**注意**: 我没读 `.L440` jump table 内容 (在 `.rodata` section, objdump 没 dump), 上面映射是从 handler 内部逻辑推回去的, **不精确**. 要精确映射需要 dump `.rodata.L440`.

### 2.3 关键 handler — `.L470 / .L537` — **下次 `ll_advertise_tx` 触发点**

`.L470` (offset 0x2be) 是 channel hop / event continuation 决策点:

```asm
2be: <.L470>:
2be:  a4 = st->byte_25                            ; ADV channels active set?
2c2:  a5 = st->byte_26                            ;  last channel idx?
2c6:  if (a4 == 0) goto .L471
2c8:  a3 = st->byte_10                            ; current channel idx
2cc:  if (a4 == a3) goto .L472                    ; reached end
2d0:  if (a3 == a5) goto .L473                    ; in some state
2d4:  st->byte_10 = a4                            ; advance channel idx
     fallthrough to .L537

2d8: <.L537>:
2d8:  a0 = s0  (= st)
2da:  call ll_advertise_tx                        ; *** 下次 ADV TX ***
```

**关键**: `.L537` 直接 `call ll_advertise_tx(st)`. 这是同一 event 内的下一个 channel TX. byte_10 = current channel idx, byte_25/26 = ADV channels active set / end marker.

Channel hop 推进语义:
- `st->byte_10` = 当前 channel index (37/38/39)
- `st->byte_25` = ?
- `st->byte_26` = ?
- 决策树: 如果还有 channel 没打 → `st->byte_10 = next; ll_advertise_tx(st)`. 否则进入 closed 路径.

(精确语义还需要 cross-ref `llAdvertiseStart` 和 `LL_AdvertiseEnalbe` — 那两个会初始化 byte_25/26)

### 2.4 通用 handler 模式 — 例 `.L443` (state[0]=0x92, ADV_DIRECT_IND post-TX)

```asm
66: <.L443>:
66-72:    BLE_SetPHYRxMode(0, 37, 0)              ; RX 模式预热, channel 37
7a-82:    ll_tx_wait_finish(1, 0, 37)             ; 等 TX 完成 + RX prep
86:       a5 = gBleIPPara[3]
8a:       if (a5 & 1):                            ; <-- TX-IRQ done flag
              // got reply (SCAN_REQ/CONNECT_IND)
9e-a2:        ble_ll_chkcrc(...)                  ; CRC 校验
a6:           if (a0 != 0) skip                   ; CRC fail
aa:           ll_advertise_legacy_rx(st)          ; 处理 RX (分派 SCAN_REQ/CONNECT_IND)
b2: <.L454>:
b2-ba:    if (st->byte_11 == 0x93) goto .L437     ; 状态变 SCAN_RSP ready → 重入主循环
be: <.L542>: ble_ll_hw_api_shut(); → .L543 → ll_advertise_status_closed(st)
```

**模式**: 每个 state handler 都做:
1. (可选) BLE_SetPHYRxMode / TX 准备
2. `ll_tx_wait_finish(1, ...)` 等 TX 完成 + 进 RX
3. 检查 gBleIPPara flag 是否有 RX 收到东西
4. 如果是 SCAN_REQ/CONNECT_IND, 进入 `ll_advertise_legacy_rx` 分派
5. 否则或 RX 完, 进 `ll_advertise_status_closed` 决定是否结束 event

### 2.5 错误路径 — `.L438`

```asm
204: <.L438>:
204: call ble_ll_hw_api_shut                       ; 硬件复位 / shut down PHY
20c: j    .L541                                    ; .L541: call ll_advertise_event_closed
```

**含义**: 状态异常或意料外路径 → 硬件 shut + 直接关 event (跳过 status_closed 决策).

### 2.6 退出 — `.L436`

```asm
d0: <.L436>:
d0-e8: epilogue restore + ret
```

ll_advertise_process 退出 = TMOS task 完成 (这个 tick), TMOS scheduler 选下一个 task. 如果 `tmos_set_event` 被 status_closed 设置, 下一个 tick 还会再调度 ll_advertise_process.

---

## 3. `ll_advertise_status_closed(st)` — **多路径汇合点**

### 3.1 反汇编 (101 行, 实际 ~30 行逻辑)

```asm
0:  <ll_advertise_status_closed>:
0:    a5 = st->byte_26
4:    if (a5 == 0) goto .L80
6:    a4 = st->byte_10
a:    if (a4 != a5) goto .L80
      // byte_26 != 0 && byte_10 == byte_26
e: <.L83>:
e-12:  tail-call ll_advertise_event_closed(st)    ; *** EVENT CLOSE ***

16: <.L80>:
16:   a4 = st->byte_25
1a:   if (a4 == 0) goto .L81
1c:   a3 = st->byte_10
20:   if (a3 == a4) goto .L82
24:   st->byte_10 = a4                            ; advance channel to byte_25

28: <.L91>:
28-36: tail-call tmos_set_event(gTmosPara[3], 2)  ; *** SET TMOS EVENT 2 ***

3a: <.L81>:
3a:   if (byte_26 == 0) goto .L83 (event_closed)
3c:   if (st->byte_10 != 0) goto .L83 (event_closed)

42: <.L84>:
42:   st->byte_10 = byte_26                       ; advance
46:   j .L91 (tmos_set_event)

48: <.L82>:
48:   if (byte_26 == 0) goto .L83 (event_closed)
4a:   j .L84 (advance + tmos_set_event)
```

### 3.2 决策树

```
inputs: st->byte_10 (current ch), st->byte_25 (?), st->byte_26 (?)

if (byte_26 != 0 && byte_10 == byte_26):
    → event_closed (event 完全结束, 触发 user cb)

elif (byte_25 != 0):
    if (byte_10 != byte_25):
        byte_10 = byte_25
        → tmos_set_event(2)   (continue, 下次 process 推进)
    else:  // byte_10 == byte_25
        if (byte_26 == 0):
            → event_closed
        else:
            byte_10 = byte_26
            → tmos_set_event(2)

else:  // byte_25 == 0
    if (byte_26 == 0):
        → event_closed
    elif (byte_10 != 0):
        → event_closed
    else:  // byte_10 == 0
        byte_10 = byte_26
        → tmos_set_event(2)
```

**简化语义** (推测):
- `byte_25` = "next channel idx" (TMOS event 触发后用这个)
- `byte_26` = "terminal/final channel idx" (达到这个就结束 event)
- `byte_10` = "current channel idx"
- 决策: 如果已到 terminal → event close (user cb); 否则推进 byte_10 + 设 TMOS event 让 process 下个 tick 再调度.

### 3.3 Tail-call 目标

| 目标 | 何时 | 含义 |
|------|------|------|
| `ll_advertise_event_closed(st)` | byte_10 已到 byte_26, 或 byte_25/26 都为 0 但 byte_10 != 0 | event 完全结束 |
| `tmos_set_event(taskID=gTmosPara[3], event=2)` | 还有 channel 没打 | 继续 event, TMOS 下个 tick 再调度 process |

**Rust 设计**: Rust scheduler 不需要 TMOS — 直接用 state machine:
- "event continuing" → schedule next ll_advertise_tx 即时调用
- "event closed" → fire user cb, reset state

---

## 4. `ll_advertise_event_closed(st)` — User cb dispatch

(Cross-reference: 已在 `ti-wch-mapping.md` §13.2 反汇编, 这里只 recap + 修正)

```asm
0:  add  sp, sp, -16
2-6:  gBleLlPara[0x7c] = 0                        ; *** 清 adv-event state byte ***
a:    sw   s0, 8(sp)
c-10: a5 = 0x91 (-111); st->byte_11 = 0x91         ; state stamp = "event closed"
16:   sw   ra, 12(sp)
1a:   a5 = &pfnAdvertiseEventCBs
1e:   s0 = *a5 = pfnAdvertiseEventCBs              ; user cb ptr
20:   if (s0 == 0) goto .L76 (epilogue)
22-28: a0 = gTmosPara[3]                           ; LL task ID
2c-30: tmos_get_task_timer(taskID, 1)              ; → a0 = remaining ticks
34-38: a0 *= 625                                   ; → timeUs = ticks * 625
3c:   t1 = s0 = user cb
3e-42: restore s0/ra/sp
44:   jr t1                                        ; *** tail-call user_cb(timeUs) ***

46: <.L76>:
46-4c: restore + ret  (no cb registered)
```

### 4.1 修正 §13.2 笔记

我之前在 `ti-wch-mapping.md` §13.2 写 `gBleLlPara[0x7c]` 是 "TX-busy guard"; 实际:
- `gBleLlPara[0x7c]` (offset 124) = **adv-event state byte** (写入 0/2/3 等, 来自 ll_advertise_tx 多处)
- `gBleLlPara[168]` (offset 0xa8) = **TX-busy guard** (ll_advertise_tx 入口检查)
- 两个**不同**的字段! 我之前混淆了, 这里修正.

`ll_advertise_event_closed` 清 `gBleLlPara[124]` = 重置 adv-event state machine, 让下次 event 可以重新开始. (不是清 busy guard, busy guard 由 IRQ handler / hardware 路径清.)

### 4.2 timeUs 语义 (重申)

```
timeUs = tmos_get_task_timer(taskID, mode=1) * 625
```
- `tmos_get_task_timer(task, 1)` 返回**距离下一个 task event 的剩余 ticks** (推测)
- 乘 625 = µs 单位 (1 tick = 625 µs = 0.625 ms, 跟 RTC 1600 Hz 对得上 — 见 `c-ground-truth-fnGetClockCBs.md`)
- 所以 `timeUs` = **距下一次 adv event 的剩余时间** (单位 µs), 不是绝对时间戳, 不是这次 event 的耗时

**Rust 设计**:
```rust
extern "C" fn my_adv_cb(time_us_remaining: u32) {
    // time_us_remaining = µs until next adv event
    // 用法: 用来调整 schedule / 决定是否能塞别的工作
}
```

---

## 5. Cindy 三问 — 浓缩答案

### 5.1 TX done 的判定源

**答**: `ll_tx_wait_finish.L83` 轮询 3 个条件, 任一满足退出:

```rust
fn wait_tx_done() {
    while !(gBleIPPara[2] & 1 != 0)       // hardware IRQ flag A
       && !(gBleIPPara[3] & 1 != 0)       // hardware IRQ flag B
       && !(*gptrLLEReg)[100] == 0 {       // LLE-busy auto-clear
        // busy-wait
    }
}
```

`gBleIPPara[2]/[3]` bit 0 由 BB/PHY IRQ handler 置位 — IRQ handler 不在 LL.o 里, 应该在 ip.o 或更低层. LLE[100] = 0 是硬件自清的 fallback.

**Rust 第一版**: 照搬 polling. 优化版用 IRQ → atomic flag → wfi 替代.

### 5.2 event_closed 的汇合路径

**答**: `ll_advertise_status_closed(st)` 是**唯一** convergence (除了 `.L438` 错误路径直接调 `ll_advertise_event_closed`).

汇合策略:
1. 所有 ADV state handler 在 PDU 处理完毕后 → `ll_advertise_status_closed(st)`
2. `status_closed` 根据 byte_10/25/26 决定 event close vs continue
3. event close → tail-call `ll_advertise_event_closed` → 清状态 + user cb
4. continue → tail-call `tmos_set_event(task, 2)` → TMOS 下个 tick 再调度 process

**`ll_advertise_process` 内部直接调用 `ll_advertise_event_closed`** 的地方:
- `.L541` (RX path 完成, scan req 已处理)
- `.L490` (scan rsp 后 cleanup)
- 错误/异常路径 (`.L438` → `.L541`)

→ 所以 event_closed 有**多个入口**, 但 status_closed 是**主要决策点**.

### 5.3 下一次 `ll_advertise_tx` 的触发点

**答**: 两条路径:

**路径 A** (同一 event 内, channel hop):
```
ll_advertise_status_closed → tmos_set_event(task, 2)
  → TMOS scheduler 下个 tick 调度 ll_advertise_process
  → process .L437 dispatch byte_11
  → .L470 (state=0x95 post-TX) decides next channel
  → .L537 直接 call ll_advertise_tx
```

**路径 B** (新 event, 由 ADV interval timer 触发):
- 还没反汇编, 应该在 `llAdvertiseStart` / `LL_AdvertiseEnalbe` 或 ADV interval timer callback
- 这是 **Phase 2** 计划的 scope (scheduler entry / interval / random delay)

---

## 6. Open questions (Phase 2/2.5/3 scope)

待 Phase 2/2.5/3 解的:

| 项 | 涉及函数 | Phase |
|----|---------|------|
| `gBleIPPara[2]/[3]` bit 0 谁置位 (IRQ handler 在哪) | bb_irq_handler 或类似 | Phase 2.5 RX ingress |
| `BLE_SetPHYRxMode` 内部细节 | `ip.o:BLE_SetPHYRxMode` | Phase 3 |
| `BLE_SetPHYTxMode` 内部细节 | `ip.o:BLE_SetPHYTxMode` | Phase 3 |
| `phy_status_clear(2)` 写哪些 RFEND 位 | `ip.o:phy_status_clear` | Phase 3 |
| `ble_ll_chkcrc` 实现 (CRC 校验) | `ll.o:ble_ll_chkcrc` 或 `ip.o` | Phase 2.5 RX |
| `ble_ll_hw_api_shut` 内部 (PHY/BB shut down) | ? | Phase 2.5 |
| `ll_advertise_legacy_rx` SCAN_REQ vs CONNECT_IND 分派 | `ll_advertise.o` | **Phase 2.5** (优先) |
| `ll_advertise_to_connection_state` (CONNECT_IND 处理) | `ll_advertise.o` | Phase 2.5 (并入) |
| `LL_AdvertiseToStandby` (advertising terminated) | `ll_advertise.o` | Phase 2/2.5 |
| `llAdvertiseStart` (new event begin) | `ll_advertise.o` | **Phase 2** |
| `llAdvertiseTimeout` (interval expire) | `ll_advertise.o` | **Phase 2** |
| `.L440` jump table 内容 → 精确 byte_11 mapping | `ll_advertise.o:.rodata.L440` | 可马上做, Phase 2 提前 |
| `pGapRoles_AppCGs` 结构 + cb 调用约定 | host/profile 层 | Phase 4 (defer) |

### 6.1 不在原计划里, 但发现的 leverage

- `ll_rx_wait_finish` (在 ip.o 里) — RX 完成函数, 跟 ll_tx_wait_finish 兄弟. 75 行, 比 tx 简单. Phase 2.5 RX ingress 时一并解.
- `BLE_SetPHYRxMode` 跟 `BLE_SetPHYTxMode` 兄弟 — Phase 3 合并解.
- 多个 sub-handler (`.L441 .L442 .L443`) 都跑相同 pattern (PHY_Rx_prep + tx_wait_finish + chkcrc + dispatch) — Phase 2.5 时可抽出 "TX→RX 转换原语" 设计.

---

## 7. Rust LL TX 完成边界设计要点

| # | Rule | Source |
|---|------|--------|
| 1 | TX 完成检测用**轮询**, 等 `IPPara_flag_A & 1` OR `IPPara_flag_B & 1` OR `LLE[100] == 0` 任一为真 | §1.5 |
| 2 | LL "process" 函数是 TMOS task callback, **不是 IRQ**. cooperative scheduler. Rust 用 state machine + executor 等价 | §2.1 |
| 3 | 当前 adv state 通过**全局指针 `gBleLlPara[100]`** 共享, 不是入参. Rust 用 RefCell + 单例 | §2.1 |
| 4 | byte_11 状态字 (0x91-0x9e) 是 adv state-machine 主分派源, 13 个状态. Rust 用 `enum AdvLLState` | §2.2 |
| 5 | byte_10 = current channel, byte_25/26 = 终端 markers. Channel hop 在 `status_closed` 决策 | §3 |
| 6 | event_closed 触发用 user cb 入参 `timeUs = ticks * 625` = **距下次 event 的剩余 µs**, 非绝对时间戳 | §4 |
| 7 | `ll_tx_wait_finish(0,...)` 实际是 "TX kick + wait" — 名字误导. Rust 拆 `tx_kick()` + `wait_done()` 两原语 | §1.3 |
| 8 | `ll_tx_wait_finish(1,...)` 是 "wait + RX prep". LLE[8]=0x2000 启 RX. LLE[100] = RX window 由 BB[0][12:13] PHY 模式选择 (1M=406/Coded=1086/2M=446) | §1.4 |
| 9 | `tmos_set_event(task, 2)` = 设 LL adv 任务 event 2 = "继续 process"; LL 主循环靠这个 reschedule, Rust 用 `Waker.wake()` 等价 | §3.3 |
| 10 | LL 错误处理 = `ble_ll_hw_api_shut()` + 走 closed 路径. Rust 异常 = `phy.shut() + close_event()` | §2.5 |
| 11 | gBleLlPara[124] (`+0x7c`) = adv-event state byte (写 0/2/3); gBleLlPara[168] = TX-busy guard. **不同字段, 之前混淆了** | §4.1 |

---

## 8. Cross-reference

- `notes/ch32-rs/ti-ble/ll-advertise-tx-disasm.md` — sibling doc (TX 装配 + MMIO 写序列)
- `notes/ch32-rs/ti-ble/peripheral-flow-deep-dive.md` (Cindy) — peripheral 全流程概览
- `notes/ch32-rs/ti-ble/ti-wch-mapping.md` §13.2 — `ll_advertise_event_closed` 早期反汇编 (本 doc §4 是修正版)
- `notes/ch32-rs/c-ground-truth-fnGetClockCBs.md` — RTC 1600 Hz / 625 µs/tick 来源
- `/tmp/wchble-disasm/ll_tx_wait_finish.dump` (203 lines)
- `/tmp/wchble-disasm/ll_advertise_process.dump` (830 lines)
- `/tmp/wchble-disasm/ll_advertise_status_closed.dump` (101 lines)
- `/tmp/wchble-disasm/ll_advertise_event_closed.dump` (93 lines)
- `/tmp/wchble-disasm/ll_rx_wait_finish.dump` (75 lines, Phase 2.5 用)

---

**作者**: Lucy (@Lucy)
**日期**: 2026-05-13 00:xx
**Phase**: 1/4 of 周围必备 LL 函数分析计划
**Next**: Phase 2 = scheduler/channel hop/interval (`llAdvertiseStart` / `llAdvertiseTimeout` / `.L440` jump table dump / channel hop 推进点)

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-tx-completion-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
