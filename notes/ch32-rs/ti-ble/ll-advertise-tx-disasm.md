# `ll_advertise_tx` + `ll_advertise_generated_scan_rsp` Disassembly Notes

**Status**: 2026-05-12 23:xx — Lucy 写, 支援 Cindy `peripheral-flow-deep-dive.md` (task #67/#68 lane).
**Source**: `libwchble.a` (CH32V208 BLE stack), extracted via
```
ar x ~/Elec/WCH/ch32v208_ble/libwchble.a ll_advertise.o
riscv64-unknown-elf-objdump -dr --disassemble=ll_advertise_tx                ll_advertise.o > /tmp/wchble-disasm/ll_advertise_tx.dump
riscv64-unknown-elf-objdump -dr --disassemble=ll_advertise_generated_scan_rsp ll_advertise.o > /tmp/wchble-disasm/ll_advertise_generated_scan_rsp.dump
```
Dumps: 645 lines (`ll_advertise_tx`) + 56 lines (`ll_advertise_generated_scan_rsp`).

**Purpose**: 把这两条最热的 ADV/SCAN_RSP TX 路径完整反向, 抽出 BB/LLE/RFEND MMIO 写序列, 让纯 Rust LL 的 TX 实现有一份可对照的 ground truth.

**⚠️ Reference-only** — 这是 Cindy 三档框架 (msg `590754ec`) 的 bucket #1: 参考实现.
- 用途: 在写 Rust LL TX 时, 拿来做硬件落点对照 + 协议序列校对.
- 禁区: 不 extern, 不 bind, 不进 ch32-hal::ble 长期 API.
- 见 `ti-wch-mapping.md` 顶部 ⚠️ STATUS 块.

---

## 0. TL;DR — 关键发现速查

| 主题 | 实测发现 | Rust 设计含义 |
|------|----------|---------------|
| 寄存器块引用方式 | 三个全局 `void*` (`gptrLLEReg` / `gptrBBReg` / `gptrRFENDReg`), 每次进函数 load 一次到 saved reg (s2/s3/a4) 复用 | Rust 用 `&'static T` 或 PAC singleton, **不要每个寄存器写都 `read_volatile(global)`** |
| BLE Access Address | `0x8e89bed6` 直接 `sw` 到 `gptrBBReg[8]` (offset 0x8) | 协议定数, ad-spec 第 6 部分 §2.1.2; Rust 用 `const` |
| BLE CRC init | `0x555555` 直接 `sw` 到 `gptrBBReg[4]` (offset 0x4) | 协议定数, Rust 用 `const` |
| ADV channel 编码 | 6-bit channel 写到 `gptrBBReg[44]` (offset 0x2c) bits 25-30 (`<< 0x19`) | 字段位置非自然; PAC 或本笔记必须固化 |
| PDU 缓冲区指针 | `buf_ptr = adv_state[76]`; 写入 `gptrLLEReg[112]` (offset 0x70) 让硬件 DMA | Rust 需要 buf-ptr 寄存器, 不是 buf 内嵌 |
| PDU 类型分派 | 状态字段 `adv_state[16]` (低 4 位): **7=ADV_EXT_IND**, **1=ADV_DIRECT_IND**, 其他 (含 6=ADV_SCAN_IND/0=ADV_IND/2=ADV_NONCONN_IND) 走通用路径 | Rust 用 `enum AdvPduType` + match |
| PHY 预热 | 入口 `phy_status_clear(2)`, 出口前 `BLE_SetPHYTxMode()` | TX 前需要 PHY 状态机重置 + 模式 kick |
| 完成回调注册 | `TMOS_SysRegister(ll_advertise_process)` 注册 LL 自己的 process handler; 不是 user cb (那是 `ll_advertise_event_closed`) | Rust 的 TX-complete 处理点 ≠ scheduler tick |
| 异步等完成 | 函数尾以 `jr t1` 尾调用 `ll_tx_wait_finish(a0=0, a1=buf, a2=length)` | tail-call: 不持锁等待, control 流交给 wait_finish |
| `ll_advertise_generated_scan_rsp` | 简单 PDU 装配: header byte 16=4 (SCAN_RSP), AdvA 6 字节, 复制 adv-data, 状态 byte 11 = 0x93 | SCAN_RSP 不重新算寄存器; ll_advertise_tx 已经 kick 过 PHY |
| 入口 guard | `if (gBleLlPara[168] != 0) goto epilogue` (early return) | LL state-machine busy 检查; Rust 必须建模 |

---

## 1. 调用上下文 / 进入路径

`ll_advertise_tx` 是 LL adv TX 的**实际硬件落点函数** — 把当前 adv 状态拆出, 配置 BB/LLE/RFEND 寄存器组, 装配 PDU header, 启动 PHY TX, 然后 tail-call 到 `ll_tx_wait_finish` 等完成.

调用者 (`grep R_RISCV_CALL ll_advertise_tx /tmp/wchble-disasm/full.dump` 反查):
- `ll.o` (LL 主调度状态机)
- `ll_advertise.o` 内部其它函数 (重试 / 多通道循环)
- `ll_advertise_event_closed` 的兄弟函数会把 `pfnAdvertiseEventCBs(timeUs)` 通知 user, 但**不会**调 `ll_advertise_tx` — TX 的发起是 LL 调度器决定, 不是 user cb 决定. (这个关系在 §13.2 of `ti-wch-mapping.md` 已经确认过.)

注: `ll_advertise_generated_scan_rsp` 由 RX 侧的 SCAN_REQ 处理路径调用 — 别处装配 SCAN_RSP, 进 wait queue, 不重新跑 `ll_advertise_tx` 的硬件配置流程.

---

## 2. 函数总览 — `ll_advertise_tx`

```c
// 推测签名 (从 a0 用法 + struct 偏移推):
void ll_advertise_tx(adv_state_t *st);   // a0 -> s0

// adv_state_t (推测部分布局, 字段名我自起):
struct adv_state_t {
    uint8_t  byte_00;            // [0]  header byte 0 dst (输出, hw header byte 0)
    /* ... */
    uint8_t  chan_or_phy;        // [10] low 7 bits -> LLE[0] (channel/PHY?)
    uint8_t  pdu_type_raw;       // [11] state stamp byte, 写 -108/-109/-110 (0x94/0x93/0x92)
    /* ... */
    uint8_t  byte_14;            // [14] low 4 bits -> adv_state[16] = pdu_type
    uint8_t  pdu_type;           // [16] PDU type (low 4 bits): 0/1/2/6/7
    uint8_t  pdu_len;            // [17] dst, payload length cursor
    uint8_t  byte_18;            // [18]
    /* ... */
    uint8_t  byte_21;            // [21]
    /* ... */
    uint16_t adv_data_len;       // [28] adv data length (halfword)
    /* ... */
    uint8_t  byte_24;            // [24]
    /* ... */
    uint32_t adv_data_ptr;       // [40] adv data buffer ptr (lw)
    uint32_t scan_rsp_data_ptr;  // [44] scan-rsp data ptr (lw, 给 generated_scan_rsp 用)
    uint32_t peer_ptr;           // [48] peer info struct ptr (lw, byte 18 = HasIRK flag)
    uint8_t  byte_52;            // [52] addr type flag (==2?)
    uint8_t  byte_53;            // [53] addr type bits (bit0 = is_random?, bit1 = peer_has_irk_check?)
    uint8_t  adva[6];            // [54-59] own AdvA
    uint8_t  byte_60_61;         // [60-61]
    uint8_t  byte_61;            // [61] peer present flag
    uint8_t  peer_addr[6];       // [62-67] peer address
    uint32_t pdu_buf;            // [76] pdu buffer ptr (LLE 用)
    /* ... */
    uint8_t  byte_94;            // [94] some 2-bit field
    uint8_t  byte_95;            // [95] EXT_ADV sub-mode (3/6/else)
    uint8_t  byte_96;            // [96] flags (bit6 = include_extra?)
    uint8_t  byte_97;            // [97]
    uint8_t  byte_98;            // [98]
    uint8_t  byte_99;            // [99]
    uint8_t  byte_100;           // [100] mode flag (==2 for ...)
    uint8_t  byte_104;           // [104] (channel?) — 6 bits -> BB[44] bits 25-30
    uint8_t  byte_106;           // [106]
    uint16_t halfword_108;       // [108]
    uint16_t halfword_116;       // [116] computed adv interval; written back to st itself
    uint8_t  byte_124;           // [124]
    uint8_t  byte_127;           // [127]
    uint32_t word_216;           // [216]
};
```

**注意**: 这些字段名是我推断的, 不是 WCH 的官方名 — WCH 源码没开. 跟 TI BLE-Stack `llConnState_t` / `advParams_t` 类似的命名是合理的方向, 但我不会强行 1:1 套.

---

## 3. 入口 prologue + busy guard (offsets 0-20)

```asm
0:   add  sp, sp, -32
2:   sw   s5, 4(sp)
4:   auipc s5, %hi(gBleLlPara)
8:   addi  s5, s5, %lo(gBleLlPara)         ; s5 = &gBleLlPara
c:   lw   a5, 168(s5)                      ; a5 = gBleLlPara[168]
10:  sw   ra, 28(sp)
12:  sw   s0, 24(sp)
14:  sw   s1, 20(sp)
16:  sw   s2, 16(sp)
18:  sw   s3, 12(sp)
1a:  sw   s4, 8(sp)
1c:  bnez a5, 24e <.L272>                  ; if gBleLlPara[168] != 0: return
20:  mv   s0, a0                           ; s0 = adv_state_t *
```

**关键**: `gBleLlPara[168]` 是 **TX-busy / state-busy guard**. 非零就直接跳到 epilogue. 这是 LL 状态机互锁: 还在前一次 TX/RX 未完成, 拒绝重入.

`.L272` 在 offset 0x24e:
```asm
24e: lw ra, 28(sp); lw s0, 24(sp); ...; addi sp, sp, 32; ret
```
单一 epilogue.

**Rust 设计**: TX 状态需要 atomic flag / state-machine 显式建模. 不能假设上一次 TX 完成 — busy 状态来自硬件中断回调路径设置.

---

## 4. PHY 状态机预热 (offsets 22-2a)

```asm
22:  li   a0, 2
24:  call phy_status_clear                 ; phy_status_clear(2)
```

`phy_status_clear(uint32_t mode)` — 把 PHY 子系统状态机置为 "ready-for-TX". 参数 2 = TX 模式 (推测).

**协议**: 1M PHY / 2M PHY / Coded PHY 切换前需要状态机 reset. WCH 的 PHY engine 在 RFEND block 里, 但 phy_status_clear 函数体我没继续展开 (645 行已经够长).

---

## 5. 三个寄存器块指针 load (offsets 2c-78)

这是整个文件最重要的硬件入口结构:

```asm
2c:  auipc s2, %hi(gptrLLEReg); addi s2, s2, %lo(gptrLLEReg)
34:  lw   a2, 0(s2)                        ; a2 = gptrLLEReg     ; LLE base
38:  li   a5, 160                          ; 0xa0
3c:  auipc s3, %hi(gptrBBReg);  addi s3, s3, %lo(gptrBBReg)
44:  sw   a5, 100(a2)                      ; LLE[100] = 0xa0
46:  lw   a5, 0(s3)                        ; a5 = gptrBBReg      ; BB base
4a:  lui  a1, 0x330                        ; (RFEND[8] mask constant)
4e:  mv   s4, s5                           ; s4 = gBleLlPara
50:  lw   a4, 44(a5)                       ; a4 = BB[44]
52:  andi a4, a4, -4                       ; clear bits 0-1
54:  ori  a4, a4, 1                        ; set bit 0
58:  sw   a4, 44(a5)                       ; BB[44] = (BB[44] & ~3) | 1
5a:  lw   a4, 0(a5)                        ; a4 = BB[0]
5c:  andi a4, a4, -385                     ; clear bits 7-8 (mask 0x180)
60:  sw   a4, 0(a5)                        ; BB[0] = ... bits 7,8 cleared
62:  lw   a4, 0(a5)                        ; reload BB[0]
64:  andi a4, a4, -385                     ; same mask (paranoid)
68:  ori  a4, a4, 256                      ; set bit 8
6c:  sw   a4, 0(a5)                        ; BB[0] = (... & ~0x180) | 0x100
6e:  auipc a4, %hi(gptrRFENDReg); addi a4, a4, %lo(gptrRFENDReg)
76:  lw   a4, 0(a4)                        ; a4 = gptrRFENDReg
78:  lw   a3, 8(a4)                        ; a3 = RFEND[8]
7a:  or   a3, a3, a1                       ; a3 |= 0x330000
7c:  sw   a3, 8(a4)                        ; RFEND[8] |= 0x330000
7e:  li   a3, 90                           ; 0x5a
82:  sw   a3, 80(a2)                       ; LLE[80] = 0x5a (90)
84:  lw   a3, 44(a4)                       ; a3 = RFEND[44]
86:  andi a3, a3, -3                       ; clear bit 1
88:  sw   a3, 44(a4)                       ; RFEND[44] = ... bit 1 cleared
```

**写序列总结** (按 MMIO 偏移整理):

| Block      | Offset | 操作 | 含义 (推测) |
|------------|--------|------|------|
| LLE | 100 (0x64)  | `= 0xa0`                | 某 timeout / window? (160) |
| LLE |  80 (0x50)  | `= 0x5a`                | (90) — 可能 prebuffer / IFS |
| BB  |  44 (0x2c)  | `= (& ~3) \| 1`        | mode 字段, 设为 1 (TX-channel mode?) |
| BB  |   0 (0x00)  | `= (& ~0x180) \| 0x100` | bit 8 set, bit 7 clear (RX/TX 方向?) |
| RFEND | 8 (0x08)  | `\|= 0x330000`         | PHY 模式 enable bits |
| RFEND | 44 (0x2c) | `&= ~2`                 | clear bit 1 (PA enable? 后面会重新 enable) |

**注**: 这些位段含义没有 datasheet (GF-1, Andelf 2026-05-11). 推测来自上下文和位模式. 真要确认得在 Rust 实现里做 GPIO 探针 trace.

---

## 6. PDU type 取值 + buffer 准备 (offsets 8a-b0)

```asm
8a:  lbu  a4, 10(s0)                       ; a4 = st->chan_or_phy (s0+10)
8e:  andi a3, a4, 127                      ; low 7 bits
92:  lw   a4, 0(a5)                        ; a4 = BB[0]      (a5 still = gptrBBReg)
94:  andi a4, a4, -128                     ; clear low 7 bits
98:  or   a4, a4, a3
9a:  sw   a4, 0(a5)                        ; BB[0].lo7 = st->chan_or_phy & 0x7f
9c:  lbu  a5, 14(s0)                       ; a5 = st->byte_14
a0:  lw   a4, 76(s0)                       ; a4 = st->pdu_buf  (PDU buffer ptr)
a2:  andi a5, a5, 15                       ; low nibble = PDU type
a4:  sb   a5, 16(s0)                       ; st->pdu_type = a5  (save to state)
a8:  sb   a5, 0(a4)                        ; pdu_buf[0] = pdu_type   (header byte 0 low nibble)
ac:  li   a4, 7
ae:  lw   s1, 76(s0)                       ; s1 = pdu_buf (reload, redundant)
b0:  bne  a5, a4, 498 <.L274>              ; if pdu_type != 7: goto .L274
```

**关键**:
- PDU type 来源 = `st->byte_14 & 0xf` (低 4 位)
- 同步写到 `st->pdu_type` (offset 16) 和 PDU buffer 第一字节 `pdu_buf[0]`
- BB[0] 低 7 位 = `st->chan_or_phy & 0x7f` — 这是物理信道 / PHY 配置

**注**: BLE 5.x ADV PDU header byte 0 = `[RFU(1) | ChSel(1) | TxAdd(1) | RxAdd(1) | PduType(4)]`. WCH 这里写的是**完整的 byte 0** (4-bit PDU type + 后面会 or 上 flag bits). 之后 0x80 = TxAdd, 0x40 = RxAdd 都是 or 进同一字节.

---

## 7. PDU type 分派 (3 个 case)

```
   pdu_type == 7  → ADV_EXT_IND 系列 (legacy 名: AUX_ADV_IND / extended adv)
                    ├── byte 95 == 3 → 走 .L276 (scannable ext + data)
                    ├── byte 95 == 6 → 走 .L285 (different ext variant)
                    └── 否则        → .L275 base ADV_EXT_IND 路径
   pdu_type == 1  → ADV_DIRECT_IND  → .L274 直传走 .L302 之外的 direct 路径
   其他 (0/2/6/默认) → ADV_IND / ADV_NONCONN_IND / ADV_SCAN_IND
                       → .L274 → .L302 通用 AdvA + adv-data 拷贝路径
```

### 7.1 `pdu_type == 7` (ADV_EXT_IND) — `.L275` 开始

```asm
b4:  li   a5, 1
b6:  sb   zero, 3(s1)                      ; pdu_buf[3] = 0
ba:  sb   a5, 2(s1)                        ; pdu_buf[2] = 1 (初始 length)
be:  lbu  a4, 95(s0)                       ; ext sub-mode
c2:  li   a1, 3
c4:  bne  a4, a1, 372 <.L275>              ; if byte95 != 3: .L275
```

`.L275` 再分支 byte 95 == 6 vs 其他.

`pdu_type == 7 && byte95 == 3` 路径 (`.L276` @ 0x260) 装配 EXT_ADV header:
```asm
260: li   a5, 8
262: sb   a5, 3(s1)                        ; pdu_buf[3] = 8
266: lhu  a5, 108(s0)                      ; halfword
26a: sb   a5, 4(s1)                        ; pdu_buf[4] = low byte
26e: lhu  a5, 108(s0)
272: lbu  a4, 97(s0)                       ; byte 97
278: srl  a5, a5, 8
27c: andi a5, a5, 15                       ; high nibble of halfword
27e: or   a5, a5, (byte97 << 4)
280: sb   a5, 5(a3)                        ; pdu_buf[5] = packed
...
```

EXT_ADV header 字段 packing — 跟 BLE 5.0 spec §2.3.4.5 Common Extended Advertising Payload Format 对应 (AdvMode / AuxPtr / TxPower 等).

我**不会**在这份笔记里把所有 EXT_ADV 子字段都解出来 — Cindy Phase 1 是 legacy connectable ADV (ADV_IND), 不需要 EXT_ADV. 标记 TODO, 等 Phase 2+ 再回来.

### 7.2 `pdu_type == 1` (ADV_DIRECT_IND) — `.L274` 入口

```asm
498: li   a4, 1
49a: bne  a5, a4, 532 <.L302>              ; if pdu_type != 1: .L302
49e: li   a5, 12
4a0: sb   a5, 17(s0)                       ; st->pdu_len = 12  (6+6 = AdvA + InitA)
4a4: lbu  a5, 53(s0)                       ; st->byte_53
4a8: andi a5, a5, 2                        ; bit 1 = peer_addr_via_irk?
4aa: beqz a5, 50c <.L303>
4ac: lw   a1, 48(s0)                       ; st->peer_ptr
4ae: lbu  a5, 18(a1)                       ; peer_ptr->byte_18 (HasIRK?)
4b2: beqz a5, 50c <.L303>
4b4: li   a2, 6
4b6: addi a1, a1, 20                       ; peer_ptr + 20 = IRK-resolved addr
4b8: addi a0, s1, 8                        ; pdu_buf + 8 = InitA dst
4bc: call tmos_memcpy                       ; memcpy(pdu_buf+8, peer_ptr+20, 6)
4c4: lw   a4, 76(s0)
4c6: lbu  a5, 0(a4)
4ca: or   a5, a5, -128                     ; pdu_buf[0] |= 0x80  (set TxAdd: random?)
4ce: sb   a5, 0(a4)
...
```

ADV_DIRECT_IND 装配:
- `pdu_len = 12` (AdvA[6] + InitA[6])
- 如果 `byte_53 & 2 && peer->byte_18` 真 → InitA 来自 `peer_ptr + 20` (resolved RPA)
- 否则 → InitA 来自 `st->peer_addr[62-67]` (.L303 路径)
- `pdu_buf[0] |= 0x80` 设 TxAdd 位 (本地随机地址)
- 之后跳 `.L304` (offset 0x4d2) 复制 AdvA 然后跳 `.L348` (设 state byte = 0x92)

### 7.3 默认路径 (ADV_IND / ADV_NONCONN_IND / ADV_SCAN_IND) — `.L302` @ 0x532

```asm
532: lhu  a2, 28(s0)                       ; adv_data_len
536: lw   a1, 40(s0)                       ; adv_data_ptr
538: addi a0, s1, 8                        ; pdu_buf + 8 = adv-data dst
53c: addi a5, a2, 6                        ; adv_data_len + 6 (AdvA)
540: andi a5, a5, 63                       ; & 0x3f
544: sb   a5, 17(s0)                       ; st->pdu_len = (adv_data_len + 6) & 0x3f
548: j    528 <.L347>                      ; .L347 调 tmos_memcpy
```

`.L347` (offset 0x528):
```asm
528: call tmos_memcpy                       ; memcpy(pdu_buf+8, adv_data_ptr, adv_data_len)
530: j    4d2 <.L304>                       ; 拷 AdvA
```

`.L304` (offset 0x4d2): 拷 AdvA 到 `pdu_buf+2`:
```asm
4d2: lw   a0, 76(s0)
4d4: li   a2, 6
4d6: addi a1, s0, 54                       ; &st->adva
4da: addi a0, a0, 2                        ; pdu_buf + 2
4dc: call tmos_memcpy                       ; memcpy(pdu_buf+2, &st->adva, 6)
4e4: lw   a5, 216(s4)                      ; gBleLlPara[216]
4e8: sll  a4, a5, 17
4ec: bgez a4, 506 <.L306>                  ; if (gBleLlPara[216] & (1<<14)) == 0:
4f0: lbu  a5, 14(s0)
4f4: andi a5, a5, 14
4f6: bnez a5, 506
4f8: lw   a4, 76(s0)
4fa: lbu  a5, 0(a4)
4fe: or   a5, a5, 32                       ; pdu_buf[0] |= 0x20  (set ChSel)
502: sb   a5, 0(a4)
506: <.L306>: li a5, 0xf92; j .L348
```

**ChSel 位决策**: 如果 `gBleLlPara[216] & 0x4000` 且 `st->byte_14 & 14 == 0` → `pdu_buf[0] |= 0x20` (ChSel = 1, 用 Channel Selection Algorithm #2). 否则 = 0 (LE Channel Selection #1).

### 7.4 ADV_DIRECT_IND TxAdd 决策 (路径汇合点 `.L304` 之前)

ADV_DIRECT_IND 的 byte 53 == 3 (legacy fallback) 路径 (`.L303` @ 0x50c):
```asm
50c: lbu  a5, 61(s0)
510: beqz a5, 51e <.L305>
512: lbu  a5, 0(s1)
516: or   a5, a5, -128                     ; pdu_buf[0] |= 0x80
51a: sb   a5, 0(s1)
51e: <.L305>: ... ; copy st->peer_addr[62-67] -> pdu_buf+8
528: call tmos_memcpy
530: j    4d2 <.L304>                      ; 然后走 AdvA + state byte
```

---

## 8. **BLE 协议常量** + channel 编码 (offsets 1a4-1ce, 在 `.L309`)

这是 ll_advertise_tx 里**最重要的一段** — BLE Advertising Channel 物理参数:

```asm
1a4: lbu  a5, 104(s0)                      ; st->byte_104 (channel idx ?)
1a8: lw   a2, 0(s3)                        ; a2 = gptrBBReg (reload via s3)
1ac: andi a5, a5, 63                       ; & 0x3f
1b0: sll  a4, a5, 25                       ; << 25  → bits 25-30
1b4: lw   a5, 44(a2)                       ; BB[44]
1b6: and  a5, a5, a0                       ; mask out bits 25-30 (a0 = 0x81ffffff = ~(0x3f<<25))
1b8: or   a5, a5, a4
1ba: sw   a5, 44(a2)                       ; BB[44].bits25-30 = st->byte_104 & 0x3f
1bc: lui  a5, 0x8e89c
1c0: addi a5, a5, -298                     ; a5 = 0x8e89bed6  ← BLE ADV Access Address
1c4: sw   a5, 8(a2)                        ; BB[8]  = 0x8e89bed6
1c6: lui  a5, 0x555
1ca: addi a5, a5, 1365                     ; a5 = 0x555555     ← BLE CRC init value
1ce: sw   a5, 4(a2)                        ; BB[4]  = 0x555555
```

**协议匹配** (Bluetooth Core Spec 6.0 Vol 6 Part B §2.1):
- BLE Advertising Access Address = **`0x8E89BED6`** (固定, 所有 ADV / SCAN_REQ / SCAN_RSP / CONNECT_IND 都用这个) → BB[8]
- BLE Advertising CRC init value = **`0x555555`** (3 字节, 高位字节按 BLE 习惯放在低位) → BB[4]
- ADV physical channel index 37/38/39 → BB[44] bits 25-30 (6-bit slot, 但 channel idx 最多 39 = 6 位)

**Rust 实现含义**:
```rust
// 协议常量, 直接编进:
const BLE_ADV_ACCESS_ADDRESS: u32 = 0x8E89_BED6;
const BLE_ADV_CRC_INIT:       u32 = 0x0055_5555;  // 24-bit, low word

// BB 寄存器写序列 (用 PAC 或 raw register):
bb.access_addr().write(|w| w.bits(BLE_ADV_ACCESS_ADDRESS));
bb.crc_init().write(|w| w.bits(BLE_ADV_CRC_INIT));
bb.chan().modify(|r, w| w.chan_idx().bits(chan_37_38_39));
```

---

## 9. PDU buffer 提交给 LLE + state machine 推进 (offsets 18a-1f8)

`.L309` (offset 0x188):
```asm
188: lw   a5, 76(s0)                       ; pdu_buf
18a: lbu  a1, 17(s0)                       ; st->pdu_len
18e: lui  a0, 0x82000
192: addi a0, a0, -1                       ; a0 = 0x81ffffff (BB[44] mask: ~(0x3f<<25))
194: sb   a1, 1(a5)                        ; pdu_buf[1] = pdu_len  ← header byte 1
198: lw   a3, 0(s2)                        ; a3 = gptrLLEReg
19c: lw   a5, 4(a3)                        ; LLE[4]
19e: or   a5, a5, 1                        ; set bit 0
1a2: sw   a5, 4(a3)                        ; LLE[4] |= 1  (enable TX engine? )
1a4: ...                                   ; (channel encode + Access Addr + CRC, §8 上面)
1ec: lw   a5, 76(s0)
1ee: sw   a5, 112(a3)                      ; LLE[112] = pdu_buf   ← DMA src ptr
1f0: auipc a5, %hi(bleClock_t+12)
1f4: lbu  a5, 0(a5)                        ; bleClock_t.byte_12  (flag?)
1f8: beqz a5, 54a <.L310>                  ; if 0: goto .L310 (no-wait register fast path)
```

**关键 LLE 写**:
- `LLE[4] |= 1`: TX-engine enable
- `LLE[112] (0x70) = pdu_buf`: DMA source 给硬件

**`bleClock_t+12` 路径分叉**: 这是一个全局 flag, 决定走两条注册路径:
- 非零 → fallthrough .L311 → `BLE_SetPHYTxMode()` + `TMOS_SysRegister(ll_advertise_process)` (busy-wait + sys-register)
- 零 → `.L310` → 直接 `TMOS_ProcessRegister(ll_advertise_process, 0)` (no busy wait)

未深挖. Rust 实现可以先只走 `.L311` 一条路.

---

## 10. PHY TX kick + callback 注册 — `.L311` (offsets 1fc-24c)

```asm
1fc: <.L311>: lw a5, 100(a3)
1fe: bnez a5, 1fc                          ; busy-wait until LLE[100] == 0
200: mv   a0, s1                           ; (s1 still = pdu_buf, but discarded by next call)
202: sb   zero, 2(gBleIPPara+2)            ; gBleIPPara[2] = 0
20a: call BLE_SetPHYTxMode                  ; kick PHY into TX mode
212: auipc a0, %hi(ll_advertise_process)
216: addi a0, a0, %lo(ll_advertise_process); a0 = &ll_advertise_process
21a: call TMOS_SysRegister                  ; TMOS_SysRegister(ll_advertise_process)
222: lw   a5, 0(s3)                        ; gptrBBReg
226: lui  a3, 0x800
22a: lw   a4, 0(a5)
22c: or   a4, a4, a3                       ; BB[0] |= 0x800000
22e: sw   a4, 0(a5)
230: lw   a4, 44(a5)                       ; BB[44]
232: andi a4, a4, -4
234: sw   a4, 44(a5)                       ; BB[44] &= ~3 (clear mode field set in §5)
236: sb   zero, 0(gBleIPPara+5)
23e: sb   zero, 0(gBleIPPara+1)
246: lw   a5, 0(s2)                        ; gptrLLEReg
24a: li   a4, 2
24c: sw   a4, 0(a5)                        ; LLE[0] = 2   ← final commit, TX go!
```

**最后一字 `LLE[0] = 2`** = **TX 实际启动 signal**. 整个 ll_advertise_tx 的所有配置都为这条 store 服务.

**Rust 序列总结** (TX 实际启动顺序):
1. PHY 状态机预热: `phy_status_clear(2)`
2. LLE/BB/RFEND 寄存器配置 (§5, §6, §8)
3. PDU buffer 装配 (§7)
4. PDU header byte 1 (length) 写 → DMA ptr 写到 `LLE[112]`
5. Busy-wait `LLE[100] == 0`
6. `BLE_SetPHYTxMode()` (RFEND 配置 PHY 模式)
7. `TMOS_SysRegister(ll_advertise_process)` 注册 TX-complete handler
8. BB 收尾配置 (mode 复位)
9. **`LLE[0] = 2`** TX kick
10. tail-call `ll_tx_wait_finish` (§11)

---

## 11. Exit 路径 — tail-call `ll_tx_wait_finish` (offsets 54a-578)

`.L310` (offset 0x54a) — `bleClock_t+12 == 0` 的简化注册路径:
```asm
54a: li   a1, 0
54c: auipc a0, %hi(ll_advertise_process)
554: call TMOS_ProcessRegister              ; TMOS_ProcessRegister(ll_advertise_process, 0)
55c: lbu  a2, 17(s0)                       ; pdu_len -> a2
560: lw   s0, 24(sp)
562: lw   ra, 28(sp)
564: lw   s2, 16(sp)
566: lw   s3, 12(sp)
568: lw   s4, 8(sp)
56a: lw   s5, 4(sp)
56c: mv   a1, s1                           ; pdu_buf -> a1
56e: lw   s1, 20(sp)
570: li   a0, 0
572: add  sp, sp, 32
574: auipc t1, %hi(ll_tx_wait_finish)
578: jr   t1                                ; jr (not jalr) → tail-call
```

**`jr t1` (jump register, no link)** = 真正的 tail-call. ll_advertise_tx 不返回, control 流交给 ll_tx_wait_finish, ra 是上一层 caller 的 ra.

`ll_tx_wait_finish` 参数:
- a0 = 0 (mode?)
- a1 = pdu_buf
- a2 = pdu_len

**注意**: 这是 `.L310` 路径的 epilogue (无 SysRegister, 用 ProcessRegister). `.L311` 路径走完之后是 fallthrough 到正常 epilogue (`.L272` @ 0x24e), **没有 tail-call**, 那个分支直接 ret. 是否调 `ll_tx_wait_finish` 由调度器决定.

---

## 12. `ll_advertise_generated_scan_rsp` 全函数 (56 行, simpler)

```asm
00000000 <ll_advertise_generated_scan_rsp>:
   0: lbu  a5, 30(s0?)  -> wait, the func takes a0 directly, no s0 yet
```

完整反汇编 (re-read for accuracy):

```asm
0:   lbu  a5, 30(a0)                       ; a5 = adv_state[30]  (adv_data_len low byte)
4:   add  sp, sp, -16
6:   sw   s0, 8(sp)
8:   sw   ra, 12(sp)
a:   add  a5, a5, 6                        ; a5 = adv_data_len + 6 (AdvA)
c:   sb   a5, 17(a0)                       ; st->pdu_len = adv_data_len + 6
10:  lw   a5, 76(a0)                       ; a5 = st->pdu_buf
12:  li   a4, 4
14:  sb   a4, 16(a0)                       ; st->pdu_type = 4   ← SCAN_RSP
18:  sb   a4, 0(a5)                        ; pdu_buf[0] = 4     ← PDU header byte 0 = SCAN_RSP
1c:  lw   a5, 76(a0)                       ; reload pdu_buf
1e:  lbu  a4, 17(a0)                       ; pdu_len
22:  mv   s0, a0
24:  sb   a4, 1(a5)                        ; pdu_buf[1] = pdu_len
28:  lbu  a5, 53(a0)                       ; st->byte_53
2c:  andi a5, a5, 1                        ; bit 0 = is_random
2e:  bnez a5, 3a <.L14>
30:  lbu  a4, 52(a0)                       ; st->byte_52
34:  li   a5, 2
36:  bne  a4, a5, 48 <.L15>                ; if byte_52 != 2 && bit0==0: skip TxAdd
3a:  <.L14>: lw  a4, 76(s0)
3c:           lbu a5, 0(a4)
40:           or  a5, a5, 64               ; pdu_buf[0] |= 0x40  ← set TxAdd
44:           sb  a5, 0(a4)
48:  <.L15>: lw  a0, 76(s0)                ; pdu_buf
4a:           add a1, s0, 54                ; &st->adva (offset 54)
4e:           li  a2, 6
50:           add a0, a0, 2                 ; pdu_buf + 2
52:           call tmos_memcpy               ; memcpy(pdu_buf+2, &st->adva, 6)
5a:  lw   a0, 76(s0)                       ; pdu_buf reload
5c:  lhu  a2, 30(s0)                       ; adv_data_len
60:  lw   a1, 44(s0)                       ; scan_rsp_data_ptr
62:  add  a0, a0, 8                        ; pdu_buf + 8 = adv data dst
64:  call tmos_memcpy                       ; memcpy(pdu_buf+8, scan_rsp_data, len)
6c:  li   a5, -109                         ; 0x93
70:  sb   a5, 11(s0)                       ; st->byte_11 = 0x93   ← state stamp = "scan_rsp ready"
74:  lw   ra, 12(sp); lw s0, 8(sp); add sp, sp, 16; ret
```

**总结** (`ll_advertise_generated_scan_rsp` 干什么):
1. 设 `st->pdu_len = adv_data_len + 6` (AdvA + scan-rsp data)
2. 写 PDU header byte 0 = 4 (SCAN_RSP), byte 1 = pdu_len
3. 根据 byte_52/byte_53 决定是否 set TxAdd (pdu_buf[0] |= 0x40)
4. memcpy AdvA → pdu_buf[2..8]
5. memcpy scan-rsp data → pdu_buf[8..8+len]
6. 设 state stamp `st->byte_11 = 0x93` (表示 "scan_rsp buffer prepared")

**注意**: 这个函数**不碰 BB/LLE/RFEND**. 它只是装配 buffer. 实际把 SCAN_RSP 发出去是别的代码 (RX 中断收到 SCAN_REQ 时, 检查 `st->byte_11 == 0x93`, 然后直接让 PHY engine 切到 TX, buffer 已经在 `LLE[112]` 等着).

**TxAdd 决策**:
- `byte_53 & 1`: 主动是 random 地址 → TxAdd = 1
- 或 `byte_52 == 2`: address type 2 (private random?) → TxAdd = 1
- 否则: TxAdd = 0 (public 地址)

---

## 13. ADV PDU type 值映射 (实测 + 协议)

WCH adv_state[16] / PDU header byte 0 低 4 位的实际取值:

| WCH `st->pdu_type` | BLE PDU 类型 | 路径 | Rust enum |
|--------------------|-------------|------|-----------|
| 0 | ADV_IND               | .L302 通用路径 | `AdvInd` |
| 1 | ADV_DIRECT_IND        | .L274 → DIRECT 路径 | `AdvDirectInd` |
| 2 | ADV_NONCONN_IND       | .L302 通用路径 | `AdvNonconnInd` |
| 3 | SCAN_REQ              | (这不是 TX 函数发送的) | RX 接收 |
| 4 | **SCAN_RSP**          | **`ll_advertise_generated_scan_rsp` 独立函数** | `ScanRsp` |
| 5 | CONNECT_IND           | (CONN setup 路径, 不在 ll_advertise_tx) | (LL_INITIATE) |
| 6 | ADV_SCAN_IND          | .L302 通用路径 | `AdvScanInd` |
| 7 | ADV_EXT_IND (AUX_ADV_IND) | .L275/.L276/.L285 EXT 分支 | `AdvExtInd` |

**ADV_IND / ADV_NONCONN_IND / ADV_SCAN_IND 走同一通用路径** — 区别只在 byte_14 低 4 位的值, 装配步骤一样. Rust 实现可以共享一条 codepath.

**SCAN_RSP 是独立函数** — 不经过 ll_advertise_tx, 因为不需要 PHY 重新配置 (RX 已经在 ADV channel).

---

## 14. State stamp 字节 (`adv_state[11]`)

这个字段是 LL 自己的状态机印章, 每个路径写不同值:

| 值 | 十六进制 | 来源路径 | 含义 (推测) |
|----|--------|--------|---------|
| -108 (0xf94? — 实际 0x94) | `0x94` | .L348 默认路径 (ADV_IND / ADV_NONCONN_IND / ADV_SCAN_IND 通用) | "adv TX buffer ready" |
| -110 (0x92) | `0x92` | `.L302` ADV_DIRECT_IND 完成 (`f9200793; b9a9`) | "direct adv buffer ready" |
| -109 (0x93) | `0x93` | `ll_advertise_generated_scan_rsp` 出口 | "scan_rsp buffer ready" |

实际写法 `li a5, -108` → `sb a5, 11(s0)`, 因为 sb 只写低 8 位, -108 = 0xf94 截取到 0x94. 同理 -110 → 0x92, -109 → 0x93.

**含义**: LL 调度器读 `st->byte_11` 来判断当前 buffer 处于哪种 PDU 状态, 决定 RX 回复路径 (例如收到 SCAN_REQ 时, 看到 0x93 才允许直接 PHY 切 TX 回 SCAN_RSP).

---

## 15. Rust LL TX 设计要点 (Iron Rules)

| #  | 规则 | 来源 |
|----|------|------|
| 1  | 寄存器块用 PAC singleton 或 `&'static`, **不要**每个 MMIO 写都 deref 全局指针 | §5 三个 `lw 0(...)` 之后用 saved reg 复用 |
| 2  | BLE Access Address `0x8E89BED6` 是协议常量, 写成 `const`, 不做配置项 | §8 |
| 3  | BLE CRC init `0x555555` 是协议常量, 写成 `const` | §8 |
| 4  | ADV channel index (37/38/39) 编码到 BB[44] bits 25-30, **不是**自然字段, 要 PAC fieldset 或本笔记永久记录 | §8 |
| 5  | PDU buffer 是**外部分配 + DMA**, 不是寄存器内嵌. ptr 写到 `LLE[112]` | §9 |
| 6  | PDU header byte 0 = `[ChSel(1) | TxAdd(1) | RxAdd(1) | PduType(4)] | RFU(1)`. **bit 7 = TxAdd**, **bit 6 = RxAdd**, **bit 5 = ChSel** | §7, §10 |
| 7  | PDU type dispatch 用 `enum AdvPduType`. 默认通用路径覆盖 IND/NONCONN/SCAN, DIRECT 单走 | §13 |
| 8  | PHY 必须 `phy_status_clear(2)` 预热, `BLE_SetPHYTxMode()` kick | §4, §10 |
| 9  | TX-complete handler 通过 `TMOS_SysRegister(handler)` 注册, **不是** ADV-event cb (那是 user 层) | §10, §13 of `ti-wch-mapping.md` |
| 10 | TX 实际启动 = `LLE[0] = 2`, 这是最后一笔 store. 之前所有配置都为它服务 | §10 |
| 11 | Tail-call `ll_tx_wait_finish(0, buf, len)` 用 `jr t1`, Rust 里用 `#[naked]` 或者干脆 inline | §11 |
| 12 | SCAN_RSP 不走 TX 函数, 独立 buffer 装配 (`ll_advertise_generated_scan_rsp`). 状态字 0x93 当握手 | §12, §14 |
| 13 | `st->byte_11` (state stamp) 是 LL 自己的状态机, RX 路径会读这个 byte 决定下一步 | §14 |
| 14 | 入口 `gBleLlPara[168]` busy-guard. Rust 用 `AtomicBool` 或 state-machine 显式建模 | §3 |
| 15 | `bleClock_t+12` flag 决定 fast/slow 注册路径 — 优先实现 `.L311` (slow path), `.L310` 是优化 | §9, §11 |

---

## 16. 已知 unknowns + 后续探查清单

待后续 (Rust 实现碰到再回查):
- [ ] LLE[100] 的具体含义 (写 0xa0 进入, busy-wait 到 0) — 推测 TX-engine 状态
- [ ] LLE[80] = 0x5a (90) — 推测 IFS / pre-buffer timing
- [ ] BB[0] bits 7-8 控制什么 — 推测 RX/TX direction enable
- [ ] BB[0] bit 23 (`| 0x800000` in §10) — 推测 TX-fifo commit
- [ ] BB[44] bits 0-1 mode field — 推测 ADV/DATA/CONN mode
- [ ] RFEND[8] bits 16-21 (`| 0x330000`) — 推测 PA enable + PHY data rate
- [ ] RFEND[44] bit 1 — 推测 PA/LNA 切换
- [ ] EXT_ADV 完整字段装配 (.L276, .L285, .L297, .L298) — Cindy Phase 1 不需要, 缓后
- [ ] `gBleLlPara[216] & 0x4000` ChSel 决策 flag — 与 BLE Channel Selection Algorithm #2 启用对应
- [ ] `gBleIPPara` byte offsets 1/2/3/5/7 含义 — IP-stack global state

不会在这份笔记里 1:1 cross-ref ch32-data fieldset (`blebb_v208.yaml` / `blelle_v208.yaml`) — 这些 YAML 还没 commit 到 ch32-data 仓库 (`find ~/Repos -name "blebb_v208.yaml"` empty 2026-05-12). Cindy 的 task #68 Phase 1 落地 ADV 时如果建了 fieldset, 那边可以反向引用回这份笔记.

---

## 17. Cross-reference

- `notes/ch32-rs/ti-ble/ti-wch-mapping.md` §13 — `LL_*EventRegister` stub 反汇编 + `ll_advertise_event_closed` user-cb 路径 (这份笔记的姊妹文档, 关注 user-cb side; 本文件关注 hardware TX side)
- `notes/ch32-rs/ti-ble/peripheral-flow-deep-dive.md` (Cindy 写) — peripheral 例子全流程 (本文件是其中一个 focused doc, 见 Cindy 末尾 5 个 focused docs 列表)
- `notes/ch32-rs/wch-ble-manual.md` — WCH 中文低功耗蓝牙手册 v1.9 pdftotext (TMOS / BLE timing / pfnGetSysClock 等上层契约)
- `notes/ch32-rs/c-ground-truth-fnGetClockCBs.md` — clock callback empirical proof (跟这份关系不大, 但都是 .a 反汇编系列)
- `/tmp/wchble-disasm/full.dump` — 完整 ll_advertise.o / ll_connect.o / ll.o objdump
- `/tmp/wchble-disasm/ll_advertise_tx.dump` — 单函数 645 行
- `/tmp/wchble-disasm/ll_advertise_generated_scan_rsp.dump` — 单函数 56 行

Bluetooth Core Spec 6.0 Vol 6 Part B 引用:
- §2.1 Packet Format → Access Address `0x8E89BED6` for adv channels
- §3.1.1 CRC → init value `0x555555`
- §2.3 Advertising Channel PDU → PDU type 0-7 enumeration
- §2.3.4.5 Common Extended Advertising Payload Format → EXT_ADV header (本文 §7.1 跳过详解)

---

**作者**: Lucy (@Lucy)
**日期**: 2026-05-12 ~23:xx
**状态**: 初稿, 等 Cindy review + 加 reverse-reference 回 `peripheral-flow-deep-dive.md`

---

## Provenance

- **Origin**: `/Users/mono/.slock/agents/997bdbdb-791a-40d4-a4f5-25dc90f2ed08/notes/ch32-rs/ti-ble/ll-advertise-tx-disasm.md` (Lucy agent workspace).
- **Migrated**: 2026-05-18, task #90 (`#ch32-rs:decce4f6`), branch `lucy/disasm-migration`.
- **Responsible agent**: Lucy.
- **Boundary**: behavioral reference only. Do **not** import `libwchble` symbols or call ROM helpers as a production fallback in the Rust/Embassy stack (see `embedded-c-ble-stack-architecture.md` §2.5 and §1.3).
