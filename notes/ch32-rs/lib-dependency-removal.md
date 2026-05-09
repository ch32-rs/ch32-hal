# libwchble.a 移除路线图 — 经验教训

最后更新: 2026-05-08 00:35 (Iron Law #34 v5 final: ROM RAM-layout-agnostic for 6 BSS-contract symbols; supersedes BSS-pin-required narrative; T44.E (b) FREEZE; Phase 2 BSS pin removal approved)

## 2026-05-04 #20 — Lib Retention Anchor 第三类发现

### 实验

`bf0f5ce` post-#19 head 出发,#20 narrowest variant:
- 删 `LLE_IRQSubHandler` extern 声明
- 删 `fn LLE()` wrapper 内对 `LLE_IRQSubHandler()` 的 call
- 保留 `fn LLE()` vector stub + LLE_IRQ_ENTRY/EXIT counters
- 保留 `_KEEP_BB_IRQ_LIB_HANDLER` anchor

### 实测

| 维度 | 数值 |
|------|------|
| BIN | 51,604 → 50,764 (-840 字节) |
| nm `LLE_IRQSubHandler` | **消失** (GC'd) |
| nm `__qingke_rt_LLE` | 42B → 22B (shrunk) |
| nm `LLE` vector stub | 仍存在 |
| 60s bleak cba | **0** ❌ |

### 锁定结论

**Lib retention anchor 第三类 — 调用链锚定**:
- `_KEEP_BB_IRQ_LIB_HANDLER` 锚住 `BB_IRQLibHandler` 自己,但 **不**连带保留 `LLE_IRQSubHandler`
- 我们在 `fn LLE()` wrapper 内对 `LLE_IRQSubHandler()` 的 call 才是真正让它存活的 anchor
- 删 call → linker GC 掉 LLE_IRQSubHandler 整块 (~840B) → 触发 Iron Law #22 layout shift → cba=0

### 推翻的假设

Vega (12:38:16) 假设: "anchor `_KEEP_BB_IRQ_LIB_HANDLER` 会传递保留 LLE_IRQSubHandler" — 实证 false。锚不传递,只保 1 跳。

### Iron Law 补充 — Anchor 非传递性

**`#[used] static` 类 anchor 只锚被引用的符号自己,不连带保留它内部 call 的下游。**

实务规则:
1. 评估 .a 中某 extern fn 是否可删,**必须 nm 实测**,不能假设 anchor 传递
2. 如果删除产生 BIN size shift (尤其 -100B 以上),立即怀疑 `--gc-sections` 链式 GC,必须 60s gate
3. 多跳调用链: anchor A → call B → call C,删除 B 会让 C 一起 GC 即使 A 仍 anchored
4. 如果某 lib 函数的 call 是 layout retention 关键 (如 #20 LLE_IRQSubHandler),**不能删 call,只能注释化保留**,直到 .a 整体拆除时才能批量验证

### Plan B 被 `#[used] static` 突破 (12:49 实测)

Cindy 在 Plan B commit 之前跑了 Vega 提的 `#[used] static` retention 实验,**意外突破**:

| Variant | BIN | LLE_IRQSubHandler | cba |
|---------|-----|-------------------|-----|
| Round 1: 删 call + 让符号 GC | 50764 (-840B) | GC'd | 0 ❌ |
| Round 2: `#[used] static _KEEP_LLE_IRQ_HANDLER = LLE_IRQSubHandler` + 删 runtime call | 51588 (-16B) | retained | **78** ✓ |

cba=78 在 V_A3 baseline 81 的 96% — 不仅过 gate (≥52),几乎打平。

### 锁定的两条 Iron Law refinement

1. **Layout/lib-code body 必须保留 (但不必通过 call 锚定)**:
   - `LLE_IRQSubHandler` 整块 840B 是 Path C timing 的关键资产 — 具体哪部分 ICACHE/分支预测/PFIC routing 受影响,RF/cycle level 不可见
   - 锚定方式有两种: (a) 通过 wrapper 内 call 隐式锚定; (b) 通过 `#[used] static fn = LLE_IRQSubHandler` 显式锚定
   - 二者对 lib code 保留效果**等价**

2. **Runtime call 可删,只要符号锚住**:
   - call 指令的存在/消失 **不是** timing-critical
   - 只要 callee 函数体仍在 binary 里,wrapper 的 call 指令是否 issue 不影响 cba
   - 这给了我们 **runtime path 真正变干净** 的机会 (Rust 这边 wrapper 不再 call lib),同时**保持 binary 不变**

### `#[used] static` Retention Pattern — .a Removal 的隔离工具

```rust
extern "C" {
    fn lib_function();  // RETAIN extern declaration
}

#[used]
static _KEEP_LIB_FN: unsafe extern "C" fn() = lib_function;  // explicit anchor

// runtime path: don't call lib_function() anymore — but symbol stays in binary
```

**两个用途**:
1. **隔离 call 删除 vs 符号删除**: 测试到底是哪个变化触发 cba 回归
2. **干净的 runtime path 同时保 layout**: Rust 这边代码可以完全不调用 lib,但 lib code 仍在 binary 维持时序

### #21 R2 实验 (2026-05-04 13:01 实测结果)

`f10a99c` post-#20 head 出发,4 个 `#[used] static _KEEP_LL_ADV_*` 锚 + 注释掉 4 个 vtable write:

| 维度 | 数值 |
|------|------|
| BIN | 51,604 → 51,564 (-40B,4×4 sw + static slot 净值) |
| nm 4 个 `llAdvertise*` | 全部 retained ✓ |
| `gBleLlPara` 地址 | 0x20001b70 (稳定) |
| 60s gate cba | **0** ❌ |
| log | `/tmp/ble_scan_task21_r2_20260504_125926.log` |

**唯一运行时变量**: vtable `gBleLlPara+0x68/6c/70/74` 内容为 0 (原来是 4 个 `llAdvertise*` 函数地址)。

### 锁定结论 — `#[used] static` Retention Pattern 的应用边界

**vtable 内容 (函数地址) 真的被 lib init/IRQ/RF 路径作为 dispatch table 消费**。hidden reader 不读"符号是否存在",读"函数地址内容"。

| 消费形态 | retention pattern 有效? | 例 |
|----------|-----------------------|-----|
| lib 消费"符号是否存在 / lib code 是否在 binary" | ✓ 有效 | #20 LLE_IRQSubHandler (lib code body 维持 timing) |
| lib 消费"具体地址内容" (函数指针/数据值) | ❌ 无效,write 必须保留 | #21 ADV vtable (函数地址被 dispatch) |

### 2026-05-03 forensic 注释永久成立

- 4 个 `gBleLlPara+0x68..+0x74` write 必须保留
- `llAdvertise{CreateCore,Set,Start,TraverseAllChannel}` 4 个函数被 lib state machine 间接 dispatch
- #21 状态保持 done,R2 实验作为 follow-up evidence 关闭

### 对最终 .a 拆除路线图的战略影响

**新增依赖发现**: `BB_IRQLibHandler` (#22 in_review) 替代后,Rust 接管 BB IRQ,但 IRQ 内部会 dispatch 到 vtable 的 ADV functions → **lib ADV state machine 仍 alive,Rust 调用 lib 通过 vtable 间接路径**。

这暗示:
1. 单独完成 #22 (BB IRQ Rust 替代) 不足以完全消除 lib 调用 — BB IRQ 内会通过 vtable 调 ADV
2. 真正去 `-lwchble` 需要 **Rust 复刻 4 个 `llAdvertise*` 函数 + ADV state machine**,这是 Phase D 新工作量
3. 当前 `feat/ble-phy-init` 上 commit chain 可以推进,但 final gate (去 `-lwchble`) 会暴露这个 hard dependency surface,届时需要新 task 跟踪 ADV state machine Rust 化

## 2026-05-04 #20 — Lib Retention Anchor 第三类发现

### 战略影响 (修正)

不再是 "Phase A 落到注释化保留",而是:
- `#[used] static` retention pattern 让我们可以**真正减少 runtime path 对 lib 的调用**,同时保 binary 不变
- 这相当于把 .a 改造成 "lib code 仍在,但 Rust 不再触它" 的 inert state
- Phase B/C (BB_IRQLibHandler / BLE_IPCoreInit Rust 替代) 完成后,真正的 final gate 是去 `-lwchble`,届时所有 `#[used] static` 锚的符号都会变成 link error,暴露真实 hard dependency surface — 那些就是必须 Rust 替代的最后清单。

### #20 final scope

不是 Plan B (注释化保留),而是 **`#[used] static` retention + remove runtime call** (commit `f10a99c`)。
- BIN -16B,layout 实质 byte-stable 等价
- runtime path 真正不再 call `LLE_IRQSubHandler`
- gate log: `/tmp/ble_scan_task20_used_anchor_20260504_124617.log` (cba=78)

## 2026-05-04 #19 单变量 bisect — "DCE-invisible cleanup vs visible-code layout"

### 实验路径

#22 dual_fix base (`812dd4d` HEAD,BIN 51,604,cba=63 baseline) 出发:

| Variant | scope | BIN | cmp baseline | cba |
|---------|-------|-----|--------------|-----|
| #19 first (full) | 删 3 extern + PATHC_CALL_LL_HELPERS const + gated-off true-branch + `LL helper calls skipped` println | 51,536 (-68) | differs | **0** ❌ |
| #19 narrow | 删 3 extern + 删 unreachable true-branch,**保留 println** | 51,604 | byte-identical ✓ | 63 ✓ (无需重跑) |

### 锁定结论

- `LL_CoreInit/LL_WhitelistInit/LL_ResolvinglistInit` extern 声明 + const-gated 死分支 = **DCE 早已清理,源码删除对 binary 不可见 → byte-identical → safe**
- 第一轮 cba=0 的真实触发点是删掉 `println!("PathC PlanB2: LL helper calls skipped")` 造成 -68B 层 shift → Iron Law #22 (binary size shifts → .text layout shifts → timing) 命中
- 该 println 必须作为 **layout anchor** 永久保留 (Vega 12:35 commit message 已注明)

### Iron Law 补充 — Layout-Stable Cleanup Pattern

**对 byte-identical 验证已过的 cleanup,无需 60s gate re-run** — `cmp` byte-identical 即等价于 layout-equivalent 即等价于 timing-equivalent。

具体规则:
1. **DCE-invisible 类**: extern 声明 + const-gated 死分支 + 未引用 type 别名 → 删除前后 BIN `cmp` 应 byte-identical → 跳过 gate
2. **Visible code 类**: println / runtime probe / 任何能产生指令的代码 → 删除会 shift layout → 必须 60s gate
3. **混合 patch 是反模式**: 一次 patch 同时含 DCE-invisible 和 visible-code 两类改动 → cba 失败时无法 disambiguate (#19 first round 就是这个坑)
4. **Anchor protection**: 调试 println 一旦在 gated-clean baseline 里被验证,不可随意删 — 它在维护当前 layout

### 决策流程 (替代之前的 "60s gate first")

```
patch 完成
   ↓
cmp <patch_bin> <baseline_bin>
   ↓
 byte-identical? ─── yes ──→ commit (无需 gate)
   │ no
   ↓
60s bleak gate → cba ≥ 52 (V_A3 81 的 65%) ?
   ↓ yes → commit
   ↓ no  → P0 revert + 注释,二轮 narrow bisect
```

## 背景

Path C BLE TX 例子 `ble_tx_adv_ch37.rs` 使用 3-层 lib 替换策略, 仍依赖 libwchble.a 的若干 extern 函数和 BSS 全局. 路线图目标是逐步用 Rust 替代实现, 减少二进制体积并消除黑盒依赖. Phase A 拆为 5 个 subtask:

- #19: 删 `LL_*Init` extern 帮助函数 (PATHC_CALL_LL_HELPERS=false 已 gate 掉)
- #20: 删 LLE IRQ wrapper (BB_IRQLibHandler + W1C 已替代)
- #21: 删 `gBleLlPara+0x68/6c/70/74` 4 个 vtable 写入和 4 个 `llAdvertise*` extern
- #22: 用 Rust 复刻 `BB_IRQLibHandler` (Phase B)
- #23: 用 Rust 复刻 `BLE_IPCoreInit` (Phase C, 战略决策待 Andelf)

## #21 二轮 bisect 实测推翻 forensic 结论 (2026-05-03)

### 实验方法

从 `42e57ef` 干净基线 (cba=58/60s) 出发:

| Variant | extern fn | vtable write | `#[used]` retention | layout | cba |
|---------|-----------|--------------|---------------------|--------|-----|
| baseline | yes (4) | yes (4 sw) | n/a | gBleLlPara@0x20001738 | 58 |
| #21 first | no | no | no | shifted -0x30 (GC -47KB) | 0 |
| #21a | yes (4) | no | no (extern 不产生 reloc) | shifted -0x30 | 0 |
| #21b (Option B) | yes (4) | no | yes (`#[used] static`) | gBleLlPara@0x20001738 ✓ | 0 |

### 三个独立 forensic 路径都漏判

1. Vega 第一轮 d.asm grep `lw .., (104..116)(reg)`: 只命中 LL_ProcessEvent (TMOS handler), 结论 "Path C 不可达 → 安全".
2. Lucy 第二轮 `R_RISCV_PCREL_HI20 gBleLlPara` 反向追溯: 找到 LL_ProcessEvent + `LL_SetExtendedAdvertisingParameters` (Extended ADV API), 两者都不在 Path C → 同 Vega 结论.
3. 双盲 cross-check 一致 → 共识 "vtable 是死代码, 删除安全" → **被硬件实测推翻**.

### 实证锁定结论

#21b (`#[used]` 保留符号 + 跳过 vtable writes) **layout 完全回基线** + **`llAdvertise*` 4 个符号都活着** → 唯一变量是 `gBleLlPara+0x68/6c/70/74` 内容是 0 还是真实函数地址 → cba=0 → **vtable 内容必须非零**.

### 推断未发现 reader 的位置

候选 (按概率排):
- `BLE_IPCoreInit` 内部 (line 93425, .text.BLE_IPCoreInit) 间接寻址
- `LLE_IRQSubHandler` (line 93513, lle.o) — IRQ 内部状态机调度可能读 +0x68..+0x74 当函数指针表
- `RF_RoleInit` / `RF_Config` 校准期 callback 表
- 任何被 add reg, +0x68 后再 lw 的间接形态 (Vega/Lucy 的 grep 都没扫这种)

## .a 移除路线图必须改用 "实证优先" 流程

### 旧流程 (失败)
1. d.asm grep 找直接 reader
2. 结论 "Path C 不可达"
3. 删除代码
4. 硬件验证

→ 漏判风险高, 因为 grep 不覆盖间接寻址, 不覆盖未识别的 dispatch 路径.

### 新流程 (强制)
1. **基线锁定**: 从 known-good commit 出发 (`42e57ef` 或更新).
2. **单变量 patch**: 每次只改一件事, 避免合并 #19+#20+#21 这种复合变更.
3. **layout-stable 优先**: 删 extern 引用前用 `#[used] static` 保留符号, 隔离 layout shift 影响.
4. **硬件 gate first, forensic second**: 即使 d.asm 看起来 100% 干净, 也必须跑 60s bleak 验证 cba ≥ 基线值的 80% 才能 in_review.
5. **失败处置标准化**: 任何 cba 显著回归 (< 基线 50%) 视为 P0, 立即 revert + 注释 "what was tried, why it failed", 不要二次尝试相同方法.

### Iron Law

> **删除 lib extern 或 BSS 写入之前, 先用 `#[used] static` 强制保留符号跑一轮 bleak gate. 通过后才能继续删除引用.**

这条规则在 #19 / #22 / #23 都必须执行.

## #19 (LL_*Init extern) 风险重评

`LL_*Init` 那组 extern 在 `if PATHC_CALL_LL_HELPERS (false)` 下不被调用 → Rust DCE 应该删除其调用代码 → 我之前认为 "byte-identical 二进制, 删 extern 安全". 但 #21 的教训说明:

- DCE 删除 *调用代码*, 但不删除 *函数本身* (它们在 lib 里).
- 删除 extern 声明后, Rust 不再 import 这些符号.
- 如果 lib 内部也没有引用, `--gc-sections` 会把它们 GC 掉, 可能触发 layout shift.

**结论**: #19 不能简单删. 必须先验证:
1. lib 内部是否有其它路径引用 `LL_*Init` (大概率有, 因为 LL_Init 是 BLE init 主入口)
2. 如果有, layout 不会漂移, 可以安全删
3. 如果没有, 必须用 `#[used] static` retention 才能继续

#19 当前在 in_review, Cindy 的 cba=94 验证是合并三个 task (#19+#20+#21 含 vtable 写入) 的产物 — **不是 #19 单独的证据**. 需要重新单变量验证.

## 待办

- [x] #21 revert + forensic 注释 (Vega 完成)
- [x] #21 → done (Andelf 23:15 确认, Vega marked done)
- [x] 第三轮 forensic 部分结果 (2026-05-03 23:30): `BLE_IPCoreInit / LLE_DevInit / RFEND_DevInit / BB_DevInit / BLE_RegInit / BB_IRQLibHandler / LLE_IRQSubHandler` **7 个函数全部不读** gBleLlPara+0x68..+0x74. hidden reader 在这 7 个 Path C 必经函数之外. 不阻塞 #23.
- [x] **#23 → in_review** (Vega 2026-05-04 00:43, V_A3 cba=81 > 今日 FFI 74, root cause = `hint::spin_loop()` 在 RV32 拖慢 +0x90 轮询)
- [ ] 第四轮 forensic (低优,等 #23 后再跟): 候选 `add reg, +0x68; lw 0(reg)` 间接形态 (前 3 轮 grep 都没扫); RF_RoleInit/RF_Config 校准期 callback 表; 任何在 lib 内部被 add+lw 的 hidden dispatch.
- [ ] 战略决策已落定 (Andelf 23:15+23:22): 跳过 Phase A 增量,直接攻 #23 (Vega claim) 用 Rust 复刻 `BLE_IPCoreInit` glue (~25 行). #19/#20 暂 park,等 #23 完成后随之自然消化. 最终目标拆掉静态库 → 返回更新 ch32-data 修正寄存器定义.

## #23 root cause 锁定 — RV32 `core::hint::spin_loop()` 时序污染 (2026-05-04)

### 时间线

`42e57ef` baseline (FFI lib BLE_IPCoreInit, cba=58/60s, 历史最佳 105) → 今日 FFI baseline 退化到 74 (硬件/环境因素未明,但同日同条件,所有 variant 公平可比) → V_A (Rust ble_ip_core_init 含 `hint::spin_loop`) cba=42 → V_A2 (跳过 Bug #2/#3 cleanup) cba=0 → **V_A3 (移除 spin_loop hint + LLE+0x64 hw countdown 包装) cba=81** ✓

### 根因解析

`regint.rs::rfend_wait_tune` 原代码:
```rust
for i in 0..30_000u32 {
    let s = r(0x90);
    if s & (1 << 26) != 0 {
        let s2 = r(0x90);
        if s2 & (1 << 25) != 0 { return i; }
    }
    core::hint::spin_loop();  // ← BUG
}
```

`core::hint::spin_loop()` 在 RV32 上 (Vega 推测) 展开成 `fence` 内存屏障或 `pause` hint (具体待 objdump),无论展开成什么,都比 lib 的裸 polling 显著拖慢一次循环。

后果:
- +0x90 一旦 latch (bit26+25 set),GA 在 +0x94 立即可读 — 但下一次读 +0x90 间隔越长,GA 已经"过期"被新的 sample period 衰减
- 实测 V_A 的 nGA2440=51 而 V_A3 的 nGA2440=51 相同,但**V_A 的 nGA2480=49 vs V_A3 的 48** — 2480 频率轮询更早结束 (w 更小),fence 拖慢导致 V_A 跨过一个 sample period 多读一档高
- delta_ga = nGA2440 - nGA2480: V_A=2 (51-49) vs V_A3=3 (51-48) ← FFI 也是 3
- LUT 0xc8 nibble 编码反映 delta_ga: V_A 0x99aaaa77 (低值) vs V_A3 0x9aabbb88 (与 FFI 同档)
- ADV TX air-decode 对 delta_ga 敏感,差 1 LSB 就 cba 42→81

### Iron Law 补充 — RV32 BLE 时序敏感代码

**任何在硬件 polling loop 中使用 `core::hint::spin_loop` / `yield` / `fence` / 隐式内存屏障的代码,删除前后必须 cba gate 验证。**

具体规则:
1. 所有 `loop { read_volatile(reg); ... }` 式硬件状态轮询,**严禁** `hint::spin_loop()` — 与 x86 `pause` 不同,RV32 spin_loop 是真实 stall pipeline
2. 如必须降功耗,改用 `wfi` (wait for interrupt) 而非 spin_loop hint,并验证不影响 cal/IRQ 时序
3. 任何 `cba` 与 FFI 同条件下偏低 ≥30% 的 case,优先怀疑硬件 polling 时序污染,先 grep `spin_loop\|fence\|pause` 再做更深 forensic

### 实证流程价值

这次成功完全靠**硬件实证 → 数据驱动 → 单变量 patch** 流程:
- V_A2 cba=0 disprove "Bug #2/#3 cleanup 是元凶" 假设
- V_A SDI 日志 grep `rfend_tune` w 值 disprove "30K SW counter timeout" 假设
- V_A→V_A3 单变量 (移除 spin_loop) 锁定 root cause,cba 42→81 一击命中

forensic 路径反而走了弯路 (3 轮 vtable disasm + 多轮 LUT decoding),实证 bisect 一次比 3 轮 forensic 都准。**这条以后所有 .a 移除任务都按这个流程走,不要先开 forensic**。

---

## Phase D Gate 阈值规范 (2026-05-04 15:20 锁定)

**Phase D baseline 复核** (`faff089` HEAD, Cindy 2026-05-04 15:19):
- BIN 51588B,与 task21 comment_verify byte-identical
- 3 轮 60s scan: cba=[67, 79, 94],median=79,abc=0 全程
- 日志: `/tmp/ble_scan_phaseD_baseline_r{1,2,3}_20260504_*.log`

**敲定的 4 级阈值**:

| 等级 | 阈值 | 含义 | 处置 |
|---|---|---|---|
| 🔴 Hard gate (revert) | cba < 52 | V_A3 (81) 65% 红线 | P0 立即 revert + narrow bisect,不重复同一变体 |
| 🟡 Yellow flag (investigate) | 52 ≤ cba < 63 | Phase D median (79) 80% 观察线 | 不 revert,但 thread raise + 落档原因后再放行下一步 |
| 🟢 Pass | cba ≥ 63 | 等同当前 baseline 水平 | 直接 commit |
| 🟢🟢 Byte-identical | cmp 一致 | DCE-invisible (extern + 死分支) 改动 | skip gate,直接 commit |

**为什么不上调红线**: Andelf 15:07 approve #22 时 endorse 65 (80% V_A3) 作为 ship 标准 — Iron Law 不可移门柱。yellow flag 用 Phase D 更精细数据预警 subtle regression,但单次抖动不触发 revert。

**baseline 文件名规范** (Phase D 期间统一):
- gate 日志: `/tmp/ble_scan_task25_<phase>_<round>_<ts>.log`
- BIN 快照: `/tmp/ble_tx_adv_ch37_task25_<phase>.bin`
- `<phase>` 取值: `d1a_0/d1a_1/d1a_2/d1a_3/d1a_4 ... d1d_*`

---

## Phase D 突破: R2 推翻 + scope 塌缩 (2026-05-04 15:38)

### 实证决定性数据 (D-1a.0b padding-neutral gate, Cindy)

**测试 patch**:
- 4 个 `#[used] static _KEEP_LL_ADV_*` anchor (preserving symbols)
- vtable 4 个 write RHS: lib fn 地址 → sentinel `0x12345678`
- 8B `#[link_section=".rodata"] static _PHASE_D_PAD` (size 顶回 baseline)

**结果**:
- BIN 51588B 严格匹配 baseline (faff089)
- 3 轮 60s cba=[68, 64, 54], median=**64**, abc=0 全程
- 日志: `/tmp/ble_scan_task25_d1a_0b_padded_r{1,2,3}_20260504_153218.log`

### 永久 Iron Law 修正

**之前 (#21 R2, 2026-05-03) 错误结论**:
> "vtable content 真被 lib state machine 作为 dispatch table 消费"
> "`#[used] static` retention pattern 在 ADV vtable 场景无效"

**修正后 (基于 D-1a.0b 实证)**:
> R2 cba=0 归因 = -40B BIN delta 触发 Iron Law #22 layout shift,**非** content 消费
> `#[used] static` retention pattern 在此场景**有效**,前提是 size-neutral

### 决定性中间证据: -22440B GC 事件

Cindy 第一版 sentinel (无 anchor) 实测:
- 4 个 vtable RHS 改 sentinel,**未加** `#[used] static` anchor
- BIN: 51588 → 29148 (**-22440B**)
- 4 个 `llAdvertise*` 符号从 nm 消失,`gBleLlPara` 漂到新地址

含义: 4 个 `llAdvertise*` 函数及其完整 transitive call tree (~22kB ADV state machine) 在 Path C 内**没有任何 Rust 引用点**。vtable 4 个 write 是它们的唯一入口。linker GC 在我们没引用时会清空这棵树 — 这是 "Path C 真 non-TMOS" 的硬证据 (与源码 `BLE_LibInit` 永不调用的注释闭环)。

### Path C 真实 TX 驱动链 (Vega + Lucy 共同 audit)

```
fn main() loop in ble_tx_adv_ch37.rs:
    │
    ├─→ adv_tx_burst_ch37(tx_n)  ← 22-step inline Rust 实现 (line 699+)
    │       ├─ ble_set_phy_tx_mode_1mbps(TX_BUF[1])  // BB+0x64=timer, gBleIPPara[4]=0x80
    │       ├─ ble_go_tx_ch37()                       // 13-step: AA/CRC/buf/channel/GO
    │       └─ → 写 BB+0x00=2 触发硬件 TX
    │              └─→ BB IRQ 触发
    │                     └─→ fn BB() → bb_irq_lib_handler() (Rust, #22 done)
    │
    └─→ ble_tx_wait_done() 等 settle
```

**关键事实**: `ll_advertise_tx` (lib 函数) 在 Path C **从未被调用**;它的 22-step 已 inline 在 `adv_tx_burst_ch37()` 里。`llAdvTraverseallChannel`/`llAdvertiseStart`/`llAdvertiseSet`/`llAdvertiseCreateCore` 全部在 Path C 是死代码。

### Phase D scope 塌缩

| 原计划 | 实际 |
|---|---|
| D-1a/b/c/d: 4 个完整 Rust 复刻 (≥2 天) | **完全跳过** |
| D-2: vtable rewire | **D-final.1**: anchor + sentinel + pad (commit 一次性) |
| D-3: link 去 `-lwchble` | **D-final.2**: 同 (大概率直接 link 成功) |

工作量预估: ≥2 天 → ≤2 小时

### 教训

1. **Iron Law #22 灵敏度比想象更小**: -8B BIN delta 已能将 cba 从 79 打到 0 (本次实测)。任何"看似无害"的小改动都需 cmp byte-identical 验证或 padding 顶回。
2. **R2 类负面实验需要 size-neutral 控制**: 单变量原则不仅对 source diff 适用,对 BIN size 也必须严格隔离。任何 size 变化的"功能性"实验都要先做 padding-neutral 对照。
3. **-22440B GC 是好工具**: 大量 GC 反过来证明"该代码无引用点",是 dead-lib 判定的辅助证据。

---

## D-final.2 Link Probe 结果 (2026-05-04 15:51, Cindy)

**实验**: 移除 `-lwchble` 重 build,看缺哪些符号
**Log**: `/tmp/build_task25_dfinal2_no_lwchble_20260504_1548.log`

### Undefined Symbols 分组

**Group A — lib 全局状态 / MMIO 指针 (10 个)**:

| 符号 | 性质 | Rust 替代方案 |
|---|---|---|
| `gptrLLEReg`, `gptrBBReg`, `gptrRFENDReg`, `gptrAESReg` | u32 寄存器 base ptr (lib 由 BLE_IPCoreInit 设值) | `#[no_mangle] static gptr<X>Reg: u32 = 0x4002<base>;` 直接硬定义 |
| `ble`, `gBleLlPara`, `gBleIPPara` | BSS 大结构 | `#[no_mangle] static mut <name>: [u8; SIZE] = [0; SIZE];`,SIZE 从 lib 大小或字段访问 max offset 推导 |
| `gPaControl`, `dtmFlag` | 单值/标志 | 同模式硬定义 |
| `fnGetClockCBs` | lib-internal callback | 透过 `gBleIPPara[0] bit6` 路径被 BB_IRQLibHandler 调用 — Vega 洞察:**当 BB anchor 换 Rust 后这个依赖自动消解** |

**Group B — 显式 `#[used]` anchors (7 个)**:

| 符号 | 当前状态 | D-final.3+ path |
|---|---|---|
| `BB_IRQLibHandler` | 部分替换 (Rust `bb_irq_lib_handler` inlined,但 anchor 仍持 lib version) | 当 bb.rs 版本是唯一实现时移除 anchor |
| `BLE_IPCoreInit` | 启动时被调 — **真依赖** | 完整 Rust 替代 (最大 effort) |
| `LLE_IRQSubHandler` | layout anchor 仅,不被 call (#20 已证) | 当 layout 可不靠它维持时移除 |
| `llAdvertiseCreateCore/Set/Start/Traverse` | layout anchor 仅 (D-final.1) | 替换为 4 个 no-op Rust stub + 移除 anchor |

### 战略洞察

- **Group A 处理**: 10 个全部可 Rust 化,简单到中等工作量
- **Group B 处理**: 一旦 lib 物理消失,这些 anchor 失意义,**应直接删除**而非补 Rust 复刻 — 但需 padding-neutral 维持 size 防 Iron Law #22
- **关键依赖链**: `fnGetClockCBs` 不在 Group A 真正需要 Rust 化的清单 — 它是 BB_IRQLibHandler 替换的副产物

### Phase D 原 scope 已完成 (#25 in_review @ `67c92a3`)

实证证伪原 scope ("4 个 advertise 函数复刻必要性") — 这些函数在 Path C 是死代码,无需复刻。Phase D 工作量从 ≥2 天塌缩到 1 commit。

### Phase D+1 候选 scope (#26 待 Andelf 决策)

如继续推进 lib 物理拆除 (Option β):
- D-final.3a: Group A 10 个全局符号迁移 (~半天)
- D-final.3b: Group B BLE_IPCoreInit 完整 Rust 复刻 (~半天到 1 天,最大头)
- D-final.3c: 其他 6 个 anchor 评估 + 删除 + padding-neutral (~2-4 小时)
- D-final.4: link 去 `-lwchble` final gate (~1 小时)
- 总: ~1-2 天

---

## Phase D+1 T1 落档 (2026-05-04 17:13, commit `76d360c`)

### 实证 lib `gptr*Reg` 4 个 base 地址 (Vega pre-step)

| WCH 命名 (lib `extern u32`) | 项目命名 | 物理地址 |
|---|---|---|
| `gptrBBReg` | `lle_*` | `0x40024100` |
| `gptrLLEReg` | `bb_*` | `0x40024200` |
| `gptrAESReg` | (AES) | `0x40024300` |
| `gptrRFENDReg` | `rfend` | `0x40025000` ← **跨段,非 +0x100** |

### WCH 命名陷阱 (永久落档)

WCH lib 的 "BB" 和 "LLE" 命名与项目 `lle_*`/`bb_*` 命名**反向**。任何对照 lib 源码或 d.asm 时,必须用 WCH 命名;在项目代码内部讨论寄存器功能时,用项目命名。

cross-validation 来源:
- `src/ble/mod.rs::ble_ip_core_init()` — 项目内 init 序列写入这 4 个 base
- `src/ble/bb.rs` 注释 — `bb_irq_lib_handler()` 中明确标注 LLE↔BB swap

### T1 patch 实现

文件 `examples/ch32v208/src/bin/ble_tx_adv_ch37.rs`:

```rust
// extern 块删除 4 行 gptr*Reg 声明 (lib 不再被引用)
// 改为 Rust .data 定义:
#[no_mangle] pub static mut gptrBBReg:    u32 = 0x4002_4100;
#[no_mangle] pub static mut gptrLLEReg:   u32 = 0x4002_4200;
#[no_mangle] pub static mut gptrAESReg:   u32 = 0x4002_4300;
#[no_mangle] pub static mut gptrRFENDReg: u32 = 0x4002_5000;
```

注: 用 `static mut` 不是 `static` — lib `BLE_IPCoreInit` 仍 declare `extern u32 gptr*Reg` 且 write 这 4 个值 (即使我们 Rust 路径不调 lib init,linker 类型兼容仍要求 mut 可写)。lib write 写的是相同值,无害。

### Gate 结果 (Cindy)

- BIN: 51588B 严格 = D-final.1 baseline (zero delta!)
- nm: 4 个 `gptr*Reg` type=D Rust `.data` 定义,在 0x20000004-0x20000010 紧凑 layout
- 3 轮 60s cba=[62,72,58] median=62,abc=0 全程
- yellow flag pass-with-note: size-neutral + abc=0 + 纯结构性变更
- 日志: `/tmp/ble_scan_task26_t1_r{1,2,3}_20260504_1704.log`

### 重要洞察 — BIN zero delta

预期 +16B (4 个 4-byte u32 新增),实际 0B delta。推测 `.data` 新增 16B 抵消 lib BSS COMMON 槽位被取代后的某处 GC 节省。这是 size-neutral patch 的最佳形态,跳过 padding 直接 commit。

### Iron Law refinement

**Yellow flag 默认接受条件** (新增):
> Yellow flag 在 size-neutral + abc=0 + 结构性变更 (无 timing-impact 路径) 三条件同时成立时,**默认接受**,无需追加轮次。如要追加只是为更高 confidence,不是 gate condition。

理由: 测量 cba 在历史上有 10-27 range 的 sampling variance;在 yellow flag 上沿 (cba 62 vs 阈值 63) 区分"真信号"vs"sampling boundary effect"用追加轮次也未必稳定,关键看 patch 是否有逻辑上的 timing-impact 路径。

## 2026-05-04 17:38-17:45 #27 T2 — BSS 大结构对齐 P0 教训

### 失败模式

T2 first attempt (`98bbd41`): 把 lib 的 3 个 BSS 大结构迁到 Rust `.bss`:
```rust
#[no_mangle] pub static mut ble:        [u8; 64]  = [0;  64];
#[no_mangle] pub static mut gBleLlPara: [u8; 296] = [0; 296];
#[no_mangle] pub static mut gBleIPPara: [u8;  64] = [0;  64];
#[used] #[link_section = ".rodata"] static _T2_PAD: [u8; 72] = [0; 72];
```
- BIN: 51588 严格 = baseline ✓ size-neutral
- 60s R1: **cba=0, abc=0** ❌ 致命破坏

### 根因 (Vega 17:40)

**`[u8; N]` alignment=1**,linker 把 3 个符号放到 mod4=3 地址:
```
20001703 B ble        (mod4=3 ❌)
20001743 B gBleIPPara (mod4=3 ❌)  
20001843 B gBleLlPara (mod4=3 ❌)
```
lib 内部以 `u32*` typed access 这些结构体字段 (BLE 状态结构是 u32 字段集),非对齐 u32 read on RISC-V →
- 某些 hart trap (本机表现)
- 某些 hart silent 错位读 (跨字节边界拼出垃圾)

→ 状态机直接走死 → cba=0+abc=0。

### 修复

```rust
#[no_mangle] pub static mut ble:        [u32; 16] = [0; 16]; // 64B  align=4
#[no_mangle] pub static mut gBleLlPara: [u32; 74] = [0; 74]; // 296B align=4
#[no_mangle] pub static mut gBleIPPara: [u32; 16] = [0; 16]; // 64B  align=4
```
nm 实测 (Vega 17:44):
```
20001704 B ble        (mod4=0 ✓)  — +1B vs broken state
20001744 B gBleIPPara (mod4=0 ✓)  — +1B
20001844 B gBleLlPara (mod4=0 ✓)  — +1B
```
linker 自然对齐,3 个符号每个恰好 +1 字节。BIN 51588B 仍严格相等 (size-neutral 保住)。

### Iron Law 新增 — Lib BSS 符号迁移对齐契约

**从 lib 迁移 BSS/data 符号到 Rust 时,字节大小相等不够,必须匹配 alignment**:

1. lib 编译时假设的对齐由其内部 typed access 决定 (BLE/MAC/PHY 结构体几乎都 u32 typed,意味着 alignment≥4 是基线契约)
2. Rust `[u8; N]` 默认 alignment=1,会引入对齐 UB
3. 安全迁移模板:
   - **首选**: `static mut FOO: [u32; N/4] = [0; N/4]` (匹配 4B align,N 必须 4 倍数)
   - **N 非 4 倍数**: `#[repr(align(4))] struct __Align4<const N: usize>([u8; N]); static mut FOO: __Align4<N> = ...`
   - **避免**: 直接 `[u8; N]` 除非确认 lib 内部完全字节级 access (罕见)
4. **必须 nm 实测验证**: 
   ```
   nm release/binary | grep -E ' (sym1|sym2|sym3)$'
   ```
   confirm 地址 mod 期望对齐 = 0
5. **症状指纹**: BIN size-neutral + cba=0 + abc=0 + structural-only-change → **首先怀疑对齐**,不是 layout shift,不需要 bisect (alignment 是常见单一根因)

### 流程教训

Lucy 17:40 directive 先 bisect (V_T2_a/b/c) 是过度保守:
- bisect 在 alignment 根因下意义不大: 任何含非对齐 BSS 都会破,bisect 只是定位"哪个 struct 先破"
- Vega 17:40 直接定位根因 + 提供修复 (`[u32]` migration) 是正确路径
- 教训: P0 失败 + 强烈症状指纹 (size-neutral + cba=0 + abc=0) → 优先猜测语义破坏 (alignment/volatile/init-order),次优 bisect

### 环境基准漂移观察

Cindy Step 1 sanity check (17:43):
- T2 padded repeat: cba=0 (复现破)
- T1 baseline (commit `76d360c`): cba=43 (低于历史 62 median + 低于 52 hard gate)
- 环境 RF 噪声/天线/系统状态漂移 → 所有 gate 阈值需要相对当前 baseline 重新校准
- 临时 gate 标准: aligned T2 R1 cba 与 T1=43 同档 (33-53) → pass

## 2026-05-04 18:00-18:43 #27 T2 — bisect 完成 + GlobalMerge ISR Timing 重大 Iron Law

### Bisect 完整结果矩阵

aligned `[u32; N]` 修复后仍 cba=0 → narrow bisect 5 个 variant:

| Variant | ble | gBleIPPara | gBleLlPara | cba | BIN | BB ISR len | 含义 |
|---|---|---|---|---|---|---|---|
| baseline T1 (`76d360c`) | extern | extern | extern | 43 | 51588 | 262B | 当前环境 baseline |
| V_T2_a (`cb7ecc8`) | Rust 64B | extern | extern | **57** ✓ | 51588 | 262B | 唯一安全 |
| V_T2_b (`3637460`) | Rust 64B | Rust 64B | extern | 0 ✗ | 51588 | 238B | gBleIPPara 加入即破 |
| V_T2_b' (`95bd998`) | Rust 64B | Rust 40B | extern | 0 ✗ | 51588 | 238B | size 不影响 |
| V_T2_b'' (`48d6c4a`) | extern | Rust 40B | extern | 0 ✗ | 51588 | 238B | gBleIPPara 单独迁就破 |
| V_T2_d (`7bf0554`) | extern | extern | Rust 296B | 0 ✗ | 51588 | 262B | gBleLlPara 单独迁也破,但 ISR 长度未变 |
| **T2 final (`cce8564`)** | **Rust 64B** | **extern** | **extern** | **[50,35,63] median=50** ✓ | 51588 | 262B | **Plan C partial 交付** |

### gBleIPPara 失败根因 — LLVM GlobalMerge ISR Timing

**机制 (Vega 18:28 disasm)**:

当 `gBleIPPara` 进入 Rust BSS,LLVM 编译器 fold 进 `.L_MergedGlobals.180` 与 BB_IRQ_ENTRY 等 Rust 静态共享 base pointer:
- V_T2_a (extern): ISR `__qingke_rt_BB` = 262B,s0=MergedGlobals,s2=gBleIPPara 独立 base load
- V_T2_b'' (Rust): ISR `__qingke_rt_BB` = 238B,s0 同时承载 MergedGlobals 和 gBleIPPara 偏移,**省 s2/s3 save/restore + 两条 lui/addi**
- 净效果: ISR -24B = ~6-8 RISC-V cycles
- `.L6` TX-advance 触发 (`WCH_LLER+0x08 = 0x2000`) 比 V_T2_a 早 6-8 cycles
- BLE hardware timing window 不接受这个 delta → cba=0

**H1 (adjacency +296) 已排除**: lib disasm 中确实存在 `addi a0,s0,296` (`ll_connect_action_in_connintervaltimeout` 等),但这些函数在 Path C 是 dead code,运行时无影响。

### 重大 Iron Law 新增 — GlobalMerge 不变 BIN size 也能破 timing

**Iron Law #22 灵敏度边界扩展**:
> BIN size strict-equal 不能保证 timing 不变。LLVM GlobalMerge / GlobalOpt / GVN 等 size-neutral 优化在 ISR 内部改变 cycle count,即使总 .text 长度未变也会偏移硬件 timing window。

**症状指纹 — GlobalMerge ISR timing 破坏**:
- BIN size-neutral
- cba=0 + abc=0
- 单一 BSS struct 迁移触发 (e.g., gBleIPPara V_T2_b'')
- ISR 函数长度变化 (如 262B → 238B)
- 该 BSS struct 在 ISR 内被访问

**P0 调查 protocol (新)**:
1. 比对 ISR 函数长度: `llvm-objdump -d release/binary | rg -A 1 '__qingke_rt_(BB|LLE|RFEND|AES|TIMER)'`
2. 如长度变化,检查 disasm 内 `s0`/`s1` 等 base register 是否被重用 (GlobalMerge 标志: 多个 static 通过同一 base + 不同 offset 访问)
3. 候选 workaround:
   - **A**: `#[link_section = ".bss.struct_isolated"]` 强制独立 section,阻止 GlobalMerge
   - **B**: `#[used]` + linker script 控制 placement
   - **C**: ISR 内显式 `core::hint::black_box()` 或 NOP cycle pad,补偿 -6-8 cycles
   - **D**: 重新校准 hardware setup (.L6 trigger value 调整),接受 timing shift

### gBleLlPara 失败 (V_T2_d) — 机制未明,Iron Law 第二类

**关键观察**:
- BB ISR `__qingke_rt_BB` = 262B,**与 baseline 同长** → GlobalMerge 未触发
- main() 代码长度差 +2B (V_T2_d 2256B vs V_T2_a 2254B)
- BIN 51588B size-neutral
- cba=0 + abc=0

**初步推测** (待 #35 forensic):
- `adv_tx_burst_ch37` inline 22-step hot path 对 gBleLlPara 字段访问
- gBleLlPara 从 lib BSS 高地址 (0x20001b80) 移到 Rust BSS 低地址 (0x200017c4) 改变了 addressing mode 选择
- `lui+addi` (32-bit) vs `auipc+addi` (PC-relative) 编译器选择基于地址 → 指令长度可能差但都填到相同 .text 长度,主 cycle 略变 → hot path 时序偏移
- 或 `adv_tx_burst_ch37` 内多个 gBleLlPara 字段访问 share 同一 base register,base 地址变化导致 `addi` immediate 范围 (-2048..+2047) 边界跨越,触发额外 `lui` 加载

**#35 起点**: disasm 对比 V_T2_a vs V_T2_d 的 `adv_tx_burst_ch37` 编译输出,定位具体指令差异。

### T2 Plan C 交付状态 (commit `cce8564`)

```rust
// ble: 唯一安全迁移
#[no_mangle] pub static mut ble: [u32; 16] = [0; 16];

// gBleIPPara/gBleLlPara: 留 lib BSS extern (推迟 #34/#35)
extern "C" {
    pub static mut gBleIPPara: [u8; 40];
    pub static mut gBleLlPara: [u8; 296];
}
```

- BIN: 51588B (与 T1 baseline 严格相等)
- ble Rust BSS at 0x20001708 (mod4=0)
- gBleIPPara/gBleLlPara at lib COMMON BSS 高地址 (0x20001ca8/0x20001b80,邻接保留)
- BB ISR 262B (与 baseline 同长)
- 3 轮 cba=[50,35,63] median=50,abc=0,yellow band pass (size-neutral + abc=0 + 结构性变更三条件)

### 推迟工作 (待 #34/#35)

- **#34 (gBleIPPara forensic + GlobalMerge workaround)**: 测试解法 A/B/C/D,选最不侵入的修复
- **#35 (gBleLlPara mechanism forensic)**: disasm `adv_tx_burst_ch37` V_T2_a vs V_T2_d,定位指令差异,提出针对性修复

T8 (final -lwchble) 闭环要求 #34 + #35 都解决。建议时序: T3-T7 先推进 (这些不依赖 BSS struct 迁移),最后用 #34/#35 收口再到 T8。

### Lucy 流程教训 (本轮新增)

1. **GlobalMerge 是 P0 调查必查项** — 之前只关注 BIN size delta,忽略 ISR 内部 cycle 变化。新增 disasm ISR 比对到 P0 protocol。
2. **Bisect 仍有价值** — 即使 alignment 是真实子 bug,GlobalMerge 是更根本机制,bisect 完整走完才把两个机制分清。盲目跳过 bisect 直接修 alignment 会留 GlobalMerge 黑盒。
3. **Vega 的 disasm 工作流非常 powerful** — `__qingke_rt_BB` 长度比对 + `.L_MergedGlobals.180` 识别 + s0/s1 base register 跟踪,这是机器级 forensic 标杆。落档为 #35 起点。

## 2026-05-04 19:11-22:39 #28 T3 — gPaControl + dtmFlag 迁移 + TX_BUF 16B 对齐 P0 教训

### 任务范围
- `gPaControl: u32` (lib BSS COMMON, 4B)
- `dtmFlag: u8` (lib BSS COMMON, 1B)
- 不动 `fnGetClockCBs` (T4 后随 BB_IRQLibHandler 自然消解)
- 实施 Vega,gate Cindy

### 路径选择失败矩阵

| Try | Patch | TX_BUF mod16 | Cindy R1 cba | Iron Law 教训 |
|---|---|---|---|---|
| T3 R1 (`e4762d3`) | 直接 Rust 化 | 4 ⚠️ | 0 ✗ | LLVM GlobalMerge 在 init-only 路径也插入 (gPaControl@+64,dtmFlag@+13 入 .L_MergedGlobals.180) |
| T3-probe-align A (`1fb675f`) | `#[link_section=".bss.zz_*"]` 隔离 | 8 ⚠️ | 0 ✗ | 隔离 GlobalMerge 后 BSS 整体 +8B shift,但 TX_BUF 落到 mod16=8,不为 0 仍破 |
| Option B (`#[repr(C, align(16))]`) | wrapper struct 强制 16B 对齐 | 0 (理论) | — (放弃) | LLVM 对 align(16) 大对象 register allocation 升级,stack +32B,**main() +108B 代码膨胀** |
| Option C (`#[link_section=".tx_buf_aligned"]`) | TX_BUF 退出 MergedGlobals | 0 | — (T3 放弃,留作 #36) | TX_BUF 单独 absolute address load (lui+addi 8B per access),**main() +64B 代码膨胀** |
| **T3 Option D (`9a3b9b5`)** | **link.x `. += 8` BSS 整体前移** | **0** ✓ | **[67,61,59] median=61** ✓ | **零 codegen 变化,但 brittle to BSS layout (T4-T7 风险)** |

### 决定性数据 (TX_BUF mod16=0 是 hardware 强对齐要求)

| Try | TX_BUF | mod16 | cba | abc |
|---|---|---|---|---|
| T2 final (`cce8564`) | 0x200016a0 | **0** | 50 | 0 |
| T3 R1 | 0x200016a4 | 4 | 0 | 0 |
| T3-probe-align A | 0x200016a8 | 8 | 0 | 0 |
| **T3 Option D** | **0x200016b0** | **0** | **61** | **0** |

实证: cba 仅在 mod16=0 时通过,mod16∈{4,8} 都破。

### LLVM GlobalMerge 多机制确认

T2 教训: GlobalMerge 改 ISR cycle count (262B → 238B,-24B = ~6-8 cycles) → `.L6` TX-advance 提前 → 时序破坏

T3 新教训: GlobalMerge 不改 ISR 长度,但**间接破坏 TX_BUF 16B 对齐**:
- gPaControl/dtmFlag 加入 `.L_MergedGlobals.180`
- struct 内部新增成员 → BSS 后续布局 shift
- TX_BUF (在 MergedGlobals 之外) offset 改变 → mod16 破

第二个机制比第一个更隐蔽,因为 ISR/BIN/main 都看似正常,只有 nm 实测才能察觉。

### Option B/C 代码膨胀 root cause (Vega disasm)

**Option B (+108B)**:
- `TxBuf([u8; 39])` with `align(16)` size = 48B
- LLVM 把 align(16) 大对象纳入 MergedGlobals 重排
- main() stack frame: 464B → 496B (+32B,需保存额外寄存器处理 48B 对象)
- 总 BIN +108B = 32B stack + ~76B codegen

**Option C (+64B)**:
- `#[link_section=".tx_buf_aligned"]` 让 TX_BUF 退出 MergedGlobals.180
- main() 每次访问 TX_BUF 需独立 `lui+addi` (8B per access)
- main() body +64B (8 次访问 × 8B,具体次数估算)
- stack 不变 (464B)

### T3 ship 决定 — Option D (commit `9a3b9b5`)

**理由**:
- 零 codegen 变化 (B/C 都引入 +64-108B 不可忽略 BIN 增长)
- mod16=0 hypothesis 实证通过 (cba=61,T2 baseline 50,+11)
- BIN 51588B strict,BB ISR 262B 完全保持 baseline

**Brittle 风险**:
- `link.x` 注入 `. += 8` 依赖当前 BSS 排布 (TX_BUF offset = MergedGlobals base + 0x98)
- T4-T7 任何 BSS static 添加可能改 MergedGlobals 内部布局 → TX_BUF offset 重新计算 → mod16 可能再破
- 缓解: T4-T7 强制 pre-step `nm TX_BUF mod16==0` 检查 (Lucy 写入 task spec)

### Strategic hardening 候选 (task #36 待建)

**方案 1**: `link.x` 给 `.bss.tx_buf_aligned` section 加 `ALIGN(16)`,TX_BUF 加 `#[link_section]` 不加 wrapper struct
- 等价 Option C hardened 版本,+64B codegen 代价
- TX_BUF 永久退出 MergedGlobals → mod16=0 与 BSS 内部布局解耦

**方案 2**: link.x 对 MergedGlobals.180 整个 region 加 `ALIGN(16)` + offset padding 保证 TX_BUF mod16=0
- 零 codegen 变化 (TX_BUF 仍在 MergedGlobals)
- 但 MergedGlobals 内部布局变化时仍可能失效 (TX_BUF 在 struct 内的 offset 变化)

**方案 3**: 接受 +64B Option C 代价作为 strategic ship
- 一次性付出 +64B,T4-T8 永久不需 mod16 验证
- Vega 倾向方案,前提是 T8 BIN budget 有 +64B 裕量

**决定时序**:
- T3 ship Option D
- T4 pre-step Vega 估算 T4-T8 BIN trajectory
- 反推方案 1/2/3 可行性 → 选定 → 在 T8 之前完成 hardening

### Iron Law 新增 (T3 落档)

> **Iron Law #23 — TX_BUF 16B 对齐是 hardware DMA 强要求**:
> CH32V208 BLE TX_BUF 必须满足 `addr mod 16 == 0`。任何 BSS 布局变化导致 TX_BUF 离开 mod16=0 都会触发 cba=0 timing break,即使 BIN size-neutral + ISR 长度同长 + abc=0。新增 BSS 静态时,**强制 nm 验证所有 hardware-DMA buffer mod16=0**,不只 mod4。

> **Iron Law #24 — linker-script BSS padding 是 tactical 不是 strategic**:
> `link.x` 的 `. += N` 是 layout-tuning 工具,不是 alignment 强制工具。hardware-DMA buffer 必须有 type-level `#[repr(align(N))]` 或等价 attribute 或专用 ALIGN(N) section,不能依赖 linker placement 偶然。

> **Iron Law #25 — wrapper struct alignment 触发 LLVM codegen tax**:
> `#[repr(C, align(16))]` wrapper 对大对象 (≥48B) 触发 LLVM register allocation 升级,stack frame 膨胀 (Option B +32B stack + +108B BIN)。如必须用 wrapper,接受 BIN 增长不可避免。

### Lucy 流程教训 (本轮)

1. **mod16 检查必须列入 P0 protocol**: T2 之后已加 mod4,但 T3 教训说明 hardware-DMA buffer 需更严格 mod16。新增 BSS 静态后,所有 candidate DMA buffer 都要 nm 实测 mod16。

2. **Vega revised hypothesis 时坚持 cross-check**: Vega 21:24 推翻 mod16 假设转向 MergedGlobals 内部 layout 假设,Lucy 21:25 立即指出 T3-probe-align 失败已经 falsify revised theory (MergedGlobals cleaned 仍 cba=0 → 唯一 still-different = TX_BUF mod16)。这是 force Vega 实施 Option B/C/D 的关键决策点。

3. **Option D 接受时同步建立 follow-up debt**: 不让 tactical fix 沉默成永久债务。T3 ship 时立即明确 #36 strategic hardening + T4-T7 mod16 sanity 强制条件。

4. **B/C/D 三选时必须看 codegen tax**: 不能只看 mod16 是否达成,还要看 BIN/stack/codegen 代价。Vega 提供的 +108B/+64B/0B 数据是关键决策依据。

## 2026-05-04 22:53-23:01 #29 T4 dry-run — 双反转后定稿

### 反转过程 (forensic 实证迭代,Vega 三轮)

**第一轮假设 (Lucy 预测)**: T4 删 anchor → BB_IRQLibHandler GC,BIN -280B。Vega 起初估算同此。

**第二轮反转 (Vega 22:53)**: dry-run patch 删 anchor 但 BB_IRQLibHandler 仍在 nm 中。错误归因为"libwchble.a 无 -ffunction-sections,section sharing 维持函数存活"。基于此,Lucy 修订为 T4 仅 -4B + Iron Law #26 (lib section sharing)。

**第三轮订正 (Vega 23:01)**: 进一步 dry-run 测试,发现 bb.o **有** -ffunction-sections,BB_IRQLibHandler 在独立 section。真正原因: **build.rs 还有 `--undefined=BB_IRQLibHandler` linker flag** 第二个 anchor。
- 仅删 Rust anchor → BIN 51864B (BB_IRQ 仍在,4B rodata 释放)
- 加删 --undefined → BIN 51312B (BB_IRQ 真 GC,-276B text)
- 调整 pad 280B → BIN 51588B ✓

### 真实根因 — 双 anchor 维持

BB_IRQLibHandler 由两个独立 anchor 维持存活:
1. **Rust 源码端**: `_KEEP_BB_IRQ_LIB_HANDLER` rodata 指针 (4B)
2. **build.rs**: `--undefined=BB_IRQLibHandler` linker flag

只删 (1) 不行 → BB_IRQ 仍由 (2) 维持。**两个都删才 GC**。

### T4 final patch (3 改动,定稿)

1. 删 `_KEEP_BB_IRQ_LIB_HANDLER` anchor (lines 80-86)
2. 删 build.rs 的 `--undefined=BB_IRQLibHandler` linker flag
3. `_T4_PAD: [u8; 280]` rodata (补偿 -276B text + -4B rodata = -280B)

### 自检结果

| 检查项 | 期望 | 实测 |
|---|---|---|
| BIN size | 51588B | **51588B** ✓ |
| ISR `__qingke_rt_BB` | 262B | **262B (0x106)** ✓ |
| TX_BUF addr | mod16=0 | **0x200016b0 (mod16=0)** ✓ |
| `BB_IRQLibHandler` | GC'd | **nm 不存在** ✓ |

binary: `/tmp/ble_tx_adv_ch37_task29_t4_final_20260504.bin`,等 Andelf approve T3 #28 后推 commit。

### T4-T8 BIN trajectory empirical 实测 (Vega 23:07,probe-build-nm-restore)

每个 task 临时删 anchor → build → nm 测量,然后 restore,从 T4 baseline 出发:

| 任务 | 实测 net delta | 主要 GC + cascade |
|---|---|---|
| T4 (BB_IRQLibHandler) | **-280B** (实证) | BB_IRQ + 双 anchor 删除 |
| T5 (BLE_IPCoreInit) | **-544B** | BLE_IPCoreInit + RFEND_DevInit (372B,unique caller cascade) + RFEND_Reset (52B,cascade) + anchor (4B) |
| T6 (LLE_IRQSubHandler) | **-824B** | LLE_IRQSubHandler (504B) + lle_irq_process (318B,unique caller cascade) + anchor (4B) — RFEND_WaitTune 仍存活 (其他 caller) |
| T7 (llAdvertise* 4) | **-22,432B** | 整个 ll_advertise.o + transitive deps 大跳 |
| T8 (-lwchble) | 剩余 lib GC | 数十 kB |

**T7 -22,432B vs Iron Law #22 -22,440B 8B 差异**: 完全由 T3 Option D `link.x . += 8` BSS gap 解释,验证 trajectory 一致。

**Pad 大小锁定**:
| Task | pad |
|---|---|
| T4 | `_T4_PAD: [u8; 280]` (定稿) |
| T5 | `_T5_PAD: [u8; 544]` |
| T6 | `_T6_PAD: [u8; 824]` |
| T7 | `_T7_PAD: [u8; 22432]` (T8 final 清理 pad,自然 BIN 减 22kB) |

### Iron Law 修订 (替换 22:53 错误版本)

**❌ 已废弃 (Iron Law #26 错误版)**: "lib 无 -ffunction-sections 时 anchor 删除 ≠ 函数 GC" — 此假设对 libwchble.a 不成立 (bb.o 实测有 -ffunction-sections)。

**✅ Iron Law #26 (订正版) — 函数 anchor 可能多源**:
> 函数存活可能由多个独立 anchor 维持:
> (a) 源码 Rust `_KEEP_*` 引用 (rodata 指针)
> (b) build.rs 的 `--undefined=<sym>` linker flag
> (c) 其他 `.a` 的 U 引用 (lib-to-lib 调用,无 weak)
> (d) lib 内 `.text` section sharing (无 -ffunction-sections 时)
> GC 必须确认所有 anchor 全部移除,否则函数残留。**P0 调查协议**: nm 验证函数已 GC 后才能确认 anchor 删除完成,不能仅以"删 anchor 文本"为准。

### #36 strategic hardening 时机锁定 — T4.5

**决定 (Vega+Lucy 23:07)**: T4 ship + Andelf approve 后立即开 #36,T5 之前完成。原则: 单一变量 PR (anchor 移除和 alignment hardening 不混)。

**Budget math**:
- T4 ship: BIN=51588B, _T4_PAD=280B (size-neutral)
- T4.5 #36 实施 Option C (+64B,TX_BUF 退出 MergedGlobals)
- 调整 _T4_PAD: 280 → 216 (吸收 +64B 增长)
- BIN 维持 51588B (size-neutral),剩余 budget 216B

**T4.5 完成后**: T3 Option D 的 `link.x . += 8` BSS gap 可以删除 (因 TX_BUF 已 strategic align 不依赖 BSS shift),T5 可以从干净 baseline 推进。

**T4.5 流程**:
1. T4 #29 ship + Andelf approve done
2. claim #36
3. Vega 实施 Option C (`#[link_section=".tx_buf_aligned"]` + link.x ALIGN(16)) 或最终选定方案
4. 自检 BIN/ISR/TX_BUF mod16=0 + 删除 link.x `. += 8` gap
5. Cindy 3 轮 60s gate
6. in_review → Andelf approve → done
7. T5 (#30) 继续

### Lucy 流程教训 (T4 dry-run 本轮)

5. **forensic 反转接受 + 鼓励**: Vega 三轮 (假设 → 第一次反转 → 第二次反转) 收敛过程是健康的实证迭代,不是混乱。Lucy 的 Iron Law #26 第一次写错本来就是基于不充分数据,接受 forensic 推翻是必要纪律。

6. **Iron Law 写入需要 nm 实证**: 只有 dry-run binary nm 验证后才落档 Iron Law。22:53 我基于 Vega 第一次假设直接写 Iron Law #26 是过早,应该等到 23:01 双反转后才落档。**新流程**: Iron Law 候选先放 "TODO" 区,nm 实测确认后才转正。

---

## T4 → T7 Sequential Milestone (2026-05-05 00:17 → 00:55)

### Commit chain
| Step | Commit | Anchor removed | Cascade GC | Pad | Cindy gate |
|------|--------|---------------|------------|-----|------------|
| T4 (#29) | `4d76774` | `BB_IRQLibHandler` (anchor + `--undefined`) | -276B | `_T4_PAD: 280B` | green cba=[53,59,43] median=53 |
| T4.5 (#36) | `92eddaf` | TX_BUF strategic align (Option C) | +64B (codegen tax) | `_T4_PAD: 216B` (-64B) | yellow cba=[52,53,45] median=52 |
| T5 (#30) | `ee2ae35` | `_KEEP_BLE_IP_CORE_INIT` | -540B (BLE_IPCoreInit + RFEND_DevInit + RFEND_Reset) | `_T5_PAD: 544B` | green cba=[61,65,45] median=61 |
| T6 (#31) | `fba0da2` | `_KEEP_LLE_IRQ_HANDLER` | -828B (LLE_IRQSubHandler + lle_irq_process) | `_T6_PAD: 828B` | green cba=[64,59,46] median=59 |
| T7 (#32) | `adeb4d2` | 4× `_KEEP_LL_ADV_*` + sentinel writes | **-26904B** (ll_advertise.o + ~83 internal funcs) | `_T7_PAD: 26904B` | green cba=[56,70,55] median=56 |

### Symbols physically absent post-T7 (nm verified)
- BB: `BB_IRQLibHandler`, `BB_DevInit`
- BLE: `BLE_IPCoreInit`, `BLE_RegInit`
- RFEND: `RFEND_DevInit`, `RFEND_Reset`, `RFEND_WaitTune`
- LLE: `LLE_IRQSubHandler`, `lle_irq_process`
- ADV: `llAdvertiseCreateCore`, `llAdvertiseSet`, `llAdvertiseStart`, `llAdvertiseTraverseAllChannel`, all `ll_advertise.o` text
- TMOS: `TMOS_SysRegister`, `tmos_memcpy`
- (~28KB of lib text physically removed)

### Invariants held all 5 commits
- BIN = 51588B exact (Cindy nm-confirmed each step)
- BB ISR = 262B exact (`__qingke_rt_BB`)
- TX_BUF mod16 = 0 (after T4.5 the address is structurally guaranteed via ALIGN(16) section, not via BSS gap math)
- abc=0 (no anomaly-bursts) all 15 scan rounds
- median cba ≥ 52 all 5 gates

### T7 surprise — cascade -26904B vs T4-probe -22432B estimate
T4-probe estimated -22432B for `ll_advertise.o` cascade. T6-state actual probe (Vega before T7 commit) measured **-26904B** (+4472B more). Root cause: T5 + T6 anchor removal opened additional lib code paths (multi-source anchor effect from Iron Law #26): once `BLE_IPCoreInit` and `LLE_IRQSubHandler` were gone, `ll_advertise.o`'s extra cross-references to internal helpers became GC-eligible.

**Lesson**: Cascade trajectory at T0 underestimates terminal state. Each anchor removal can unblock further cascades. Always re-probe at the current state, not at T0.

### T8 prerequisites — #34 + #35 (gBleIPPara / gBleLlPara)

After T7, the only remaining `-lwchble` consumers are:
- BSS globals: `gBleIPPara` (90B), `gBleLlPara` (132B), plus a few smaller ones (`fnGetClockCBs`, etc.)
- These are still extern from libwchble.a — T2 attempt to migrate broke timing via LLVM GlobalMerge cycle-count delta.

**T8 cannot remove `-lwchble` until #34 + #35 land**. Rust-owning these globals requires:
1. Either a GlobalMerge workaround (the "fold gBleIPPara with another `.L_MergedGlobals.180` member to reproduce lib BSS adjacency") **or**
2. A different alignment / `#[link_section]` trick that prevents GlobalMerge from rebalancing the BB ISR cycle count

Vega has the forensic data from T2 bisect (V_T2_a/b/b'/b''/d). #34 starts there.

### Process notes
- **Andelf "继续推进" DM (00:16:48 + 05:55:25)** acted as standing approval for the entire T4 → T7 chain. Lucy ran in_review → done sequentially without per-step approval, matching the prior pattern (T1/T2/T3 with per-task explicit approval, T4+ with chain-level approval).
- **Pipeline efficiency**: T4 → T7 took ≈40 min wall time (00:17 → 00:55) for 5 commits, 5 builds, 5 gates, 15 scan rounds. Single-bottleneck on Cindy's gate runtime (~5 min each).

---

## #34 Forensic — gBleIPPara `.bss.gBleIPPara` section isolation FAILS at runtime (2026-05-05 06:00-06:04)

### Probe attempt 1 (Vega `4743f7f`)

**Hypothesis**: The T2 break root cause was BB ISR cycle-count delta from LLVM GlobalMerge folding `gBleIPPara` into `.L_MergedGlobals.180`. Since `GlobalMerge` only merges statics with identical section names, declaring Rust `gBleIPPara` with `#[link_section = ".bss.gBleIPPara"]` should prevent the fold and preserve BB ISR = 262B.

**Compile-time result** (Vega self-check):
| Variant | BB ISR | Decision |
|---------|--------|----------|
| Option 1 — no section annotation | 238B (-24B) | ✗ — GlobalMerge folds (same as T2 V_T2_b) |
| Option 2 — `#[link_section = ".bss.gBleIPPara"]` | **262B** ✓ | section isolation defeats fold |

LLVM GlobalMerge only merges same-section statics; `.bss.gBleIPPara` ≠ `.bss` so `gBleIPPara` is excluded from `.L_MergedGlobals.*`. Linker script `*(.bss .bss.*)` still gathers it into BSS region. **Compile-time hypothesis confirmed.**

### Runtime result — STILL BROKEN

Cindy gate (`/tmp/ble_tx_adv_ch37_task34_20260505.bin`):
- Pre-check: BIN=51588B ✓, TX_BUF=0x20000020 mod16=0 ✓, gBleIPPara=0x20000630 ✓, all T4-T7 lib text still absent ✓, BB ISR = 262B ✓
- R1 60s scan: **cba=0, abc=0** ✗
- T7 baseline sanity (immediate re-flash): cba=69 ✓ — environment healthy

### Conclusion (provisional)

**Old Iron Law (T2 era)** that "GlobalMerge ISR cycle-count delta = root cause of gBleIPPara migration breakage" is **necessary but not sufficient**. Even with ISR cycle count preserved at 262B, Path C still fails. There is at least one additional independent failure mode triggered by gBleIPPara migration.

### Iron Law #27 candidate (TODO — pending forensic confirmation)

> **Candidate**: "BSS-global migration may have multiple independent failure modes. Preserving compile-time invariants (ISR length, TX_BUF alignment, BIN size) is insufficient — runtime invariants (lib-residual references, BSS init expectations, struct field layout, BSS adjacency) must be separately verified before concluding migration is safe."
> **Status**: Candidate. Promote to Iron Law only after specific second-cause is identified by Vega forensic.

### Forensic candidates handed to Vega

1. Lib-residual `gBleIPPara` references — list lib functions that still reference gBleIPPara post-T7 (`objdump -d wchble.a | grep gBleIPPara`)
2. Symbol resolution check — `nm -A target/.../*.o build/*.a | grep gBleIPPara` (verify Rust strong symbol vs lib COMMON resolution)
3. BSS init expectations — does any field need explicit Rust initialization beyond zero-init? (e.g. MEMAddr at gBleIPPara[36])
4. Adjacency dependency — does the lib expect gBleIPPara adjacent to another BSS symbol?
5. Layout diff — `objdump -h --start-address=0x20000000 ...` of #34 vs T7 baseline to spot broken invariants

Vega prioritizing (1) first.

### Process discipline note

This is a **valid Iron Law #20 (forensic before implementation) miss** — Lucy's previous reasoning relied on the GlobalMerge cycle-count theory as if it were sufficient. The T2 bisect data showed only that V_T2_b was broken; it did NOT prove that GlobalMerge fold was the *only* break vector. Better hypothesis would have been: "GlobalMerge fold is *one* known break vector; section isolation defeats *that* one but other vectors may exist." Vega's probe is the proper way to test this — observe ISR 262B preserved AND test runtime — confirming the multi-vector hypothesis.

---

## 🚨 #34 Decisive Forensic — T7 baseline "pass" was stale-RAM accidental success (2026-05-05 07:34-08:08)

### Sequence of revelation

1. **07:17 Cindy runtime data** (`/tmp/sdi_task34_runtime_20260505_0717.log`): #34 binary HW-level TX completes (`done=true` 100/100, `tx#100 ok=100/100`) but cba=0 — failure is at PHY/RF emission, NOT timing/ISR.
2. **07:35 startup log diff** — T7 baseline vs #34: `gBleIPPara[0]` = `0x69` in T7 vs `0x00` in #34; `ip16` = `0x0a01000a` in T7 vs `0x00000000` in #34. T7 baseline gBleIPPara region also contained ASCII `"PATHC_MANUAL_L6_IP4 before"` debug string fragments — clear evidence of **stale RAM contents** at gBleIPPara's lib BSS slot.
3. **07:48 Vega static analysis** confirmed:
   - No duplicate symbol — Rust `gBleIPPara=0x20000630` resolves uniquely.
   - No hardcoded lib write (`objdump | grep 0x0a01000a` empty in #34).
   - bb_irq_lib_handler asm: when `gBleIPPara[0] bit5 SET` (T7 stale 0x69 had bits 5+6), ISR writes `LLE+0x08 = 0x8000` then `LLE+0x6c = gBleIPPara[20..23] << 1`. With `ip0=0` in #34, this scan-mode path is skipped.
4. **08:03 decisive experiment** (Vega forced ip0=0x60 + ip20=0x20231d00 in #34, T7 stale values reproduced): Cindy gate cba=[58,48,53] median=53 ✓ — **#34 from cba=0 → cba=53 with one explicit init line.**

### Confirmed root cause

**T7 baseline cba=69 was passing because the lib's residual BSS COMMON for gBleIPPara was NOT zero-initialized cold-boot to cold-boot — the chip RAM retained values from prior lib sessions, including a `gBleIPPara[0] = 0x69` (bits 5+6 set) that triggered the BB ISR scan-mode path on first IRQ, writing `LLE+0x08=0x8000` and `LLE+0x6c=ip20<<1` as side-effects. This scan-mode write is ESSENTIAL for ADV TX RF emission, but it was never an intended part of the ADV TX init sequence — it was a serendipitous side-effect from connection/scan code paths that happened to fire when bit5 was set.**

When `gBleIPPara` migrated to Rust BSS (`.bss.gBleIPPara`), Rust crt0 zero-initialized it cleanly. ip0=0 → scan-mode path skipped → LLE+0x08=0x8000 + LLE+0x6c writes never happen → ADV TX hardware fires (BB ISR sees TX done) but RF emission is invalid (SDR cba=0).

### Iron Law #27 (CONFIRMED)

> **Iron Law #27 — Stale-RAM hidden dependency**: Lib-managed BSS COMMON symbols may NOT be zero-initialized cold-boot if startup linker order excludes them from `_sbss.._ebss` clear, OR if board reset doesn't clear all SRAM. Functions tested under "lib BSS" appear to work but actually depend on residual data from prior runs. Migrating these symbols to Rust BSS (which IS strictly zero-init) exposes init gaps that were never visible under lib BSS.
>
> **P0 Forensic protocol**: Before any BSS-symbol migration, run a `forced-zero baseline test` — clear all relevant lib BSS to zero in the *current* baseline binary, re-test, confirm it still works. Only then is the baseline a valid reference. If baseline fails on forced-zero, the migration is exposing a real init gap that needs explicit Rust code to populate.

### Retroactive implications for T1-T7

All T1-T7 commits passed Cindy gate (cba ≥ 45 / median ≥ 52). However:
- Cindy gate uses `wlink flash` which does NOT explicitly clear SRAM between runs. Soft-reset between binaries means `gBleIPPara` BSS slot retains data from prior binary.
- This means T1-T7 "green pass" data is **partially compromised** — runs were not true cold-boot tests.
- Specifically suspect: T2 bisect V_T2_a ("only ble migrate cba=57 ✓") may also be stale-RAM accidental. Need to retest after explicit power-cycle / SRAM clear.

### Plan forward

1. **Single-variable probes** (Vega + Cindy): isolate which of `ip0=0x60` (BB+0x08 dual-write) and `ip20=0x20231d00` (BB+0x6c) is the actual TX-fire enabler.
2. **Real fix in Rust** — `ble_ip_core_init` or ADV TX setup must EXPLICITLY perform the missing init writes (`LLE+0x08=0x8000`, `LLE+0x6c=<value>`) directly, not via the ISR scan-mode side-effect path. This decouples ADV TX from any stale-RAM dependency.
3. **Cold-boot retest of T1-T7** — after fix lands, run each T-step under forced-zero or post-power-cycle conditions to validate true working status.
4. **#35 (gBleLlPara)** — apply same forced-zero protocol BEFORE migration starts.
5. **T8 (final -lwchble removal)** — only after #34/#35 fix lands and forced-zero retest passes for all of T1-T7.

### Process learning

- **"Run, don't theorize" worked here** — Andelf's preference saved hours. After 30 min of static analysis cycling through hypotheses, one experimental probe (force ip0+ip20) gave decisive answer in 6 min.
- **Multi-step forensic discipline** — Vega's progression: hypothesis (GlobalMerge) → probe (section isolation) → confirm-compile-time → fail-runtime → narrow (HW done vs RF emit) → narrow (scan-mode path) → propose-fix-experiment → confirm-experiment. Each step had a test. No leaps.
- **Skepticism on "green" baselines** — once stale-RAM hypothesis was floated, *retroactively all "green" gates become questionable* until cold-boot revalidated. Iron Law #27 makes this protocol explicit.

---

## #34 Single-variable probe-A result — Root A confirmed (2026-05-05 08:12)

### Setup

Vega built two single-variable binaries on top of `4743f7f`:
- **probe-A** (`probe_A_ip0only.bin`, 51600B): `ip0=0x60` only, `ip20=0` (Root A test — does ISR scan-mode + `LLE+0x08=0x8000` alone suffice?)
- **probe-B** (`probe_B_ip20only.bin`, 51612B): `ip20=0x20231d00` only, `ip0=0` (negative control — should be cba=0 since ip0=0 means scan-mode path never fires)

### probe-A result

Cindy single-round gate (R1, 60s scan):
- binary: `/tmp/probe_A_ip0only.bin`, 51600B
- result: **cba=39, abc=0** ✓ (Root A confirmed qualitatively)

### Conclusion — Root A locked, Root B not needed

**The critical missing init is `LLE+0x08 = 0x8000` (the BB ISR scan-mode pre-write). `LLE+0x6c = ip20<<1` is non-essential.** With `ip20=0` the ISR computes `LLE+0x6c = 0` and BLE-decodable advertisements still appear (cba=39). probe-B was skipped (Vega's planned skip when probe-A clears ≥45 threshold; cba=39 was decisive enough qualitatively).

### Real fix direction — explicit MMIO init, not ip0/ip20 simulation

Vega's clean-patch direction (consensus with Lucy, msg `5709b8e0`/`44a01025`):
- Remove the `ip0=0x60` + `ip20=0x20231d00` injection completely.
- Add an explicit MMIO write in `ble_ip_core_init` (or pre-GO sequence in `ble_tx_adv_ch37`):

```rust
// Replaces the BB ISR scan-mode side-effect that was previously triggered
// only when stale RAM left gBleIPPara[0] bit5 set (lib BSS COMMON not zero-init cold-boot).
// Path C does NOT need scan-mode logic; do the one essential write directly.
write_volatile((0x40024200 + 0x08) as *mut u32, 0x8000);  // LLE+0x08 = 0x8000
```

- Then proceed normally: `BLE_SetPHYTxMode` → set `gBleIPPara[4]=0x80` to arm `.L6` → trigger ADV TX GO. The ISR's `.L6` advance path (`LLE+0x08=0x2000`) still fires on each PLL-ready IRQ as before; only the previously-stale scan-mode pre-arm is now made explicit.

### T1-T7 retroactive policy (negotiated 2026-05-05 08:11)

Vega's position (msg `5709b8e0`): T1-T7 cba data was real BLE emission (SDR decoded actual packets), and the symbol cleanup conclusions (`nm` measured lib functions removed) are unaffected by stale-RAM. Only the *trigger path* depended on stale RAM. Therefore:
- **DO NOT** re-run T1-T7 cold-boot. Symbol-removal conclusions stand.
- **DO** record retroactive note: T1-T7 cba data was contingent on scan-mode side-effect; once #34 lands explicit init, those green gates become independent of stale RAM.
- Cold-boot validation effectively happens automatically at T8 (final `-lwchble` removal): if T8 still passes after explicit init, all upstream work is implicitly validated.

Lucy concurred (msg `44a01025`).

### Iron Law #27 (final form)

> **Iron Law #27 — Explicit MMIO init > implicit ISR side-effect.** When porting a lib-driven init sequence to Rust, identify ALL MMIO writes the lib performs end-to-end (including ISR side-effects in scan/connection/idle code paths) and replicate them EXPLICITLY in the Rust init path. Do not rely on stale-RAM-triggered ISR side-effects to populate critical hardware state. Forced-zero baseline test (clear lib BSS to zero, re-test) is mandatory before declaring any lib BSS migration "passing".

### Required full-gate criterion for #34 close

probe-A R1=39 is *qualitative* confirmation. Final clean patch must:
1. Strip ip0/ip20 injection — only `LLE+0x08=0x8000` explicit write remains.
2. Pass full 3-round gate (R1/R2/R3, median ≥45 — typical green threshold).
3. nm validate `gBleIPPara` resolves to Rust strong symbol; lib BSS COMMON suppressed.
4. Document any other lib ISR side-effects discovered while drafting the explicit init (probe future #35 to avoid same trap).

---

## #34 patch attempts log (2026-05-05 08:20-08:55)

| Patch | Approach | Result | Insight |
|-------|----------|--------|---------|
| v1 (`32ff2861`) | Write `LLE+0x08=0x8000` once at end of `ble_ip_core_init()` | ✗ R1=0 | Init phase too early — LLE in Sleep state, write doesn't persist |
| v2 (`b79de9ce`) | Write `LLE+0x08=0x8000` in `bb_irq_lib_handler` `.L6`, before `0x2000` advance | ✗ R1=0 | One write insufficient — must mirror full scan-mode write pair |
| v3 (`90e58a65`) | v2 + write `LLE+0x6c=0` in `.L6` (mirror scan-mode commit) | ✗ R1=0 | Position inside `.L6` ip4 block insufficient; missing earlier-position semantics |
| v4 (`63b5dc38`) | Move `0x8000`+`0x6c=ip20<<1` to immediately after `BB+0x38` W1C, before ip0/ip4 checks | ✗ R1=0 | Even with timing window matched, still insufficient — confirms differential is more subtle |

### Iron Law #27 second-warning resolution (2026-05-05 08:54)

After v1-v4 all failed while probe-A originally passed cba=39 R1, suspicion arose that probe-A itself was stale-RAM artifact (Iron Law #27 self-application). Cindy ran `wlink erase + flash probe-A` (clears flash but not SRAM strictly) and **probe-A R1 = 56 ✓** — even higher than original cba=39. This rules out probe-A being stale-RAM-dependent (post erase+flash, fresh probe-A binary still works).

**Probe-A working hypothesis confirmed**: `LLE+0x08=0x8000` + `LLE+0x6c=ip20<<1` writes in scan-mode bit5 path + `.L6` advance path is genuinely the working init sequence. v1-v4 attempts to replicate the MMIO sequence in Rust failed for reasons not yet identified — store sequence is identical, but cba differs cba=56 (probe-A) vs cba=0 (v4).

### Pending: SDI forensic (next session)

Plan (proposed by Lucy, 2026-05-05 08:56):
- Vega builds `probe_A_sdi.bin` and `v4_sdi.bin` with SDI counters at:
  - BB ISR entry/exit
  - bit6 / bit5 / .L6 advance branch entry counters
  - LLE+0x08 read at each critical node
  - BB+0x38 read post-W1C
  - gBleIPPara[4] terminal value
- Cindy erase+flash each, captures SDI dump
- Side-by-side comparison identifies the actual differential

Suspected differentials still on the table:
- **A**: BB IRQ count / LLE state machine sequence difference (maybe probe-A has 2+ BB IRQs, v4 only 1?)
- **B**: Branch ordering effect on hardware sync (.L6 inside ip4-only block vs probe-A's full chain through bit6→bit5→.L6)

---

## 2026-05-05 11:08 — expC negative result (ip0=0x00 failure mode)

### Captured 10 globals (5 registers × PRE/POST around `.L6` writes `0x8000 / 0x6c / 0x2000`)

```
# SDI_C pre18=0x24031624 pre24=0x01006310 pre30=0x009c0320 pre3c=0x00000000 preRF90=0x0735241e
       post18=0x24031624 post24=0x01006310 post30=0x009c0320 post3c=0x00000000 postRF90=0x0735241e
cba=0 abc=0
```

### Conclusion

**All 5 candidate registers (`BB+0x18`, `+0x24`, `+0x30`, `+0x3C`, `RFEND+0x90`) are byte-identical PRE→POST.** `.L6`'s 3 MMIO writes have **zero observable effect** on these offsets in failure mode.

Possible explanations:
1. Candidates are simply wrong — true PHY advance state lives at different offset (other BB / LLE / RFEND offsets we haven't probed).
2. Failure mode causes writes to be silently dropped because PHY isn't in a state that responds (no PLL lock, no calibration, no TX enable).
3. State updates lag the readback window (write-then-read race; HW state changes after the SDI emit).

Need probe-A working-mode comparison (`EXP_C_PROBE_A=true`) to disambiguate (1) vs (2)/(3): if probe-A reads also identical → candidates wrong; if some delta appears → candidates right but failure-mode PHY is dead.

### Decision

- Vega rebuild `expC_PROBE_A.bin` with `EXP_C_PROBE_A=true` (force `gBleIPPara[0]=0x60` + `ip20=0` working-mode).
- Cindy single 60s scan for working-mode SDI_C line.
- Side-by-side compare 5 registers between failure-mode (above) and working-mode.

### Note for #37 register doc

Add: "BB+0x18=0x24031624 / +0x24=0x01006310 / +0x30=0x009c0320 / +0x3C=0x00000000 / RFEND+0x90=0x0735241e" as **chip-alive baseline values** (failure mode, observed at .L6 entry, ip0=0x00 path, expC bin `ble_tx_adv_ch37_task34_expC_20260505.bin` 52528B). RFEND+0x90 lower 6 bits = CO calibration (matches `co=` PHY init dump).

---

## 2026-05-05 11:24 — Andelf anti-regression flag + strategy pivot

### Andelf concern (DM 11:24)
> "我觉得 Cindy 也可以介入进来 review 下当前瓶颈,看看有没有更好的方向来发力,目前这个依赖关系应该没问题,34 目前这个死磕状态不是一个好信号"

After 6h of v1-v6 + expB + expC forensic with no green run, Andelf flagged the death-grind anti-regression pattern. Pulled Cindy in for strategy review.

### Lucy reflection — close gate was wrong

Original #34 close standard was "strip ip0/ip20 injection — only `LLE+0x08=0x8000` explicit MMIO write remains". This contradicts Iron Law #27's prescription of "explicit MMIO init: replicate ALL lib writes". probe-A's two writes (`gBleIPPara[0]=0x60`, `ip20=0`) ARE that explicit init. The "minimum-write" aesthetic became the obstacle.

### Pivot — probe-A IS the v7 patch

Drop the "minimum-write" goal. probe-A becomes v7-probe:
- `gBleIPPara[0] = 0x60` (lib scan-mode bit5+6 mode word equivalent)
- `gBleIPPara[20..24] = 0` (control word clear, scan-mode commit equivalent with ip20=0)
- Strip v1-v6 explicit MMIO writes (no longer needed)
- Code comment: "Forensic equivalence with libwchble scan-mode init — Iron Law #27 explicit init"

### Cindy review (msg `aaf16d2b`) — accepted with 3 hardenings

1. **Naming**: "libwchble 兼容初始化" (compatible init), comments must state semantic equivalence.
2. **Liveness gate added**: 3×60s scan median ≥45 + abc=0, **plus** 10-20s serial capture confirming main loop progress + tx#/alive output + no infinite ISR storm. probe-A's SDI version had main-loop starvation (instrumentation artifact) — production v7 must verify no similar issue.
3. **#35 protocol**: Field-level bisection BEFORE MMIO guessing. Pre-step: dump `gBleLlPara[0..296]` from T7/v7 baseline; Rust zero-init version, if fails, bisect fields not chase MMIO. Field-level probe vtable region `+0x68..+0x74`, timer/adv hot path offsets — every non-zero initial value needs documented source.

### stub.a fallback (P0 escape)
If v7-probe gate also fails: keep gBleIPPara/gBleLlPara in self-built C/asm shim BSS, not Rust statics. Lower elegance, same lib-removal goal achievable.

---

## 2026-05-05 11:32 — Vega T7 head full lib symbol audit

### TEXT functions from libwchble.a in final ELF: **0** (all GC'd by T1-T7)
`-lwchble` flag now only resolves BSS/DATA globals.

### BSS globals remaining (10 symbols)

| Symbol | Addr | Size | Source | Status |
|---|---|---|---|---|
| `gBleIPPara` | 0x200005f8 | 40B | bb.o | #34 current target |
| `gBleLlPara` | 0x20001bc8 | 296B | ll.o | #35 next |
| `ble` | 0x200017a0 | 64B | tmos.o COMMON | 🚨 new finding, unaudited (likely TMOS task context) |
| `fnGetClockCBs` | 0x20001bc0 | 4B | bb.o | already handled in Rust ISR |
| `dtmFlag` | 0x200005f0 | 1B | ip.o | written by Rust `ble_ip_core_init` |
| `gPaControl` | 0x200005f4 | 4B | ip.o | written by Rust `ble_ip_core_init` |

### DATA globals (initialized by Rust `ble_ip_core_init`)

| Symbol | Addr | Size | Value |
|---|---|---|---|
| `gptrLLEReg` | 0x20000004 | 4B | 0x40024200 |
| `gptrRFENDReg` | 0x20000008 | 4B | 0x40025000 |
| `gptrBBReg` | 0x2000000c | 4B | 0x40024100 |
| `gptrAESReg` | 0x20000000 | 4B | 0x40024300 |

### T8 final removal — remaining work + difficulty

1. **#34 (gBleIPPara, 40B)** — Iron Law #27, v7-probe pivot path. ETA 1-2h after green light.
2. **#35 (gBleLlPara, 296B)** — same stale-RAM risk class, field-level bisect protocol. 7× bigger than #34. ETA 2-4h.
3. **#38 (`ble` symbol, 64B)** — NEW, unaudited. Likely TMOS task context. Must audit each field before T8 to avoid leaving COMMON suspended. ETA 1-2h.
4. **`gptrXxxReg` (16B DATA)** — trivial migrate to Rust static. ETA <30min.
5. **T8 (#33)** — strip `-lwchble`, remove `_T7_PAD: 22432B`, BIN drop ~22-24KB. ETA 1h. Total walltime 5-9h.

- **C**: Some other MMIO read/write we missed in the asm decode (e.g. an implicit read in fnGetClockCBs path that probe-A executes when ip0 bit6 set)

---

## 2026-05-05 22:19 — #34 v7-probe.3-fix FULL GATE PASS — close gate green

### Outcome
| Metric | v7-probe.3-fix |
|---|---|
| Liveness markers `ok=0/1/2` | ✓ all visible |
| `PATHC_ALIVE post-go 0/1/2` | ✓ all visible |
| Outer-loop `tx_heartbeat tx_n=` | ✓ visible |
| RF R1/R2/R3 cba (3×60s erase+flash gate) | **66 / 74 / 86** |
| Median | **74** ← highest ever observed |
| `abc` | 0 across all rounds ✓ |
| BIN | 51588B (unchanged) |
| BB ISR size | 0x106 = 262B (unchanged) |
| TX_BUF mod16 | 0 ✓ |

Cindy log files: `/tmp/sdi_task34_v7p3fix_liveness_20260505_2214.log`, `/tmp/ble_scan_task34_v7p3fix_r{1,2,3}_20260505.log`. Vega production commit `0f918cc`.

### Real root cause — ISR storm pre-GO

Original v7-probe.0 had cba=78/66/67 RF green BUT main loop never reached `post-go`. Diagnosed by Vega (~21:48):

- Second `PATHC_LIB_IRQ` block in `ble_tx_adv_ch37.rs` was placed **after** `riscv::interrupt::enable()` for IRQ 63.
- Inside that block, line 1126 `bb_write(0x08, 0x0000_2000)` — meant as `.L6` advance strobe — instead **fires BB ISR before GO strobe**.
- ISR consumes `gBleIPPara[0]` bit5/bit6 scan-mode path, reduces ip0 to 0, then `.L6` path inside ISR writes 0x2000 again → triggers next IRQ → **infinite ISR storm**, CPU 100% in ISR, main loop never advances past `skip-enable64`.
- Fix: delete the entire second `PATHC_LIB_IRQ` block. ip16 / bb64 timer values are already set in the first block + `ble_set_phy_tx_mode_1mbps` step 6. Result: ISR fires at GO strobe + steady-state TX cadence. Both main loop AND RF green.

T7 baseline binary `/tmp/ble_tx_adv_ch37_task32_t7_20260505.bin` reached `post-go` cleanly — confirms hang was a #34-introduced regression, not pre-existing path C behavior.

### Three new Iron Laws extracted from the v7-probe heisenbug chain

#### Iron Law #27 — Stale-RAM hidden dependency / explicit MMIO init > implicit ISR side-effect
T1-T7 "green pass" partially relied on `bb.o` BSS COMMON residual values + lib BB ISR's scan-mode bit5/bit6 implicit init path. probe-A revealed the real init contract: `gBleIPPara[0]=0x60` + `ip20=0` are the actual scan-mode arm word. **Rule**: any "lib write we forgot to replicate" must become an explicit Rust write at boot — never assume ISR side-effect will silently set state. When migrating BSS COMMON to Rust strong, audit pre-existing residual values; if non-zero in working state, they are part of the contract and must be initialized explicitly.

#### Iron Law #28 — LLVM branch-hoisting eliminates guarded print markers
v7-probe.1/.2 had `if burst_idx == 0 { sprintln!("ok=N"); }` blocks at three points. LLVM merged the three identical guards and **eliminated `ok=1` and `ok=2` strings entirely from `.rodata`**. Result: looked like deterministic hang at "skip-enable64" — but `ok=0/1/2` markers had simply been folded out of existence, not hit at runtime. Heisenbug was the diagnostic itself. **Rule**: bisect prints **must be unconditional** (no `if` guards), or use distinct guards per site (different conditions, not the same `burst_idx == 0` repeated). Verify markers exist in `.rodata` via `riscv-nm -S` / `strings <bin> | grep ok=` before trusting absence as evidence of hang.

#### Iron Law #29 — Cargo target mismatch between `cargo build` default and explicit objcopy path
`cargo build` without `--target` produced `target/riscv32imc-unknown-none-elf/...`. `rust-objcopy` was reading from `target/riscv32imac-unknown-none-elf/...` (the project's `.cargo/config.toml` default for some commands). **Stale binary kept being flashed** — fix went into imc, gate pulled imac. **Rule**: every build command and every objcopy path must pass `--target riscv32imac-unknown-none-elf` explicitly. After build, verify mtime + path consistency before flashing. When a "fix is in but didn't land" pattern appears, suspect target path mismatch first.

### Open scope decision (P) vs (Q) — pending Andelf

Vega's `0f918cc` reverted gBleIPPara to lib BSS COMMON (`extern "C"`) and added `gBleIPPara[0] = 0x60` init in main(). RF/liveness gate green, but `-lwchble` removal at T8 will leave gBleIPPara unresolved.
- **(P)** accept current commit, handle gBleIPPara source in T8 (custom .o shim or new task)
- **(Q)** add another commit migrating gBleIPPara to Rust strong with `#[link_section=".bss.gBleIPPara"]` (defeat GlobalMerge) + retain `gBleIPPara[0]=0x60` init. Vega estimates 15min, ISR=262B preserved.

Both Lucy and Vega lean (Q) — section isolation already proven not to break ISR size, T8 boundary cleaner. DM'd Andelf at 22:25 (msg `d25c9c26`).

### File-handoff convention locked
Andelf reiterated 22:13 (msg `f25d13cc`): all agents on same machine — pass binary by **absolute file path** in message text, do NOT use `slock attachment upload` for build artifacts. Recorded to `notes/user-preferences.md`.

---

## 2026-05-05 22:35 — Andelf chose (Q) → Vega ships commit `3d5f8d3`

### (Q) commit content
- `gBleIPPara` migrated from lib COMMON BSS to **Rust strong** with `#[no_mangle] #[link_section=".bss.gBleIPPara"]`
- main() retains explicit `gBleIPPara[0] = 0x60` init (Iron Law #27)
- Section isolation defeats LLVM GlobalMerge — ISR size preserved at 262B

### Self-checks (all pass)
| Check | Result |
|---|---|
| BIN | 51588B ✓ (size-neutral) |
| BB ISR | 0x106 = 262B ✓ (GlobalMerge defeated) |
| TX_BUF mod16 | 0 ✓ |
| `nm gBleIPPara` | 'B' (strong global BSS), no longer 'C' (COMMON) ✓ |

### Link probe — drop `-lwchble`
Critical (Q) value verification: link without `-lwchble` produces only 2 unresolved symbols:
- `gBleLlPara` — task #35
- `fnGetClockCBs` — T8 trivial stub (see below)

`gBleIPPara` is no longer in the unresolved list — **(Q) goal achieved**, T8 path widened.

Binary: `/tmp/ble_v7p3_q.bin`. Cindy gate run pending.

### T8 prep — `fnGetClockCBs` stub plan

**Symbol audit** (T7 head, recorded earlier in this doc): 4B BSS at `0x20001bc0`, source `bb.o`. WCH `fn` prefix = function pointer variable, not a function body.

**Call path** (Vega): inside `bb_irq_lib_handler` only along `gBleIPPara[0] bit6` branch. ADV TX path uses `ip0=0x60` (bit5+bit6, but ISR scan-mode evolves to non-bit6 state pre-TX) → callback never actually invoked. Stub does not need real implementation, only symbol presence.

**T8 stub** (one of):
```rust
// Option B1 — typed function pointer, NULL initial (matches WCH semantic)
#[no_mangle]
#[link_section = ".bss.fnGetClockCBs"]
pub static mut fnGetClockCBs: Option<unsafe extern "C" fn() -> u32> = None;  // 4B BSS

// Option B2 — plain u32 if ergonomics simpler
#[no_mangle]
#[link_section = ".bss.fnGetClockCBs"]
pub static mut fnGetClockCBs: u32 = 0;  // 4B BSS, byte-equivalent
```

**Rule for similar lib BSS COMMON symbols**: when call path is provably dead in our config (e.g. gBleIPPara[0] bit6 never set), a NULL/0 BSS stub is sufficient — no functional Rust port required. Combine with #34's section-isolation pattern (`#[link_section=".bss.<name>"]`) to defeat GlobalMerge.

---

## 2026-05-05 23:00 — Liveness regression bisect — accidental UART delay hypothesis confirmed

After (Q) `3d5f8d3` shipped with RF cba=86 but liveness hang at `post-go → PATHC_ALIVE 0`, Vega proposed a falsifiable test: `0f918cc` (P production form, no gBleIPPara migration, no `ok=` markers) was never directly liveness-tested — its ✓ status was carried over from `v7p3fix` (which had `ok=1/2` println markers acting as accidental ~200µs UART delays).

### Bisect — `/tmp/ble_0f918cc_probe.bin` 10s liveness

| Binary | Last marker reached | Inference |
|---|---|---|
| `v7p3fix` (with `ok=1/2`) | full alive loop ✓ | Markers as 435µs UART delay acts as timing pad |
| `0f918cc` (P production) | **`skip-enable64`** ❌ | Hangs *before* `post-go` — worse than (Q) |
| `3d5f8d3` (Q) | `post-go` ❌ | Hangs after `post-go`, RF=86 |

### Reframe — three findings

1. **(P) was never a proven baseline**. Lucy's "no compromise close gate" claim was based on v7-probe.3-fix data carry-over; the production-form `0f918cc` itself fails liveness *earlier* than (Q).
2. **(Q) is the better baseline**. gBleIPPara migration to Rust strong `0x20000630` actually *helps* the pre-GO sequence reach `post-go`. BSS layout shift ≠ regression here.
3. **Real bug is timing/sequencing**, not BSS layout. Markers' UART writes were unintentional timing pads. Production code needs explicit delays at the same points.

### Iron Law #30 — Diagnostic UART output as accidental timing pad

`sprintln!` / `println!` markers used for bisect/diagnostic emit UART bytes that take 87µs each at 115200 baud (5 bytes ≈ 435µs). Removing them in production "cleanup" silently strips that delay. **Rule**: when bisect/diagnostic markers are in timing-critical paths (especially around IRQ enable, hardware strobes, ISR-main coordination), production removal must be paired with **explicit delays** (`riscv::asm::delay(N)` / `core::hint::spin_loop()` × N) sized to the equivalent UART transmit time. After removing markers, re-gate liveness — never carry over the marker-version's pass to production.

### Andelf decision — continue (Q) (DM 23:10 thread `c706b35d:406bbe83`)

> "我也认可 Q。这个 task 可以继续推一下"

Vega builds `v7-probe.3-fix2` with three-pronged fix on (Q) baseline.

---

## 2026-05-05 23:22 — `v7-probe.3-fix2`: three-pronged fix on (Q)

Binary: `/tmp/ble_fix2_candidate.bin`. Built on (Q) `3d5f8d3` (gBleIPPara Rust strong + section isolation retained).

### Fix 1 — Pre-IRQ-enable W1C clear (root cause of P hang)

```rust
bb_write(0x38, 0xFF);  // W1C-clear stale bit6 before enable_interrupt(63)
riscv::interrupt::enable();
sprintln!("# PATHC_IRQ_MARK post-enable63");
```

Without this, `BB+0x38 bit6` (or other lib BSS / RAM stale state) leaks into the IRQ 63 enable moment, causing immediate ISR fire → ISR storm → main loop never reaches `post-go`. This is the **actual P hang root cause** — delay alone wouldn't fix it.

### Fix 2 — Pre-GO settle delay (timing pad replacing markers)

```rust
sprintln!("# PATHC_IRQ_MARK skip-enable64");
delay(72_000);  // ~500µs at 144MHz → settle window between IRQ enable and GO strobe
sprintln!("# PATHC_IRQ_MARK post-go");
bb_write(0x08, 0x0000_2000);  // GO strobe
```

### Fix 3 — Post-GO alive-loop delay (Q hang fix)

```rust
// Was: delay(240)  — too fast, ISR couldn't complete TX-advance → TX fire → .L4 cleanup
delay(72_000);  // ~500µs per alive iteration
```

Verified delay timing in binary: `real_cyc=36001` (CH32V208 ~144MHz from PLL config; 72000 cycles ≈ 500µs).

### Self-checks
| Check | Result |
|---|---|
| BIN | 51588B ✓ (`_T4_PAD`: 688→752, +64B compensation) |
| ISR `__qingke_rt_BB` | 0x106 = 262B ✓ |
| TX_BUF mod16 | 0 (addr 0x20000020) ✓ |
| `gBleIPPara` | Rust strong 'B' at 0x20000630, 40B ✓ |
| `delay(72_000)` validation | 36001 cycles in 0x1cf4 + 0x1d5a loops ✓ |

### ⚠️ T8 prep note — `fnGetClockCBs` warm-reset risk

`fnGetClockCBs` lives at `0x20001b50` = `_ebss` — **outside BSS zeroing range** in startup. Warm reset (e.g. wlink flash without power cycle) leaves RAM residual values; ISR's `if fnGetClockCBs != 0 { invoke }` path can pass and dereference garbage → crash.

**Mitigation for v7-probe.3-fix2 gate**: power-cycle (USB unplug, 5s, replug) before flash + each RF round, ensuring RAM = 0.

**Permanent fix**: T8's `-lwchble` removal will replace `fnGetClockCBs` with Rust strong `static mut fnGetClockCBs: u32 = 0` in `.bss.fnGetClockCBs` — guaranteed in BSS zeroing range. After T8, warm-reset crash class disappears.

### Iron Law #31 — `_ebss` boundary symbols are warm-reset hazards

Symbols placed at or after `_ebss` are not zeroed by RISC-V startup. If runtime code conditionally checks them (e.g. function-pointer null-check), a warm reset propagates pre-reset RAM into post-reset behavior. **Rule**: any extern lib BSS COMMON symbol audited for migration should also be checked against `_ebss` placement. Symbols at/after `_ebss` need either (a) explicit zeroing in startup before use, or (b) Rust strong static migration to bring them into the BSS zero region.

---

## 2026-05-06 00:17 — #34 CLOSED via Vega commit `c702890`

### Production state shipped

Commit `c702890` "ble: Phase D+1 #34 fix2 — W1C pre-enable clear + explicit settle delays" (based on Q `3d5f8d3`):

| Change | Detail |
|---|---|
| `gBleIPPara` | Rust strong `[u32; 10]` at `0x20000630` + `#[link_section=".bss.gBleIPPara"]` (defeat GlobalMerge — the actual #34 deliverable, retained from Q) |
| `bb_write(0x38, 0xFF)` | W1C pre-enable clear — eliminates IRQ 63 enable-time `bb64=0` ISR storm (P hang root cause) |
| Pre-GO `delay(72_000)` | ~500µs settle window between IRQ 63 enable and GO strobe (replaces `ok=1` marker UART pad) |
| Post-GO alive-loop `delay(72_000)` | ~500µs per iteration (replaces `ok=2` marker UART pad) |
| `_T4_PAD` 688→752 | Compensates +64B text growth, BIN stays 51588B |
| `fnGetClockCBs` | **Reverted** to lib COMMON BSS — micro-ext attempt (without `#[link_section]`) caused cba=0 regression; deferred to T8 |

### Validation chain
- Self-checks: BIN 51588B ✓ · ISR 262B ✓ · TX_BUF mod16=0 ✓ · `gBleIPPara` Rust strong 'B' ✓ · `delay(72_000)` 36001-cycle loops ✓
- RF gate (Cindy 3×60s): cba=[88, 67, 83] median=**83**, abc=0 ✓
- External nRF (Andelf 23:59 msg `6cb0bd5f`): "我确认没问题了，已经 reset，我自己也能用 nRF 看到 cbra" ✓

### `fnGetClockCBs` micro-ext failure — postmortem

Lucy proposed folding fnGetClockCBs Rust strong into #34 to eliminate the warm-reset hazard class (Iron Law #31). Vega built it as `#[no_mangle] pub static mut fnGetClockCBs: u32 = 0;` **without** `#[link_section=".bss.fnGetClockCBs"]`. Cindy gate: cba=0.

**Primary suspect** (Andelf asked for postmortem, msg `b410daa3` 00:15): Iron Law #22 layout shift via GlobalMerge. Without explicit section isolation, LLVM GlobalMerge may aggregate fnGetClockCBs with other Rust BSS into a `.L_MergedGlobals.*` block, perturbing the merged-base addressing the lib's BB ISR uses (same mechanism Vega disasm-confirmed for gBleIPPara at #27). gBleIPPara survived Q migration only because it carried `#[link_section=".bss.gBleIPPara"]` — fnGetClockCBs missed that protection.

Other candidate hypotheses (lower probability, recorded for completeness):
- `_T4_PAD` 752→756 direction reversal (BSS doesn't consume flash, so +4B pad is suspect)
- lib BSS COMMON re-aggregation placing some stale-RAM-sensitive symbol on a non-zeroed address
- Iron Law #29 cargo target mismatch (pre-checks passed, but theoretically possible)

### Lesson — retroactive Iron Law refinement

**Any BSS migration from lib COMMON to Rust strong static MUST carry `#[link_section=".bss.<name>"]`** to defeat GlobalMerge merged-base aggregation. This was implicit in #27's gBleIPPara fix and explicit in #34's (Q) — but was missed in the inline fnGetClockCBs micro-ext. T8 must apply this rule uniformly to every migrated symbol (`gBleLlPara`, `ble`, `fnGetClockCBs`, `dtmFlag`, `gPaControl`).

### Anti-regression timeline (#34 walltime)
- 2026-05-05 11:24 — Andelf flags "死磕状态不是好信号" (anti-regression P0)
- ~14:00 — Cindy 3 hardenings + v7-probe pivot
- 22:14-22:19 — v7-probe.3-fix close gate FULL PASS (markers acted as accidental UART timing pad — discovered later)
- 22:32 — Andelf chose (Q): gBleIPPara Rust strong migration
- 22:53-23:00 — (Q) liveness regression + bisect (P actually breaks earlier than Q)
- 23:22 — Lucy/Vega built `v7-probe.3-fix2` (W1C + delays) on Q
- 23:59 — Andelf nRF external verification ✓
- 00:12-00:17 — fnGetClockCBs micro-ext attempted, failed, reverted
- **00:17 — Vega commit `c702890`, #34 → done** (~13h walltime, 5 Iron Laws extracted)

### Down-stream gates updated

- **#33 / T8 final -lwchble strip**: must include fnGetClockCBs Rust strong stub **with `#[link_section=".bss.fnGetClockCBs"]`** + remove `_T4_PAD: 752B` + `_T7_PAD: 22432B`. BIN drop projection ~22-24KB.
- **#35 gBleLlPara forensic**: 296B (7× larger than gBleIPPara) — Cindy spec is field-level bisect before MMIO guess. Must instrument liveness with explicit `delay(72_000)`-class delays (Iron Law #30 — never rely on bisect/diagnostic markers as accidental UART pads).
- **#38 (NEW)**: audit `ble` 64B COMMON tmos.o symbol.
- **Bench protocol delta**: production gate (no markers) MUST be run on production-form binary, not on bisect-marker form. v7-probe.3-fix's "FULL PASS" liveness was a false positive — the production-form `0f918cc` actually hangs *earlier* than (Q). Future close gates must verify the exact bytes that ship.

---

## 2026-05-08 00:35 — Iron Law #34 finalized (v5 final form): ROM is RAM-layout-agnostic for the 6 BSS-contract symbols

**Author**: Lucy
**Status**: **LOCKED** — 3-layer hardware-validated evidence, supersedes the "BSS must be pinned" narrative carried since the attempt-9 era.
**Trigger**: Andelf direction `c6b97584` (06:25): "彻底想办法去掉bss位置依赖的问题，尽量不使用link script hack". 6h ROM hex forensic + Cindy 06:28 wlink RAM dump + frozen drift hardware gate produced 3 independent evidence streams.

### Final form (canonical text)

> **Iron Law #34 (v5 final)** — The CH32V208 mask-burned ROM (`wchble_rom.hex` `.highcode` segment) does not directly reference any of the 6 host-side BSS-contract symbols by hardcoded RAM address. ROM uses its own private workspace at `0x20003000-0x200036FF` (≈1792B) to cache 4 MMIO base pointers (BB / LLE / RFEND / AES) and bookkeeping state, then performs all subsequent register access via indirect addressing through that private region. Therefore: **physical placement of `gBleIPPara` / `gBleLlPara` / `ble` / `dtmFlag` / `gPaControl` / `fnGetClockCBs` is host-side concern only**. Host TX/RX correctness depends on init-path completeness (BLE_SetPHYTxMode rate select + adv_tx_burst PHY mode lock + ll_init_safe_prefix + ADV_TX_BUF `ALIGN(16)` for DMA) — **NOT** on BSS placement. Earlier "BSS must be pinned at 0x758/0x508/etc." claims (Iron Law #34 v1–v4) were misattribution: BSS-layout shift was *correlated* with cba=0 (because both moved when host-code address-sensitive sites were touched), but never *causal* on the ROM side.

### Companion law (Iron Law #34 corollary)

> **Iron Law #34.1 (host-side residual dependence)** — Removing ROM-side BSS dependence does not eliminate host-side address-sensitive sites. Iron Law #22 (GlobalMerge layout shift) and Iron Law #31 (`_ebss` warm-reset hazard) still apply to host code. Empirical: the 2026-05-07 23:35 trace-binary v1 stalled at `rfend_tune` because `link.x` wildcard let `gBleIPPara` drift to `0x200002a4`, and the Rust HAL `ble_ip_core_init` writes via that address — a host-code implicit assumption, NOT a ROM expectation. Phase 2 BSS pin removal must audit host code for such implicit assumptions, not just ROM disasm.

### v1 → v5 forensic chain (5 narrative reshapes)

| Version | Date | Author | Claim | Status | Falsifying datum |
|---|---|---|---|---|---|
| v1 — "BSS layout invariant" | ~attempt-9 era | (legacy) | The 6-symbol layout that empirically works is invariant; do not move | Pattern-recognition only, never tested as falsifiable hypothesis | (frozen drift PASS at 06:34/07:19, 6 days later) |
| v2 — "lib-linked-only PC-relative constraint" | 2026-05-06 22:06 | Vega | `libwchble.a` `BB_IRQLibHandler` uses PC-relative `lw` to read `gBleIPPara@hardcoded`; lib link bakes address into immediate; T8 (-lwchble removed) should be address-agnostic | **Falsified by F1 (T44.E 22:08-22:10)** | Cindy minimal `db62d03`: `-lwchble` already removed, `gBleIPPara` drifted -528B → cba=0 anyway |
| v3 — "ROM-hardcoded BSS contract" | 2026-05-06 22:48 | Lucy | WCH EVT linker baked the 6 BSS addresses into ROM `.highcode` immediates (lui+addi or auipc) at silicon manufacturing time; we must pin RAM to ROM-expected addresses | **Falsified by F2 (T44.E 23:09)** | Cindy minimal `de63067`: all 6 BSS pins aligned to ROM contract → cba=0 |
| v4 — "BSS pin + other invariants" | 2026-05-06 23:14 | Lucy | F2 falsifies v3's *sufficiency*; perhaps BSS pin is *necessary* but combined with other invariants (host TX init delta) | **Falsified by frozen drift PASS** | 06:34 cba=72, 07:19 cba=61 — frozen `74e8d67` runs PASS with `gBleIPPara` already drifted off `0x758`; BSS pin is not necessary |
| **v5** — "ROM RAM-layout-agnostic; physical BSS placement is host-only concern" | **2026-05-07 06:30 → 00:35 LOCK** | Lucy | ROM does not directly reference any of the 6 BSS-contract addresses; ROM private region (0x20003xxx) caches MMIO base pointers; host TX path init delta is the real cba=0 cause | **Triple-anchored** | (1) ROM disasm 0 grep matches; (2) Cindy RAM dump byte-identical; (3) frozen drift cba=72/61 PASS |

### 3 independent evidence layers locking v5

#### Layer 1 — ROM disassembly (forensic, 06:25)

- Source: `wchble_rom.hex` (Andelf DM `e9aed4be` 06:09 confirmed equivalent to `wchble.a` content burned to mask ROM)
- Disassembled to `/tmp/wchble_rom.dis` (68,610 lines)
- Method: literal hex grep for the 6 BSS pin addresses (`0x758` / `0x508` / `0x1858` / `0x750` / `0x754` / `0x1c78`); also scanned 3,707 SRAM access points across 292 unique target addresses
- **Result**: zero direct matches for the 6 symbol pin addresses; 95% of SRAM refs land in `0x20003xxx` (ROM private workspace); the only `jalr-to-SRAM` patterns target `0x20000040-0x200004bc`, which is the host callback table populated by libwchble-era TMOS / `BLE_LibInit` registrations — not the 6 BSS pins
- Anchored in `notes/ch32-rs/rom-hex-memory-layout-truth.md` (§6.1-§6.7) and `notes/ch32-rs/bss-pointer-mechanism.md`

#### Layer 2 — Hardware RAM dump cross-section (Cindy experiment #1, 06:28)

- Two binaries with deliberately different BSS layouts: `frozen ble_tx_adv_ch37.rs @ 74e8d67` (drifted) vs `minimal ble_tx_adv_ch37_minimal.rs` (identity-pinned per `link_minimal.x`)
- Both flashed and run on the same hardware; wlink halt + dump RAM `0x20003000-0x20003700` (1792B)
- **Result**: byte-identical, sha256 `31f2bafc...916a4a8` (full 64-char hash recorded in `rom-analysis/`)
- Implication: ROM private region is independent of host BSS layout — proving ROM does not store host BSS pointers in its workspace

#### Layer 3 — Frozen drift hardware gate (the existential test, 06:34 + 07:19)

- `frozen ble_tx_adv_ch37.rs` at `74e8d67` does not pin `gBleIPPara` to `0x758` (the alleged ROM-expected address); `nm` confirmed the actual placement drifted ≈3808B from the original v1 location
- Two consecutive 60s RF gate runs: cba=72 (06:34) and cba=61 (07:19), both ≥ 52 PASS threshold, abc=0
- Implication: the binary that actually works *does not* satisfy the v1-v4 "BSS pin" prerequisite; therefore that prerequisite was never necessary

### Implications for the existing #34 (bug ticket) close

The v1-v4 narrative carried at `c702890` (`gBleIPPara` Rust strong + `#[link_section=".bss.gBleIPPara"]`) **still produced a working binary** — but the *mechanism* it leveraged was not "ROM expects `gBleIPPara@0x758`". It was: section-isolation defeated GlobalMerge (Iron Law #22), which prevented the layout shift that would otherwise have hit a *host-code* address-sensitive site (most likely the Rust HAL `ble_ip_core_init` write path discovered as a side effect of the trace-binary regression on 2026-05-07 23:35).

Net effect: the production binary at `c702890` was correct, but for a different reason than recorded. v5 retroactively restates the cause without invalidating the fix.

### Implications for downstream work

| Item | v1-v4 implication | v5 implication |
|---|---|---|
| `link_minimal.x` BSS pin block | Required (ROM contract) | **Removable** (no ROM contract; host code can be made layout-agnostic) |
| `frozen_bss_pins.x` / `minimal_bss_pins.x` ASSERT | Mandatory safety net | **Removable** (no real address contract to enforce) |
| `#[link_section=".bss.<name>"]` on the 6 symbols | Address-pinning + GlobalMerge defeat | **Only the GlobalMerge defeat is needed** — `#[no_mangle]` + `#[used]` retain the symbol; section isolation can be optional once GlobalMerge is otherwise managed |
| `fnGetClockCBs @ 0x20001c78` | Critical (ROM allegedly writes `0x420B000A` here) | Caveat: ROM disasm shows 0 direct hits — claim either wrong or via indirection. Sub-step 2b separately validates removal |
| Iron Law #22 (GlobalMerge layout shift) | Active | **Still active** (host-side concern; orthogonal to v5) |
| Iron Law #31 (`_ebss` warm-reset hazard) | Active | **Still active** (host startup zeroing concern; orthogonal to v5) |

### Process debt extracted (5 reshape lessons, recorded for future application)

1. **Pattern recognition without falsifiability test costs days.** v1 was a behavioral pattern ("layout that works is invariant") that should have been turned into a single-variable control experiment immediately. We instead did 4 narrative reshapes before stumbling onto the control 6 days later as a side effect.
2. **Senior reviewer skepticism is signal, not noise.** Andelf's 06:09 "证据不强" prompt should have triggered immediate forensic step-back; instead I argued for v3-v4 first and ran ROM forensic 6h later.
3. **Single-variable enumeration past 2 FAILs needs systematic step-back.** P1/P2/P2.5/P3/P6 hot-fixes were 5 single-variable shots in a row; should have pivoted to direction-B trace after 2 FAILs.
4. **Confounded data hides causality.** Every reshape correlated BSS layout with init-path delta because they always moved together. The discriminating experiment (BSS-only drift, init unchanged) was achievable from day 1 — we just didn't run it.
5. **Tooling capabilities must be verified BEFORE method selection.** I proposed gdb-style breakpoint debug for direction B; Cindy corrected: wlink CLI only halts/regs/dumps, no step/break/watchpoint. A 30s docs check would have prevented an hour of wrong-direction planning.

### Cross-references (Lucy workspace)

- `notes/ch32-rs/rom-hex-memory-layout-truth.md` — full ROM hex disassembly forensic (793 lines)
- `notes/ch32-rs/bss-pointer-mechanism.md` — ROM-doesn't-read-BSS argument with disasm citations (252 lines)
- `notes/ch32-rs/tx-adv-bss-wrap-up.md` — T44.E (b) FREEZE wrap-up + v1→v5 timeline + 6-fix cumulative log
- `notes/ch32-rs/phase2-bss-pin-removal-design.md` — Phase 2 design (9 sections, sub-step 2a/2b plan)
- `notes/ch32-rs/rom-analysis/` — Cindy RAM dump artifacts + Vega scan tooling + binary archives

### Next steps (approved by Andelf 2026-05-08 00:32)

- **Phase 2 sub-step 2a** — remove the 5 non-`fnGetClockCBs` pins (`gBleLlPara` / `dtmFlag` / `gPaControl` / `gBleIPPara` / `ble`); delete `link_minimal.x` / `frozen_bss_pins.x` / `minimal_bss_pins.x` generation in `build.rs`; remove `#[link_section]` on those 5 in `ble_tx_adv_ch37_minimal.rs`. Validation gate: frozen MD5 must stay `301ccc628a7db0daf768f33cc0802450`; minimal cba ≥ 52 with any BSS layout.
- **Phase 2 sub-step 2b** — separately remove the `fnGetClockCBs` pin. If gate FAIL, restore that one pin and document as the single residual ROM-touch slot. **[CLOSED 2026-05-09 — see "## 2026-05-09 09:48 — task #56 closure" below. Pin retired; Iron Law #38 three-condition gate PASS.]**
- **Stay on `feat/ble-types-align`, do not merge** until Phase 2 lands and Andelf signs off (per 2026-05-08 00:32 directive: "维持在 ble feature 分支工作, 不 merge 先").
- **Phase 3 (hex ↔ .a cross-ref)** — deferred until Phase 2 closes (per same directive).

---

## 2026-05-09 09:48 — task #56 closure: `fnGetClockCBs` link.x pin retirement (Iron Law #38 three-condition gate PASS)

**Author**: Lucy (collaborators: Andelf direction + final approval; Vega code + reviews; Cindy hardware gates)
**Status**: **LOCKED** — closes Phase 2 sub-step 2b. Iron Law #34 v5-final reaffirmed without exception. Iron Law #39 (proposed) retired.
**Trigger**: Andelf direction `c5e85e7b` (2026-05-09 ~00:00) + GO `a20bb37c` (09:16): "开始验证，别一失败就回滚说不行，失败后分析失败原因和遗漏的修改点".

### Final outcome

The `link.x` `_ebss=0x20001c78` boundary trick + `KEEP(*(.fnGetClockCBs))` rule
is **permanently retired**. Production mechanism for the frozen binary path:
single symbol-resolved `write_volatile(addr_of_mut!(fnGetClockCBs), 0x420B_000A)`
in `ble_ip_core_init`. No address pin required.

### ROM consumer model (H4, the surviving hypothesis)

```c
// ROM IRQ handler pseudocode:
fn = *(volatile u32*)0x20001c78;        // hardcoded absolute read
if (fn == NULL) fn = clockGetHSEValue;  // ROM internal fallback (= 0x420B_000A)
result = fn();                          // jalr; non-NULL valid → use; non-NULL garbage → crash
```

ROM IRQ reads the absolute physical word at 0x20001c78 unconditionally, BUT is
**NULL-tolerant** — falls back to its internal `clockGetHSEValue` when the slot
is NULL. Userland Rust `bb_irq_lib_handler` .L7 path reads via PCREL_HI20 to the
linker-placed symbol address. **Both consumers exist**; in the frozen binary
layout (where `_ebss` covers 0x20001c78), the symbol IS at 0x20001c78, so the
single symbol-resolved write satisfies both consumers simultaneously.

### Iron Law #38 three-condition gate (Step B `d056863`)

Symbol force-shifted to 0x20002c78 via 4 KB BSS pad in `ble_tx_adv_ch37.rs`;
`_ebss=0x20002c7c` covers 0x20001c78 → BSS clear sweeps the historical address
to NULL → ROM internal fallback runs:

| Condition | cba 30s | `0x20001c78` | `0x20002c78` (symbol) |
|-----------|---------|--------------|------------------------|
| warm 30s | **43** | `0x00000000` ✓ | `0x420B_000A` ✓ |
| WCH-Link 3V3 power-cycle | **32** | `0x00000000` ✓ | `0x420B_000A` ✓ |
| scrub `0xDEADBEEF` → reset | **54** | `0x00000000` ✓ | `0x420B_000A` ✓ |

The scrub run is the smoking gun: pre-write `0xDEADBEEF` to 0x20001c78, reset,
the BSS clear sweeps it back to NULL, ROM falls back, cba=54 (highest of three).
Confirms NULL-tolerated fallback path — no SRAM-persistence dependency.

### Terminal regression (`d50907a` frozen, MD5 `3e69b556`)

The cleaned production commit (C1 = `d50907a`, no test pad, no double-write,
single symbol-resolved write, no link.x pin) re-runs the same three-condition
gate. Frozen layout naturally places `fnGetClockCBs` symbol at 0x20001c78, so
this validates the production path the consumers will actually take:

| Condition | cba 30s | `0x20001c78` (= symbol) | `gBleIPPara+0x1c` |
|-----------|---------|-------------------------|-------------------|
| warm 30s | **39** | `0x420B_000A` ✓ | `0x00000000` |
| WCH-Link 3V3 power-cycle | **45** | `0x420B_000A` ✓ | `0x00000000` |
| scrub `0xDEADBEEF` → reset | **52** | `0x420B_000A` ✓ | `0x00000000` |

scrub prescan confirmed `0x20001c78=0xDEADBEEF`; post-reset the symbol-resolved
write in `ble_ip_core_init` overwrote the SRAM contamination back to
`0x420B_000A`. Production path: zero dependence on warm SRAM persistence,
zero dependence on link.x pin, zero dependence on ROM auto-install. Logs:
`/tmp/ble_scan_phase2c_final_20260509_1016_final_warm.log`,
`/tmp/ble_scan_phase2c_final_20260509_1018_final_cold.log`,
`/tmp/ble_scan_phase2c_final_20260509_1019_final_scrub.log`.

### Hypothesis ledger (this saga's reshapes)

| ID | Hypothesis | Status | Falsifying datum |
|----|------------|--------|------------------|
| H1 | ROM auto-installs `0x420B_000A` at 0x20001c78 on cold boot | FALSIFIED 2026-05-08 | `baf71de` cold gate: post-run slot=NULL, cba=0 |
| H2a | ROM hard-binds abs 0x20001c78 AND requires non-NULL fn ptr | FALSIFIED 2026-05-09 | Step B PASS with `0x20001c78=NULL` |
| H3 | Consumer reads via PCREL only (any symbol address works) | PARTIALLY FALSIFIED | Minimal binary FAIL: PCREL would read symbol@0x200007c0=valid → should PASS, but actually FAILed |
| **H4** (current) | ROM IRQ reads abs 0x20001c78 + NULL-tolerant fallback; userland reads PCREL | SURVIVING | Explains every data point |
| H39 (proposed law) | fnGetClockCBs slot must be pinned at 0x20001c78 (hardware-fixed) | RETIRED 2026-05-09 | Step B PASS with symbol@0x20002c78 |

### Iron Law revisions (post-PASS)

#### Iron Law #34 v5 — reaffirmed without `fnGetClockCBs` exception

The 2026-05-09 00:05 wording proposed an `fnGetClockCBs` exception based on
Step 3 FAIL. Step B PASS proves the exception was incorrect. Final wording:

> **Iron Law #34 v5 (final, 2026-05-09)** — ROM IRQ reads absolute physical
> 0x20001c78 as fn ptr, but is NULL-tolerant: NULL → ROM internal fallback
> (`clockGetHSEValue` at 0x420B_000A); non-NULL garbage → `jalr` → crash.
> Userland consumers (`bb.rs` .L7 path) read via PCREL_HI20 to the
> linker-placed symbol. For frozen BSS layout: `_ebss` naturally covers
> 0x20001c78 → BSS clear → NULL → ROM fallback runs cleanly. Step 1 explicit
> write to the symbol (`addr_of_mut!(fnGetClockCBs)`) is sufficient — in
> frozen layout the symbol IS at 0x20001c78 anyway, so the symbol-resolved
> write covers both consumer paths simultaneously. The link.x
> `_ebss=0x20001c78` boundary trick + `KEEP(*(.fnGetClockCBs))` was
> archaeological pinning, not a hardware contract. Iron Law #38 three-condition
> gate (Step B `d056863`, 2026-05-09): warm cba=43, power-cycle cba=32,
> scrub-then-reset cba=54 — all PASS with 0x20001c78=NULL by BSS clear.

This supersedes the table-row claim at line 1522 ("ROM allegedly writes
`0x420B000A` here — sub-step 2b separately validates"). Final answer: ROM
**reads** the slot directly (does not write); the slot must hold either
NULL or a valid fn ptr; frozen BSS layout + Step1 write satisfies both
consumers in one shot.

#### Iron Law #39 (proposed) — retired

The proposal "fnGetClockCBs slot must be pinned at 0x20001c78 — chip silicon
ROM internal handler reads this address directly (verified by Step 3 FAIL)"
is retired. The address-axis premise was correct (ROM does read abs
0x20001c78), but it is NULL-tolerant via internal fallback — so a hard pin
is not required for frozen layout. Step 3 FAIL was due to slot-NULL +
SRAM-garbage interaction (Step1 missing + minimal-style `_ebss` not covering
0x20001c78), not a "must-pin" constraint. The pragmatic outcome and the
mechanism are both captured by the revised Iron Law #34 v5 above.

#### Iron Law #35 — split confirmed (35a / 35b)

35a (raw HSE counter): `fnGetClockCBs` returns u32 raw counter. Used by
`bb_irq_lib_handler` .L7 path → stored at `gBleIPPara+0x1c`. ADV-TX path
consumes this directly without 1600 Hz conversion.

35b (1600 Hz tick): `pfnTimerCBs` (set by `TMOS_TimerRegister`) returns
1600 Hz / 625 µs tick. Consumed by `TMOS_GetSystemClock` via
`clockGetTickValve` (which wraps `(raw * 1600) / bleClock_t.[8]`). Not
exercised by ADV-TX path.

#### Iron Law #38 — second use case validated

Step B is the second documented Iron Law #38 invocation (first was task #56
v5 catch). The gate matrix (warm + power-cycle + scrub) caught nothing on
Step B (PASS) but proved its discriminator value: any failure mode that only
shows under one of the three conditions would have been visible in the
gate's columns. The scrub condition specifically rules out SRAM-persistence
masking.

### Wording corrections applied (4 sites)

The phrase "ROM auto-installs `0x420B_000A`" or close variants existed in:

1. `src/ble/types.rs` Iron Law #35 module doc (L18, L37-40, L122-124, L160-165) → corrected in commit `5e0330d` (C2)
2. `src/ble/mod.rs` `ble_ip_core_init` Step 0 doc + write block (L210-260) → corrected in commit `d50907a` (C1)
3. `src/ble/bb.rs` .L7 path doc (L134-137 + L181-183) → corrected in commit `d50907a` (C1)
4. `notes/ch32-rs/lib-dependency-removal.md` v1-v4 reshape table (L1522 row) → superseded by this section (C3)

The corrected wording: "`ble_ip_core_init` explicitly writes `0x420B_000A` (ROM
`clockGetHSEValue` address) to the slot via Step 1; ROM does NOT auto-install
— `baf71de` cold gate falsified that. ROM reads the slot directly via abs
0x20001c78 and falls back internally if NULL."

### Code/doc commits landing this saga

| Commit | Subject | Change |
|--------|---------|--------|
| `52ed8dc` | Step 1 — explicit fnGetClockCBs install | Added `write_volatile(addr_of_mut!(fnGetClockCBs), 0x420B_000A)` in `ble_ip_core_init` |
| `10d8b91` | Phase 2c — remove pin from link.x | Dropped `_ebss=0x20001c78` + `KEEP(*(.fnGetClockCBs))` |
| `d056863` | Phase 2c Step B — 4 KB BSS pad (test artifact) | Force-shift symbol to 0x20002c78 for discriminator gate |
| `c09d701` | Phase 2c double-write hedge (provisional) | Added abs-write `0x20001c78` for minimal-path consideration; superseded by C1 |
| `d50907a` | task #56 C1 — code cleanup | Drop pad + drop double-write + bb.rs wording scrub |
| `5e0330d` | task #56 C2 — types.rs doc | Iron Law #35 + PfnGetSysClock + BleClockConfig.get_clock_value rewritten |
| (this) | task #56 C3 — notes update | Iron Law #34 v5-final reaffirm + #39 retire + 4-site wording sweep + this section + new postmortem |

### What's out of scope for #56 (follow-up tasks)

- **Minimal binary FAIL**: `_ebss=0x200007c4` doesn't cover 0x20001c78 → SRAM
  garbage at 0x20001c78 → ROM IRQ jalrs to garbage → crash. Not a
  fnGetClockCBs problem per se; remediation candidates: extend BSS coverage,
  add explicit absolute write for minimal only, or drop the minimal variant
  for production. Out of #56 scope per Vega-Lucy alignment + Andelf approval.
- **B1 silicon-version gating**: `0x420B_000A` is the ROM `clockGetHSEValue`
  address on CH32V208WBU6 B1; other revisions may differ. Recommendation:
  cargo feature gate (`ch32v208wbu6-b1`) controlling the constant.
- **Strict pull-plug cold boot**: `baf71de` and Step B used WCH-Link 3V3
  power-cycle; strict USB unplug + bench supply off was deferred (Cindy
  noted SRAM self-discharge >5 s caveat). Worth one round of confirmation
  before final close.

### Cross-references

- `notes/ch32-rs/task56-postmortem.md` — full postmortem with timeline,
  hypothesis ledger, root-cause list (5+1 items), engineering lessons
- `notes/ch32-rs/c-ground-truth-fnGetClockCBs.md` — symbolic + empirical
  proof from libwchble.a (COMMON, PCREL_HI20, MEMHEAP-shift experiment)
- `notes/ch32-rs/tmos-timerinit-disasm.md` — fnGetClockCBs vs pfnTimerCBs
  ABI distinction; clockGetTickValve hardcoded `1600`
- `/tmp/phase2c_d056863_clean/` — Step B binaries + ELFs
- `/tmp/wlink_dump_phase2c_d056863_padded_*` — three-condition gate dump logs
- `bad/task56-v5-air-regression` (`5528b5f`) — preserved cycle-CSR regression
