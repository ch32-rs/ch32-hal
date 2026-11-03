# CH32V4 驱动边界条件复查（2026-11-03）

本次上机验证在 CH32V467WEU6 上发现并修复了两个挂起。两者都不是时序/性能问题，而是
**在错误的状态下轮询了错误的状态位**：

| # | 位置 | 现象 | 根因 |
| --- | --- | --- | --- |
| 1 | `src/spi.rs` `blocking_flush` / `async flush` | `hal_spi` 第一个 `blocking_write` 永久自旋 | CH32V4x7 在 `SPE=0` 时 `STATR.TXE` 读 0，而 `prepare()` 在使能 SPE 之前先 flush |
| 2 | `src/flash/v4x7.rs` `program_page` | `flash-write-test` 报 `FLASH_RESULT=FAIL code=21`（`Timeout`） | 快速页编程装载阶段等了 `BSY|WRBSY`；RM 要求只等 `WRBSY`，而 `BSY` 在整个 256 字节装载期间保持置位 |

两个修复都已在硬件上复验通过（见第 6 节），并与 embassy-stm32 的处理路径做了对照（第 2、3 节）。

---

## 1. 主机测试 harness 与 `x86_64-unknown-linux-gnu`

`tools/{i2c,usbhs,sdio,eth,i3c}-host-tests/README.md` 里的命令写死了：

```sh
--target x86_64-unknown-linux-gnu
```

**这不是 harness 的硬性要求，而是 CI/参考环境（Linux）的三元组。** 原因如下：

- 这些 crate 是普通的 `std` 主机程序：`build.rs` 把生产源码（如 `src/i2c.rs`）拷进
  `OUT_DIR`，配一份手写的、由 RAM 后备的 PAC（`tools/*/src/common.rs`），
  不包含芯片级 `pac.rs`，也没有 `#[link_section]`。
- 因此它们与宿主对象格式无关。2026-11-03 在本机 macOS（`aarch64-apple-darwin`）**不加
  `--target`** 直接跑全部 harness，全部通过：

| harness | 结果 |
| --- | --- |
| `tools/i2c-host-tests` | 23 passed |
| `tools/usbhs-host-tests` | 27 passed |
| `tools/sdio-host-tests` | 34 passed |
| `tools/eth-host-tests` | 16 passed |
| `tools/i3c-host-tests` | 50 passed |
| `tools/spi-host-tests`（新增） | 13 passed（含 5 项 SPI 生命周期回归） |
| `tools/flash-host-tests`（新增） | 3 passed |

真正不能在 macOS 上跑的是 **HAL / 示例 crate 自身的 `cargo test`**：生成的 PAC 与
`qingke-rt` 使用 ELF 段名（`.vector_table.*`、`.init.rust`、`.trap.rust`），rustc 对
Mach-O 会报 `invalid Mach-O section specifier`。这与 `tools/` 下的 harness 是两回事。
建议把 README 里的 `--target` 说明为“CI 参考命令（可选）”，避免误导。

---

## 2. SPI：`flush` 在 `SPE=0` 时死锁

### 2.1 现象与定位

`hal_spi` 在 SPI1 的第一条 `blocking_write` 处卡死，`SPI_RESULT` 停在 1、`SPI_CASES=0`。
暂停内核后：

- `PC = 0x396c`，落在 `Spi<SPI1, Async>::blocking_flush` 的自旋循环；
- `SPI1_CTLR1 = 0x00000324`（`MSTR/SSM/SSI=1`，**`SPE=0`**）；
- `SPI1_STATR = 0x1`（只有 bit0；bit1 TXE=0）。

调试器手动写 `CTLR1 |= SPE` 后 `STATR` 立即变为 `0x3`（RXNE=1、**TXE=1**），证实
**V4x7 的 `TXE` 只在 `SPE=1` 时有意义**。

调用路径：`new_inner()` 留下 `SPE=0` → `blocking_write()` → `prepare()` →
**先** `blocking_flush()` → 轮询 `TXE && !BSY` → 永远不成立。

### 2.2 修复

- `src/spi.rs:284` `blocking_flush()`：进入轮询前先查 `CTLR1.SPE`，为 0 直接返回 `Ok`
  （禁用状态下不可能有在途传输）。
- `src/spi.rs:632` `async flush()`：同一边界，同样先查 `SPE`。

### 2.3 与 embassy-stm32 对照

embassy-stm32 的同类路径**同样有“先判使能再轮询”的保护**，只是位置不同：

- `embassy-stm32/src/spi/mod.rs:661-694` `set_direction()`：先读
  `let was_enabled = cr1().spe()`，只有 `was_enabled` 时才
  `while !sr().read().txe() {}`、`while sr().read().bsy() {}`，然后才动 `SPE`。
  注释明确写的是“等待 TXE=1 再等 BSY=0，之后才能安全关闭 SPE”。
- `blocking_write()`（`mod.rs:818`）先置 `SPE=true` 再写数据，不存在“禁用时 flush”的窗口。
- embassy-stm32 的 `embedded_hal::spi::SpiBus::flush`（`mod.rs:2354`、异步版 `2387`）
  直接 `Ok(())`，因为 `blocking_write`/`write` 自身已等到传输结束。

结论：**处理路径的原则一致**（先确认外设处于可轮询状态，再等状态位），差异在于
ch32-hal 把 `flush` 暴露成独立 API，并且 `prepare()` 会在使能之前调用它，所以必须在
`flush` 内部做 guard；`blocking_flush` 与 async `flush` 两处都要。

### 2.4 同类教训（embassy-stm32 也有）

`mod.rs:835-846` 记录：STM32 spi_v1/v2 的 `BSY` 会过早清零，官方 workaround 是等 `RXNE`
（STM32 gotcha g68）。这与本次问题同属“状态位语义随 IP 版本变化，必须按 RM 逐位确认”。

---

## 3. Flash：快速页编程等错了 busy 位

### 3.1 现象与定位

`hal_flash --features hal,flash-write-test` 在第一次 512 字节快速页写时报
`FLASH_RESULT=FAIL code=21`。临时打印得到 `Error::Timeout`。

失败时的寄存器：

- `FLASH_STATR = 0x1` → `BSY=1`、`WRBSY=0`；
- `FLASH_CTLR = 0x00010000` → `FTPG=1`，`PGSTRT=0`；
- `FLASH_ADDR = 0x080F0000`（擦除阶段留下的地址）。

即：驱动在数据装载循环里等 `BUSY_MASK = BSY|WRBSY` 清零，但 `BSY` 要到 `PGSTRT`
之后才清零，而 `PGSTRT` 永远执行不到 → 50M 次轮询后 `Timeout`。

CH32V407RM §32.4.3：

| 位 | 名称 | 语义 |
| --- | --- | --- |
| 0 | `BSY` | 操作忙（整个快速页编程期间保持置位） |
| 1 | `WRBSY` | 快速页编程时“编程数据正在写入” |

RM §32.5.6 步骤 5–9：

1. 32 位方式写 Flash 地址；
2. **等待 `WRBSY` 为 0**，再写下一个字（共 64 次）；
3. 置 `PGSTRT`；
4. **等待 `BSY` 为 0**（或 `EOP`）。

### 3.2 修复

- `src/flash/v4x7.rs:19` 新增 `WRBSY_MASK = 1 << 1`；
- `src/flash/v4x7.rs:404-421` 新增 `wait_write_buffer()`，只等 `WRBSY`，并保留
  `WRPRTERR` → `Protected` 的映射；
- `src/flash/v4x7.rs:466` `program_page` 的装载循环改用它；`PGSTRT` 之后仍用
  `wait_idle()`（`BSY`）。

修复后 `flash-write-test` 全流程 PASS（4 KiB 擦、快速页写、重复写拒绝、半字写、
32 KiB 块擦、NOR 分区编码）。

### 3.3 与 embassy-stm32 对照

embassy-stm32 对不同系列的“操作忙”和“队列/子阶段忙”做了显式区分，路径与本次修复一致：

| 系列 | 等待条件 | 说明 |
| --- | --- | --- |
| STM32H7 `flash/h7.rs:236-262` | `!sr.bsy() && !sr.qw()` | `BSY` = 整次操作；`QW` = 写队列满。每个 32 位字写完后 `wait_ready()`，与 V4x7 的 `WRBSY` 完全同构 |
| STM32C5/H5 `flash/c5.rs:213-220, 336-342` | `!bsy && !dbne` / `bsy \|\| dbne \|\| wbne` | 数据缓冲空、写缓冲空都是独立位 |
| STM32G0 双 bank `flash/g.rs:161-175` | `bsy \| bsy2 \| cfgbsy` | “状态位不是同一个名字” |
| STM32F1/F3 `flash/f1f3.rs:56-66` | 置 `STRT` 后先空读一次再查 `BSY` | `BSY` 有一拍延迟 |

结论：**embassy-stm32 的成熟做法正是“按操作阶段选择对应的 busy 位”**；本仓库的 V4x7
后端首次实现时把两者合并成一个掩码，属于对 RM 细节的误读，已按 RM 修正。`embassy-stm32`
还把错误提取集中在每族的 `get_result()`（`Protected/Seq/Prog/...`）；本仓库对应的是
`wait_idle`/`wait_write_buffer` 里的 `wrprterr` 检查，加上 `Verify`/`NotErased`/`Timeout`。

---

## 4. 其他结构性差异（供后续评审参考）

| 主题 | embassy-stm32 | ch32-hal（V4x7） |
| --- | --- | --- |
| 忙等上限 | 无限自旋 / 由中断唤醒 | `Config::poll_limit`（默认 50M 次），超时返回 `Timeout` 并保留 `recover()` |
| 异步等待 | 外设 IRQ + waker | 同样用 IRQ + waker；blocking 路径用自旋 |
| 错误检查 | 每族 `get_result(sr)`，一次映射全部错误位 | 分阶段检查 `wrprterr`，其余错误在 `operation()` 前检查 `Seq`/`Busy` |
| flush 语义 | `SpiBus::flush` 为空操作（写路径已等待） | `flush` 是真实的等待 API，因此需要自己的状态边界 guard |

`poll_limit` 是一个有意的取舍：避免无限挂起，但让“硬件卡死”表现为 `Timeout` 而不是
永久阻塞；`recover()` 允许在硬件空闲后重试。

---

## 5. 本次上机验证摘要（CH32V467WEU6）

设备曾因旧版 `hal_time_scale,clock-slow` 把 HCLK 留在约 39 kHz 而无法 attach
（`81 55 01 01`）。恢复方式是 `wlink erase --method power-off --chip CH32V4X7`
（断电整片擦除，不动 option bytes）。恢复后：

- PAC-only：`isa`、`cpuid`、`rtt` 正常；
- `hal_init`（默认/分频）、`hal_blinky`、`hal_rcc`（HSI 9 组 / HSE 14 组）、
  `hal_embassy`、`hal_time_scale`（默认/PLL/AHB÷512）、`hal_systick_stress`
  （512 轮，默认与分频；负向对照按预期 `FAIL code=13`）PASS；
- `hal_dma`（24 通道/216 拷贝）、`hal_usart`（10/10 TX）、`hal_timer`（24 cases）、
  `hal_i2c`（配置/清理）、`hal_spi`（24 cases，修复后）、`hal_dac`、`hal_rng`、
  `hal_adc`、`hal_can`（72 帧 loopback）、`hal_i3c`、`hal_flash`（只读 + 写测试）PASS；
- `hal_eth` 初始化成功（PHY ID `0x73719011`），无网线故未验帧收发；
  `hal_sdio` 无卡时正确报告 “no card response”。

**注意**：持续 RTT/调试访问会 halt 内核，导致 `hal_usart`/`hal_timer`/`hal_rcc`
的时序断言出现假失败（code 11）。时序类用例应 `probe-rs download` + `reset`
自主运行后再 `probe-rs read` RAM 标记，本次均按此方式取得 PASS。

---

## 6. 后续处理（2026-11-03 已完成）

1. **已完成**：新增 `tools/spi-host-tests` 与 `tools/flash-host-tests`，直接编译生产
   源码（`build.rs` 注入，不维护副本）：
   - SPI：模型编码“`SPE=0` 时 `STATR.TXE=0`”，覆盖 `blocking_flush`/async `flush`
     的禁用路径、`blocking_write` 经 `prepare()` 的首个 flush（原 `hal_spi` 死锁）、
     以及使能后 `TXE && !BSY` 的等待路径（async 在 `BSY=1` 时保持 Pending）。
   - Flash：模型编码“`FTPG` 后 `BSY` 保持到 `PGSTRT`，`WRBSY` 只标记字装载”，
     覆盖 `wait_write_buffer` 忽略 `BSY`、`program_page` 在装载期 `BSY=1` 时完成、
     以及 `WRBSY` 卡住时仍返回 `Timeout`。`build.rs` 会去掉 ELF 专用
     `#[link_section]`，并用 CH32V467WEU6 的 992 KiB / 256 字节页替换
     `FLASH_SIZE`/`WRITE_SIZE`（主机没有生成的内存布局）。
2. **回归有效性已做反向验证**：临时把 `program_page` 的等待改回 `wait_idle`，
   flash 用例以 `Err(Timeout)` 失败（与板上 `code=21` 一致）；临时删除两处
   `SPE` guard，SPI 用例在 20 秒超时内不返回（复现死锁）。恢复源码后全部通过。
3. **已完成**：`examples/ch32v467/README.md` 与 `local-docs/ch32v4-migration.md` 增补
   2026-11-03 的板测恢复与结果，并修正“调试连接待恢复”的旧表述。
4. **已完成**：`tools/*-host-tests/README.md` 注明 `--target x86_64-unknown-linux-gnu`
   只是 Linux CI 参考，macOS 可原生运行；新增的两个 harness 也包含该说明。

剩余（非本轮范围）：`Flash::blocking_write` 走 `FLASH_BASE = 0x0800_0000` 的绝对地址，
主机无法执行；host 用例只驱动同一 `program_page` 例程，公共封装与擦除流程仍由
`hal_flash --features flash-write-test` 在板上覆盖。
