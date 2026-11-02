# CH32V4 分批提交清单

整理日期：2026-11-03。对应 `feat/ch32v4-support`，最初整理时 HAL HEAD 为
`dffc47e`，ch32-data HEAD 为 `081bbdb`，本地 qingke HEAD 为 `5046007`。

## 2026-11-03 执行调整

- ch32-data #78 已合并，用户允许临时开发分支使用 Git 依赖；HAL 与 V467 示例固定到
  生成 PAC 的 `e150d5303c6980d4d1cc708944654d3b9f47295a`，对应数据源 `844640b`。
  根依赖和 build-dependency 使用同一个 revision；示例 qingke/qingke-rt 改为 crates.io 0.8.3。
  下文“先使用本地 PAC”的内容保留为整理时的历史条件。
- #232 已通过 merge 纳入本分支，保留主线提交身份；未重复提交该修复。
- `init()` 总会初始化 DMA；首批将 RCC/启动、EXTI、Embassy、DMA 及基础例程一起提交，
  避免引入临时空 DMA 实现。原 H01/H02/H03/H04 合并为一个基础批次。
- 外设仍按 USART、Timer、I2C、SPI、DAC/RNG、ADC、Flash、CAN、USBHS、SDIO、ETH、I3C
  依次提交。共享模块 gate、生成代码 allowlist、Cargo feature/bin 声明随批次加入。
- 每批将暂存树导出到 `/tmp/ch32v4-commit-review/hal`，使用独立 target 构建
  V467 blocking/Embassy、V307 Embassy 及本批例程后才提交。
  I2C/USBHS/SDIO/ETH/I3C 批次另外执行对应主机测试。
  主机模型仍使用本地生成 PAC；固件构建使用固定 Git PAC，两者的来源范围分别记录。
- 下面 17 批表为最初拆分方案；实际为一个基础批次、12 个外设批次和一个文档收尾批次。

下文保留最初准备的提交边界；实际提交及验证结果以“HAL 提交完成记录”为准。
整理前 HAL 有 26 个已跟踪文件修改、90 个新增文件；ch32-data 有 18 个已跟踪文件修改；
两个仓库暂存区均为空，本地 qingke 无未提交修改。本文不计入上述新增文件数量。

## 先处理的基线

- `src/embassy/time_driver_systick.rs` 与主线修复 `e194315`（#232）完全一致。
  后续提交前先纳入该主线修复，再拆本分支改动；不要重新提交为 V4 新功能。
  当前没有执行 merge/rebase，也没有改变现有分支历史。
- 保留用户要求的本地 `ch32-metapac`。根 Cargo patch、示例 patch 和主机测试中的
  `../ch32-data/build/ch32-metapac` 依赖必须在说明中写明。
  这些提交在本地相邻仓库布局下开发；上游可独立复现和发布前，须发布对应 PAC 并切换版本。
- 不提交生成的 PAC、target、临时日志。先提交数据源修正，再重新生成 PAC。
- 示例继续使用本地 qingke；根 HAL 的最低版本提升到 0.8.3，匹配 V3V feature。
- 每个驱动批次连同例程、测试、相关 README 段落一起提交。
  不把整个最终版 `src/lib.rs`、`build.rs` 或示例 Cargo.toml 提前放进启动批次。

## ch32-data：6 批

| 编号 | 建议 commit subject | 完整文件边界 | HAL 前置关系 |
| --- | --- | --- | --- |
| D01 | `fix(ch32v4): correct V3V core and 32-bit SysTick metadata` | 六个 `data/chips/CH32V{407,467}{RET6,VET6,WEU6}.yaml`；`data/registers/systick_v4x7.yaml` | H01/H03 |
| D02 | `fix(ch32v4): correct RCC and DMA register metadata` | `data/registers/rcc_v4x7.yaml`、`data/registers/dma_v4x7.yaml`、`data/dma/CH32V_V4X7_DMA2.yaml`、`data/family/CH32V4X7.yaml` | H01/H04；USART5 TX 映射为 DMA2 CH4 |
| D03 | `fix(ch32v4): correct ADC DAC and RNG metadata` | `data/registers/adc_v4x7.yaml`、`data/peripherals/V4x7_ADC_DAC.yaml`、`data/peripherals/V4x7_MISC.yaml` | H09/H10；DAC 使用 v3，RNG 使用独立内核时钟 |
| D04 | `fix(ch32v4): correct USBHS DMA address width` | `data/registers/usbhs_v4x7.yaml` | H13 |
| D05 | `fix(ch32v4): align SDIO and Ethernet fields with reference manual` | `data/registers/sdio_v4x7.yaml`、`data/registers/emac_v4x7.yaml` | H14/H15；可在实际 review 时再拆 SDIO/ETH |
| D06 | `fix(ch32v4): clarify I3C controller and FIFO semantics` | `data/registers/i3c_v4x7.yaml` | H16；包括 legacy I2C 的 MEND 约束 |

以上覆盖数据仓库全部 18 个修改文件。数据验证需使用该仓库已有生成及检查流程，
HAL 构建必须使用这些修正重新生成的 PAC；不能仅因 YAML diff 小就省略生成验证。

## HAL：按依赖提交 17 批

下表 `examples/...` 均指 `examples/ch32v467/`。每一行均包含该批次新增的
Cargo feature、bin 声明及 README 对应章节；共享文件的拆分规则见下一节。

| 编号 | 建议 commit subject | 主要文件 / 边界 | 验证重点 |
| --- | --- | --- | --- |
| H01 | `feat(ch32v4): add core clock and minimal HAL bring-up` | 根 `Cargo.toml`；`src/rcc/{mod.rs,v4x7.rs}`；`src/delay/{mod.rs,impl_systick_v4x7.rs}`；`build.rs` 的 V3V 分类与初始隔离；`src/lib.rs` 初始隔离；`examples/.../src/lib.rs`、`src/bin/{blinky,hal_init,hal_blinky,hal_rcc}.rs`；基本 HAL/clock features | 六芯片无 Embassy 构建；HSI/HSE/PLL/分频边界；PAC-only 示例回归 |
| H02 | `feat(exti): support CH32V4 GPIO interrupt routing` | `src/exti.rs`；`examples/.../src/bin/hal_exti.rs`；exti feature | 路由/屏蔽/取消路径；输入板测仍待接线 |
| H03 | `feat(embassy): add CH32V4 32-bit SysTick time driver` | `src/embassy/{mod.rs,systick_time_scale.rs,time_driver_systick_32bit.rs}`；`build.rs` 禁止未支持的 V4 TIM time driver；`examples/.../src/bin/{hal_embassy,hal_systick_stress,hal_time_scale}.rs`；Embassy/stress features | 分频换算、软件进位、alarm；自然溢出与最新版完整板测待补 |
| H04 | `feat(dma): support CH32V4 channel windows and transfer errors` | `src/dma/{mod.rs,dma_bdma.rs,dma_v4x7.rs}`；`build.rs` DMA2 元数据选择；`examples/.../src/bin/hal_dma.rs` | DMA1 13 / DMA2 11 通道；扩展窗口；取消/错误/剩余计数 |
| H05 | `feat(usart): support CH32V4 USART and DMA transfers` | `src/usart/{mod.rs,tests.rs}`；`src/patches.rs` 的 USART1 双位 remap；`examples/.../src/bin/hal_usart.rs` | 时钟/分频/remap；DMA 回环待板测 |
| H06 | `feat(timer): support CH32V4 timers and PWM` | `src/timer/{mod.rs,low_level.rs,simple_pwm.rs,complementary_pwm.rs,tests.rs,register_tests.rs}`；`build.rs` timer UP DMA；`examples/.../src/bin/hal_timer.rs` | 寄存器和 PWM 边界；实际波形待板测 |
| H07 | `feat(i2c): integrate checked DMA transfers and lifecycle tests` | `src/i2c.rs`、`src/i2c/tests.rs`、`tools/i2c-host-tests/`；`examples/.../src/bin/hal_i2c.rs` | 主机生命周期、STOP/NACK、DMA 错误/取消；真实设备待板测 |
| H08 | `feat(spi): support CH32V4 blocking and async transfers` | `src/spi.rs`、`src/spi/tests.rs`；`examples/.../src/bin/hal_spi.rs` | 频率、模式、DMA；I2S 不在本批 |
| H09 | `feat(ch32v4): support DAC and RNG` | `src/dac.rs`、`src/dac/tests.rs`、`src/rng.rs`、`src/rng/tests.rs`；DAC pin/DMA 生成；`examples/.../src/bin/{hal_dac,hal_rng}.rs` | D03；DAC 输出、RNG 时钟与错误处理；可按外设再拆 |
| H10 | `feat(adc): add CH32V4 ADC backend` | `src/adc/{v4x7.rs,tests.rs}`；ADC 模块和 DMA 生成；`examples/.../src/bin/hal_adc.rs` | D03；配置和 DMA；模拟精度待板测 |
| H11 | `feat(flash): add CH32V4 flash backend` | `src/flash/{mod.rs,v4x7.rs,v4x7_tests.rs}`；根 memory-split-usr feature；`examples/.../flash-test.x`、`build.rs`、`src/bin/hal_flash.rs` | 容量/分区边界；写测试须显式 feature；不修改 option bytes |
| H12 | `feat(can): add CH32V4 CAN backend` | `src/can/{v4x7.rs,v4x7_types.rs,v4x7_tests.rs}`；CAN 模块/pin 生成；`examples/.../src/bin/hal_can.rs` | 位时序、过滤器和错误；总线板测待补 |
| H13 | `feat(usbhs): add CH32V4 USB device backend` | `src/usbhs/v4x7*.rs`、`tools/usbhs-host-tests/`；USBHS 模块/pin 生成；`examples/.../src/bin/hal_usbhs.rs`、`usbhs_echo_test.py` | D04；EP 生命周期；双实例按封装；device-only |
| H14 | `feat(sdio): add CH32V4 SD storage and DMA backend` | `src/sdio/`、`tools/sdio-host-tests/`；SDIO 模块/pin/DMA 生成；`src/patches.rs` 的 SDIO cfg；`examples/.../src/bin/hal_sdio.rs` | D05；SD 卡存储、阻塞/异步 DMA；无 SDIO IO/MMC/UHS |
| H15 | `feat(eth): add CH32V4 embassy network backend` | `src/eth/{v4x7.rs,v4x7_ring.rs}`、`tools/eth-host-tests/`；ETH 模块选择；`examples/.../src/bin/hal_eth.rs`、`eth_echo_test.py` | D05、H14 的测试代理；真实 PHY/link/frame 待测 |
| H16 | `feat(i3c): add CH32V4 controller with CCC DAA and DMA` | `src/i3c.rs`、`src/i3c/{transfer.rs,dma.rs}`、`tools/i3c-host-tests/`；I3C 模块/pin/DMA 生成；`examples/.../src/bin/hal_i3c.rs` | D06、H04、H14；FIFO/CCC/DAA/DMA 主机覆盖；原生 I3C target 板测待补 |
| H17 | `docs(ch32v4): record migration status and validation boundaries` | `local-docs/ch32v4-migration.md`、本文；README 剩余总览更新 | 审核记录与最终树一致；不将未测项写成通过 |

H01 将当前完整 RCC 与阻塞 delay 一起纳入，避免为重建历史阶段而临时实现 HSI stub。
H01 必须暂时屏蔽尚未提交的 V4 外设模块及对应生成代码，H04～H16 逐项解除；
不能把最终工作区直接作为 H01 快照。H03 后才承诺有/无 Embassy 的完整构建组合。

I3C 当前 FIFO、DMA API、测试模型、例程 feature 有交叉引用，先作为一个可验证的批次。
若审阅认为过大，须先构造并验证不含 DMA 的中间版本，再拆 FIFO/CCC/DAA 和 DMA；
仅按新文件 `dma.rs` 单独暂存不能形成正确边界。

## 共享文件的拆分规则

| 文件 | 拆分归属 |
| --- | --- |
| 根 `Cargo.toml` | H01：qingke 0.8.3、六芯片 feature、本地 PAC patch；H11：memory-split-usr |
| `build.rs` | H01：内核选择、V4 初始 pin/DMA allowlist；H03：time-driver 限制；H04：DMA2；H06：timer UP；H09：DAC；H10：ADC1/2；H16：I3C。随每批驱动扩展 allowlist |
| `src/lib.rs` | H01 建立隔离；各后端随对应批次启用。ADC/CAN/ETH 需同步替换旧版 gate，不能同时启用旧、新模块 |
| `src/patches.rs` | H05：USART1；H14：SDIO |
| 示例 `Cargo.toml` | H01：HAL 可选依赖、芯片透传、patch、启动/时钟例程；H03：executor/time/critical-section；其余依赖/feature/bin 随驱动加入 |
| 示例 `README.md` | 按对应 bin/feature 的说明段落分批加入；总览与最终状态在 H17 收尾 |
| 示例 `src/lib.rs` | H01：32-bit PAC 访问修正、PE8/PE9 LED；原 PAC-only bin 必须一起验证 |
| 示例 `build.rs` | H11：flash-test.x 链接与 rerun-if-changed |

注意 `tools/eth-host-tests/src/lib.rs` 和 `tools/i3c-host-tests/src/lib.rs`
均引用 `tools/sdio-host-tests/src/common.rs`，因此当前顺序要求 H14 在 H15/H16 之前。
这次整理保留现有测试结构；后续如果抽公共 fixture，应单独验证后再改提交边界。

## 每批执行与验收

1. 保留原工作区；基于已纳入 #232 的基线逐批构造暂存内容。
   对共享文件使用按块暂存或构造精确 index 内容；不得整文件提前暂存。
2. 将该批次暂存快照导出到临时目录，与最终工作区分开构建。
   只测试最终工作区不足以证明前序提交可构建。
3. 运行本批测试及受影响旧系列回归；主机 harness 见各自 README。
   基础 RCC/DMA/生成代码批次回归六个 V4 和 V003/V103/V307/H417，
   H03 起检查 blocking/Embassy 两种配置。
4. 示例指定独立 `CARGO_TARGET_DIR=/tmp/...`，避免现有示例 target 路径问题。
   避免 cargo fmt 跟随测试 harness 的 path 模块批量格式化生产文件。
5. 检查 `git diff --cached --check`、完整 staged diff 和文件清单，确认无跨批依赖后再提交。
   不把缺少硬件的项目标成硬件通过；可提交软件实现，但说明验收范围。

## HAL 提交完成记录（2026-11-03）

主线 #232 通过 merge commit `dbbeac5` 纳入。功能提交按以下顺序落地，
每批均在导出的暂存树中构建 V467 blocking/Embassy、V307 Embassy 及本批例程后提交。

| 顺序 | Commit | 范围 |
| --- | --- | --- |
| 1 | `db2dc1c` | Git PAC、V3V 启动、RCC、GPIO/EXTI、Embassy、DMA、基础例程 |
| 2 | `deaa262` | USART |
| 3 | `29ef3c1` | Timer/PWM |
| 4 | `f531340` | I2C 与 DMA 生命周期测试 |
| 5 | `c89b80a` | SPI |
| 6 | `8eb844c` | DAC/RNG |
| 7 | `13fd572` | ADC |
| 8 | `06aab41` | Flash |
| 9 | `e830289` | CAN |
| 10 | `3524c95` | USBHS device |
| 11 | `b8546db` | SDIO SD 存储 |
| 12 | `7aa2cb2` | Ethernet |
| 13 | `accbadf` | I3C FIFO/CCC/DAA/DMA |
| 14 | 文档收尾提交 | 完整例程 README、迁移记录、本文 |

最终代码树使用固定 Git PAC 完成以下复验：

- 六款 V407/V467 的 blocking/Embassy 库构建，共 12 组；
  V003/V103/V307/H417 V5F 的 Embassy 回归，共 4 组，全部通过。
- V467 默认 PAC-only 全部 bin、`embassy,usbhs,eth` 下全部可用 bin 构建通过。
- 额外构建通过：独立 blocking `hal_init`；I3C DMA 默认、AHB 分频、remap、RET6、
  DMA+DAA 回环例程，共 6 组。回环只构建，原始 I3C timing 编译夹具不代表有效板上配置。
- 各批主机测试通过：I2C 23、USBHS 27、SDIO 34、ETH 16、I3C 50，共 150 项。
  主机模型使用本地生成 PAC；固件构建使用 Git PAC，未把前者描述为独立 Git fixture 验证。
- 日志：`/tmp/ch32v4-stage-commits.log`、`/tmp/ch32v4-git-final-matrix.log`、
  `/tmp/ch32v4-git-example-variants.log`。

这轮只做构建与主机验证，没有新增板测。自然 SysTick 溢出、GPIO 输入及其他外设的
真实硬件验收继续按迁移文档执行。提交保留在本地开发分支，尚未推送或创建 HAL PR。

## 最初整理与数据 PR 记录

### ch32-data 已提交 PR（2026-11-03）

- PR：[ch32-rs/ch32-data#78](https://github.com/ch32-rs/ch32-data/pull/78)，目标 `main`，
  分支 `fix/ch32v4-register-baseline`。
- D01～D06 已按表中边界提交并推送，commit 依次为 `21c627b`、`653fb74`、
  `fc65721`、`5d8831e`、`d12dde7`、`202089b`；数据仓库工作区干净。
- 完整 `./d gen`、workspace tests、六个 V4 型号及 V307 PAC 构建、V003 PAC 元数据
  主机测试均通过。构建 target 为 `riscv32imc-unknown-none-elf`。
- HAL 尚未提交；以下准备记录保留为此前整理时的状态。

- 已完成文件归类、数据与 HAL 依赖排序、共享文件边界和测试 fixture 依赖核对。
- #232 文件与主线一致已经通过 `git diff e194315 -- src/embassy/time_driver_systick.rs` 核对。
- 本次只整理文档与复跑最终工作区主机测试，不新建分支、不修改暂存区、不执行提交。
- 本次主机复验全部通过：I2C 23、I3C 50、USBHS 27、SDIO 34、ETH 16，
  共 150 项。日志位于 `/tmp/ch32v4-commit-prep-{i2c,i3c,usbhs,sdio,eth}.log`。
- HAL 与 ch32-data 的 `git diff --check` 均通过；两个仓库的暂存区仍为空。
- 各批中间快照尚未构造或构建；上表是提交边界清单，不能标记为 17 个已验证提交。
- 最近的六芯片及旧系列库构建、I3C 示例构建记录见迁移文档；
  本次整理不替代逐批提交时的快照验证。
- CRC/watchdog 上游 PR 参考及 OPA 待办继续保留在迁移文档；尚无这些外设的新实现进入本次提交范围。
