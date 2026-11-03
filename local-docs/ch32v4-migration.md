# CH32V407 / CH32V467 HAL 与 Embassy 迁移计划

更新日期：2026-11-03。范围以 `../ch32-data` 的 V4x7 寄存器定义为基线，
硬件验收以 CH32V467WEU6、PE8/PE9 LED 和 WCH-Link 为准。

2026-11-03 的变更归类、依赖顺序和提交边界见 [分批提交清单](ch32v4-commit-plan.md)。

2026-11-03 依赖更新：ch32-data #78 已合并为 `844640b`；HAL 的运行时/构建依赖和
V467 示例均固定使用生成 PAC 仓库的 Git revision
`e150d5303c6980d4d1cc708944654d3b9f47295a`。示例的 qingke/qingke-rt 使用 crates.io
0.8.3，固件构建不再依赖相邻 PAC/qingke checkout。下文原本地 patch 流程保留为历史记录；
当前 `tools/` 主机模型仍按各自 README 使用相邻目录中生成的 PAC 寄存器源文件。

## 顺序与验收

| 阶段 | 范围 | 验收 |
| --- | --- | --- |
| 0：基线 | 纳入 main 的 SysTick 一致性读取修复 #232；核对 V3V 内核、PAC 元数据、运行时 feature；固定本地依赖与 ISA 配置 | cpuid、isa、RTT 示例构建；板上身份和 ISA 检查保持一致 |
| 1：HAL 最小启动 | 六个芯片 feature；显式内核选择；HSI 20 MHz、AHB/APB 分频、时钟缓存、RCC 门控与复位；隔离未适配驱动及其生成代码 | 使用 ch32_hal::init() 的 RTT 示例运行；缓存与 RCC 寄存器一致；六个封装分别构建；有/无 Embassy 构建 |
| 2：GPIO/AFIO/EXTI | GPIO v3 复用；V4x7 remap 与 EXTI 路由、中断和异步等待 | HAL 驱动 PE8/PE9；输入、上下拉、边沿中断、异步唤醒 |
| 3：Embassy | SysTick 驱动、executor、中断绑定、休眠唤醒 | 多任务 Timer、多截止时间、重复唤醒；时间单调且周期正确 |
| 4：完整 RCC | HSE、PLL、总线限制、Flash 等待周期、外设内核时钟、完整 refresh | 逐档切换；独立频率观测；Embassy 与外设时序正确 |
| 5：DMA | DMA1 13 通道、DMA2 11 通道；普通/扩展窗口、请求映射、IRQ | 内存复制、边界长度、两类窗口、完成/取消/错误清理及重新启动 |
| 6：常用外设 | USART blocking → DMA async；timer/PWM、I²C、SPI | 串口回环、PWM 波形、实际 I²C 设备、SPI 回环；取消/超时/重新使用 |
| 7：后续外设 | Flash、ADC/DAC、USBHS、CAN、SDIO、Ethernet；I3C、PSRAM/FSMC、LTDC、ARGB 分项推进 | 按驱动提供硬件验收与独立提交/PR |

阶段 3 是第一轮交付目标：正式 HAL 初始化后，运行 Embassy 多任务闪灯及 RTT
心跳，贯通时钟、GPIO、中断、时间与休眠唤醒。该目标不依赖 DMA。

## 阶段 0～1 的边界

- 现有 PAC-only cpuid/isa/blinky/rtt 保留作参照；新增 HAL 启动示例。
- V4x7 的 GPIO 为 v3，但 RCC、AFIO、DMA、USART 等有独立版本；不能因名字相似就复用。
- V3V 内核选择不能隐式决定 SysTick 布局。延时和 Embassy 驱动按实际外设能力选择。
- 阶段 1 只允许 HSI 时钟源；refresh 对尚未支持的 HSE/PLL 状态明确拒绝，避免缓存假频率。
- 不接入假 DMA/空实现；模块与生成的 pin/DMA trait、IRQ 一起隔离。
- 不修改 option bytes；沿用 PAC 默认 c512_r200 布局（992 KiB flash、200 KiB RAM）。
- 保留 soft-float RV32IMC ABI，使用 +xwchc,+zve64x,+zvl64b,+zvbb；不启用未经验证的 CAS。
- 本地 PAC 尚未发布时，通过根目录和示例 Cargo.toml 的 patch 指定本地版本，
  不把开发机绝对路径写入 HAL 依赖。
  Cargo 会检查所有芯片 feature 的依赖映射，所以旧芯片构建也需要这份 PAC。
  上游提交前需先发布包含 V4x7 的 PAC、更新版本要求并移除本地 patch；
  当前阶段只保证下面记录的本地开发流程。

## 验证策略

1. 每阶段构建相关 feature 组合；V407/V467 六个封装各自构建，有/无 Embassy。
   重要提交回归现有 CI 的系列矩阵，区分构建覆盖与真实硬件覆盖。
2. 核对生成的 RCC gate/reset、pin remap、DMA 请求和中断；必要时反汇编确认访问和启动。
3. 板上记录时钟配置、原始 RCC 值、预期与结果。计时通过独立观测核对，
   不用同一时钟源相互证明；无独立测量时只声明功能验证。
4. 阶段 3 专用程序独占 SysTick，将 32 位计数值放到进位边界附近，
   检查软件扩展时间的单调性、跨边界 alarm、维护中断与临近截止时间。
   V4x7 的 CNT/CMP 为 32 位；不能使用 V3 的 64 位访问及高字视图。
5. 保留现有 cpuid/isa 回归；硬件测试不更改外部用户配置或 option bytes。

## 2026-11-03 板测恢复与回归结果

旧版 `hal_time_scale,clock-slow` 把 HCLK 留在约 39 kHz，WCH-Link attach 返回
`81 55 01 01`，重新插拔、低速连接、connect-under-reset 均无效。恢复手段是
`wlink erase --method power-off --chip CH32V4X7`（probe 侧断电整片擦除，不动
option bytes）；返回后等 2–3 秒再 attach 即恢复。

恢复后重新执行了此前的板测。时序类用例不接持续 RTT：先 `probe-rs download`、
`reset` 自主运行，再 `probe-rs read` 读 RAM 标记（调试 attach 会 halt 内核，
曾使 USART/Timer/RCC 断言假失败）。

- PAC-only：`isa`、`cpuid`、`rtt` 正常。
- HAL：`hal_init`（默认/分频）、`hal_blinky`、`hal_rcc`（HSI 9 组 / HSE 14 组）、
  `hal_embassy`、`hal_time_scale`（默认/PLL/AHB÷512）、`hal_systick_stress`
  （默认与分频各 512 轮；负向对照按预期 `FAIL code=13`）、`hal_dma`（24 通道、
  216 次复制）、`hal_usart`（10/10 TX）、`hal_timer`（24 项）、`hal_i2c`
  （地址探测/清理）、`hal_spi`（24 项）、`hal_dac`、`hal_rng`、`hal_adc`、
  `hal_can`（72 帧回环）、`hal_i3c`（寄存器/生命周期）、`hal_flash`
  （只读 + `flash-write-test`）均通过。
- `hal_eth` 初始化成功（PHY ID `0x73719011`），无网线未验帧收发；`hal_sdio`
  无卡时正确报告 “no card response”。

本轮发现并修复了两个真实驱动边界问题：

1. `src/spi.rs` 的 `blocking_flush` / async `flush` 在 `SPE=0` 时轮询 `STATR.TXE`。
   CH32V467 实测 `SPE=0` 时 TXE 恒为 0，而 `prepare()` 在使能 SPE 前先 flush，
   导致第一次 `blocking_write` 永久自旋。修复为禁用状态直接返回。详见
   [review-conclusion](review-conclusion.md) 第 2 节。
2. `src/flash/v4x7.rs` 的快速页编程在装载阶段等待 `BSY|WRBSY`。RM 32.5.6 要求
   每个 32 位字只等 `WRBSY`，`BSY` 要到 `PGSTRT` 之后才清零；原实现永远到不了
   `PGSTRT`，`flash-write-test` 报 `code=21`（`Timeout`）。修复为分阶段等待。
   详见 review-conclusion 第 3 节。

对应主机模型：新增 `tools/spi-host-tests`（5 项生命周期回归）和
`tools/flash-host-tests`（3 项），直接编译生产源码；已用“临时回退旧实现”反向
验证两者都会失败/挂起。`tools/*-host-tests` 在 macOS 上原生运行，
`--target x86_64-unknown-linux-gnu` 只是 Linux CI 参考。

剩余硬件门槛：PE8/PE9 LED 目视、SysTick 自然 32 位回绕（约 172 秒）、
USART/SPI 回环接线、I²C EEPROM、外部 I3C/CAN/SD/Ethernet 对端、ADC/DAC 模拟
测量，以及 USBHS 枚举/回显。

## 实施记录：阶段 0～1

阶段 0、1 已实现并完成下述验证；阶段 2 已实现并完成本地构建及主机回归，
GPIO 输出已上机通过；按用户要求暂缓 GPIO 输入/EXTI 接线测试。
阶段 3 已完成默认和分频配置的上机验证；阶段 4 的高速时钟树已实现并进入扩展回归，
阶段 5 的 DMA 后端、阶段 6 的 USART/timer/PWM/I²C/SPI 已实现；2026-11-03
设备恢复后已完成这些外设的默认板测（结果见下文“板测恢复”一节）；
阶段 7 已接入 DAC/RNG/ADC/Flash/CAN，并增加 USBHS device 后端和 CDC 回显例程；
主机/构建验证与板上验收分别记录，SDIO 等后续外设继续实施。

### 基线与修改

- main 的 #232（e194315）一致性读取实现已同步到工作区；该文件与 main 一致。
  当前分支未做 merge/rebase，未提交此次修改。
- ch32-data 基于 081bbdb；六个 V407/V467 YAML 的内核改为 qingke-v3v，
  product_type 同步改为 V3V vector core，并重新生成 PAC。该仓库的修改也尚未提交。
- qingke 本地示例基于 5046007；HAL 最低版本提升为 qingke/qingke-rt 0.8.3，
  以保证 V3V leaf feature 存在。原有 ISA 配置保留。
- 工具链：rustc 1.101.0-nightly (8006091eb 2026-10-17)。
- 增加六个芯片 feature、V3 内核系列选择、HSI RCC、分频缓存、独立 V4x7 阻塞延时，
  以及 hal_init 示例。未适配的模块、pin/DMA trait 和 DMA IRQ 生成同步隔离。
- 阶段 1 当时对 EXTI feature、TIM 时间驱动给出明确编译错误；阶段 2 已解除 EXTI 限制。
  Embassy 的构建和初始化已接通，但阶段 3 的运行验收尚未进行。

### 本地生成与构建

在 `../ch32-data` 运行：

```sh
cargo run --bin ch32-data-gen
cargo run --bin ch32-metapac-gen -- 'CH32*' CH641 CH643
```

本次实际生成覆盖所有具有 GPIOA 定义的芯片。CH645 是已有的不完整定义，
默认全量 PAC 生成在该芯片上报 GPIOA must exist；因此排除 CH645，未修改其数据。
上面的显式选择避免该已知问题，并保留 HAL 既有芯片 feature。

在 HAL 根目录运行（Cargo.toml 的本地 PAC patch 生效）：

```sh
cargo build --release --no-default-features \
  --features ch32v467weu6,rt,memory-x,defmt \
  --target riscv32imc-unknown-none-elf
cargo build --release --no-default-features \
  --features ch32v467weu6,embassy,rt,memory-x,defmt \
  --target riscv32imc-unknown-none-elf
```

在 `examples/ch32v467` 运行：

```sh
cargo run --release --bin hal_init --features hal
cargo run --release --bin hal_init --features hal,clock-dividers
cargo run --release --bin hal_init --features embassy
cargo run --release --bin hal_init --features embassy,clock-dividers
```

### 验证结果

- 六个 V407/V467 封装分别完成 blocking / embassy 两种 Release 构建，共 12 组通过。
  另有 V467WEU6 无 rt/无 defmt/无 embassy 的最小 feature 构建通过。
- 旧系列 CI 对应的 12 个芯片 Release 构建全部通过：L103、V003、V006、V103、
  V203、V208、V303、V305、V307、X035、CH641、CH643。
- 临时主机回归直接加载实际 RCC 源码与 PAC、用 RAM 模拟寄存器：
  225 组 AHB/APB 分频配置的写入、频率缓存、timer 倍频与 refresh 一致性通过；
  非 HSI 源拒绝路径通过。此测试不模拟晶振或真实时钟切换时序。
- WEU6 板上四组 hal_init 全部 PASS，并持续输出 RTT 心跳：

| 模式 | SYSCLK | HCLK | PCLK1 | PCLK2 | CFGR0 |
| --- | --- | --- | --- | --- | --- |
| blocking，默认 | 20 MHz | 20 MHz | 20 MHz | 20 MHz | 0x20000000 |
| blocking，分频 | 20 MHz | 10 MHz | 2.5 MHz | 5 MHz | 0x20002580 |
| embassy 初始化，默认 | 20 MHz | 20 MHz | 20 MHz | 20 MHz | 0x20000000 |
| embassy 初始化，分频 | 20 MHz | 10 MHz | 2.5 MHz | 5 MHz | 0x20002580 |

这些频率为 HSI 标称值结合分频得到的缓存值；寄存器读回一致，未做外部仪器测频。
阻塞模式检查零延时与 1 µs 延时，并使用 HAL 延时输出心跳；Embassy 模式检查
SysTick 已启用、HCLK/8、向上计数及 SEVONPEND，心跳使用 CPU spin loop。
这不代表多任务 Timer、真实 alarm IRQ 或休眠唤醒已验收。

- hal_init ELF 保留 xwchc2p2、zve64x、zvl64b、zvbb，无 A/F 扩展。
- 三个 V467 封装分别构建全部示例（含 hal_init）通过；原 PAC-only 示例构建通过。
- WEU6 板上 cpuid 报告完成，F=false、V=true；isa 的 WCH 压缩字节/半字访存、
  e8/e16/e32/e64 的 vadd 与 vror 均 PASS，VLEN=64。
  CPUID 仍广告 A 位；本轮未据此开启 A 扩展或证明 CAS 安全。
- 阶段 1 的 EXTI 和 TIM 时间驱动负向构建检查通过；阶段 2 起 EXTI 可用，TIM 驱动仍拒绝。
- 本地日志放在 /tmp/ch32v4-*；构建仍有既有 visibility/dead-code/unused-dependency
  警告，xwchc 仍有工具链不稳定 feature 警告。

## 实施记录：阶段 2（GPIO 输出已验收，输入/EXTI 待验收）

### 实现

- 复用 GPIO v3 的 Input/Output/Flex；新增 HAL PE8/PE9 交替输出示例 hal_blinky。
- 接入 AFIO v4x7 的四个 EXTICR，按 pin/4 与 pin%4 选择端口路由。
- EXTI wait 按“屏蔽本线 → 路由/边沿 → W1C 清除本线旧 pending → 最后使能”配置。
  IRQ 只处理已使能的 GPIO 0～15 pending，保留被屏蔽 GPIO 和内部唤醒源 16～21。
  继续复用 EXTI0～4、EXTI9_5、EXTI15_10 中断入口和 AtomicWaker 等待机制。
- USART1 remap 低位写 PCFR1[2]，高位写 PCFR2[26]（usart1_rm1），保留其他位。
  串口驱动和其他外设仍按原计划后续接入。
- 新增 hal_exti 有限次测试；使用手动 poll 并核对 IRQ 唤醒标记，不依赖 executor/Timer，
  避免把阶段 3 的时间驱动验证混入本阶段。

### 本地验证

- 六个 V407/V467 芯片分别完成 exti、exti+embassy Release 构建，共 12 组通过。
- V467 三个封装分别完成 exti、exti+embassy 的全部示例构建，共 6 组通过。
- 旧系列 CI 对应的 12 个芯片全部完成 embassy+exti Release 构建回归；
  覆盖 L103、V003、V006、V103、V203、V208、V303、V305、V307、X035、CH641、CH643。
- 临时主机测试提取实际 EXTI future/IRQ 代码，模拟 pending 的 W1C 语义；
  AFIO 和 USART1 patch 使用实际本地 PAC。覆盖 16 条 GPIO 线 × 5 个端口 × 3 种边沿
  的 240 组配置、相邻路由位保留、屏蔽/清 pending/使能顺序、Drop 取消、IRQ 完成后
  重用等待，以及 IRQ 对 masked GPIO/内部线的保留。USART1 四种 remap 编码及无关位保留通过。
  这些测试验证寄存器访问逻辑，不模拟真实总线时序、引脚或 PFIC。
- hal_exti ELF 含 EXTI0～4、EXTI9_5、EXTI15_10 入口和 GPIO_EXTI_RESULT；
  ISA attributes 保留 xwchc/zve64x/zvl64b/zvbb，无 A/F。
- 主机测试源码和日志：/tmp/ch32v4-stage2-test-gen.py、/tmp/ch32v4-stage2-test.rs、
  /tmp/ch32v4-stage2-host.log；构建日志：/tmp/ch32v4-stage2-*。

### 上机准备与验收顺序

1. 准备 V467WEU6、WCH-Link、RTT；先运行 hal_blinky，观察 PE8/PE9 交替。
   此步骤不需要新增跳线。HSI 和延时为标称值；逻辑分析仪可选，用于独立测量。
2. 确认 PA0/PB1/PB8/PC13 在板上引出且没有其他外部驱动；将 PA0 分接到三个输入。
   不可用时先修改示例引脚及配套 EXTI/路由断言，再构建后烧录。
3. 运行 hal_exti：逐个输入上下拉，然后八轮并发 rising/falling/any-edge、取消后重用、
   旧 pending 清除。验证独立 IRQ 与两个共享 IRQ、GPIO B/C 路由、真实 ISR 唤醒。
   RTT 需输出每轮 PASS 和 GPIO_EXTI_RESULT=PASS；失败时检查符号值和接线。
4. 用户要求暂缓 GPIO 输入接线，因此先执行不依赖输入的阶段 3；EXTI 硬件验收保留待办。

GPIO 输出已在连接的 WEU6 上验收：连续切换 PE8/PE9，输出锁存和引脚电平均与预期一致，
RTT 输出 GPIO_OUT_RESULT=PASS。无外部仪器测波形，LED 的视觉表现待用户确认。
GPIO 输入、上下拉和 EXTI 外部边沿仍未测试。

## 实施记录：阶段 3（2026-11-01）

### 上机发现与修正

- V4x7 的 SysTick 实际为 32 位 CNT 和 32 位 CMP，0x0c、0x14 为保留地址。
  板上高字写入读回为 0、低字进位后高字仍为 0；本地 SDK 的 core_riscv.h 同样以
  uint32_t CNT/CMP 和两个 RESERVED 字段定义。此前 PAC 的 64 位和高字视图是项目扩展，
  不属于该硬件。已修正 ch32-data 的 systick_v4x7.yaml 并重新生成本地 PAC，
  阻塞延时和 PAC-only 示例也移除高字访问。其他系列的 #232 一致性读取修复仍保留。
- 上机复现旧时间换算在 HCLK=20 MHz、SysTick=2.5 MHz、tick_hz=1 MHz 时偏快 25%。
  HCLK=10 MHz 时旧换算也将偏快 25%（1.25 MHz 截断为每 tick 1 count）。
  新驱动按计数频率进行比例换算，now 向下取整、alarm 向上取整，避免非整数比截断。
- V4x7 改用专用驱动，不复用 V3 的 64 位硬件计数。临界区内维护 last_low/epoch，
  空队列仍安排维护中断；下一次比较取任务截止时间、半周期维护点和本轮计数末尾中的最早值。
  因此 CMP 始终位于当前 32 位周期，跨进位后再安排后续 deadline。
- 维护周期最多 2^31 个计数。默认 HCLK/8 下约 859 秒，分频测试下约 1718 秒；
  不支持将中断连续屏蔽这么久，也不支持运行中由其他代码改写/重置 SysTick。

### 硬件验收

新增 hal_embassy 示例，使用 embassy-executor 0.10 的 platform-riscv32/thread executor：

- 默认 HCLK=20 MHz 和 AHB /2、APB1 /4、APB2 /2 两种配置全部 EMBASSY_RESULT=PASS。
- 专用程序在尚无应用 timer 时预置计数接近末尾：第一次验证连续读取跨进位，
  第二次让三个独立任务的 alarm 跨进位；软件时间保持单调且与预期 epoch/计数换算一致。
- 三个任务分别执行 16 轮 100/150/230 ms Timer，PE8/PE9 由两个任务切换，
  六次 1 s 心跳均正常，任务完成标记最终为 7。
- 验证取消较远 deadline 后重用、已过去的 deadline、连续 1 µs 临近 deadline。
- 八轮显式在临界区内执行 WFI，等待 SysTick pending 后退出临界区并由 ISR 唤醒 future。
  测试会消费已有事件后继续等待，避免把 WFI 因旧事件立即返回误当作睡眠。
- 主机单调时钟观测的默认心跳间隔约 0.947～1.066 s，分频约 0.936～1.081 s；
  RTT/USB 带来抖动。这排除了原比例换算偏差，不代表经过校准的 HSI 精度或低功耗电流验收。

### 本地回归与记录

- 六个 V407/V467 芯片的 exti、exti+embassy 共 12 组 Release 构建通过；
  旧系列 CI 的 12 个芯片 embassy+exti 回归构建通过。
- 三个 V467 封装的 exti、exti+embassy 全部示例共 6 组构建通过；
  修正 PAC 后的 blocking hal_blinky 再次上机 GPIO_OUT_RESULT=PASS。
- 临时主机回归加载实际 32 位驱动状态机、时间比例换算、本地 PAC 和 Embassy 通用队列，
  验证无用户 timer 的维护中断、连续跨三个周期的远期 alarm、不提前唤醒、过期后重用、
  MAX sentinel；9 种计数频率各 10,000 随机样本与 u128 参考换算及向上取整检查通过。
- 生成脚本：/tmp/ch32v4-stage3-test-gen.py；主机日志：/tmp/ch32v4-stage3-host.log。
  板上日志：/tmp/ch32v4-stage3-default-timed-run.log、/tmp/ch32v4-stage3-divided-timed-run.log；
  主机时间记录：/tmp/ch32v4-stage3-{default,divided}-host-timing.json。

下一步可推进阶段 4 的完整 RCC；阶段 2 的 GPIO 输入/EXTI 硬件验收仍单独保留。

### SysTick 跨进位强化板测（2026-11-01）

用户要求实际确认漏计风险，新增 `hal_systick_stress`，使用独立外设 TIM2 作为
1 MHz 标称参考计数和看门狗。它仍与 SysTick 共用 HSI，因此只能验证相对计时、
分频与时间换算，不能证明 HSI 的绝对精度。

- 每轮暂停 SysTick，预置 CNT 到末尾附近，通过一次 `Instant::now()` 建立边界前基线，
  注册跨进位的 3 ms Timer。测试不直接写 CMP，要求驱动自行设置 CMP=0xffffffff。
- 恢复计数后，前台在确认 CMP 被 ISR 重新设置前不再调用 `Instant::now()` 或 poll Timer，
  避免前台读时间替 ISR 补进位。TIM2 独立唤醒测试任务并检查超时。
- 四组交替测试：不屏蔽中断，以及屏蔽中断跨进位 50/500/2000 µs；每次完成后验证
  64 位软件 epoch 恰好增加一次、时间与硬件 CNT 对应、alarm 不早于 deadline。
- 默认 HCLK=20 MHz、分频 HCLK=10 MHz 各运行三次，每次 512 轮；脱离 RTT 持续访问、
  复位后自主运行，完成后才读取结果及统计。合计 **3072 轮全部 PASS**。
- 默认配置 3 ms Timer 的实际等待为 3036～3098 µs；分频配置为 3076～3152 µs。
  这些数值包含 ISR 和 executor 的唤醒延迟，是本次样本，不能当作最坏延迟保证。
  两秒窗口内 Embassy 与 TIM2 相对计时差不超过 1 µs；默认 Embassy tick 分辨率为 1 µs，
  SysTick 原始计数分辨率分别为 0.4/0.8 µs，均为标称值。
- 负向对照：启用 `stress-disable-systick-irq` 后，首轮即因 CMP 保持末尾、CNTIF pending
  被 TIM2 看门狗检出，输出 FAIL code=13，证明测试不会因前台补偿而假通过。
- 初版 TIM2 每 10 ms 通过 ISR 扩展计数，在调试连接下出现少计 10 ms；改为完整
  16 位周期 65.536 ms 后消失。RTT 连接下还观察到数十毫秒延后，因此额外进行了上述
  脱离 RTT 的复测。调试暂停会干扰此类中断计时，不能把短周期参考的漏计归因于 SysTick。

结论：本次没有复现 SysTick 漏进位，继续保留 32 位 SysTick 驱动，不因此切换 TIM。
软件扩展依然依赖维护中断及时执行；默认最长半周期约 859 秒，不允许屏蔽中断超过该约定。
该加速测试不代替自然完整周期（默认约 28.6 分钟）、超长屏蔽或外部频标验证。
TIM 的软件扩展同样需要及时处理中断，改用 16 位 TIM 本身并不能消除这一约束。

运行：在 `examples/ch32v467` 中执行
`cargo run --release --bin hal_systick_stress --features embassy`，分频时追加 `,clock-dividers`。
负向对照追加 `,stress-disable-systick-irq`（预期 FAIL），结束后应重新烧录正常配置。
RAM 符号 `SYSTICK_STRESS_RESULT`：1=运行中、2=成功、10～17=断言失败；
`SYSTICK_STRESS_STATS` 依次为轮数、最短/最长等待、TIM2 两秒窗口、Embassy 两秒窗口，单位 µs。
记录：`/tmp/ch32v4-stress-{default,divided,negative}.log`、
`/tmp/ch32v4-stress-detached.json`。

## 实施记录：阶段 4（2026-11-01）

### 事实基准

- WCH CH32V407RM V1.3（官方页面标注 2026-10-22）：§3.3 时钟树、§3.4 RCC 寄存器、
  §32.4.4 FLASH_CTLR。官方原站下载接口返回刷新错误，实际读取的是同版官方 PDF 的
  ch32-riscv-ug/CH32V407 镜像，并与原站文件元数据的名称/版本核对。
  本地 `/tmp/CH32V407RM.PDF`，SHA256
  `77de94a3c80e64104242a5d7486a9a23caa417d3476284fd852f0d5ce89f7216`。
- WCH CH32V407DS0 V1.4：表 3-2 限定默认 LDO 下 HCLK 最高 200 MHz，PB1/PB2 不超过 HCLK；
  表 3-15 通用 PLL 输入 5～32 MHz、输出 100～400 MHz。本轮采用该额定范围，
  没有加入 EVT 特定温度/电压条件下的 240 MHz 超频预设。
  本地 `/tmp/CH32V407DS0.PDF`，SHA256
  `048dbf5f4a0f31ddbadc518d88c6bccf683f82d90f0e598d91d255960a87a78a`。
- EVT：`EXAM/GPIO/GPIO_Toggle/User/system_ch32v4x7.c` 与
  `EXAM/SRC/Peripheral/{inc,src}/ch32v4x7_rcc.*`。
  Flash 使用 SCKMOD（访问时钟不超过 100 MHz），不是旧系列 LATENCY 等待周期字段。

### 实现

- RCC Config 支持 HSI/HSE/SYSPLL、32 种通用 PLL 倍频（含半整数）、USBHS PLL、ETH PLL，
  以及五种 SYSPLL 来源和 /1～/8 分频。保留原 AHB/APB 分频 API 与 HSI 默认配置。
- 配置在写寄存器之前检查依赖和频率；拒绝缺失的 HSE/PLL、非法 USBHS 参考频率、
  超出额定范围的通用 PLL/HCLK、非 25 MHz 的 ETH PLL 参考。
- 切换先返回 HSI、关闭 SYSPLL gate 与各 PLL，之后配置/等待目标源，设置 Flash 访问分频、
  重新锁定普通/快速编程锁，再切换 SYSCLK。启动/切换具有有界轮询。
  不访问 option bytes，不修改 RAM/code 分配。
- `rcc::refresh(hse)` 解码全部 SYSCLK/SYSPLL 来源、通用 PLL 半整数倍率及总线分频；
  ETH*2/3 使用 500 MHz*2/3 的整数 Hz 表示，而不是复制 EVT 的 333000000 近似常数。
- ch32-data 修正 `USNHS_PLLRDY` 拼写为 `USBHS_PLLRDY`，重新生成本地 PAC。
- SysTick 与 blocking delay 改用 SYSCLK/AHB 分频的比例换算，避免先截断 HCLK/8：
  例如 HSI、AHB /256 时真实计数频率为 9765.625 Hz。TimeScale 初始化时约分，
  平常使用 64 位商/余数换算，极端比例的中间乘法溢出时才使用 128 位计算；
  now 向下取整、deadline 向上取整，保留 MAX sentinel 和饱和行为。

### 验证与剩余范围

- `hal_rcc` 默认 9 组、启用 `clock-hse` 后 14 组自主运行均 PASS；
  覆盖 HSI 20 MHz、HSE 25 MHz、通用 PLL 的 160/250/350/400 MHz、USBHS 的
  120/240/320 MHz、ETH 的 250 MHz/166666666 Hz SYSCLK，HCLK 最高 200 MHz，
  以及最终降频回到 HSI。检查总线时钟缓存、refresh、Flash 分频/锁和 TIM2 倍频。
  相对计时通过 TIM2 与 SysTick 比对；没有外部频标校准。
- `RCC_SNAPSHOTS` 保存运行时 RCC/Flash 读回和计数差，便于运行后复核；
  结果见 `/tmp/ch32v4-stage4-rcc-board.json`。连续 RTT 访问会干扰短测量窗口，
  板测采用先自主运行再读取 RAM 的方式。
- 源码内 RCC 单元测试覆盖 225 组 AHB/APB 分频、40 组 SYSPLL 来源/分频、
  原生 HSI/HSE、半整数倍率及五类错误配置；8 个主机测试通过。
  比例换算按 6 种源频率×9 种 AHB 分频与 u128 参考对比；测试 tick_hz=1、1 MHz、1 GHz，
  并回归无用户 timer 的维护中断、跨三个 epoch 的 alarm 和重用。
- `hal_time_scale` 用暂停的真实 SysTick 检查包括 3000000000 和接近 MAX 的计数值，
  用于暴露低速分频下累计的频率截断错误；`hal_systick_stress` 新增 `clock-pll`，
  并可用 `stress-natural-wrap` 在不改写 CNT 的情况下再等待一个完整硬件周期。
- RTC/低速时钟驱动尚未接入；ADC/I2S/RNG/LTDC 等专用内核时钟选择将在对应驱动迁移时
  加入，当前没有把它们的专用时钟冒充为已验证的总线时钟。下一阶段为 DMA 和常用外设。

### 当前板测中断点（2026-11-01，已于 2026-11-03 解除）

> 该中断点已由断电擦除恢复；慢速/PLL 与压力测试结果见文首“2026-11-03 板测恢复与回归结果”。
> 仅 `stress-natural-wrap`（自然 32 位回绕）仍未上板完成。

- 比例换算修改后，默认 `hal_time_scale` 已在板上通过；RCC 的 9/14 组再次通过。
  新驱动状态机和比例换算的 5 个主机测试通过。
- 旧版 `hal_time_scale,clock-slow` 把 HCLK 留在约 39 kHz，之后探针报 protocol error。
  已尝试低速连接、connect-under-reset、WCH 芯片复位及显式 V4x7 family attach，均未恢复。
  未发送解保护或 option-byte 修改命令。
- 测试例程已改为检查后恢复 HSI /1，并成功构建；尚无法刷入。
  需要物理复位/恢复连接后验证慢速与 PLL 比例换算，重跑默认/分频/PLL 压力测试和负向对照。
- `embassy,clock-pll,stress-natural-wrap` 已准备自然完整周期验证，预计约 172 秒；
  失败码为 18，结果保存在 `NATURAL_WRAP_STATS`。此项尚未上板完成。
  前述 3072 轮是比例换算改动之前的结果，不能代替当前版本回归。
- 当前证据不足以认定 SysTick 需要换为 TIM，也不足以宣布新版本验证完成。
  若恢复后的自然溢出或中断压力测试出现无法可靠维护 epoch 的结果，再推进 TIM 驱动方案。

## 实施记录：阶段 5（DMA，2026-11-01）

### 事实基线与数据修正

- 使用本地 CH32V407RM V1.3 第 11 章、表 11-2/11-3、DMA 寄存器列表，
  以及 EVT `ch32v4x7.h`、`ch32v4x7_dma.c` 和 `DMA_MEM2MEM` 例程交叉核对。
- DMA1 为 13 通道，DMA2 为 11 通道；两者 CH1～CH7 从 +0x08 开始，步长 0x14；
  CH8 起从 +0x90 开始，步长 0x10。扩展标志/清除寄存器分别为 DMA1 +0xf0/+0xf4、
  DMA2 +0xd0/+0xd4。IRQ 号独立于寄存器窗口，DMA2_CH6/7 仍使用普通窗口。
- 修正 ch32-data 的 USART5_TX 请求为 DMA2_CH4，原先误填 CH5；手册图和表均指向 CH4。
  修正 ECH 索引注释，并补全 SIZE=3 的 64 位定义及芯片修订限制。
  HAL 的 Word 仍只支持 8/16/32 位，本阶段不绕过硬件批次限制开放 u64 DMA。
- 重新生成 data/PAC。六个芯片各 24 通道的编号、单独 IRQ 与 EVT 常量逐项核对通过，
  生成数据中的 USART5_TX 也均为 CH4。

### 实现与验证

- 恢复 V4x7 的 DMA 模块、Config 中断优先级、初始化、通道 singleton trait 和 IRQ 生成。
  使用独立 `src/dma/dma_v4x7.rs` 寄存器后端，复用现有 Transfer/Future/环形缓冲 API；
  DMA1/2 保留各自 PAC 类型，不把 DMA2 强转为 DMA1 的布局。
- 初始化先开时钟、停用通道并清标志/pending，再启用中断。配置仅在传输停止或完成后进行；
  启动前 fence，取消关闭通道及所有 IE 并清标志，支持完成或取消后的再次使用。
  同次 IRQ 的 HT/TC 分别处理，避免只清一个；未启用 TCIE 时也能查询硬件完成标志。
- V4x7 手册要求 EN=0 时才能改 CIRC；环形缓冲的 `stop()` 因此记录停止请求，
  在下一次 TC IRQ 中关闭通道，避免运行中修改 CIRC，也避免把前几圈的完成计数当作本次停止完成。
- 与现有 HAL 一致，传输错误采用 panic；V4x7 后端在 panic 前停止通道并清中断标志。
  这不表示提供可恢复的 `Result` 错误 API，也尚未在真实硬件制造总线错误验证。
- 真实后端源码及本地 PAC 通过 RAM 寄存器主机测试：所有 24 通道的窗口和标志寻址、
  完成唤醒、HT+TC 同时置位、取消/重启、无 TCIE 的 blocking_wait 后重用、
  circular 完成后保持运行、跨多圈后 stop 等待新 TC，以及错误停机/清理。
  连同现有 ringbuffer 测试共 13 项通过。
  RAM 测试显式清理模拟状态，不模拟 DMA 总线搬运和真实 W1C/中断时序。
  临时 harness 为 `/tmp/ch32v4-dma-tests`，日志 `/tmp/ch32v4-dma-host.log`。
- 新增 `hal_dma` 板测程序：遍历 24 通道，共 216 次复制，覆盖长度 1/2/3/31/256/65535、
  8/16/32 位宽、首尾 guard、无请求时取消及随后重用；每次完成有独立 Embassy 超时。
  已完成 release 构建和链接，BSS 为 132768 字节，适配当前默认 200 KiB SRAM。
  2026-11-03 已上板：24 通道、216 次复制全部通过；中断延迟和最坏时序未单独测量。
- 最终版本的六个 V407/V467 芯片 × blocking/Embassy+EXTI 共 12 组 release 构建通过；
  V003、V103、V307（带 Embassy）、H417 V3F 的代表配置也构建通过。
  V307 不带 Embassy 时现有 OTG/USBHS 模块引用可选 embassy_time 导致构建失败，
  与本次 DMA 改动无关，本阶段未顺带修改 USB 驱动。

阶段 5 后按 USART、timer/PWM、I²C、SPI 的顺序推进；仅恢复已经适配模块的 pin/request trait。

## 实施记录：阶段 6a（USART，2026-11-01）

### 基线与接入

- 核对本地 CH32V407RM V1.3 第 17 章、17.3 波特率公式、17.10 STATR/BRR/CTLR，
  及 EVT `ch32v4x7_usart.c`。USART1 使用 PCLK2，USART2～10 使用 PCLK1；
  16 倍过采样，最高 12.5 Mbps 对应 PCLK=200 MHz、BRR=16。
- V4x7 的基础 STATR/DATAR/BRR/CTLR1～3 与当前通用驱动兼容；CTLR4 未用于普通 UART。
  恢复全部 10 个 USART 的驱动模块、pin trait 和固定 DMA request trait。
  USART1 继续使用阶段 2 验证过的 PCFR1/PCFR2 分裂 remap patch，其余实例由元数据生成。
- V4x7 的异步 TX-only 构造器增加显式 USART IRQ binding，构造时启用该中断，
  保证 `flush()` 的 TC 唤醒有实际处理器；其他系列的构造器签名保持原样。

### 修复

- 波特率直接对 PCLK/baud 做 64 位四舍五入，写入 16 位 BRR，替代容易在 200 MHz
  发生 `25*clock` 溢出、且小数进位会丢失的旧公式。校验 0、超高/超低波特率后才写寄存器。
  reconfigure 保留原 IRQ enable 状态，半双工配置可清回 full-duplex，RX 策略字段同步更新。
- DMA TX 对空缓冲返回成功、超 65535 返回 BufferTooLong；使用与 EVT 相同的 RW0 清 TC
  方式保留接收标志，完成或取消后关闭 DMAT。`flush()` 完成/取消均关闭 TCIE。
- RX 在开启 DMAR 之前清历史状态；等待期间仅查询 STATR，不再在每次 poll 时读取 DATAR，
  避免与 DMA 抢收字节。错误和 IDLE 路径关闭 DMAR 后才读 DATAR；IDLE IRQ 立即停止请求，
  读取长度之前停止 DMA，避免返回的长度继续变化。错误优先于同次 DMA 完成。
- V4x7 对 TX/RX 使用时钟引用计数，双向初始化只 reset 一次；释放某一半只关闭对应方向，
  最后一半释放时才关闭外设时钟和 IRQ。配置失败会释放时钟并断开已配置的引脚。

### 验证与剩余范围

- 真实 USART 源码副本、本地 PAC、真实 DMA 后端与 Embassy waker 在主机 RAM 寄存器上回归。
  4 个持久化测试位于 `src/usart/tests.rs`，覆盖波特率边界/舍入、高频、48 种 framing/半双工配置、
  无效配置不写寄存器、错误优先级；临时 harness 另测 6 项异步与生命周期行为。
  连同 8 个已有 DMA/环形缓冲测试共 18 项通过。
  对临时 common::Reg 加读取计数和手册指定的 RW0/W1C 模拟，确认无事件的重复 RX poll
  不消费 DATAR。此模拟不能证明总线时序、电气波形或真实数据接收。
  harness `/tmp/ch32v4-usart-tests`；日志 `/tmp/ch32v4-usart-host.log`。
- 六个芯片 × blocking/Embassy+EXTI 的 12 组 release 构建通过，另回归 V003、V103、
  V307（Embassy）、H417 V3F 代表配置；完整结果 `/tmp/ch32v4-usart-matrix-final.log`。
- 新增 `hal_usart`：无接线测试 10 个实例的 blocking TX、DMA TX、TC flush、发送取消和重用，
  包含扩展 DMA 窗口，并用 SysTick 对照发送耗时；两者共享系统时钟，不校准振荡器。
  同一例程对全部 10 路 RX pin/DMA 构造调用进行编译检查，此函数不在运行时执行。
- `usart-loopback` 可选 PA9→PA10 接线，验收固定长度 RX、IDLE、接收取消/重用、释放 RX 后 TX；
  仍等待 GPIO 输入接线与探针恢复，不把发送完成标志当作实际收到数据的证明。
- 默认、APB 分频、200 MHz PLL 与 PLL+APB 分频+loopback 四组例程完成构建和链接。
  2026-11-03 默认配置已上板：TX 10/10 通过；`usart-loopback` 仍待 PA9→PA10 接线。
  USART 的 9 位数据传输、LIN/IrDA/智能卡、流控电气行为不在本轮板测覆盖内。

后续推进 timer/PWM，然后 I²C、SPI；SysTick 自然周期、DMA 实物和 USART 板测继续保留为硬件验收门槛。

## 实施记录：阶段 6b（timer/PWM，2026-11-01）

- 依据本地 CH32V407RM V1.3 第 14～16 章、EVT `ch32v4x7_tim.c`，核对
  `timer_v4x7.yaml` 与 V3 布局差异。V4 的 TIM1/8 为高级定时器、TIM2～5 为
  16 位通用定时器、TIM6/7 为基本定时器；不把 TIM2/5 当作 32 位。
- 开放八个 timer 实例，复用 low-level、SimplePwm 和 ComplementaryPwm；
  恢复 pin、CH1～4 DMA trait，并补生成 UP DMA trait。实例频率取 RCC 的
  PCLK1_TIM/PCLK2_TIM 缓存，沿用 APB 分频后的定时器倍频规则。
- 修复通用实现中 ARR+1/PSC+1 的频率回读、32 位 ARR 少减 1、重载时 URS
  配置被覆盖、RW0 标志使用 read-modify-write 可能清掉并发标志的问题。
- PWM 配置为边沿模式保留可写的 100% CCR，避免 ARR=65535 时 duty=65536
  无法写入；中心对齐用 2×ARR 计算周期与最大 duty。补修互补 PWM 的 CH4
  disable，以及 DTG 的 110/111 分段编码和 255/508 附近的边界。
  dead time 参数单位明确为定时器输入时钟 tick，最大 4032 ticks。
- 新增持久测试 `src/timer/{tests,register_tests}.rs` 与死区编码穷举回归。
  临时完整源代码主机 harness `/tmp/ch32v4-timer-tests` 使用生成 PAC 和模拟
  寄存器；30 项通过（12 项 timer/PWM，18 项已有 DMA/USART），包括全部
  u16 dead time 请求的解码对照、PWM 0/50/100%、分频边界和中断清除写入值。
  该模拟不证明实际 PWM 波形、死区或 IRQ 时序。
- 新增 `hal_timer`：八个定时器分别用 50/1000/10000Hz 更新 IRQ 对照 Embassy
  时间窗口；六个 PWM 实例三种计数模式与四通道 duty 回读；最后在 TIM1
  全重映射 PE9/PE8 输出 1kHz 互补 PWM，保留示波器验收入口。
- 当前默认及 PLL+APB 分频配置已构建链接；本轮实际 attach 仍返回 Protocol
  error，故没有烧录，也不声称 24 组定时器 IRQ 或互补输出已上机通过。
  恢复调试连接后读取 `TIMER_RESULT`（1 运行、2 通过、10～16 失败）、
  `TIMER_CASES`（应为 24）、`TIMER_SNAPSHOTS`，再测 PE9/PE8 波形。
- 六个 V407/V467 型号各 blocking/Embassy，以及 V003/V103/V307/H417 四组
  回归共 16 个库构建通过。共享 PWM 修正亦经过该构建矩阵。

继续按 I²C → SPI → 阶段 7 推进；timer Embassy time driver 尚未开放 V4，
当前仍使用单独验证中的 32-bit SysTick 后端。

I²C 预审发现 EVT `I2C_Init` 在 PCLK≥60MHz 时将 CTLR2.FREQ 钳位到 60，
RTR 按该值计算；旧 HAL 直接把 PCLK MHz 写进 6-bit FREQ，在 100/200MHz
时会截断。下述阶段 6c 已修正该差异。

## 实施记录：阶段 6c（I²C，2026-11-01）

- 依据 CH32V407RM V1.3 第 18 章与 EVT `ch32v4x7_i2c.c`、`I2C_DMA`，
  接入单个 I2C（非 I3C）：PB6/PB7 默认、PB8/PB9 重映射；
  I2C_EV/I2C_ER；TX=DMA1_CH6、RX=DMA1_CH7；PCLK1 驱动。
- V4 FREQ 钳位到 60，RTR 采用同一频率域值；CCR 用实际 PCLK1 向上取整，
  避免超过请求的 SCL 频率。配置限制为 0 < SCL ≤ 400kHz、V4 输入时钟至少 4MHz，
  CCR 必须能用 12 位表达。无效配置在 RCC/remap/pin 写入前拒绝。
  100/400kHz 已覆盖，低频上限同时受实际 PCLK 与 CCR 容量约束。
- 修复共享 driver 的成功路径清理 guard 意外发 STOP、embedded-hal 两个
  transaction trait 的 todo，以及未保护的错误/超时退出。异步帧使用 Embassy
  超时，等待 STOP 采用可取消轮询；结束帧等待 STOP 完成再返回。
- 连续读操作按总长度合并，覆盖单字节在清 ADDR 前撤 ACK、双字节 POS/BTF、
  最后三字节 ACK/BTF 收尾。异步读取前缀使用 DMA，最后最多三字节用事件
  IRQ + CPU 收尾，避免相邻 read 缓冲区边界提前 ACK 或 NACK。
  写阶段在 BTF 后才能 STOP 或 repeated START，空写仅进行地址探测。
  R→W 在读出末尾数据前预置 START，下一阶段复用待处理的 SB/START，
  避免 NACK 后释放总线再尝试启动；1/2/3/5 字节尾部均有模型回归。
- DMA 请求在 channel 就绪后启用；错误优先于同时 DMA 完成；取消/超时关闭
  DMA 和事件/错误 IRQ 请求。挂起 START/STOP 使用 SWRST 恢复并重装时序。
  STAR1 用单次快照及 RW0 mask 清除已观察错误；ARLO 优先，失去主机身份
  后不发 STOP。V4 的 BERR 按手册返回 Bus，保留其他系列原有忽略策略。
- blocking 构造不启用无 binding 的 IRQ；async 初始化完成再 unpend/enable。
  driver 持有 SCL/SDA，Drop 时断开引脚并关闭 IRQ、PE 和 RCC 时钟。
  7-bit 地址及 DMA 长度在开始事务前验证。
- 持久测试 `src/i2c/tests.rs` 覆盖时序边界、输入校验。
  `/tmp/ch32v4-i2c-tests` 使用完整当前 driver 源码、真实 PAC/DMA 和可控的
  寄存器/总线模型；45 项通过（15 项 I²C，30 项已有 timer/DMA/USART）。
  覆盖 1/2/3/4/5/31 字节与分散缓冲读取、W→R/R→W/连续 W、零长度写、
  错误与 DMA 同时完成、RXNE/BTF 未就绪、START/STOP 挂起、取消和超时重用，
  并穷举 64 组错误组合。模型不能证明真实线路的 ACK/STOP 或 IRQ 时序。
- 新例程 `hal_i2c` 默认验证六组映射/速度配置、异步地址探测与清理；
  无外接从机时 NACK/Timeout 仅记入 I2C_PROBES，不冒充通信验收。
  可选 `i2c-eeprom` 要求 PB6/PB7 上接带上拉的 3.3V 24C02、地址 0x50；
  只设置地址指针并读数据，比较 blocking/DMA、短读、连续读及取消后重用，
  不写 EEPROM 非易失数据。
- 默认与 PLL/APB 分频 + EEPROM 配置已构建链接；2026-11-03 默认配置已上板，
  地址探测与清理检查通过；外部 24C02 从机/示波器验收仍待进行。
- 六个 V407/V467 型号各 blocking/Embassy，加 V003/V103/V307/H417，
  含上述 I²C 修改的 16 组库构建全部通过。

后续继续 SPI，然后阶段 7；上述主机和构建结果均不替代 I²C 设备板测。

## 实施记录：阶段 6d（SPI，2026-11-01）

事实基线为 CH32V407RM V1.3 第 20 章和 EVT `ch32v4x7_spi.c`。
SPI1 使用 PCLK2，SPI2/3 使用 PCLK1，三路均为 `spi_v4x7`；恢复 pin/DMA trait
生成和模块入口。当前 API 提供软件 NSS 的主机模式、8/16-bit、四种时钟模式、
MSB/LSB 顺序和 blocking/DMA 收发；slave/I2S/CRC 不在此次 API 实现范围内。

- 分频按真实比例向较慢方向选择，避免整数截断后超出请求频率；拒绝 0 和低于
  PCLK/256 的请求，构造器在修改引脚/remap/时钟前验证配置。
- CPOL/CPHA/字长更新前关闭 SPE；重配时同步更新 HSCR。V4 高速接收只在 DIV2
  启用，HSRXEN2 还要求 PCLK >= 120 MHz，依据手册 20.4.10。
- write 使用单线输出模式避免无接收消费者时产生 OVR；read 使用双线全双工、
  显式发 dummy，避免 RXONLY 主机自动产生多余时钟。16-bit dummy 使用 u16。
  RX-only 构造器只提供 RX DMA 时以软件发送 dummy；新增
  `new_rxonly_with_tx_dma` 可用 TX DMA 产生 dummy，不需要 MOSI 引脚。
- 允许 transfer 两端长度不等：公共部分全双工，尾部补零读或丢弃接收值。
  调用者控制 CS，阶段之间允许停钟。空缓冲直接成功，DMA 单次长度上限 65535。
- DMA 完成后等待 TXE 且 BSY 清除再完成操作。新增真正的 blocking/async flush。
  V4 异步构造器要求 SPI IRQ binding；错误 ISR 停止请求并唤醒 future，错误优先于
  同时到来的 DMA 完成。其他系列保留原有构造器签名。
- future 取消/错误时先释放 DMA transfer，再清理 SPI 请求、错误中断与 SPE；
  按 DATAR→STATR 清 OVR、STATR→CTLR1 清 MODF、RW0 清 CRCERR，支持后续重用。
  不支持的方向在开始传输前返回 UnsupportedOperation。
- 持久测试 `src/spi/tests.rs` 覆盖分频边界、无超速、四种模式/位序、HSCR 及错误优先级。
  临时 `/tmp/ch32v4-spi-tests` 在已有 I²C harness 上增加真实 SPI PAC 和寄存器模型；
  **58 项通过**（13 项 SPI，45 项已有回归）。driver 副本只增加测试模块声明，并将
  `core::ptr` 四处字读写替换为测试代理以模拟数据回环；算法和 DMA 后端使用现有源码。
  覆盖 8/16 位、精确 blocking 帧数、dummy 字宽、不等长度、in-place、软件发 dummy、
  取消/重用、BSY 等待及取消、配置修改时 SPE 必须关闭、三类错误 IRQ 唤醒和恢复。
  DMA 测试通过手动完成模拟硬件，不能据此声称已验证真实 DMA 数据内容或波形。
- 六个 V407/V467 型号各 blocking/Embassy，加 V003/V103/V307/H417，16 组库构建通过。
  `hal_spi` 默认及 `embassy,clock-pll,clock-dividers,spi-loopback` 均成功构建链接。
  默认三路各运行八组模式/位序组合，覆盖 blocking/DMA、8/16 位、重配和取消/重用；
  可选 SPI1 PA7→PA6 回环比较数据。RX-only 两种构造器仅验证完成，不对悬空 MOSI
  形成的接收数据作断言。2026-11-03 默认配置已上板，24 项配置/生命周期检查通过
  （修复了 `SPE=0` 时 `blocking_flush` 的死锁）；回环配置仍待 PA7→PA6 接线。

后续进入阶段 7，先核对 ADC/DAC/RNG 的寄存器和内核时钟，再推进其余外设。
DAC 当前沿用 `dac_h4` 布局，不能仅解除旧驱动 gate；双通道时钟生命周期和 DMA
取消也需要审查。Flash、USBHS、CAN、SDIO、Ethernet 及专用外设仍未完成，
当前不满足“绝大多数外设驱动完成”的整体验收条件。

## 实施记录：阶段 7a（DAC/RNG，2026-11-01）

基线为 CH32V407RM V1.3 第 13、30 章及 EVT `ch32v4x7_dac.c`、
`ch32v4x7_rng.c`、`RCC_RNGCLKConfig`。尚未进行新的板测。

### DAC

- 核对 0x00～0x30 的寄存器地址、两个通道各占 16 位的控制字段及全部八种触发编码，
  确认与现有 `dac_v3` 定义一致；ch32-data 的 V4x7 DAC 从 `h4` 改为复用 `v3`，
  重新生成 PAC。硬件未更名，只统一等价的寄存器视图，保留其他 H4 芯片定义。
- 接通模块、PA4/PA5 pin trait 以及 DMA2_CH3/CH4。双通道 acquire 只复位一次，
  拆分后释放某通道只关闭该通道，最后一个通道释放后才关外设时钟；保存并释放引脚。
- 支持单/双通道 8/12 位输出、左右对齐、输出缓冲、软件/定时器/EXTI 触发、
  噪声/三角波配置和双通道一次写入软件触发。
- 新增 `try_write` 报告过长缓冲或未配置硬件触发，保留 `write` 便利入口。
  空缓冲无操作，长度上限 65535；软件触发不产生 DAC DMA 请求，明确拒绝用于 DMA。
  先配置 DMA 再开请求；取消时先停止 DMA，再关对应 DAC 通道/请求，支持重用和循环模式。
- 正常 DMA 完成只关请求，保留通道使能；最后一个样本可能还在 DHR，需要后续触发
  才到 DOR。修复旧实现完成时立即关 EN、可能提前终止最后一次输出的问题。
  标量 `Bit12Left` 接收未移位 12 位值；DMA `ValueArray::Bit12Left` 使用预移位至
  bits 15:4 的半字，文档明确两者区别，原有标量行为保持一致。
- 持久 4 项寄存器/输入测试，临时 4 项生命周期/DMA 模型测试通过；覆盖 split 两种
  drop 顺序、只复位一次、单通道取消不影响另一通道、循环取消、完成保持 EN 和非法输入。

### RNG 和 RCC

- 核对 RNG `v3` 的 3 个寄存器布局与 V4 一致，解除 gate。ch32-data 的内核时钟
  由错误的 HCLK 改为 RNG。RCC 增加 `RngClockSource`：SYSCLK / USBHS PLL / ETH PLL，
  PLL 来源在初始化前验证，`rng_source_frequency()` 读取选择与 ready 状态。
  PLL 选择按 EVT 写 RNGSRC 和共享 I2SPLLSRC，后者也会决定 I2S2/I2S3 的 PLL 来源。
- `new`/`reset` 不等待熵；首次 ready 的字在后续读取时丢弃。消除旧 reset→next_u32→
  reset 的错误恢复递归和 seed recovery 的无限等待。
- 新增 `try_next_u32`、`blocking_next_u32`、`async_next_u32`，显式返回硬件错误。
  读取前后都检查当前和锁存错误，错误优先于 ready；零是合法的随机字，
  不再把单个零误判为 seed failure。CEIS 用 EVT 同样的写零掩码清除，保留 SEIS。
- 异步读取在注册 waker/开 IE 后复查状态；完成、失败或取消都关 IE；Drop 关 RNGEN、
  PFIC IRQ 和门控。`rand_core::try_fill_bytes` 现在会返回错误，其他非 fallible
  入口遇硬件错误 panic，避免无限恢复；字节填充支持尾部及错误时保留已完成的前缀。
- 持久 2 项状态/错误编码测试、临时 7 项生命周期/事件模型测试通过，覆盖 IRQ 唤醒、
  取消重用、使能 IE 时到达的数据/错误、读 DR 后出现错误、所有字节尾部和 rand_core。
  RCC 原 8 项加 3 项 RNG 检查，共 11 项通过；验证 SYSCLK 与 AHB 分频分离、
  PLL 不 ready 时返回 None，以及缺失 PLL 的配置在初始化前被拒绝。

### 验证与剩余范围

- `/tmp/ch32v4-analog-tests` 使用当前 DAC/RNG 源码副本，除加载测试模块外无算法修改；
  使用真实 PAC/DMA 加寄存器模型，新增 17 项，连同先前 58 项共 **75 项通过**。
  模型手动完成 DMA、注入 RNG 数据和事件；不证明真实模拟输出、熵质量或物理 DMA 时序。
- 六个 V407/V467 型号各 blocking/Embassy，加 V003/V103/V307/H417，共 16 组构建通过。
- `hal_dac`、`hal_rng` 默认及 PLL/APB 分频 + RNG USBHS PLL 配置均构建链接通过。
  DAC 示例比较 DOR、双通道/软件触发、三种 DMA 格式、最后样本、取消重用和三角波变化；
  不接输入线也可做数字路径检查，但实际 PA4/PA5 电压精度仍需外部测量。
  RNG 示例记录 32 个字、非整字节读取、中断清理、drop/recreate，不对熵质量作结论。
- 2026-11-03 DAC/RNG 默认配置已上板通过；模拟输出精度与熵质量仍需外部测量。
  下一步为 ADC 的独立布局、时钟/校准、blocking/DMA 与 ADC↔DAC 板测，
  然后推进 Flash、USBHS、CAN、SDIO、Ethernet 及其他专用外设。

## 阶段 7b：ADC 独立后端（2026-11-01）

### 事实基准与 PAC

- 对照 CH32V407RM V1.3 第 12 章、数据手册 V1.4 §3.3.21/22、EVT 的
  `ch32v4x7_adc.c`、`ch32v4x7_rcc.c` 和 `ADC/ADC_DMA/ADC_DMA/User/main.c`。
- 修正 `adc_v4x7.yaml` 的 H4 遗留命名：0x10 为 SAMPTR2，RDATAR 低 16 位为
  完整 DATA，IDATAR1 为完整 JDATA；删除错误的 TKEY/charge 描述，章节改为 12。
  不改变实际寄存器地址，已重新生成 data 和 PAC。
- ADC1/2 共享 ADC1_2 中断，固定 DMA 请求分别为 DMA1_CH12/CH13。
  温度 IN16、Vref IN17 只允许 ADC1；未虚构 H4 的 PGA/触摸功能。

### 时钟与驱动

- RCC 增加 ADC 来源（HCLK 或 USBHS PLL /1～32）、/2/4/6/8 分频和占空比配置；
  按 EVT，两个来源都还经过 APB2 分频。初始化前验证 PLL 依赖和 30 MHz 上限。
  默认 /8 在 HCLK 200 MHz 时为 25 MHz；`adc_frequency()` 返回整数 Hz，内部保留源/除数
  比例进行频率和采样时间检查，不先截断 ADC 频率再判断。
- 新 `src/adc/v4x7.rs` 独立于旧系列，实现有界校准/阻塞单次读取、共享中断异步读取、
  1～16 rank 的规则扫描 DMA。DMA buffer 必须是完整帧，长度不超过 65535 半字；
  空 buffer 不启动。覆盖普通和 AUX 扩展采样时间，共 12 种。
- 通道描述借用引脚/内部通道；重复通道的采样时间冲突提前拒绝。温度采样按 17.1 us
  检查真实时钟比例；例如 25 MHz 即使使用 239.5 周期也会拒绝，需降低 ADC 时钟。
  Vref 仅使用 1200 mV 标称值，不宣称温度/电压已作出厂校准。
  同时限制 ADC 转换速率不超过数据手册的 2 MS/s：例如 30 MHz 下 1.5 周期采样
  加 12.5 周期转换会超限，需要 2.5 周期或更长的采样时间。
- 每次操作结束或取消时停止 ADC/请求，DMA guard 在 DMA 对象释放前执行；中断处理不读
  RDATAR，不与 DMA 抢数据。ADC1/2 分别维护 waker，drop 一路不禁用共享 PFIC vector 或另一路
  时钟。校准超时会关闭 ADC 和其时钟；可再次校准或重新创建。
- 稳定等待使用 core delay，不占用 Embassy 的 SysTick。当前采用至少 2 ADC 周期与
  20 us 对应 core cycles 的较大值，再作保守余量；实际启动/模拟稳定性仍需板测。
  为确保每次配置前已停止转换，单次读取包含重新上电稳定开销；高速采样使用扫描 DMA。

### 验证与边界

- 持久 5 项 ADC 寄存器/采样编码/序列与速率、温度约束测试；临时 ADC 模型增加 8 项生命周期测试，
  连同前期测试共 **88 项通过**。模型直接加载当前 ADC 文件，使用真实 PAC/DMA 驱动，模拟
  自校准完成、EOC、共享 IRQ 和 DMA 完成。覆盖超时、取消重用、drop 隔离、16 rank 跨寄存器、
  正常 DMA 完成，以及 AUX 扩展采样切回普通采样。RCC **14 项通过**。
- `/tmp/ch32v4-adc-tests` 继承前期 USART/I²C/SPI/DAC/RNG 模型；其历史源码副本未修改算法。
  ADC 通过路径直接加载 `src/adc/v4x7.rs`，没有替换驱动算法。模型不能证明真实模拟数值、
  校准时序、外设 DMA 请求可靠性或最高采样速率。
- 六个 V407/V467 型号 × blocking/Embassy，加 V003/V103/V307/H417 的 16 组构建；
  全部默认 Embassy 示例、ADC 的 PLL/APB 分频/USBHS PLL 和可选 DAC 回环配置构建链接通过。
- `hal_adc` 默认不接线：ADC1 Vref/温度、ADC1/2 共享 IRQ、ADC1 双通道 DMA、ADC2 PA0
  DMA 数字路径、取消重用与 drop 隔离。浮空 PA0 只检查 12 位结果范围，不判断模拟精度。
  `adc-loopback` 可选连接 PA4→PA0，核对三个 DAC 输出点；精密模拟性能需外部测量。
- ADC_RESULT：1 运行、2 通过、10～23 失败；ADC_CLOCK_HZ、ADC_SAMPLES 和
  ADC_CANCEL_PENDING 用于复核。异步读取可用 Embassy with_timeout 包裹并安全取消。
- 当前仍未上板：探针能枚举，连接 CH32V467WEU6 仍返回 Protocol error。用户物理恢复连接后
  按原顺序补 SysTick 自然回卷及外设验收，不修改 option bytes、保护状态或 RAM/code 分配。
- 当前支持独立规则转换；注入组、双 ADC 同步、模拟看门狗和硬件触发 API 尚未实现。
  后续继续 Flash、USBHS、CAN、SDIO、Ethernet、I3C 及专用外设。

## 实施记录：阶段 7c（Flash，2026-11-01）

- 依据 RM 第 32 章、数据手册及 EVT FLASH_Program，增加独立 V4x7 后端。
  普通编程以 16 位写入，快速编程为 256 字节页；普通擦除为 4 KiB，快速擦除为
  32 KiB。V4x7 没有旧系列的快速 256 字节擦除。几何从 MEMORY_LAYOUT 获取。
- 擦除后原始读值为 0xE339，而非 0xFFFF。原始 Flash 只实现 ReadNorFlash；
  显式 into_nor_flash(from, to) 提供数据分区，以逐字节 XOR 将擦除值归一为 0xFF，
  才实现 embedded-storage NorFlash。该编码会改变存储表示，不能用于固件镜像。
  不实现 MultiwriteNorFlash；调用者仍须遵守擦除后再编程的约定。
- 写入前检查整个请求的边界、对齐、写保护及擦除读值，数据源无需对齐；操作后逐字节
  或半字校验。WRP31 保护第 31～247 扇区，读保护时首个 4 KiB 也拒绝写入。
  不解保护，不写 option bytes，不改变 RAM/code 分配。
- 保留 Flash 时钟设置，编程/擦除前退出增强读取模式，完成后恢复并锁定。
  BSY/WR_BSY 置位时不写控制寄存器。超时后保留清理状态，必须保留驱动并在硬件空闲后
  调用 recover() 完成恢复；轮询上限不是墙钟时间。每次物理操作屏蔽中断，32 KiB
  擦除的数据手册上限为 200 ms，应用应计入中断延迟。
- 擦写函数及其被调函数必须位于零等待 Flash。增加 memory-split-usr 转发 PAC linker
  feature，使代码限制在零等待区；运行时检查三个擦写入口的地址及 OBR 所示区域。
  此检查不能替代对自定义链接脚本和全部被调函数布局的审查。
- 新增 hal_flash：默认只读取和检查无效请求；显式 flash-write-test 才允许擦写。
  该 feature 启用分区链接并以 NOLOAD 预留 0x080F0000..0x080F8000 的末尾 32 KiB，
  覆盖 4 KiB/32 KiB 擦除、两种编程、未对齐源、重复编程拒绝和编码 NOR 数据分区。
  ELF 确认预留段文件长度为 0；本地 probe-rs ELF loader 跳过空段，不将其作为下载数据。
  测试范围内原有数据会被删除。2026-11-03 已用 `flash-write-test` 实际执行
  4 KiB 擦除、快速页写、重复写拒绝、半字写、32 KiB 块擦除和编码 NOR 分区，全流程通过
  （修复了快速页装载阶段等待 `BSY|WRBSY` 导致的超时）。
- 104 个主机测试通过，其中 6 个持久化 Flash 几何/边界/保护/编码测试和 10 个临时
  控制器生命周期模型测试；覆盖超时延迟清理、失败解锁、保护错误、校验失败和模式恢复。
  16 组库构建矩阵通过；默认只读和可选写入例程，以及 Embassy+PLL 200 MHz 写入配置
  构建链接通过。ELF 中擦写入口位于零等待区，预留段和固件数据不重叠。
  记录：/tmp/ch32v4-flash-host.log、/tmp/ch32v4-flash-matrix.log、
  /tmp/ch32v4-flash-example-{read,write,pll}.log、/tmp/ch32v4-flash-elf-layout.txt。
- 主机模型不能证明物理 Flash 编程、ECC、真实耗时和耐久性；当前探针连接仍报
  Protocol error，实际板测待恢复后进行。后续推进 CAN、USBHS、SDIO、Ethernet、I3C 等。

## 实施记录：阶段 7d（CAN，2026-11-01）

- 依据 RM 第 25 章及 EVT CAN TestMode/Networking，增加独立 can_v4x7 后端并恢复
  CAN 引脚 trait 生成。V4x7 只有 CAN1，三个 TX 邮箱、两个相邻 RX FIFO、14 个过滤器组，
  APB1 为内核时钟；AFIO 支持 PA11/PA12、PB8/PB9、PD0/PD1 三组映射。
  现有 ch32-data 基线与这些事实相符，本阶段不需修改 PAC。
- 支持 11/29 位数据帧和远程帧，0～8 字节 DLC；接收到 DLC 9～15 时按 Classical CAN
  的 8 字节处理，远程帧无数据。打包按整个 29 位 ID 左移 3 位，避免旧代码拆分 ID 的错误。
- 波特率搜索使用精确整除，接受 1 kbit/s～1 Mbit/s 内硬件可表示的值，BRP 使用完整
  10 位（1～1024），不将四舍五入的结果冒充指定波特率。显式 BitTiming 校验范围和时钟。
  标准 TS1 1～16、TS2 1～8、SJW 1～4，扩展 long-TS1 保持关闭；不实现 CAN FD。
- 过滤器支持 16/32 位、列表/掩码及两路 FIFO 分配；exact() 提供 ID/IDE 和可选 RTR
  匹配，accept_all() 接受全部。原始 Filter.registers 可表达四个 16 位列表项及两对掩码。
  检查 bank <14 后才写寄存器，FINIT 期间先停用目标 bank，再更新并激活。
- 提供 blocking/nonblocking 和 Embassy async；embedded-can blocking/nb traits 接通，
  trait receive 默认 FIFO0，显式 receive/try_receive 可选两个 FIFO。阻塞发送等待实际完成，
  异步发送也等待 TXOK 或错误，未将“写入邮箱”当作发送成功；不替换其他待发送帧。
  blocking 使用有界寄存器轮询，async 超时由调用者使用 with_timeout 包裹。
- TX ISR 在 W1C 清 RQCP 前保存三个邮箱各自的完成结果；新提交只清该邮箱旧结果。
  RX0/RX1 中断分别唤醒任务，FIFO overrun 清标志后报告丢帧，保留现有队列供重试读取。
  SCE 唤醒错误等待者；先确认旧 ERRI，再读取最新 LEC/BOFF，避免错误已恢复却留下
  ERRI 导致反复唤醒，也避免在读取“无错误”后清除新来的事件。
- 取消异步发送只 abort 该 future 已提交的邮箱；等待满邮箱时取消不会影响其他提交。
  取消接收关闭所选 FIFO 与错误中断；drop 关闭中断、请求 abort/sleep、断开引脚及关时钟。
  取消不能撤回总线上已经发出的帧；Mailbox 结果只在该邮箱被复用或控制器销毁前有效。
- 新增 hal_can，默认 SilentLoopback，无需外部收发器和接线。先以 125/500/1000 kbit/s
  做阻塞回环，再测试 72 个标准/扩展数据/远程帧、两个 FIFO、bank13、过滤拒收、
  pending receive 取消/复用、FIFO overrun 及 drop 时钟释放。CAN_RESULT 为 1=运行、
  2=通过、10～26=失败；CAN_SNAPSHOTS 和 CAN_FRAMES 用于读回。
- 合并主机测试 128 项通过，包含 10 个持久化 CAN 测试及 14 个临时寄存器生命周期测试；
  覆盖三个邮箱、TX/RX/SCE 实际 waker 调用、立即完成、取消与满邮箱、错误恢复、
  W1C 清标志、过滤器和初始化失败清理。16 组库矩阵通过，hal_can 默认及
  Embassy+PLL+APB1 /4 配置均构建链接通过。
  记录：/tmp/ch32v4-can-host.log、/tmp/ch32v4-can-matrix.log、
  /tmp/ch32v4-can-example-{default,pll}.log。
- 2026-11-03 72 帧内部回环已上板通过；物理位时序、收发器、外部总线仲裁、
  错误恢复和电气质量仍需外部 CAN 环境。后续推进 USBHS、SDIO、Ethernet、I3C 等。

## 阶段 7e：USBHS device

- 基准：CH32V407RM 第 22 章及官方 EVT 的
  `USBHS/Device/CH372Device/User/ch32v407_usbhs_device.c`、USB 定义头文件、RCC UTMI
  时钟函数。USBHS1/2 分别使用自身总线门控、复位、UTMI 门控、中断和软件状态；
  共用的 480 MHz PLL 由 RCC 配置，驱动销毁单个实例不关闭共享 PLL。
- 修正 ch32-data `usbhs_v4x7.yaml` 的 15 个 device DMA 地址字段：24 位改为完整
  32 位，与 RM 和 EVT 中写入完整 SRAM 指针一致。重新生成数据及全系列 PAC，
  加入 `0x200xxxxx` 地址往返测试，避免字段 setter 截断 SRAM 高位。
- 独立 `src/usbhs/v4x7.rs` 对接 `embassy-usb-driver`，不改动 V3 USBHS 后端。
  物理端点 1–7 支持独立 IN/OUT 分配；同编号两方向的 MPS 必须一致。
  EP0 保留共享 64 字节缓冲区，非控制端点每方向独占缓冲；内部 SRAM 校验和
  4 字节对齐在初始化时完成。驱动、总线与端点保持调用者缓冲的可变借用生命周期，
  用户 read/write 临时切片不直接参与 DMA。
- 实现总线 enable/disable/reset、端点 enable/STALL、手动 DATA toggle、
  异步 IN/OUT、控制 SETUP/data/status、状态包 ACK 后设置地址，以及 suspend/resume。
  新 SETUP 先快照 8 字节，再使旧控制传输失效；即使 SETUP 已到而 ISR 尚未执行，
  旧传输也检查硬件 pending SETUP，防止提交过期地址或数据。
- 按 RM 的 W0C 清除各端点 DONE/NAK_ACT；TRANSFER 总标志是只读汇总位，
  不向其写 1。ISR 扫描全部物理端点，处理同时完成、重复 OUT toggle 与 FIFO overflow。
  取消 IN 后已复制的 owned packet 可继续等待主机，下一次 write 等其完成才复用；
  取消 OUT 不丢弃已收包。复位、disable 和 STALL 切换推进 generation，唤醒旧等待者。
- 新增 `hal_usbhs --features usbhs`：全速 CDC 二进制回显，支持 USBHS1
  PA12/PA11 和 `usbhs2` PB7/PB6。默认 HSI 供 USBHS PLL，支持 HCLK 200 MHz
  的 `clock-pll` 和 25 MHz 晶振供 USB PLL 的 `clock-hse`。RET6 不提供 USBHS2。
  Linux `usbhs_echo_test.py` 通过 DTR 打开 CDC，检查 63/64/65 等包边界和全部字节值。
  独立 PTY 回归完成 120 次、62160 字节往返，并确认超时与数据损坏会报错；这不是 USB 板测。
- 主机组合测试 155 项通过，其中 USBHS 27 项：PAC 地址/宽度、MPS、SRAM 范围、
  分配失败不泄漏、完整 HS 512 字节收发和 7 对端点、IRQ 实际唤醒、同时完成、取消、
  STALL/disable、复位、控制状态阶段、新 SETUP 抢占（含 ISR 前到达）、畸形 SETUP 恢复、
  地址提交时机、重复包、FIFO overflow、挂起恢复、双实例隔离及 ISO toggle 规则。
  注入测试复现并修复 ISR 两次读取 EP0 RX 之间到达 SETUP 时将其误当 OUT data 的竞态：
  SETUP 分类和普通 RX 分支使用同一次 RX 快照；快照之后的新 SETUP 留待后续中断处理。
  组合 harness 为 `/tmp/ch32v4-usbhs-tests`。USBHS 独立模型已落到
  `tools/usbhs-host-tests`，27 项再次通过；直接引用生产驱动和相邻 ch32-data 的生成
  PAC，避免复制驱动实现。运行：
  `cargo test --manifest-path tools/usbhs-host-tests/Cargo.toml --target x86_64-unknown-linux-gnu -- --test-threads=1`。
  模型不证明实际 USB 协议、电气或 DMA 总线时序。
- 构建：六个 V407/V467 芯片各 blocking、Embassy/EXTI 两组，加 V003/V103/V307/H417
  四组旧系列，共 16 项通过；CDC 的默认、PLL、HSE、USBHS2、RET6/USBHS1、
  VET6/USBHS2 六组 release ELF 构建通过。CDC 依赖关闭不需要的默认 HID feature。
- 当前限制：默认 FS；HS 包长与寄存器逻辑已有模型覆盖，但 PHY、回退、ISO 时序和
  remote wakeup 尚待硬件。Embassy USB 0.6 缺少 other-speed configuration 描述符，
  不能只把 speed 改成 High 就宣称完整双速设备。端点 9–15 的 AF 复用、burst、LPM、
  host 模式待后续；没有 VBUS 检测，因此不产生物理拔线的 PowerRemoved 事件。
- 硬件验收：恢复调试连接后先跑默认 USBHS1，检查主机枚举、打开关闭、120 次二进制
  回显、USB reset 后重连、拔插后重新枚举；再测 HSE/HCLK PLL 和 USBHS2。
  `USBHS_STAGE`、echo packets/bytes、error counters 提供 RTT 断连后的观察点。
  连接已于 2026-11-03 恢复，但尚未接 USB 主机做枚举/回显测试；不把主机测试或
  构建通过记作板测通过。后续继续 SDIO、Ethernet、I3C 等。

### 下一阶段：SDIO 基线审查

- RM 第 24 章和 EVT `SDIO/SDIO_SD/User/sdio.c` 是下一阶段基准。V4x7 保留
  V3 风格命令/数据寄存器，并增加 DCTRL2；不能只扩大模块 cfg 就认定驱动可用。
- RM 给出的 SDIO_CK 为 HCLK/(CLKDIV+2)，CLKDIV 为 8 位，实际分频范围 2–257；
  bypass 要求 HWFC_EN。旧驱动允许除数 258，转换 u8 后会回绕，且没有检查请求频率为 0。
  V4x7 新实现必须拒绝不可表示的频率，不能通过截断而超频。
- 400 kHz 识别阶段要求 HCLK 不超过 102.8 MHz 才能用现有分频器表示；200 MHz 下
  最低约 778 kHz，不能直接按默认高速配置初始化 SD 卡。先使用 HSI 20 MHz 验证，
  对不支持的 HCLK 返回 BadClock，不在驱动内偷偷修改整个系统时钟。
- EVT 和 DMA2 表均指定 SDIO 使用 DMA2 channel 4；RCC 仅有门控，无 SDIO reset 位。
  需要显式清理控制器状态。当前 V3 驱动包含无界 wait_idle/abort 等待，数据完成也未
  同时确认 DMA 完成/错误。V4x7 接入时需覆盖无卡超时、CRC/overrun/underrun、DMA 错误、
  取消及重新使用，然后提供默认只读卡信息/块读取例程；写卡测试另行显式启用。

### 阶段 7f：SDIO 接入与验证（2026-11-01）

- 新增独立 `src/sdio/v4x7.rs` 后端，接入 V4 引脚和 DMA 生成；保留旧系列后端。
  提供 1/4 位 SDSC/SDHC/SDXC 初始化、SDR12/SDR25、单双块及多块 DMA 读写、SD Status，
  同时支持阻塞和 Embassy API。每次最多 511 个 512 字节块，受 DMA 16 位计数限制。
- CK/CMD 固定 PC12/PD2；D0/D1 支持 PC8/PC9 或 PB14/PB15 成组重映射；D2/D3 为
  PC10/PC11。DMA 固定 DMA2_CH4，采用 32 位数据宽度；使用四字节对齐的 `DataBlock`。
- 识别阶段检查 HCLK/400 kHz 分频可表示性；高速切换先检查 CMD6 支持和 busy 状态，
  确认选中函数后才加速。繁忙或不支持高速时保留默认速率。bypass 默认关闭，启用时同时
  开启 HWFC_EN。驱动不修改系统时钟。板测例程的 PLL 配置单独选 HCLK 100 MHz。
- 修正 SD Status FIFO 字序/端序；卡容量用 u64 块数保留最大 2 TiB SDXC 边界，
  避免依赖 `sdio-host` 的 u32 块数计算。SDSC 使用字节地址，高容量卡使用块地址。
- 数据成功同时要求 DMA 完成、剩余数量为零、DATAEND、状态机空闲及无错误；再次检查
  完成后出现的错误。阻塞等待有轮询上限，异步数据等待还有软件超时。DMA 增加 V4 专用
  checked 构造方法，错误停止通道、锁存并唤醒，SDIO 转成 `Error::Dma`；普通构造方法
  保留错误 panic 行为。取消或数据失败先停止 DMA，再清理控制器、断开卡状态，需重新初始化。
- 无 SDIO RCC reset 位，显式停止命令/数据状态机并关闭电源。依据 RM §24 修正
  `ch32-data` 的 DCTRL2.SLV_CK_PHASE 描述：host 模式为 0 时清 DTEN 会清 FIFO，
  初始化与传输均显式选择该模式；已重新生成 data/PAC。
- 可复现测试位于 `tools/sdio-host-tests`，直接编译生产 SDIO/DMA 和生成寄存器。
  34 项通过（含 8 项 DMA ring-buffer 回归），覆盖命令/DMA 次序、回调唤醒、晚到错误、
  超时、取消、卡忙、无卡、参数/地址/容量边界及高速切换回退和失败恢复。数据写入仅在 RAM
  模型中执行；模型地址映射、时序和电气限制见该目录 README。
- 修改 DMA 后的其他外设主机回归 155 项通过；6 个 V4 型号各自阻塞/Embassy，以及
  V003/V103/V307/H417，共 16 组 release 库构建通过。`hal_sdio` 的默认、4 位、重映射、
  PLL 高速、APB 分频、bypass、RET6 共 7 组 release 构建通过。
- 新增只读板测例程 `hal_sdio`，读取卡信息、SD Status、单/多块数据并比较，检查取消后的
  重新初始化及 Drop 后门控/DMA 停止。无卡返回独立结果，不记作读卡通过。设备仍不能 attach，
  本阶段没有 SDIO 上机结果。真实卡兼容性、总线时序、信号质量和写卡均未验证；eMMC、8 位、
  SDIO functions、1.8 V/UHS 暂未实现。后续继续 Ethernet、I3C 和其他专用外设。

### 下一阶段：Ethernet 基线审查

- 事实基准为 RM §29、EVT `ETH/MAC_RAW/ETH_Driver/eth_driver.c` 及外设库
  `ch32v4x7_eth.c`。当前数据定义为 `emac_v4x7`，已有 HAL 的 v1 后端只针对旧系列，
  `src/lib.rs` 仍排除 V4。V4 MACCR/MACFCR/PMT 与旧系列有差异，需单独接入。
- MAC 位于 0x40028000，DMA 位于 +0x1000，使用自身的描述符链；不使用通用 DMA 通道。
  内置 10/100M PHY 默认 SMI 地址为 1，无需外接 RMII GPIO；PHY LED 默认 PE8/PE9，
  与现有心跳输出冲突，接入时需选择是否启用 LED 或重映射到 PD14/PD15。
- EVT 依赖 25 MHz HSE 驱动 ETH PLL，再开启 ETHMAC 门控、总线复位、释放 PHY PD/RSTN，
  最后等待 DMA software reset。RCC 已有 ETH PLL 配置及源校验，驱动应验证该前提并对
  所有复位/SMI 等待设上限；不继承 EVT 部分无界等待和用零数据表示 SMI 失败的接口。
- RM §29.2.1.20 明确 PHY_CR 总是读零；控制位须完整写入或保存软件影子，不能用 RMW
  保留配置，也不能依赖其中 SPEED/DUPLEX 读取链路。链路能力应通过 SMI PHY 状态取得。
- 下一实现需提供有借用约束的 DMA ring/buffer、收发 token、错误帧回收、DMA 错误恢复、
  链路中断/唤醒及 `embassy-net-driver` 接口，并用生产代码模型检查 OWN/FS/LS、CRC 长度、
  token 取消/Drop 和 late IRQ。实机先做无网线初始化/PHY ID，再做物理链路和帧收发；
  目前调试连接未恢复，不把构建或 RAM 描述符模型视作联网验证。

### 阶段 7g：Ethernet 接入（2026-11-01）

- 新增 V4 独立后端 `src/eth/v4x7.rs` 和描述符辅助模块，接入 `embassy-net-driver`；
  不复用 V307/H417 的 PHY 和 DMA 配置。要求 RCC 先启用 25 MHz HSE/ETH PLL，MDC 分频
  按 RM 的 20–200 MHz HCLK 档位选择，验证不超过 2.5 MHz。驱动只操作 ETHMAC 门控，
  不关闭可能供其他外设使用的 ETH PLL。
- 修正 `ch32-data/emac_v4x7` 的旧版残留：PHYLINK_SR/IER 应在 bit 11，移除 bit 31；
  DMA 总线模式仅 SR/DSL，操作模式没有旧版 PBL/RSF/OSF/RTC/TTC 等配置；添加 MACCR.RD，
  按寄存器位表将 FES 标为只读，并移除位表列为保留的 LM。PHY_CR 明确始终读零，
  SPEED/DUPLEX 标为只读。已重新生成 data/PAC，初始化测试检查 DMA/MMC 写值不含保留位。
- RM 与 EVT 的冲突单列为待上机确认项：MACCR 图中有 LM 而位表将 [13:12] 列为保留；
  FES 在位表为 RO，但 EVT 写 FES；DMASR/DMAIER bit 13 列为保留，但 EBS 描述又提及 bit 13，
  EVT 库也包含标准 FBE 宏。当前驱动按位表不写 FES/LM、不启用或清除 bit 13；链路速度从
  内置 PHY page 0 状态 0x1a 获取，异常进程停止报错并提供显式 `recover()`，不声称这些
  冲突已由 RAM 模型解决。
- PHY 控制完整写入复位/掉电/地址位，SMI 不用零值冒充超时。识别允许 PHY 延迟就绪，
  PHY ID/复位重试有独立次数限制；SMI 和 DMA reset 有轮询上限。配置 page 7 链路变化中断，
  清 page 0 中断源，使用 BMSR 双读处理 latch-low；获取 PHY 解析速度/双工，支持自动协商
  及对端强制速率的 parallel detection。GPIO PHY LED 功能默认关闭，不占用 PE8/PE9。
- `Resources<TX,RX>` 必须为 static，DMA 写入区域使用 UnsafeCell；忘记 driver 或 token
  不会令 DMA 指针指向失效栈内存。每帧单一四字描述符/缓冲区，链式队列，OWN 交接使用
  volatile 和内存屏障。RX 校验 ES/FS/LS/长度并扣除 FCS；无效帧回收；RX token Drop 回收
  描述符，TX token 未消费则不提交，短帧补零至 60 字节。保留软件校验和，MTU 1514。
- ISR 注册实际 waker、确认快照内 W1C 标志，PHY 源先屏蔽再在前台清理/重开，避免持续 IRQ。
  队列满等待 TX 完成唤醒；正常断链停止产生的状态在临界区单独确认，避免当作故障。
  SMI/停止故障静默 MAC/DMA，显式恢复丢弃旧队列并重新初始化；Drop 复位并关闭门控，
  late IRQ 由 ACTIVE 状态拒绝。恢复需应用持有 driver；没有后台自动重启机制。
- `tools/eth-host-tests` 直接编译生产后端和生成寄存器，16 项模型测试通过，覆盖链表、
  唤醒、满队列、长度/FCS、时钟、延迟就绪、超时、取消/回调 unwind、断链/重连、恢复失败
  及 Drop。主机 peer 脚本的 10 次/4466 字节回显、超时和损坏注入路径通过 socket 模型。
- 新增 `hal_eth`：无网线先检查 PHY ID，再等待链路并回显自定义 EtherType 帧；
  `eth_echo_test.py` 默认验证 100 帧，最大 1500 字节负载。GPIO LEDs 不占用，PHY jack/
  磁性器件和 Linux 有线接口需准备。当前调试连接未恢复，没有 PHY 实测或真实链路/帧收发结果。
  10/100M、半双工、协商/复位时序、DMA 总线时序、FES/bit 13 冲突均待硬件验收。
- 暂不提供 MAC/PHY loopback API、PTP、硬件 checksum、WoL、VLAN/jumbo frame 和 TCP/IP
  栈例程；后续继续 I3C、PSRAM/FSMC、LTDC/ARGB 等专用外设及整体验收。
- 最终软件回归：6 个 V4 型号各自阻塞/Embassy，加 V003/V103/V307/H417，16 组 release
  库构建通过；`hal_eth` 默认、PLL、APB 分频和 RET6 共 4 组 release 构建通过。
  再生 PAC 后，SDIO/DMA 34 项及其他外设 155 项主机回归通过；格式与两仓 diff 检查通过。

### 阶段 7h：I3C 首轮接入（2026-11-01）

- 按 RM 第 19 章与 EVT `I3C_DMA_Controller` 核对：I3C 是独立的 HCLK 外设，
  并非 I2C 寄存器组的扩展。采用独立 `src/i3c.rs` 和传输状态机；传统 I2C 的单方向消息
  在上层实现 `embedded-hal::i2c::I2c` 与 Embassy async trait。GPIO 使用 EVT 的
  AF push-pull 配置，接入默认 PB10/PB11 与重映射 PB4/PB5 的生成 pin traits。
- 修正 ch32-data 中 TMODE（FIFO 预装载，并非 target mode）、MID（消息序号）、
  XDCNT、ABT、PERR 和 TXLASTF/RXLASTF 描述，重新生成全部 PAC。
- 第一轮采用 byte FIFO 的 blocking / interrupt 后端：支持 I2C 同向 operation
  合并、SDR 私有事务及其重复起始、普通广播 CCC、无 defining byte 的单目标直接 CCC，
  以及 ENTDAA。ENTDAA 使用调用者提供的地址池及结果数组，验证保留/重复/本机地址，
  分别保存 PID/BCR/DCR，并检查结束时的硬件目标数量；不继承 EVT 的无界数组及累积 PID 问题。
- RM 19.4.1 与 19.4.2 的 MEND 说明明确要求 MTYPE=4（legacy I2C）时 MEND 不能为 0。
  因此不承诺完整 I2C 兼容：跨方向 transaction/write_read 在硬件操作前返回
  UnsupportedTransaction。需重复起始的 I2C 设备继续使用独立 I2C 外设；不能用多个 STOP
  替代 trait 约定的 repeated start。该限制也加入模型断言，不能用模型构造出手册禁止的支持。
- 使用 TMODE 预装载，先设 CRINIT 再解除内部复位；关停 IBI/hot-join 接收。
  按 EVR 的 FIFO 请求逐项服务，最后等待 FCF，错误优先于完成；RX FIFO 在 FCF 后
  仍可排空。提前结束私有读返回 ShortRead，不等待不存在的数据。
- blocking 使用总轮询步数上限，async 使用 Embassy timeout；取消/失败关闭控制器
  请求并设置 NeedsRecovery，禁止直接复用。显式 reinitialize 只重建控制器，不代表
  已对物理总线做恢复；DAA 失败时外部设备可能已经部分接受地址，结果仅为暂存。
- 时序采用绑定 HCLK 的显式 raw counters，不给出未经 RM/目标器件约束验证的自动
  FREE/混合总线计算公式。运行中 HCLK 缓存变化会拒绝新事务。无通信默认速率承诺。
- 主机测试直接编译生产后端及 PAC，覆盖控制/数据 FIFO、CCC、ENTDAA、短读、错误、
  超时、取消、复用与 Drop。新增 hal_i3c：默认只做寄存器生命周期检查；i3c-daa
  必须显式提供审核后的 TIMINGR0/1，才运行 RSTDAA → ENTDAA → GETPID。
- 未上板，不将模型/构建通过当作通信验收。现有探针连接故障尚无恢复验证。
  板测需先恢复设备连接，准备原生 I3C target、PB10/PB11 与共地及匹配的上拉。
  原有 SysTick 自然回卷验收仍保留。
- 下一步为 DMA1 CH4 TX / CH5 RX 接入与回归（控制 FIFO 为 CH8，独立评估是否需要）；
  FIFO 中断后端尚不能据此宣称高速 SDR 能力。之后补真实设备与中断压力验收，
  再推进 IBI/hot-join、RSTACT/完整 CCC 变体及其他专用外设。
- 本轮最终软件验证：I3C 30 项、Ethernet 16 项、SDIO/DMA 34 项主机测试通过；
  六个 V4 型号各自 blocking/Embassy 与 V003/V103/V307/H417 共 16 组 release 库构建通过。
  hal_i3c 默认、AHB 分频、引脚重映射、RET6 与 DAA 分支共 5 组 release 构建通过。
  DAA 构建使用 raw 寄存器 fixture，仅用于编译分支检查，未烧录或资格验证通信时序。
  rustfmt 与两仓 diff 检查通过；结果均不替代上机验收。


## I²C DMA 收尾与上游 PR 对账（2026-11-01）

### I²C DMA 软件收尾

- 本轮按用户所写 I²C 核验；独立 I²C 的 DMA 已在阶段 6c 接入，TX=DMA1_CH6、
  RX=DMA1_CH7。I3C 是另一外设，其 DMA1_CH4/CH5 接入仍是阶段 7h 的待办，
  本轮结果不代表 I3C DMA 已实现。
- 将先前位于 `/tmp/ch32v4-i2c-tests` 的 I²C 生命周期模型固化为
  `tools/i2c-host-tests`。构建时读取当前生产 `src/i2c.rs`，挂接原有单元测试与
  生命周期测试；直接使用当前 DMA 后端和生成 PAC，不保留独立驱动副本。
- 23 项测试通过：15 项 I²C 时序/事务/生命周期，8 项 DMA 寄存器窗口/环形缓冲回归。
  覆盖短读及 DMA 前缀/CPU 尾部、分散 read、双向 repeated START、空写、
  外设错误与 DMA 同时完成、取消、超时及重新使用。
- `hal_i2c` 默认 Embassy 配置和 PLL/APB 分频 + `i2c-eeprom` 两组 release
  构建链接通过。格式检查及 diff 检查通过；本轮未改动 I²C 生产驱动逻辑。
- 模型在请求使能时模拟 DMA 完成，不能证明真实请求路由、IRQ 延迟和 ACK/NACK 波形。
  板测仍需恢复调试连接，准备 3.3V 24C02（0x50）、PB6/PB7、共地及外部上拉；
  用逻辑分析仪/示波器核验 100/400kHz、repeated START、读尾 NACK 和 STOP。

### 上游已有实现及复用建议

通过 GitHub API 拉取 `ch32-rs/ch32-hal` 全部 129 个 PR 的标题、状态和正文，
其中 17 个 open、92 个 merged、20 个 closed；另读取 #215、#199、#227 的变更范围。
以下是本次查询快照，状态应在实际移植时重新确认。

- **CRC / IWDG / WWDG：优先参考 [#215](https://github.com/ch32-rs/ch32-hal/pull/215)**。
  当前 open，标题限定 V208/V305；包含 `src/crc.rs`、`src/wdg/{iwdg,wwdg}.rs`
  和 CRC 例程。V4 当前 PAC 分别为 crc/h4、iwdg/v00x、wwdg/common。
  CRC 所用 DATAR/IDATAR/CTLR 与字段访问器在当前 PAC 中存在，具备复用基础，
  但尚未编译移植或上机验证，不能据此声明已支持 V4。
  接入时审查 RCC gate/reset、CRC 数据位序/已知向量、看门狗同步与实际 LSI、
  调试冻结及复位原因；PR 中还有无界寄存器等待，需结合 V4 的错误策略审查。
- **OPA：未找到相关 PR**。在所拉取 PR 标题/正文中检索 OPA、opamp、
  operational amplifier 和中文运放相关词，并单独检索 OPA，均无命中。
  这不排除未提交分支或外部仓库实现；V4 OPA 仍按独立任务排期。
- **RTC/BKP、I2S、DVP、FSMC、PSRAM：本次标题/正文检索未找到对应实现 PR**。
  这些项目继续保留本地迁移任务。
- **I²C slave：[#199](https://github.com/ch32-rs/ch32-hal/pull/199)** 已有 blocking /
  async + DMA 实现，正文说明仅在 V203 验证；另有较早的 #164。
  它们属于 slave 扩展，不是当前 V4 master DMA 收尾；移植时审查与本地事务/清理修复的冲突。
- **Timer 编码器：[#227](https://github.com/ch32-rs/ch32-hal/pull/227)** 提供 QEI mode 3、
  输入滤波和 16 位计数，正文记录 V307 板测；可作为 timer 高级模式的移植起点。
- **USART circular DMA RX：#228；USBHS 等时传输：#229；ETH：#231**
  都可列为后续参考，分别核对 V4 DMA 窗口、独立 USBHS/ETH 后端，不能直接视为兼容。

### 当前 open PR 列表

| PR | 标题 |
| --- | --- |
| [#234](https://github.com/ch32-rs/ch32-hal/pull/234) | Update usbpd in examples to version 3 |
| [#233](https://github.com/ch32-rs/ch32-hal/pull/233) | fix: remove unconditional embassy-time dependencies from peripheral drivers |
| [#231](https://github.com/ch32-rs/ch32-hal/pull/231) | Improve eth/v1 initialization, ownership, and interrupt handling |
| [#230](https://github.com/ch32-rs/ch32-hal/pull/230) | fix(usbd): clear STATUS_OUT when a new setup transaction arrives |
| [#229](https://github.com/ch32-rs/ch32-hal/pull/229) | usbhs: isochronous support |
| [#228](https://github.com/ch32-rs/ch32-hal/pull/228) | feat(usart): ring-buffered rx over circular dma |
| [#227](https://github.com/ch32-rs/ch32-hal/pull/227) | feat(timer): quadrature encoder driver |
| [#226](https://github.com/ch32-rs/ch32-hal/pull/226) | fix(usart): keep TE / RE as they were when reconfiguring |
| [#224](https://github.com/ch32-rs/ch32-hal/pull/224) | feat(delay): Initial V4 nanosecond delay |
| [#220](https://github.com/ch32-rs/ch32-hal/pull/220) | fix: clean up chip-specific HAL and example builds |
| [#215](https://github.com/ch32-rs/ch32-hal/pull/215) | Add watchdog and CRC support for V208 and V305 |
| [#214](https://github.com/ch32-rs/ch32-hal/pull/214) | feat(examples): add ch32v208 sensor demos driven by our own edrv drivers |
| [#199](https://github.com/ch32-rs/ch32-hal/pull/199) | Complete I2C slave implementation with blocking and async + DMA modes |
| [#194](https://github.com/ch32-rs/ch32-hal/pull/194) | ADC async single shot |
| [#193](https://github.com/ch32-rs/ch32-hal/pull/193) | Update README support table |
| [#192](https://github.com/ch32-rs/ch32-hal/pull/192) | CH32X035 usb support |
| [#164](https://github.com/ch32-rs/ch32-hal/pull/164) | Add I2C slave driver and examples |


## 阶段 7h 续：I3C byte DMA 首轮接入（2026-11-02）

### 范围与依据

- 用户确认上轮所指为 I3C DMA；独立 I²C DMA 保持阶段 6c 实现。
- 重新读取 CH32V407RM V1.3 §19.3.3/19.3.4、§19.4.3、§19.4.4/5，以及官方
  `openwch/ch32v407_ch32v467` 的 `EVT/EXAM/I3C/I3C_DMA_Controller/User/main.c`。
  手册 SHA256 与先前记录一致：
  `77de94a3c80e64104242a5d7486a9a23caa417d3476284fd852f0d5ce89f7216`。
- ch32-data 基线确认：TX=DMA1_CH4、RX=DMA1_CH5，控制 FIFO 为 CH8。
  本轮只接入 byte data DMA：RDR=0x10、TDR=0x18，RXTHRES/TXTHRES 均为 0；
  不启用控制 FIFO DMA，不修改 PAC 数据定义。

### 实现与边界

- 新增 `I3c::new_with_dma`，持有带 `TxDma<T>` / `RxDma<T>` 类型约束的通道；
  build.rs 生成 I3C TX/RX 通道绑定，普通 `new` 继续使用原 FIFO 后端。
- Embassy 私有事务和受支持的 legacy I²C 事务，在每个方向至多一个连续缓冲区时
  自动使用 DMA。支持单独写/读、私有 W→R/R→W 重复起始，各方向最多 65535 字节。
  有多个同方向缓冲区的 scatter 事务整体走原 FIFO 后端，不在消息中途重装 DMA。
- CCC、ENTDAA 和 blocking 方法仍使用 FIFO；IBI/hot-join、RSTACT、控制 FIFO DMA
  和 scatter DMA 仍未实现。legacy I²C 的 MEND=1 约束不变，跨方向 transaction /
  write_read 继续在硬件操作前返回 UnsupportedTransaction。
- 在控制字入队前建立数据 DMA、开启对应请求。CPU 只投递最多两个控制字，不访问
  DMA 正在收发的数据寄存器。使用 checked DMA API，将 DMA 错误返回为 Error::Dma。
- 成功要求对应 DMA 完成且剩余计数为零、全部控制字已排队、FCF 到达；最终重新检查
  外设错误及 DMA 错误。已经观察到 FCF 而 DMA 尚未完成时，不重复使能 FCF IRQ，
  等待 DMA 唤醒或超时。DMA 完成不能替代总线 STOP/FCF。
- 成功、失败和取消均先关闭 DMA 请求，再停止/释放 DMA transfer。失败和取消随后
  禁用控制器并保留 NeedsRecovery，必须在应用恢复空闲物理总线后显式 reinitialize。
  不把控制器重置宣称为总线清除或 target 动态地址恢复。

### 软件验证

- `tools/i3c-host-tests` 共 50 项通过：原 I3C 30 项、DMA/环形缓冲 8 项、
  新增 I3C DMA 12 项。模型直接编译生产 I3C/DMA 和生成 PAC，按 FIFO 就绪状态搬运
  实际 buffer 字节；覆盖 1/3/4/7/8/9/31/257/65535 字节、双向重复起始、legacy
  限制、scatter/CCC fallback、DMA 与 FCF 的两种完成顺序、残余计数、checked DMA
  错误 IRQ 唤醒、外设错误优先、短读、取消、超时、恢复及硬件操作前校验。
- 六个 V407/V467 型号各 blocking/Embassy，加 V003/V103/V307/H417 共 16 组
  release 库构建通过。
- `hal_i3c` 的 FIFO、DMA、DMA+AHB 分频、DMA+remap、DMA+RET6 和
  DMA+DAA+echo fixture 共 6 组 release 构建通过。echo 分支的 raw timing fixture
  仅用于编译分支检查，未经通信资格验证，未烧录。
- 独立 I²C/DMA 的 23 项主机回归通过；相关驱动及测试的 rustfmt 与本 HAL 的
  git diff --check 通过。主机模型不能证明真实 DMA 请求路由、
  电气时序、外设访问宽度是否被硅实现接受、IRQ 压力下可靠性或最大 SDR 速率。

### 板测准备

- 当前 CH32V467WEU6 板，默认 PB10=SCL、PB11=SDA、共地；上拉及电压遵循 target
  规范。调试连接恢复后再烧录，本轮没有新的连接恢复或硬件通信结论。
- 准备原生 I3C echo target，支持 RSTDAA/ENTDAA/GETPID，并将最后一次私有写入内容
  回显给私有读取；需同时支持 STOP 分隔和 repeated START，数据长度至少 257 字节。
  普通 I²C 从机不能承担这项 DAA/私有传输验收。
- 按实际 HCLK、target 与总线审核 I3C_TIMINGR0/1；不得直接把构建 fixture 当作
  可用时序。显式启用 `i3c-dma-loopback` 才向 target 写入测试 payload，不能用于普通
  传感器或现有应用总线。
- 运行 1/7/8/9/31/257 字节分帧写读及 repeated-start 回显；检查 I3C_DMA_RESULT=2、
  数据相等、两个 DMA EN 与请求位清除。用逻辑分析仪检查 START/地址/数据/STOP，
  再做 NACK、超时、取消恢复和代表性中断压力测试；通过前不声明最高通信速率。
- 后续独立评估 scatter DMA / 控制 FIFO DMA，以及 IBI/hot-join 和更多 CCC 形式。
