# CH32H417 Dual-Core (V3F+V5F) 实验记录

> 2026-08-20 更新: 双核唤醒已跑通。早期"失败"结论已修正。

## 已验证（最终状态）

| 项目 | 状态 |
|------|------|
| V3F blinky (GPIO+RCC+Delay, PF2/LED0) | 通过 |
| +a 原生原子操作 | 通过 |
| probe-rs flashing (WCH-Link) | 通过 |
| 共享 SRAM (0x20100000) 读写 — HSI 25MHz | 通过 |
| 共享 SRAM 读写 — PLL 400MHz | **通过**（早期误判，见下） |
| V5F 唤醒 (CPU 写 WAKEIP+SCTLR SENDEVENT) | **通过** |
| V5F 写 ITCM (0x200A0100 标记) | **通过** |
| V5F 独立 GPIO 翻转 (PF0/LED1) | **通过** |

## 双核启动流程（工作版本）

1. V3F `hal::init()` (Pll400MHsi) → 配置 GPIO/LED
2. V3F 调用 `qingke::pfic::wake_other_core(0x08002000)`：
   - `PFIC_WAKEIP1 (0xE000E724) = entry`（清 SHUTDOWN 位）
   - `PFIC_SCTLR (0xE000ED10) |= 1<<5`（SENDEVENT）
3. V5F 从 0x08002000 启动，qingke-rt 初始化，跑 `main()`
4. 两核独立运行

qingke 的 `wake_other_core` **原始实现（RM 地址）完全正确**。

## 关键教训

### 1. SENDEVENT 只能由 CPU 总线写入触发

`wlink write-mem` 通过 debug bus 写 SCTLR **不会**产生 SENDEVENT 事件。
手动调试唤醒必须让 V3F 代码亲自写寄存器。这是早期"手动唤醒失败"的根因。

### 2. 共享 SRAM 一直可用

早期结论"hal::init() 后 SRAM 写被丢弃"是**采样方法学错误**：
- wlink halt 是异步的，dump 可能落在迭代间隙
- 循环第一次写 counter=0 到 SRAM，读到 0 被误读为"写失败"
- 用多时间点采样 + 读写回环验证后，SRAM 在 400MHz PLL 下完全正常

### 3. V5F 从 flash 执行很慢

V5F 启动没有配置 flash latency（SDK 在 handle_reset 里
`FLASH_ACTLR |= 3`）。V5F 的 5M NOP 忙等实际跑得很慢（>2s）。
SDK 的 highcode 模型（代码搬到 ITCM/共享 SRAM 零等待执行）解决此问题。
可选优化：V5F 启动加 flash latency 配置。

## SDK 参考

- 唤醒序列 (main.c): `NVIC_WakeUp_V5F(addr)` = WAKEIP[1]+SCTLR SENDEVENT，
  之后 V3F 进 STOP_WFE，V5F 就绪后 HSEM 唤醒 V3F
- FreeRTOS 双核: V3F 代码搬 0x20100000, V5F 代码搬 0x200A0000 (ITCM),
  共享数据 `.shared_data` @ 0x20178000
- V5F handle_reset: 先 `FLASH_ACTLR |= 3` 再搬代码（flash 加速）

## TODO

- [ ] V5F 启动加 flash latency 配置
- [ ] 跨核通信 demo（V5F 写 ITCM 标记 → V3F 读取显示）
- [ ] HSEM/IPC 驱动
- [ ] highcode (Flash→RAM) 链接模型
- [ ] defmt-rtt 在 WCH-Link 上的输出（probe-rs 后端）
