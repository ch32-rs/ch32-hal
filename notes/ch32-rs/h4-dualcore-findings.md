# CH32H417 Dual-Core (V3F+V5F) 实验记录

> 2026-08-13, nanoCH32H417 开发板, WCH-LinkE v2.20

## 已验证

| 项目 | 状态 |
|------|------|
| V3F blinky (GPIO+RCC+Delay, PF2) | 通过 |
| +a 原生原子操作 (ITCM/DTCM) | 通过 |
| probe-rs flashing (WCH-Link) | 通过 |
| defmt 编译 + RTT 符号保留 (`--undefined=_SEGGER_RTT`) | 通过 |
| ITCM 0x200A0000 CPU 写 | 通过 |

## 未通过

| 项目 | 状态 |
|------|------|
| V5F 唤醒 (WAKEIP+SCTLR SENDEVENT) | 失败 — 寄存器写入无效果 |
| V5F 唤醒 (PFIC_IRER12/13 0xE1B0/0xE1B4) | 偶然成功一次，不可复现 |
| 共享 SRAM 0x20100000 CPU 写 (V3F) | 失败 — `sw` 指令静默丢弃, wlink 可写 |
| probe-rs RTT 输出 (WCH-Link 后端) | 失败 — 数据在 buffer 但 host 读不到 |
| SDI print (wlink) | 失败 — "Chip doesn't support SDI print" |

## 根因分析

SDK (C) 双核工作依赖 boot ROM 初始化，裸 Rust flash 启动缺失：

1. **Bus matrix**: 共享 SRAM (0x20100000) 路由在 boot ROM 激活。
   我们的裸启动跳过后, V3F CPU 无法访问该区域 (wlink/debug bus 可以)。
2. **代码模型**: SDK 用 highcode 模型 (Flash→RAM 搬迁):
   - V3F 代码 → 0x20100000 (共享 SRAM)
   - V5F 代码 → 0x200A0000 (ITCM)
   我们是 Flash 原地执行 (0x08000000 / 0x08002000)。
3. **V3F STOP**: SDK 在 `NVIC_WakeUp_V5F()` 后立即
   `PWR_EnterSTOPMode(WFE)`, V5F 就绪后通过 HSEM 唤醒 V3F。
   这个电源状态变化可能是 V5F 实际启动的触发条件。

## SDK 参考

- `EVT/EXAM/CPU/OS/FreeRTOS/FreeRTOS_Core/{V3F,V5F}/Ld/Link_*.ld`
  - 共享数据段 `.shared_data` @ 0x20178000 (32KB, 两核同址)
  - V3F RAM_CODE @ 0x20100000, V5F RAM_CODE @ 0x200A0000
- 唤醒序列 (main.c):
  ```c
  NVIC_WakeUp_V5F(addr);      // WAKEIP[1]=addr; SCTLR|=1<<5
  HSEM_ITConfig(HSEM_ID0);    // 硬件信号量中断
  NVIC->SCTLR |= 1<<4;        // SEVONPEND
  RCC_HB1PeriphClockCmd(PWR); // PWR 时钟
  PWR_EnterSTOPMode(WFE);     // V3F 睡眠
  ```
- 跨核同步: HSEM (0xE000C000) + IPC (0xE000D000), 非裸共享内存

## TODO

- [ ] 逆向 boot ROM 初始化序列 (bus matrix / SRAM 路由)
- [ ] 验证 V5F 在 STOP 后唤醒假设
- [ ] HSEM/IPC 驱动
- [ ] highcode (Flash→RAM) 链接模型
