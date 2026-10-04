//! # RCC 时钟树参考（阅读用，不参与 HAL 运行时逻辑）
//!
//! 本文件汇总：
//! - 你期望的目标 API（`Clocks` / `Config` / `init` / `uninit` / preset）
//! - WCH CSDK 里 **预设时钟的命名习惯**（以本地 `~/WCH/CH32H417EVT` + 公开 V30x `system_*.c` 为准）
//! - **当前 ch32-hal** 各 `rcc_impl` 的 `Config` 形状与 preset 常量
//!
//! 打开方式：IDE 直接读此文件，或 `cargo doc --open` 后搜 `clock_tree_reference`。
//!
//! ---
//!
//! ## 目标架构（你的思路）
//!
//! ```text
//!   Config  (~= RM 时钟树，可嵌套子结构)
//!      │
//!      ├─ preset: Config::with_*() / const 工厂 → 填好一整棵树的常用组合（对齐 CSDK #define）
//!      │
//!      ▼
//!   init(config)  ──►  RCC/FLASH/…  ──►  更新 Clocks 缓存
//!   uninit()      ──►  回到安全默认（HSI、关 PLL 等，语义需文档化）
//!
//!   clocks()      ──►  &Clocks  （整系统时钟状态的快照，驱动只读这里）
//! ```
//!
//! **Preset 应是「返回完整 `Config` 的函数/常量」**，而不是第二套 `SysClk` 枚举 + 隐藏 `match`。
//!
//! 可选补充（非主路径）：`refresh(hse)` = 仅从寄存器重算 `Clocks`（bootloader / 未走 `init` 时）。

/// WCH CSDK：各系列 **宏命名** 与 **入口函数** 对照（便于定 Rust `with_*` 名字）。
pub mod csdk_naming {
    //! ## CH32H417（`system_ch32h417.c`，EVT `~/WCH/CH32H417EVT`）
    //!
    //! **选一个配方**：在文件顶部 **只启用一个** `#define`，值为目标 `SystemClock`（Hz）。
    //!
    //! | C 宏名 | 数值 | 配套 C 函数 |
    //! |--------|------|-------------|
    //! | `SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE` | 400000000 | `SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE()` |
    //! | `SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI` | 400000000 | `…_HSI()` |
    //! | `SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE` | 480000000 | `…_HSE()` |
    //! | `SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI` | 480000000 | `…_HSI()` |
    //! | `SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE` | 480000000 | `…_HSE()`（另配 VDDK 1.25V） |
    //! | `SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI` | 480000000 | `…_HSI()` |
    //!
    //! **命名规律**（H4 特有，信息量大）：
    //! ```text
    //! SYSCLK_<SYS MHz>M_CoreCLK_V5F_<v5f MHz>M_V3F_<v3f MHz>M_<HSE|HSI>
    //! ```
    //!
    //! **调度**：`SystemInit()` → `SetSysClock()` → 上表之一；无宏则保持 **HSI 25 MHz**。
    //!
    //! **读回频率**：`SystemAndCoreClockUpdate()`（读 `SWS`、`SYSPLL_SEL`、PLL 域、`HPRE`/`FPRE`）。
    //! - `SystemClock` = SYSCLK（PLL 路径上的系统时钟）
    //! - V5F 核频 ≈ `SystemClock >> HPRE`
    //! - V3F / `HCLKClock` ≈ 再 `>> FPRE`
    //!
    //! ---
    //!
    //! ## CH32V2/V3/F2（`system_ch32v30x.c` 等，公开 SDK 常见写法）
    //!
    //! **旧版/简写**（仍常见）：只标 SYSCLK，不标 HSE/HSI
    //! - `SYSCLK_FREQ_72MHz` / `SYSCLK_FREQ_144MHz` → `SetSysClockTo72()` / `SetSysClockTo144()`
    //!
    //! **新版**（带振荡器来源，与 HAL 现有常量更接近）：
    //! - `SYSCLK_FREQ_144MHz_HSE` / `SYSCLK_FREQ_144MHz_HSI`
    //! - `SetSysClockTo144_HSE()` / `SetSysClockTo144_HSI()`
    //!
    //! 另有直通：`SYSCLK_FREQ_HSE`（`HSE_VALUE`）、`SYSCLK_FREQ_HSI`（`HSI_VALUE`）。
    //!
    //! **命名规律**（V3 系）：
    //! ```text
    //! SYSCLK_FREQ_<频率>MHz[_HSE|_HSI]   →   SetSysClockTo<频率>[_HSE|_HSI]()
    //! ```
    //!
    //! **读回**：多数例程用 `SystemCoreClock` 变量 + `SystemCoreClockUpdate()`（无 H4 双核分频语义）。
    //!
    //! ---
    //!
    //! ## CH32V00x / V003 / X035 等（24M HSI、PLL×2 类）
    //!
    //! 典型：`SYSCLK_FREQ_48MHz` / `SYSCLK_FREQ_48MHz_HSE`，函数 `SetSysClockTo48()` 等。
    //! （具体以各 `system_ch32v00x.c` / `system_ch32v003.c` 为准；本地 EVT 仅 H417 时查对应 SDK 包。）
    //!
    //! ---
    //!
    //! ## Rust preset 命名建议（对照 C，待你拍板）
    //!
    //! | 风格 | 示例 | 说明 |
    //! |------|------|------|
    //! | A. 贴近 HAL 现状 | `Config::SYSCLK_FREQ_144MHZ_HSE` | 与 v1/v3 一致，全大写+MHZ |
    //! | B. 贴近 CSDK V30 | `config::preset::mhz144_hse()` | 小写+MHz 后缀 |
    //! | C. 贴近 CSDK H417 | `Config::with_480m_v5f_240m_v3f_120m_hse()` | 树形语义，冗长但无二义性 |
    //! | D. 分层 | `Config::evt_h417::sysclk_480m_v5f_240m_hse()` | 芯片子模块 + C 宏语义缩短 |
    //!
    //! 推荐：**树形 `Config` + `impl Config` 上 `const fn with_*` / 关联函数**，名字按系列选 B 或 D；
    //! H4 用 D/C，V3 用 B/A 均可与现有 `SYSCLK_FREQ_*` 做 type alias 过渡。

    /// H417 EVT 中出现的 C 宏名（复制粘贴对照表）。
    pub const H417_C_MACROS: &[&str] = &[
        "SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE",
        "SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI",
        "SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE",
        "SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI",
        "SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE",
        "SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI",
    ];

    /// V30x SDK 常见 C 宏名（公开仓库 `system_ch32v30x.c`）。
    pub const V30X_C_MACROS_EXAMPLES: &[&str] = &[
        "SYSCLK_FREQ_HSE",
        "SYSCLK_FREQ_HSI",
        "SYSCLK_FREQ_48MHz_HSE",
        "SYSCLK_FREQ_144MHz_HSE",
        "SYSCLK_FREQ_144MHz_HSI",
        "SYSCLK_FREQ_144MHz", // 旧简写
    ];
}

/// 当前 HAL **公共**类型（`src/rcc/mod.rs`），各 family 共用部分。
pub mod hal_common_today {
    //! ```text
    //! Clocks {
    //!     sysclk, hclk, pclk1, pclk2,
    //!     pclk1_tim, pclk2_tim,  // pub(crate)，定时器内核时钟
    //! }
    //! HSI_FREQ, LSI_FREQ, Hse { freq, mode }, LsConfig { … }
    //! init(Config), refresh(Option<Hertz>)   // uninit 尚未实现
    //! ```
    pub const NOTE: &str = "H4 另有 CoreClocks { sysclk, hclk, v5f, v3f } + core_clocks()";
}

/// 各 family 的 **Config 字段 ≈ 时钟树**（现状摘要，2026-03）。
pub mod families {
    //! `#[cfg]` → `rcc_impl` 路径见 `mod.rs`。

    pub mod v003 {
        //! **芯片**：ch32v003 等 | **HSI** 24 MHz | **PLL** 固定 ×2
        //!
        //! ```text
        //! Config {
        //!     hse: Option<Hse>,
        //!     sys: Sw,              // HSI / HSE / PLL
        //!     pll_src: Pllsrc,      // HSI / HSE（PLL 时）
        //!     ahb_pre: Hpre,
        //!     apb2_pre: Ppre,       // 主要影响 ADC 分频域
        //! }
        //! ```
        //!
        //! **树**：`HSI/HSE → [PLL×2] → SYSCLK → HPRE → HCLK`；APB2 单独 PPRE2。
        //!
        //! **HAL presets**（`impl Config`）：
        //! - `SYSCLK_FREQ_48MHZ_HSI`
        //! - `SYSCLK_FREQ_48MHZ_HSE`
        //! - `SYSCLK_FREQ_24MHZ_HSE`
        //!
        //! **CSDK 类比**：`SYSCLK_FREQ_48MHz` + `SetSysClockTo48()` 类。
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_48MHZ_HSI",
            "SYSCLK_FREQ_48MHZ_HSE",
            "SYSCLK_FREQ_24MHZ_HSE",
        ];
    }

    pub mod v00x {
        //! **芯片**：ch32v00x / ch32m0（非 v003）| **HSI** 24 MHz | **PLL** ×2
        //!
        //! ```text
        //! Config { hse, sys, pll_src, hb_pre: Hpre }   // 无独立 APB 字段
        //! ```
        //!
        //! **树**：与 v003 类似，更简；PCLK1/2 在 init 里均等于 HCLK。
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_24MHZ_HSI",
            "SYSCLK_FREQ_48MHZ_HSI",
            "SYSCLK_FREQ_24MHZ_HSE",
            "SYSCLK_FREQ_48MHZ_HSE",
        ];
    }

    pub mod v1 {
        //! **芯片**：ch32v1 / ch32l1 | **HSI** 8 MHz
        //!
        //! ```text
        //! Config {
        //!     hse, sys: Sw,
        //!     pll_src, pll: Option<Pll { prediv, mul }>,
        //!     ahb_pre, apb1_pre, apb2_pre,
        //! }
        //! ```
        //!
        //! **树**：`HSI/HSE → [÷prediv] → PLL VCO → SYSCLK → HPRE → HCLK → PPRE1/2 → PCLK`
        //! 定时器：APB 分频时 PCLK×2（STM32 规则）。
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_48MHZ_HSE",
            "SYSCLK_FREQ_72MHZ_HSE",
            "SYSCLK_FREQ_96MHZ_HSE",
            "SYSCLK_FREQ_48MHZ_HSI",
            "SYSCLK_FREQ_72MHZ_HSI",
            "SYSCLK_FREQ_96MHZ_HSI",
        ];
    }

    pub mod v3 {
        //! **芯片**：ch32v2 / v3 / f2 | **HSI** 8 MHz | 可选 USBHS PLL（d8c）
        //!
        //! ```text
        //! Config {
        //!     hse, sys: Sw,
        //!     pll_src: PllSource,     // HSI / HSE / [PLL2]
        //!     pll: Option<Pll { prediv, mul }>,
        //!     pllx: Option<Pllx>,     // 多 PLL，多为 todo
        //!     ahb_pre, apb1_pre, apb2_pre,
        //!     ls: LsConfig,
        //!     hspll_src, hspll,      // USBHS 相关
        //! }
        //! ```
        //!
        //! **树**：最接近「完整时钟树 Config」的现有实现；与 CSDK `SetSysClockTo144_HSE` 等一一可对应。
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_96MHZ_HSE",
            "SYSCLK_FREQ_144MHZ_HSE",
            "SYSCLK_FREQ_144MHZ_HSI",
            "SYSCLK_FREQ_96MHZ_HSI",
        ];
    }

    pub mod x0 {
        //! **芯片**：ch32x0 / ch643 | **HSI** 48 MHz | **无 PLL 选择**（仅 HSI）
        //!
        //! ```text
        //! Config { ahb_pre: Hpre }   // SYSCLK 恒为 48M，只分频 HCLK
        //! ```
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_48MHZ_HSI",
            "SYSCLK_FREQ_24MHZ_HSI",
            "SYSCLK_FREQ_16MHZ_HSI",
            "SYSCLK_FREQ_12MHZ_HSI",
        ];
    }

    pub mod ch641 {
        //! **芯片**：ch641 | **HSI** 24 MHz | **PLL** 固定 HSI×2
        //!
        //! ```text
        //! Config { sys: Sw, ahb_pre, apb2_pre }
        //! ```
        pub const PRESETS: &[&str] = &[
            "SYSCLK_FREQ_48MHZ_HSI",
            "SYSCLK_FREQ_24MHZ_HSI",
        ];
    }

    pub mod h4 {
        //! **芯片**：ch32h417 | **HSI** 25 MHz | 多 PLL + `SYSPLL_SEL` + **HPRE/FPRE 双核语义**
        //!
        //! **API（preset）**：
        //!
        //! ```text
        //! Config::with_sysclk_400m_v5f_400m_v3f_100m_hse()
        //! Config::with_sysclk_480m_v5f_240m_v3f_120m_hsi()
        //! Config::with_hsi()
        //!   .with_hse(hse)
        //!   .with_ls(ls)
        //! ```
        //!
        //! **树（RM / CSDK）** 应更接近：
        //!
        //! ```text
        //! Osc: HSI | HSE
        //!   → Main PLL | USBHS PLL | ETH PLL | …  (SYSPLL_SEL)
        //!   → SYSCLK
        //!   → HPRE → V5F / AHB 域
        //!   → FPRE → HCLK / V3F
        //!   → PPRE1/2 → PCLK + 外设 kernel (CFGR2)
        //! ```
        //!
        pub const PRESET_TO_C: &[(&str, &str)] = &[
            ("Config::with_hsi()", "(none, POR HSI 25M)"),
            ("Config::with_sysclk_400m_v5f_400m_v3f_100m_hse()", "SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE"),
            ("Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi()", "SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI"),
            ("Config::with_sysclk_480m_v5f_240m_v3f_120m_hse()", "SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE"),
            ("Config::with_sysclk_480m_v5f_240m_v3f_120m_hsi()", "SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI"),
            ("Config::with_sysclk_480m_v5f_480m_v3f_120m_hse()", "SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE"),
            ("Config::with_sysclk_480m_v5f_480m_v3f_120m_hsi()", "SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI"),
        ];
    }
}

/// 从 C 宏名到 **建议** Rust preset 函数名（草案，未实现）。
pub mod preset_naming_draft {
    //! H417：保留 C 里的数字语义，用 `with_` 前缀 + snake_case：
    //!
    //! | C macro | 草案 Rust |
    //! |---------|-----------|
    //! | `SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE` | `Config::with_400m_v5f_400_v3f_100m_hse()` |
    //! | `SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE` | `Config::with_480m_v5f_240_v3f_120m_hse()` |
    //!
    //! V307：
    //!
    //! | C macro | 草案 Rust |
    //! |---------|-----------|
    //! | `SYSCLK_FREQ_144MHz_HSE` | `Config::with_144mhz_hse()` 或保留 `SYSCLK_FREQ_144MHZ_HSE` const |
    pub const H417_EXAMPLE: &str = "Config::with_480m_v5f_240_v3f_120m_hse()";
    pub const V30_EXAMPLE: &str = "Config::with_144mhz_hse()";
}

/// `cfg` → family 路由（与 `mod.rs` 一致）。
pub const FAMILY_ROUTING: &[(&str, &str)] = &[
    ("ch32v003", "v003.rs"),
    ("ch32v0/ch32m0 (not v003)", "v00x.rs"),
    ("ch32v1 / ch32l1", "v1.rs"),
    ("ch32v2 / ch32v3 / ch32f2", "v3.rs"),
    ("ch32x0 / ch643", "x0.rs"),
    ("ch641", "ch641.rs"),
    ("ch32h4 (rcc_h4)", "h4/mod.rs"),
];
