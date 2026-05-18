//! CH32H4 RCC — minimal port of the EVT SDK clock recipes.
//!
//! WCH's CH32H4 SDK ships clock setup as a fixed set of `#ifdef SYSCLK_*`
//! recipes in `system_ch32h417.c::SetSysClock`. We mirror them as enum
//! variants + dispatch. The user picks one — every prescaler, multiplier,
//! Flash wait-state and PLL selector along the path is baked into the
//! recipe.
//!
//! ## What is NOT here
//!
//! - USBSS / ETH / SERDES PLLs — these are entirely driver-owned. The USB
//!   3.0 driver enables USBSS PLL when it runs, the Ethernet driver
//!   enables ETH PLL when it runs, etc. RCC `init()` only touches USBHS
//!   PLL, and only when the chosen sysclk recipe routes through it
//!   (`Pll480M*` variants).
//! - Per-pin kernel-clock muxes (CFGR2 fields like LTDC / SAI / RNG /
//!   HSADC source select). Same story — handled by the relevant driver.
//! - Arbitrary user PLL multipliers / prescalers. CH32H4's secondary PLLs
//!   have fixed output frequencies (USBHS 480M, USBSS 125M, ETH 500M,
//!   SERDES 500M) — exposing freeform configuration would let users
//!   produce combinations the silicon never validated.
//!
//! Add new recipes here as the project's needs grow (480M overdrive, lower
//! frequencies, etc.).

use crate::pac::rcc::vals::{
    Fpre, Hpre, Pllmul, Pllsrc, Sw as Sysclk, SyspllSel, UsbhspllRefsel, Usbhspllsrc,
};
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

/// Internal RC oscillator. Unlike most other ch32 families (8 MHz),
/// CH32H4's HSI runs at 25 MHz — matching the EVT board's HSE crystal —
/// which is why the HSE/HSI variants of the SDK recipes share identical
/// PLL multiplier values.
const HSI_FREQUENCY: Hertz = Hertz(25_000_000);

/// Target system clock recipe. Mirrors EVT SDK's `SetSYSCLK_*` functions
/// in `system_ch32h417.c`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SysClk {
    /// Power-on default: HSI 25 MHz, all PLLs off, no Flash wait state.
    Hsi,
    /// SYSCLK 400 MHz via main PLL fed by HSE (25 MHz × 16). V5F=400, V3F=100, HCLK=400.
    Pll400MHse,
    /// SYSCLK 400 MHz via main PLL fed by HSI (25 MHz × 16). V5F=400, V3F=100, HCLK=400.
    Pll400MHsi,
    /// SYSCLK 480 MHz via USBHS PLL (fixed 480M output), USBHS PLL fed by HSE.
    /// V5F=240, V3F=120, HCLK=240.
    Pll480MHse,
    /// SYSCLK 480 MHz via USBHS PLL (fixed 480M output), USBHS PLL fed by HSI.
    /// V5F=240, V3F=120, HCLK=240.
    Pll480MHsi,
}

pub struct Config {
    pub sysclk: SysClk,
    pub ls: super::LsConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            sysclk: SysClk::Hsi,
            ls: super::LsConfig::default(),
        }
    }
}

pub unsafe fn init(config: Config) {
    let (sysclk_hz, hpre, _fpre) = match config.sysclk {
        SysClk::Hsi => (HSI_FREQUENCY, 1, 1),
        SysClk::Pll400MHse => {
            init_400m(Pllsrc::HSE);
            (Hertz(400_000_000), 1, 4)
        }
        SysClk::Pll400MHsi => {
            init_400m(Pllsrc::HSI);
            (Hertz(400_000_000), 1, 4)
        }
        SysClk::Pll480MHse => {
            init_480m(Usbhspllsrc::HSE);
            (Hertz(480_000_000), 2, 2)
        }
        SysClk::Pll480MHsi => {
            init_480m(Usbhspllsrc::HSI);
            (Hertz(480_000_000), 2, 2)
        }
    };

    // APB prescalers stay at their POR default (DIV1) — every SDK recipe
    // does the same. PCLKx_TIM doubling logic isn't applied either; H4 RM
    // table 4.2 confirms TIM kernel clock follows PCLKx directly when
    // PPRE=DIV1.
    let hclk = Hertz(sysclk_hz.0 / hpre as u32);
    super::CLOCKS.sysclk = sysclk_hz;
    super::CLOCKS.hclk = hclk;
    super::CLOCKS.pclk1 = hclk;
    super::CLOCKS.pclk2 = hclk;
    super::CLOCKS.pclk1_tim = hclk;
    super::CLOCKS.pclk2_tim = hclk;
}

// SDK port: SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE/HSI
//
// Drives SYSCLK from the main PLL (×16, /1 prediv). Source is HSE or HSI.
// HCLK = SYSCLK / 1 = 400 MHz, V3F core = SYSCLK / 4 = 100 MHz.
unsafe fn init_400m(pll_src: Pllsrc) {
    if pll_src == Pllsrc::HSE {
        enable_hse();
    }

    RCC.pllcfgr().modify(|w| {
        w.set_pllmul(Pllmul::MUL16);
        w.set_pll_src_div(0); // PLL_SRC_DIV = N-1; 0 means /1
        w.set_pllsrc(pll_src);
    });

    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    // SYSPLL_SEL = PLL_CLK (main PLL output) via the 3-step
    // gate-down → write SEL → gate-up handshake. Gate-up is done implicitly
    // by `set_sw(Sysclk::PLL)` below.
    RCC.pllcfgr().modify(|w| {
        w.set_syspll_gate(false);
        w.set_syspll_sel(SyspllSel::PLL_CLK);
    });

    RCC.cfgr0().modify(|w| {
        w.set_hpre(Hpre::DIV1);
        w.set_fpre(Fpre::DIV4);
    });

    set_flash_latency_2();
    switch_sysclk_to_pll();
}

// SDK port: SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE/HSI
//
// Drives SYSCLK from the USBHS PLL (fixed 480 MHz output). Both HSE and
// HSI variants land at REFSEL=25 MHz directly because both oscillators
// run at 25 MHz on CH32H4. HCLK = SYSCLK / 2 = 240 MHz, V3F core =
// SYSCLK / 2 = 240 MHz / 2 = 120 MHz.
unsafe fn init_480m(usbhs_src: Usbhspllsrc) {
    if usbhs_src == Usbhspllsrc::HSE {
        enable_hse();
    }

    RCC.pllcfgr2().modify(|w| {
        w.set_usbhspll_refsel(UsbhspllRefsel::F25MHZ);
        w.set_usbhspllsrc(usbhs_src);
    });
    RCC.ctlr().modify(|w| w.set_usbhs_pllon(true));
    while !RCC.ctlr().read().usbhs_pllrdy() {}

    RCC.pllcfgr().modify(|w| {
        w.set_syspll_gate(false);
        w.set_syspll_sel(SyspllSel::USBHS_PLL);
    });

    RCC.cfgr0().modify(|w| {
        w.set_hpre(Hpre::DIV2);
        w.set_fpre(Fpre::DIV2);
    });

    set_flash_latency_2();
    switch_sysclk_to_pll();
}

// Shared helpers ----------------------------------------------------------

unsafe fn enable_hse() {
    RCC.ctlr().modify(|w| w.set_hseon(true));
    while !RCC.ctlr().read().hserdy() {}
}

/// FLASH.ACTLR.SCK_CFG = 2 (HCLK/2). The EVT SDK hardcodes this for every
/// 400 MHz / 480 MHz recipe; it gives Flash 2 wait cycles which is the
/// minimum the silicon supports at HCLK ≥ 100 MHz.
unsafe fn set_flash_latency_2() {
    FLASH.actlr().modify(|w| w.set_sck_cfg(2));
}

/// Re-open the SYSPLL_SEL gate and route SYSCLK from PLL. The 3-step
/// gate-down → SEL → gate-up handshake is required when changing
/// SYSPLL_SEL; this is the closing half.
unsafe fn switch_sysclk_to_pll() {
    RCC.pllcfgr().modify(|w| w.set_syspll_gate(true));
    RCC.cfgr0().modify(|w| w.set_sw(Sysclk::PLL));
    while RCC.cfgr0().read().sws() != Sysclk::PLL {}
}
