//! CH32H4 RCC — full clock tree (RM §3.4, EVT `system_ch32h417.c`).
//!
//! EVT presets: [`Config::with_sysclk_*`] builders (names mirror C `SYSCLK_*` macro segments).
//!
//! ```ignore
//! hal::init(hal::Config {
//!     rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
//!     ..Default::default()
//! });
//! ```

mod ls;
mod measure;
mod mux;
mod pll;
mod prescale;

pub use mux::KernelMux;
pub use pll::{AuxPlls, UsbhsPll, UsbssPll};

use crate::pac::rcc::vals::{
    Adcpre, Fpre, Hpre, Mco, Pllmul, Pllsrc, Ppre, Sw as SysclkSw, SyspllSel, UsbhspllRefsel,
    Usbhspllsrc,
};
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

use prescale::{ahb_from_sysclk, hclk_from_ahb};

/// HSI frequency (25 MHz on CH32H4).
pub const HSI_FREQUENCY: Hertz = Hertz(25_000_000);

/// LSI frequency (~40 kHz, DS table 3-14).
pub const LSI_FREQUENCY: Hertz = Hertz(40_000);

/// Typical nanoCH32H417 HSE crystal.
pub const HSE_FREQUENCY_25M: Hertz = Hertz(25_000_000);

/// AHB / APB / V5F / ADC prescalers applied to the selected SYSCLK.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct BusPrescalers {
    pub hpre: Hpre,
    pub ppre1: Ppre,
    pub ppre2: Ppre,
    pub fpre: Fpre,
    pub adcpre: Adcpre,
}

impl Default for BusPrescalers {
    fn default() -> Self {
        Self {
            hpre: Hpre::DIV1,
            ppre1: Ppre::DIV1,
            ppre2: Ppre::DIV1,
            fpre: Fpre::DIV1,
            adcpre: Adcpre::DIV2,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct McoConfig {
    pub source: Mco,
}

/// WCH EVT-validated clock recipe (internal; use [`Config::with_*`] presets).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Recipe {
    /// Power-on style: HSI 25 MHz.
    Hsi,
    /// Main PLL ×16 → 400 MHz SYSCLK.
    M400V5f400V3f100 { osc: Oscillator },
    /// USBHS PLL 480 MHz, V5F=240 MHz, V3F=120 MHz.
    M480V5f240V3f120 { osc: Oscillator },
    /// USBHS PLL 480 MHz, V5F=480 MHz, V3F=120 MHz (EVT also sets VDDK 1.25 V).
    M480V5f480V3f120 { osc: Oscillator },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Oscillator {
    Hsi,
    Hse,
}

pub struct Config {
    recipe: Recipe,
    /// Board HSE; `None` uses [`HSE_FREQUENCY_25M`] on HSE recipes.
    pub hse: Option<super::Hse>,
    pub bus: BusPrescalers,
    pub mux: KernelMux,
    pub aux_plls: AuxPlls,
    pub ls: super::LsConfig,
    pub mco: Option<McoConfig>,
}

impl Default for Config {
    fn default() -> Self {
        Self::with_hsi()
    }
}

impl Config {
    /// HSI 25 MHz (no PLL). Matches EVT when no `SYSCLK_*` macro is defined.
    pub fn with_hsi() -> Self {
        Self::base(Recipe::Hsi)
    }

    /// `SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE`
    pub fn with_sysclk_400m_v5f_400m_v3f_100m_hse() -> Self {
        Self::base(Recipe::M400V5f400V3f100 { osc: Oscillator::Hse })
    }

    /// `SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI`
    pub fn with_sysclk_400m_v5f_400m_v3f_100m_hsi() -> Self {
        Self::base(Recipe::M400V5f400V3f100 { osc: Oscillator::Hsi })
    }

    /// `SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE`
    pub fn with_sysclk_480m_v5f_240m_v3f_120m_hse() -> Self {
        Self::base(Recipe::M480V5f240V3f120 { osc: Oscillator::Hse })
    }

    /// `SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI`
    pub fn with_sysclk_480m_v5f_240m_v3f_120m_hsi() -> Self {
        Self::base(Recipe::M480V5f240V3f120 { osc: Oscillator::Hsi })
    }

    /// `SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE`
    pub fn with_sysclk_480m_v5f_480m_v3f_120m_hse() -> Self {
        Self::base(Recipe::M480V5f480V3f120 { osc: Oscillator::Hse })
    }

    /// `SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI`
    pub fn with_sysclk_480m_v5f_480m_v3f_120m_hsi() -> Self {
        Self::base(Recipe::M480V5f480V3f120 { osc: Oscillator::Hsi })
    }

    /// Override the external crystal description (frequency / bypass).
    pub fn with_hse(mut self, hse: super::Hse) -> Self {
        self.hse = Some(hse);
        self
    }

    /// Low-speed / RTC configuration.
    pub fn with_ls(mut self, ls: super::LsConfig) -> Self {
        self.ls = ls;
        self
    }

    fn base(recipe: Recipe) -> Self {
        Self {
            recipe,
            hse: None,
            bus: BusPrescalers::default(),
            mux: KernelMux::default(),
            aux_plls: AuxPlls::default(),
            ls: super::LsConfig::default(),
            mco: None,
        }
    }
}

pub unsafe fn init(config: Config) {
    while !RCC.ctlr().read().hsirdy() {}

    let _rtc = ls::init_ls(&config.ls);
    mux::apply_mux(&config.mux);

    let (sysclk_hz, bus, _v3f_hz, use_pll_sw) = recipe_params(&config);

    RCC.cfgr0().modify(|w| {
        w.set_hpre(bus.hpre);
        w.set_ppre1(bus.ppre1);
        w.set_ppre2(bus.ppre2);
        w.set_fpre(bus.fpre);
        w.set_adcpre(bus.adcpre);
    });

    if use_pll_sw {
        match config.recipe {
            Recipe::M400V5f400V3f100 { osc } => {
                let src = match osc {
                    Oscillator::Hse => Pllsrc::HSE,
                    Oscillator::Hsi => Pllsrc::HSI,
                };
                init_400m(src, config.hse);
            }
            Recipe::M480V5f240V3f120 { osc } | Recipe::M480V5f480V3f120 { osc } => {
                let src = match osc {
                    Oscillator::Hse => Usbhspllsrc::HSE,
                    Oscillator::Hsi => Usbhspllsrc::HSI,
                };
                init_480m(src, config.hse);
            }
            Recipe::Hsi => unreachable!(),
        }
    } else if let Some(hse) = config.hse {
        enable_hse(hse);
        RCC.cfgr0().modify(|w| w.set_sw(SysclkSw::HSE));
        while RCC.cfgr0().read().sws() != SysclkSw::HSE {}
    } else {
        RCC.cfgr0().modify(|w| w.set_sw(SysclkSw::HSI));
        while RCC.cfgr0().read().sws() != SysclkSw::HSI {}
    }

    if let Some(mco) = config.mco {
        RCC.cfgr0().modify(|w| w.set_mco(mco.source));
    }

    pll::apply_aux(&config.aux_plls);

    if !use_pll_sw {
        let ahb = ahb_from_sysclk(sysclk_hz, bus.hpre);
        if hclk_from_ahb(ahb, bus.fpre).0 > 100_000_000 {
            set_flash_latency_2();
        }
    }

    // HSE recipes use the default crystal when no override was supplied.
    // Measure with the same frequency that was used to configure the PLL.
    let hse_hz = config.hse.map(|h| h.freq).or_else(|| match config.recipe {
        Recipe::M400V5f400V3f100 { osc: Oscillator::Hse }
        | Recipe::M480V5f240V3f120 { osc: Oscillator::Hse }
        | Recipe::M480V5f480V3f120 { osc: Oscillator::Hse } => Some(HSE_FREQUENCY_25M),
        _ => None,
    });
    refresh_clocks(hse_hz);
}

/// Re-read RCC and update [`super::clocks`].
pub(crate) unsafe fn refresh_clocks(hse: Option<Hertz>) {
    let clocks = measure::measure(hse);
    super::set_clocks(clocks);
}

fn recipe_params(config: &Config) -> (Hertz, BusPrescalers, Hertz, bool) {
    match config.recipe {
        Recipe::Hsi => (HSI_FREQUENCY, config.bus, HSI_FREQUENCY, false),
        Recipe::M400V5f400V3f100 { .. } => (
            Hertz(400_000_000),
            BusPrescalers {
                hpre: Hpre::DIV1,
                ppre1: Ppre::DIV1,
                ppre2: Ppre::DIV1,
                fpre: Fpre::DIV4,
                adcpre: config.bus.adcpre,
            },
            Hertz(100_000_000),
            true,
        ),
        Recipe::M480V5f240V3f120 { .. } => bus_480(Bus480::V5f240, config),
        Recipe::M480V5f480V3f120 { .. } => bus_480(Bus480::V5f480, config),
    }
}

enum Bus480 {
    V5f240,
    V5f480,
}

fn bus_480(kind: Bus480, config: &Config) -> (Hertz, BusPrescalers, Hertz, bool) {
    let (hpre, fpre, v3f) = match kind {
        Bus480::V5f240 => (Hpre::DIV2, Fpre::DIV2, Hertz(120_000_000)),
        Bus480::V5f480 => (Hpre::DIV1, Fpre::DIV4, Hertz(120_000_000)),
    };
    (
        Hertz(480_000_000),
        BusPrescalers {
            hpre,
            ppre1: Ppre::DIV1,
            ppre2: Ppre::DIV1,
            fpre,
            adcpre: config.bus.adcpre,
        },
        v3f,
        true,
    )
}

unsafe fn enable_hse(hse: super::Hse) {
    RCC.ctlr().modify(|w| w.set_hsebyp(hse.mode == super::HseMode::Bypass));
    RCC.ctlr().modify(|w| w.set_hseon(true));
    while !RCC.ctlr().read().hserdy() {}
}

unsafe fn init_400m(pll_src: Pllsrc, hse: Option<super::Hse>) {
    if pll_src == Pllsrc::HSE {
        enable_hse(hse.unwrap_or(default_hse()));
    }

    RCC.pllcfgr().modify(|w| {
        w.set_pllmul(Pllmul::MUL16);
        w.set_pll_src_div(0);
        w.set_pllsrc(pll_src);
    });
    RCC.ctlr().modify(|w| w.set_pllon(true));
    while !RCC.ctlr().read().pllrdy() {}

    RCC.pllcfgr().modify(|w| {
        w.set_syspll_gate(false);
        w.set_syspll_sel(SyspllSel::PLL_CLK);
    });

    set_flash_latency_2();
    switch_sysclk_to_pll();
}

fn default_hse() -> super::Hse {
    super::Hse {
        freq: HSE_FREQUENCY_25M,
        mode: super::HseMode::Oscillator,
    }
}

unsafe fn init_480m(usbhs_src: Usbhspllsrc, hse: Option<super::Hse>) {
    if usbhs_src == Usbhspllsrc::HSE {
        enable_hse(hse.unwrap_or(default_hse()));
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

    set_flash_latency_2();
    switch_sysclk_to_pll();
}

unsafe fn set_flash_latency_2() {
    FLASH.actlr().modify(|w| w.set_sck_cfg(2));
}

unsafe fn switch_sysclk_to_pll() {
    RCC.pllcfgr().modify(|w| w.set_syspll_gate(true));
    RCC.cfgr0().modify(|w| w.set_sw(SysclkSw::PLL));
    while RCC.cfgr0().read().sws() != SysclkSw::PLL {}
}

pub use pll::{enable_eth_pll, enable_serdes_pll, enable_usbhs_pll, enable_usbss_pll};
