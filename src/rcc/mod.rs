//! Reset and clock control (RCC).
//!
//! # API shape (all chip families)
//!
//! | Item | Role |
//! |------|------|
//! | [`Config`] | Family-specific clock setup (presets + optional fields) |
//! | [`init`] | Apply `Config` to hardware and refresh the global cache |
//! | [`refresh`] | Read RCC registers and refresh the cache (bootloader / dual-stage init) |
//! | [`clocks`] | Cached AHB/APB frequencies for drivers |
//! | [`HSI_FREQ`] / [`LSI_FREQ`] | Nominal RC oscillator rates |
//! | [`Hse`] | External clock description when HSE is used |
//!
//! On CH32H4, [`clocks`] additionally reports the V3F/V5F core frequencies
//! ([`Clocks::v3f`], [`Clocks::v5f`]); there is no separate `core_clocks()`.
//!
//! Pass the board HSE crystal frequency to [`refresh`] whenever SYSCLK can be sourced from HSE
//! or PLL fed by HSE (same requirement as `HSE_VALUE` in the WCH C SDK).

use crate::time::Hertz;

mod shared;

/// 时钟树与 CSDK 命名对照（阅读用，见文件内文档）。
#[doc(hidden)]
pub mod clock_tree_reference;

#[cfg(ch32v003)]
#[path = "v003.rs"]
mod rcc_impl;

#[cfg(all(any(ch32v0, ch32m0), not(ch32v003)))]
#[path = "v00x.rs"]
mod rcc_impl;

#[cfg(any(ch32v1, ch32l1))]
#[path = "v1.rs"]
mod rcc_impl;

#[cfg(any(ch32v2, ch32v3, ch32f2))]
#[path = "v3.rs"]
mod rcc_impl;

#[cfg(any(ch32x0, ch643))]
#[path = "x0.rs"]
mod rcc_impl;

#[cfg(ch641)]
#[path = "ch641.rs"]
mod rcc_impl;

#[cfg(rcc_h4)]
#[path = "h4/mod.rs"]
mod rcc_impl;

pub use rcc_impl::*;

/// Nominal HSI frequency for the selected chip.
pub const HSI_FREQ: Hertz = HSI_FREQUENCY;

#[cfg(rcc_h4)]
const DEFAULT_FREQUENCY: Hertz = Hertz(25_000_000);
#[cfg(all(any(ch32x0, ch643), not(rcc_h4)))]
const DEFAULT_FREQUENCY: Hertz = Hertz(48_000_000);
#[cfg(all(
    not(rcc_h4),
    not(any(ch32x0, ch643)),
    any(ch32v003, ch32v0, ch32m0, ch641)
))]
const DEFAULT_FREQUENCY: Hertz = Hertz(24_000_000);
#[cfg(all(
    not(rcc_h4),
    not(any(ch32x0, ch643, ch32v003, ch32v0, ch32m0, ch641))
))]
const DEFAULT_FREQUENCY: Hertz = Hertz(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
    pclk1: DEFAULT_FREQUENCY,
    pclk2: DEFAULT_FREQUENCY,
    pclk1_tim: DEFAULT_FREQUENCY,
    pclk2_tim: DEFAULT_FREQUENCY,
    #[cfg(rcc_h4)]
    v3f: DEFAULT_FREQUENCY,
    #[cfg(rcc_h4)]
    v5f: DEFAULT_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    /// AHB clock (on H4: V3F core / HCLK domain after `FPRE`).
    pub hclk: Hertz,
    pub pclk1: Hertz,
    pub pclk2: Hertz,
    pub(crate) pclk1_tim: Hertz,
    pub(crate) pclk2_tim: Hertz,
    /// V3F (boot core, hart 0) core frequency. H4 only.
    #[cfg(rcc_h4)]
    pub v3f: Hertz,
    /// V5F (second core, hart 1) core frequency. H4 only.
    #[cfg(rcc_h4)]
    pub v5f: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

/// External high-speed clock (HSE).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Hse {
    pub freq: Hertz,
    pub mode: HseMode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HseMode {
    Oscillator,
    Bypass,
}

#[cfg(not(ch32v208))]
pub const LSI_FREQ: Hertz = Hertz(40_000);
#[cfg(ch32v208)]
pub const LSI_FREQ: Hertz = Hertz(32_768);

#[allow(dead_code)]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LseMode {
    Oscillator,
    Bypass,
}

pub struct LseConfig {
    pub frequency: Hertz,
    pub mode: LseMode,
}

pub enum RtcClockSource {
    LSE,
    LSI,
    HSE,
    DISABLE,
}

pub struct LsConfig {
    pub rtc: RtcClockSource,
    pub lsi: bool,
    pub lse: Option<LseConfig>,
}

impl LsConfig {
    pub const fn default_lse() -> Self {
        Self {
            rtc: RtcClockSource::LSE,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator,
            }),
            lsi: false,
        }
    }

    pub const fn default_lsi() -> Self {
        Self {
            rtc: RtcClockSource::LSI,
            lsi: true,
            lse: None,
        }
    }

    pub const fn off() -> Self {
        Self {
            rtc: RtcClockSource::DISABLE,
            lsi: false,
            lse: None,
        }
    }
}

impl Default for LsConfig {
    fn default() -> Self {
        Self::default_lsi()
    }
}

#[allow(unused)]
impl LsConfig {
    pub(crate) fn init(&self) -> Option<Hertz> {
        todo!()
    }
}

pub unsafe fn init(config: Config) {
    rcc_impl::init(config);
}

/// Re-measure RCC and update [`clocks`] (on H4 that includes `v3f` / `v5f`).
///
/// Use after C `SystemInit`, a bootloader, or the other core has configured clocks.
pub unsafe fn refresh(hse: Option<Hertz>) {
    rcc_impl::refresh_clocks(hse);
}

pub(crate) use shared::{apb_timer_clk, set_clocks};
