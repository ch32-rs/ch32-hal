use crate::time::Hertz;

const DEFAULT_FREQUENCY: Hertz = Hertz(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
    pclk1: DEFAULT_FREQUENCY,
    pclk2: DEFAULT_FREQUENCY,

    pclk1_tim: DEFAULT_FREQUENCY,
    pclk2_tim: DEFAULT_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    /// AHB clock
    pub hclk: Hertz,
    /// APB1 clock
    pub pclk1: Hertz,
    /// APB2 clock
    pub pclk2: Hertz,

    pub(crate) pclk1_tim: Hertz,
    pub(crate) pclk2_tim: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

#[cfg(ch32v0)]
#[path = "v0.rs"]
mod rcc_impl;

#[cfg(any(ch32v1, ch32l1))]
#[path = "v1.rs"]
mod rcc_impl;

#[cfg(any(ch32v2, ch32v3, ch32f2))]
#[path = "v3.rs"]
mod rcc_impl;

#[cfg(ch32x0)]
#[path = "x0.rs"]
mod rcc_impl;

#[cfg(ch641)]
#[path = "ch641.rs"]
mod rcc_impl;

pub use rcc_impl::*;

#[cfg(not(ch32v208))]
pub const LSI_FREQ: Hertz = Hertz(40_000);
#[cfg(ch32v208)]
pub const LSI_FREQ: Hertz = Hertz(32_768);

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum LseMode {
    Oscillator, // (LseDrive),
    Bypass,
}

pub struct LseConfig {
    pub frequency: Hertz,
    pub mode: LseMode,
}

pub enum RtcClockSource {
    LSE,
    LSI,
    // HSE divided by 128
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
                mode: LseMode::Oscillator, // (LseDrive::MediumHigh),
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

impl LsConfig {
    pub(crate) fn init(&self) -> Option<Hertz> {
        todo!()
    }
}

pub unsafe fn init(config: Config) {
    rcc_impl::init(config);
}
