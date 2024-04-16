use crate::time::Hertz;

// const HSI_FREQUENCY: Hertz = Hertz(48_000_000);

// const LSI_FREQUENCY: Hertz = Hertz(40_000);
// CH32FV208 use 32.768KHz LSI

// Power on default: HPRE = 0b0101 = Div6
const DEFAULT_FREQUENCY: Hertz = Hertz(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
    pclk1: DEFAULT_FREQUENCY,
    pclk2: DEFAULT_FREQUENCY,
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
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

#[cfg(ch32v0)]
pub mod v0;
#[cfg(ch32v1)]
pub mod v1;
#[cfg(any(ch32v2, ch32v3, ch32f2))]
pub mod v3;
#[cfg(ch32x0)]
pub mod x0;

#[cfg(ch32v0)]
pub use v0::Config;
#[cfg(ch32v1)]
pub use v1::Config;
#[cfg(any(ch32v2, ch32v3))]
pub use v3::Config;
#[cfg(ch32x0)]
pub use x0::Config;

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
    HSE_DIV_128,
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
    #[cfg(any(ch32v3, ch32v2))]
    v3::init(config);

    #[cfg(ch32v0)]
    v0::init(config);

    #[cfg(ch32x0)]
    x0::init(config);
}
