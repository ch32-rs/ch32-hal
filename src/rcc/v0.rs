use crate::pac::rcc::vals::{Hpre as AHBPrescaler, Ppre as APBPrescaler, Sw as Sysclk};
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

const HSI_FREQUENCY: Hertz = Hertz(24_000_000);

const LSI_FREQUENCY: Hertz = Hertz(128_000);

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum HseMode {
    /// crystal/ceramic oscillator (HSEBYP=0)
    Oscillator,
    /// external analog clock (low swing) (HSEBYP=1)
    Bypass,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Hse {
    /// HSE frequency.
    pub freq: Hertz,
    /// HSE mode.
    pub mode: HseMode,
}

// PLLSRC, the div ratio is set in Pll.prediv
#[derive(Clone, Copy, PartialEq)]
pub enum PllSource {
    HSI,
    HSE,
}

pub struct Config {
    // won't close hsi
    // pub hsi: bool,
    pub hse: Option<Hse>,
    pub sys: Sysclk,

    pub pll_src: PllSource,

    pub ahb_pre: AHBPrescaler,
    pub apb2_pre: APBPrescaler,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // hsi: true,
            hse: None,
            sys: Sysclk::HSI,
            pll_src: PllSource::HSI,
            ahb_pre: AHBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
        }
    }
}
