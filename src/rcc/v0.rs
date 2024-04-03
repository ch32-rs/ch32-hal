use crate::pac::rcc::vals::{Hpre as AHBPrescaler, Pllsrc as PllSource, Ppre as APBPrescaler, Sw as Sysclk};
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

pub struct Config {
    // won't close hsi
    // pub hsi: bool,
    pub hse: Option<Hse>,
    pub sys: Sysclk,

    pub pll_src: PllSource,

    pub ahb_pre: AHBPrescaler,
    // ADCPRE is actually splitted from APB2
    pub apb2_pre: APBPrescaler,
}

impl Config {
    pub const SYSCLK_FREQ_48MHZ_HSI: Config = Config {
        hse: None,
        sys: Sysclk::PLL,
        pll_src: PllSource::HSI_MUL2,
        ahb_pre: AHBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };

    pub const SYSCLK_FREQ_48MHZ_HSE: Config = Config {
        hse: Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Oscillator,
        }),
        sys: Sysclk::PLL,
        pll_src: PllSource::HSE_MUL2,
        ahb_pre: AHBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // hsi: true,
            hse: None,
            sys: Sysclk::HSI,
            pll_src: PllSource::HSE_MUL2,
            ahb_pre: AHBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
        }
    }
}

#[allow(unused_variables)]
pub(crate) unsafe fn init(config: Config) {
    if config.sys == Sysclk::HSE || (config.sys == Sysclk::PLL && config.pll_src == PllSource::HSE_MUL2) {
        // enable HSI
        RCC.ctlr().modify(|w| {
            w.set_hsebyp(config.hse.unwrap().mode == HseMode::Bypass);
            w.set_hseon(true);
        });
        while !RCC.ctlr().read().hserdy() {}
        RCC.cfgr0().modify(|w| w.set_sw(Sysclk::HSE));
    }

    let mut sysclk = 24_000_000;
    if config.sys == Sysclk::HSI {
        // enable HSI
        // default enabled
        sysclk = HSI_FREQUENCY.0;
    } else if config.sys == Sysclk::HSE {
        // enable HSE
        RCC.cfgr0().modify(|w| w.set_sw(Sysclk::HSE));
        while RCC.cfgr0().read().sws() != Sysclk::HSE {}
        sysclk = config.hse.unwrap().freq.0;
    } else if config.sys == Sysclk::PLL {
        RCC.cfgr0().modify(|w| w.set_pllsrc(config.pll_src));
        RCC.ctlr().modify(|w| w.set_pllon(true));
        while !RCC.ctlr().read().pllrdy() {}

        RCC.cfgr0().modify(|w| w.set_sw(Sysclk::PLL));
        while RCC.cfgr0().read().sws() != Sysclk::PLL {}

        match config.pll_src {
            PllSource::HSI_MUL2 => sysclk = HSI_FREQUENCY.0 * 2,
            PllSource::HSE_MUL2 => sysclk = config.hse.unwrap().freq.0 * 2,
            _ => unreachable!(),
        };
    }

    // TODO: handle AHBPrescaler and APBPrescaler

    super::CLOCKS.sysclk = Hertz(sysclk);
    super::CLOCKS.hclk = Hertz(sysclk);
    super::CLOCKS.pclk1 = Hertz(sysclk);
    super::CLOCKS.pclk2 = Hertz(sysclk);
}
