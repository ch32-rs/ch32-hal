//! CH32V305/307, CH32F205/207
use core::ops;

use crate::pac::rcc::vals::{
    Hpre as AHBPrescaler, PllMul, PllxMul, Ppre as APBPrescaler, Prediv as PllPreDiv, Sw as Sysclk,
};
use crate::pac::{EXTEND, FLASH, RCC};
use crate::time::Hertz;

const HSI_FREQUENCY: Hertz = Hertz(8_000_000);

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

#[derive(Clone, Copy)]
pub struct Pll {
    /// PLL pre-divider
    pub prediv: PllPreDiv,

    /// PLL multiplication factor.
    pub mul: PllMul,
}

// PLLSRC, the div ratio is set in Pll.prediv
#[derive(Clone, Copy, PartialEq)]
pub enum PllSource {
    /// HSI or HSI div 2
    HSI,
    // HSE or HSE div (2 or Prediv)
    HSE,

    // PLL2, PREDIV1SRC
    PLL2,
}

pub struct Pllx {
    pub prediv2: PllPreDiv,
    pub pll2: Option<PllxMul>,
    pub pll3: Option<PllxMul>,
}

pub struct Config {
    // won't close hsi
    // pub hsi: bool,
    pub hse: Option<Hse>,
    pub sys: Sysclk,

    pub pll_src: PllSource,
    pub pll: Option<Pll>,

    // TODO: optional
    pub pllx: Option<Pllx>,

    pub ahb_pre: AHBPrescaler,
    pub apb1_pre: APBPrescaler,
    pub apb2_pre: APBPrescaler,

    pub ls: super::LsConfig,
    // /// Per-peripheral kernel clock selection muxes
    // pub mux: super::mux::ClockMux,
}

impl Config {
    pub const SYSCLK_FREQ_96MHZ_HSE: Config = {
        Config {
            hse: Some(Hse {
                freq: Hertz(8_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL6,
            }),

            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: super::LsConfig::default_lsi(),
        }
    };
    pub const SYSCLK_FREQ_144MHZ_HSE: Config = {
        Config {
            hse: Some(Hse {
                freq: Hertz(8_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL18,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: super::LsConfig::default_lsi(),
        }
    };
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // hsi: true,
            hse: None,
            sys: Sysclk::HSI,
            pll_src: PllSource::HSI,
            pll: None,
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: super::LsConfig::default(),
        }
    }
}

pub(crate) unsafe fn init(config: Config) {
    // Configure HSI
    while !RCC.ctlr().read().hsirdy() {}
    let hsi = Some(HSI_FREQUENCY);

    // Configure HSE
    let hse = match config.hse {
        None => {
            RCC.ctlr().modify(|w| w.set_hseon(false));
            None
        }
        Some(hse) => {
            RCC.ctlr().modify(|w| w.set_hsebyp(hse.mode != HseMode::Oscillator));
            RCC.ctlr().modify(|w| w.set_hseon(true));
            while !RCC.ctlr().read().hserdy() {}
            Some(hse.freq)
        }
    };

    // Configure PLLs.
    let pll_clk = {
        // Disable PLL
        RCC.ctlr().modify(|w| w.set_pllon(false));
        if let Some(pll) = config.pll {
            let pll_src = match config.pll_src {
                PllSource::HSI => hsi.unwrap(),
                PllSource::HSE => hse.unwrap(),
                PllSource::PLL2 => todo!(),
            };
            let in_freq = pll_src / pll.prediv;
            let vco_freq = in_freq * pll.mul;

            RCC.cfgr0().modify(|w| {
                w.set_pllsrc(config.pll_src == PllSource::HSE || config.pll_src == PllSource::PLL2);
                w.set_pllmul(pll.mul);
            });
            RCC.cfgr2().modify(|w| {
                w.set_prediv1(pll.prediv);
                w.set_prediv1src(config.pll_src == PllSource::PLL2);
            });
            if config.pll_src == PllSource::HSI {
                match pll.prediv {
                    PllPreDiv::DIV1 => EXTEND.ctr().modify(|w| w.set_pll_hsi_pre(true)), // set no divided
                    PllPreDiv::DIV2 => EXTEND.ctr().modify(|w| w.set_pll_hsi_pre(false)),
                    _ => panic!("invalid combination"),
                }
            }

            // Enable PLL
            RCC.ctlr().modify(|w| w.set_pllon(true));
            while !RCC.ctlr().read().pllrdy() {}
            Some(vco_freq)
        } else {
            None
        }
    };

    // Configure sysclk
    let sys = match config.sys {
        Sysclk::HSI => hsi.unwrap(),
        Sysclk::HSE => hsi.unwrap(),
        Sysclk::PLL => pll_clk.unwrap(),
        _ => unreachable!(),
    };
    let hclk = sys / config.ahb_pre;
    let (pclk1, pclk1_tim) = calc_pclk(hclk, config.apb1_pre);
    let (pclk2, pclk2_tim) = calc_pclk(hclk, config.apb2_pre);

    // 当 AHB 时钟来源的预分频系数大于 1 时，必须开启预取缓冲器

    FLASH.ctlr().modify(|w| {
        w.set_sckmode(sys.0 <= 72_000_000);
        w.set_enhancemode(true);
    });

    RCC.cfgr0().modify(|w| {
        w.set_sw(config.sys);
        w.set_hpre(config.ahb_pre);
        w.set_ppre1(config.apb1_pre);
        w.set_ppre2(config.apb2_pre);
    });
    while RCC.cfgr0().read().sws() != config.sys {}

    super::CLOCKS.sysclk = sys;
    super::CLOCKS.hclk = hclk;
    super::CLOCKS.pclk1 = pclk1;
    super::CLOCKS.pclk2 = pclk2;
}

pub fn calc_pclk<D>(hclk: Hertz, ppre: D) -> (Hertz, Hertz)
where
    Hertz: ops::Div<D, Output = Hertz>,
{
    let pclk = hclk / ppre;
    let pclk_tim = if hclk == pclk { pclk } else { pclk * 2u32 };
    (pclk, pclk_tim)
}

impl ops::Div<PllPreDiv> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: PllPreDiv) -> Hertz {
        Hertz(self.0 / (rhs as u32 + 1))
    }
}

impl ops::Mul<PllMul> for Hertz {
    type Output = Hertz;
    fn mul(self, rhs: PllMul) -> Hertz {
        match rhs {
            PllMul::MUL15 => Hertz(self.0 * 15),
            PllMul::MUL16 => Hertz(self.0 * 16),
            PllMul::MUL18 => Hertz(self.0 * 18),
            PllMul::MUL6_5 => Hertz(self.0 * 13 / 2),
            // PllMul::MUL2 => Hertz(self.0 * 2),
            _ => Hertz(self.0 * (rhs as u32 + 2)),
        }
    }
}

impl ops::Div<AHBPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: AHBPrescaler) -> Hertz {
        let raw = rhs as u32;
        if raw >= 0b1000 {
            // 2, 4, 8
            let d = raw - 0b1000 + 1;
            Hertz(self.0 >> d)
        } else {
            // DIV1
            self
        }
    }
}

impl ops::Div<APBPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: APBPrescaler) -> Hertz {
        let raw = rhs as u32;
        if raw >= 0b100 {
            // 2, 4, 8, 16
            let d = raw - 0b100 + 1;
            Hertz(self.0 >> d)
        } else {
            // DIV1
            self
        }
    }
}
