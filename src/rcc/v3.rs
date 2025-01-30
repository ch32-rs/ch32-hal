//! CH32V2, CH32V3
use core::ops::{self, Div};

pub use crate::pac::rcc::vals::{
    Hpre as AHBPrescaler, PllMul, PllxMul, Ppre as APBPrescaler, Prediv as PllPreDiv, Sw as Sysclk, Usbpre,
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

    // PLL2 or not, selected by PREDIV1SRC
    #[cfg(d8c)]
    PLL2,
}

pub struct Pllx {
    pub prediv2: PllPreDiv,
    pub pll2: Option<PllxMul>,
    pub pll3: Option<PllxMul>,
}

#[derive(Clone, Copy, PartialEq)]
// USBHSPLLSRC bits in rcc_cfgr2
pub enum HsPllSource {
    HSI,
    HSE,
}

// TODO: should be in PAC
// USBHSDIV bits in rcc_cfgr2
#[derive(Clone, Copy, PartialEq)]
pub enum HsPllPrescaler {
    DIV1 = 0b000,
    DIV2 = 0b001,
    DIV3 = 0b010,
    DIV4 = 0b011,
    DIV5 = 0b100,
    DIV6 = 0b101,
    DIV7 = 0b110,
    DIV8 = 0b111,
}

impl Div<HsPllPrescaler> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: HsPllPrescaler) -> Hertz {
        Hertz(self.0 / (rhs as u32 + 1))
    }
}

#[derive(Clone, Copy, PartialEq)]
pub enum HsPllRef {
    _3M = 0b00,
    _4M = 0b01,
    _8M = 0b10,
    _5M = 0b11,
}

#[derive(Clone, Copy)]
pub struct HsPll {
    pub pre: HsPllPrescaler,
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

    pub hspll_src: HsPllSource,
    pub hspll: Option<HsPll>,
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
            hspll_src: HsPllSource::HSE,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV2,
            }),
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
            hspll_src: HsPllSource::HSE,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV2,
            }),
        }
    };
    pub const SYSCLK_FREQ_144MHZ_HSI: Config = {
        Config {
            hse: None,
            sys: Sysclk::PLL,
            pll_src: PllSource::HSI,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL18,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: super::LsConfig::default_lsi(),
            hspll_src: HsPllSource::HSI,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV2,
            }),
        }
    };
    pub const SYSCLK_FREQ_96MHZ_HSI: Config = {
        Config {
            hse: None,
            sys: Sysclk::PLL,
            pll_src: PllSource::HSI,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL12,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV4, // 24MHz
            apb2_pre: APBPrescaler::DIV4,
            ls: super::LsConfig::default_lsi(),
            hspll_src: HsPllSource::HSI,
            hspll: Some(HsPll {
                pre: HsPllPrescaler::DIV2,
            }),
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
            hspll_src: HsPllSource::HSE,
            hspll: None,
        }
    }
}

#[allow(unused_variables)]
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
    //
    // TODO: Configure PLL2 and PLL3
    #[cfg(d8c)]
    let (pll2_clk, pll3_clk): (Option<Hertz>, Option<Hertz>) = if let Some(_pllx) = config.pllx {
        todo!()
    } else {
        (None, None)
    };
    // Configure PLL
    let pll_clk = {
        // Disable PLL
        RCC.ctlr().modify(|w| w.set_pllon(false));
        if let Some(pll) = config.pll {
            let pll_src = match config.pll_src {
                PllSource::HSI => hsi.unwrap(),
                PllSource::HSE => hse.unwrap(),
                #[cfg(d8c)]
                PllSource::PLL2 => todo!(),
            };
            let in_freq = pll_src / pll.prediv;
            let vco_freq = in_freq * pll.mul;

            // Usb clock must be 48MHz
            let usb_pre = calc_usbpre(vco_freq);
            if let Some(usb_pre) = usb_pre {
                RCC.cfgr0().modify(|w| w.set_usbpre(usb_pre));
            }
            // TODO: handle USBHS clk

            RCC.cfgr0().modify(|w| w.set_pllmul(pll.mul));

            match config.pll_src {
                PllSource::HSI => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(false)); // use HSI or HSI/2
                    match pll.prediv {
                        PllPreDiv::DIV1 => EXTEND.ctr().modify(|w| w.set_pll_hsi_pre(true)), // set no divided
                        PllPreDiv::DIV2 => EXTEND.ctr().modify(|w| w.set_pll_hsi_pre(false)),
                        _ => panic!(
                            "If HSI is selected as the PLL source (PLLSRC), only dividers DIV1 and DIV2 are available"
                        ),
                    }
                }
                #[cfg(d8c)]
                PllSource::HSE => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(true));
                    RCC.cfgr2().modify(|w| {
                        w.set_prediv1src(false); // HSE
                        w.set_prediv1(pll.prediv);
                    });
                }
                #[cfg(d8c)]
                PllSource::PLL2 => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(true));
                    RCC.cfgr2().modify(|w| {
                        w.set_prediv1src(true); // PLL2
                        w.set_prediv1(pll.prediv);
                    });
                }
                // FV203/V303
                #[cfg(any(d6, d8))]
                PllSource::HSE => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(true));
                    match pll.prediv {
                        PllPreDiv::DIV1 => RCC.cfgr0().modify(|w| w.set_pllxtpre(false)),
                        PllPreDiv::DIV2 => RCC.cfgr0().modify(|w| w.set_pllxtpre(true)),
                        _ => panic!(),
                    }
                }
                // CH32V20x_D8
                #[cfg(all(ch32v2, d8))]
                PllSource::HSE => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(true));
                    match pll.prediv {
                        PllPreDiv::DIV4 => RCC.cfgr0().modify(|w| w.set_pllxtpre(false)),
                        PllPreDiv::DIV8 => RCC.cfgr0().modify(|w| w.set_pllxtpre(true)),
                        _ => panic!(),
                    }
                }
                // CH32F20x_D8W、、CH32V20x_D8W
                #[cfg(d8w)]
                PllSource::HSE => {
                    RCC.cfgr0().modify(|w| w.set_pllsrc(true));
                    match pll.prediv {
                        // When using USB and ETH together, the HSE frequency must be 32MHz
                        PllPreDiv::DIV2 if usb_pre == Some(Usbpre::DIV5) => {}
                        PllPreDiv::DIV4 => RCC.cfgr0().modify(|w| w.set_pllxtpre(false)),
                        PllPreDiv::DIV8 => RCC.cfgr0().modify(|w| w.set_pllxtpre(true)),
                        _ => panic!(),
                    }
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

    // Configure HSPLL for USBHS

    #[cfg(d8c)]
    if let Some(hspll) = config.hspll {
        // Disable HSPLL
        RCC.cfgr2().modify(|w| w.set_usbhs_pllalive(false));
        let hspll_src = match config.hspll_src {
            HsPllSource::HSI => hsi.unwrap(),
            HsPllSource::HSE => hse.unwrap(),
        };
        let in_freq = hspll_src / hspll.pre;
        let ref_clk = match in_freq {
            Hertz(3_000_000) => HsPllRef::_3M,
            Hertz(4_000_000) => HsPllRef::_4M,
            Hertz(8_000_000) => HsPllRef::_8M,
            Hertz(5_000_000) => HsPllRef::_5M,
            _ => panic!(),
        };

        RCC.cfgr2().modify(|w| {
            w.set_usbhs_prediy(hspll.pre as u8);
            w.set_usbhs_ckpef_sel(ref_clk as u8);
            w.set_usbhs_pll_src(if config.hspll_src == HsPllSource::HSI {
                true
            } else {
                false
            });
            w.set_usbhs_clk_src(true);
        });
        RCC.cfgr2().modify(|w| w.set_usbhs_pllalive(true));
    };

    // Configure sysclk
    let sys = match config.sys {
        Sysclk::HSI => hsi.unwrap(),
        Sysclk::HSE => hse.unwrap(),
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

    super::CLOCKS.pclk1_tim = pclk1_tim;
    super::CLOCKS.pclk2_tim = pclk2_tim;
}

fn calc_pclk<D>(hclk: Hertz, ppre: D) -> (Hertz, Hertz)
where
    Hertz: ops::Div<D, Output = Hertz>,
{
    let pclk = hclk / ppre;
    let pclk_tim = if hclk == pclk { pclk } else { pclk * 2u32 };
    (pclk, pclk_tim)
}

fn calc_usbpre(pllclk: Hertz) -> Option<Usbpre> {
    // output 48MHz
    match pllclk.0 {
        48_000_000 => Some(Usbpre::DIV1),
        96_000_000 => Some(Usbpre::DIV2),
        144_000_000 => Some(Usbpre::DIV3),
        #[cfg(d8w)]
        240_000_000 => None,
        _ => None,
    }
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
            #[cfg(d8c)]
            PllMul::MUL6_5 => Hertz(self.0 * 13 / 2),
            // All the others are covered by this case
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
