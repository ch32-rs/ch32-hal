//! RCC init for CH32V1, CH32L1.
//!
//! Differences:
//!
//! - USBPre is 2bit in CH32L1, 1bit in CH32V1.
//! - PllMul is different.
//! - Flash latency is different.

use core::ops;

use crate::pac::rcc::vals::{
    Hpre as AHBPrescaler, PllMul, Pllsrc as PllSource, Ppre as APBPrescaler, Sw as Sysclk, Usbpre,
};
use crate::pac::{EXTEND, FLASH, RCC};
use crate::time::Hertz;

pub const HSI_FREQUENCY: Hertz = Hertz(8_000_000);
pub const LSI_FREQUENCY: Hertz = Hertz(40_000);

#[derive(Clone, Copy)]
pub enum PllPreDiv {
    DIV1 = 1,
    DIV2 = 2,
}

#[derive(Clone, Copy)]
pub struct Pll {
    /// PLL pre-divider
    pub prediv: PllPreDiv,

    /// PLL multiplication factor.
    pub mul: PllMul,
}

pub struct Config {
    // won't close hsi
    // pub hsi: bool,
    pub hse: Option<super::Hse>,
    pub sys: Sysclk,

    pub pll_src: PllSource,
    pub pll: Option<Pll>,

    pub ahb_pre: AHBPrescaler,
    pub apb1_pre: APBPrescaler,
    pub apb2_pre: APBPrescaler,
}

impl Config {
    pub const SYSCLK_FREQ_48MHZ_HSE: Config = Self {
        hse: Some(super::Hse {
            freq: Hertz(8_000_000),
            mode: super::HseMode::Oscillator,
        }),
        sys: Sysclk::PLL,
        pll_src: PllSource::HSE,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL6,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };
    pub const SYSCLK_FREQ_72MHZ_HSE: Config = Self {
        hse: Some(super::Hse {
            freq: Hertz(8_000_000),
            mode: super::HseMode::Oscillator,
        }),
        sys: Sysclk::PLL,
        pll_src: PllSource::HSE,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV2,
        apb2_pre: APBPrescaler::DIV2,
    };
    pub const SYSCLK_FREQ_96MHZ_HSE: Config = Self {
        hse: Some(super::Hse {
            freq: Hertz(8_000_000),
            mode: super::HseMode::Oscillator,
        }),
        sys: Sysclk::PLL,
        pll_src: PllSource::HSE,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL12,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV2,
        apb2_pre: APBPrescaler::DIV2,
    };
    pub const SYSCLK_FREQ_48MHZ_HSI: Config = Self {
        hse: None,
        sys: Sysclk::PLL,
        pll_src: PllSource::HSI,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL6,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };
    pub const SYSCLK_FREQ_72MHZ_HSI: Config = Self {
        hse: None,
        sys: Sysclk::PLL,
        pll_src: PllSource::HSI,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV2,
        apb2_pre: APBPrescaler::DIV2,
    };
    pub const SYSCLK_FREQ_96MHZ_HSI: Config = Self {
        hse: None,
        sys: Sysclk::PLL,
        pll_src: PllSource::HSI,
        pll: Some(Pll {
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL12,
        }),
        ahb_pre: AHBPrescaler::DIV1,
        apb1_pre: APBPrescaler::DIV2,
        apb2_pre: APBPrescaler::DIV2,
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
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
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
            RCC.ctlr().modify(|w| w.set_hsebyp(hse.mode != super::HseMode::Oscillator));
            RCC.ctlr().modify(|w| w.set_hseon(true));
            while !RCC.ctlr().read().hserdy() {}
            Some(hse.freq)
        }
    };

    // Configure PLLs.
    // Configure PLL
    let pll_clk = {
        // Disable PLL
        RCC.ctlr().modify(|w| w.set_pllon(false));
        match config.pll {
            None => None,
            Some(pll) => {
                let pll_src = match config.pll_src {
                    PllSource::HSI => hsi.unwrap(),
                    PllSource::HSE => hse.unwrap(),
                };
                let in_freq = pll_src / pll.prediv;
                let vco_freq = in_freq * pll.mul;

                // Usb clock must be 48MHz
                if let Some(usb_pre) = calc_usbpre(vco_freq) {
                    RCC.cfgr0().modify(|w| w.set_usbpre(usb_pre));
                }

                RCC.cfgr0().modify(|w| w.set_pllmul(pll.mul));

                match config.pll_src {
                    PllSource::HSI => {
                        RCC.cfgr0().modify(|w| w.set_pllsrc(PllSource::HSI)); // use HSI or HSI/2
                        match pll.prediv {
                            PllPreDiv::DIV1 => EXTEND.ctr().modify(|w| w.set_hsipre(true)), // set no divided
                            PllPreDiv::DIV2 => EXTEND.ctr().modify(|w| w.set_hsipre(false)),
                        }
                    }
                    PllSource::HSE => {
                        RCC.cfgr0().modify(|w| w.set_pllsrc(PllSource::HSE));
                        match pll.prediv {
                            PllPreDiv::DIV1 => RCC.cfgr0().modify(|w| w.set_pllxtpre(false)),
                            PllPreDiv::DIV2 => RCC.cfgr0().modify(|w| w.set_pllxtpre(true)),
                        }
                    }
                }

                // Enable PLL
                RCC.ctlr().modify(|w| w.set_pllon(true));
                while !RCC.ctlr().read().pllrdy() {}
                Some(vco_freq)
            }
        }
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

    // flash latency
    #[cfg(ch32v1)]
    match sys.0 {
        0..=24_000_000 => FLASH.actlr().modify(|w| w.set_latency(0)),
        24_000_001..=48_000_000 => FLASH.actlr().modify(|w| w.set_latency(1)),
        48_000_001..=72_000_000 => FLASH.actlr().modify(|w| w.set_latency(2)),
        _ => FLASH.actlr().modify(|w| w.set_latency(2)),
    }
    #[cfg(ch32l1)]
    match sys.0 {
        0..=40_000_000 => FLASH.actlr().modify(|w| w.set_latency(0)),
        40_000_001..=72_000_000 => FLASH.actlr().modify(|w| w.set_latency(1)),
        72_000_001..=144_000_000 => FLASH.actlr().modify(|w| w.set_latency(2)),
        _ => FLASH.actlr().modify(|w| w.set_latency(2)),
    }

    RCC.cfgr0().modify(|w| {
        w.set_sw(config.sys);
        w.set_hpre(config.ahb_pre);
        w.set_ppre1(config.apb1_pre);
        w.set_ppre2(config.apb2_pre);
    });
    while RCC.cfgr0().read().sws() != config.sys {}

    refresh_clocks(config.hse.map(|h| h.freq));
}

pub(crate) unsafe fn refresh_clocks(hse: Option<Hertz>) {
    super::set_clocks(clocks_from_registers(hse));
}

fn clocks_from_registers(hse: Option<Hertz>) -> super::Clocks {
    let cfgr = RCC.cfgr0().read();
    let sys = sysclk_hz(cfgr.sws(), &cfgr, hse);
    let hclk = sys / cfgr.hpre();
    let (pclk1, pclk1_tim) = calc_pclk(hclk, cfgr.ppre1());
    let (pclk2, pclk2_tim) = calc_pclk(hclk, cfgr.ppre2());
    super::Clocks {
        sysclk: sys,
        hclk,
        pclk1,
        pclk2,
        pclk1_tim,
        pclk2_tim,
    }
}

fn sysclk_hz(sw: Sysclk, cfgr: &crate::pac::rcc::regs::Cfgr0, hse: Option<Hertz>) -> Hertz {
    match sw {
        Sysclk::HSI => HSI_FREQUENCY,
        Sysclk::HSE => hse.expect("RCC: HSE frequency required"),
        Sysclk::PLL => {
            let src = match cfgr.pllsrc() {
                PllSource::HSI => {
                    let div2 = !EXTEND.ctr().read().hsipre();
                    if div2 {
                        HSI_FREQUENCY / 2u32
                    } else {
                        HSI_FREQUENCY
                    }
                }
                PllSource::HSE => {
                    let hse = hse.expect("RCC: HSE frequency required");
                    if cfgr.pllxtpre() {
                        hse / 2u32
                    } else {
                        hse
                    }
                }
            };
            src * cfgr.pllmul()
        }
        _ => HSI_FREQUENCY,
    }
}

fn calc_pclk<D>(hclk: Hertz, ppre: D) -> (Hertz, Hertz)
where
    Hertz: ops::Div<D, Output = Hertz>,
{
    let pclk = hclk / ppre;
    let pclk_tim = super::apb_timer_clk(hclk, pclk, true);
    (pclk, pclk_tim)
}

fn calc_usbpre(pllclk: Hertz) -> Option<Usbpre> {
    // output 48MHz
    match pllclk.0 {
        48_000_000 => Some(Usbpre::DIV1),
        #[cfg(ch32l1)]
        96_000_000 => Some(Usbpre::DIV2),
        72_000_000 => Some(Usbpre::DIV1_5),
        _ => None,
    }
}

impl ops::Div<PllPreDiv> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: PllPreDiv) -> Hertz {
        match rhs {
            PllPreDiv::DIV1 => self,
            PllPreDiv::DIV2 => self / 2u32,
        }
    }
}

impl ops::Mul<PllMul> for Hertz {
    type Output = Hertz;
    fn mul(self, rhs: PllMul) -> Hertz {
        #[cfg(ch32v1)]
        match rhs {
            PllMul::MUL16_ALT => Hertz(self.0 * 16),
            _ => Hertz(self.0 * (rhs as u32 + 2)),
        }
        #[cfg(ch32l1)]
        match rhs {
            PllMul::MUL18 => Hertz(self.0 * 18),
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
