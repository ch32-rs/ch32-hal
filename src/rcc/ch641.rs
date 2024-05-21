//! RCC for CH641.
//!
//! - No HSE
//! - No PLLSRC selection, only HSI
//! - No PLLMUL selection, always *2
use core::ops;

use crate::pac::rcc::vals::{Hpre as AHBPrescaler, Ppre as APBPrescaler, Sw as Sysclk};
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

pub const HSI_FREQUENCY: Hertz = Hertz(24_000_000);

// Ref: CH641DS. Typical value: 400 kHz, min: 100 kHz, max: 700 kHz
pub const LSI_FREQUENCY: Hertz = Hertz(400_000);

pub struct Config {
    pub sys: Sysclk,
    // PLL is always HSI * 2
    pub ahb_pre: AHBPrescaler,
    // ADCPRE is actually splitted from APB2
    pub apb2_pre: APBPrescaler,
}
impl Config {
    pub const SYSCLK_FREQ_48MHZ_HSI: Config = Config {
        sys: Sysclk::PLL,
        ahb_pre: AHBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };
    pub const SYSCLK_FREQ_24MHZ_HSI: Config = Config {
        sys: Sysclk::HSI,
        ahb_pre: AHBPrescaler::DIV1,
        apb2_pre: APBPrescaler::DIV1,
    };
}
impl Default for Config {
    fn default() -> Self {
        Config {
            sys: Sysclk::HSI,
            ahb_pre: AHBPrescaler::DIV3,
            apb2_pre: APBPrescaler::DIV1,
        }
    }
}

#[allow(unused_variables)]
pub(crate) unsafe fn init(config: Config) {
    let sysclk = match config.sys {
        Sysclk::HSI => {
            // HSI default enabled
            HSI_FREQUENCY.0
        }
        Sysclk::PLL => {
            RCC.ctlr().modify(|w| w.set_pllon(true));
            while !RCC.ctlr().read().pllrdy() {}

            RCC.cfgr0().modify(|w| w.set_sw(Sysclk::PLL));
            while RCC.cfgr0().read().sws() != Sysclk::PLL {}

            // MUL2 is the only option, always *2
            HSI_FREQUENCY.0 * 2
        }
        _ => unreachable!(),
    };

    if sysclk >= 24_000_000 {
        FLASH.actlr().modify(|w| w.set_latency(0b01)); // 1 等待（24MHz<HCLK≤48MHz）
    }

    RCC.cfgr0().modify(|w| {
        w.set_hpre(config.ahb_pre);
        w.set_ppre2(config.apb2_pre); // FIXME: this is undocumented, only for ADC2?
    });

    // TODO: handle APBPrescaler
    let hclk = match config.ahb_pre {
        AHBPrescaler::DIV1 => sysclk,
        AHBPrescaler::DIV2 => sysclk / 2,
        AHBPrescaler::DIV3 => sysclk / 3,
        AHBPrescaler::DIV4 => sysclk / 4,
        AHBPrescaler::DIV5 => sysclk / 5,
        AHBPrescaler::DIV6 => sysclk / 6,
        AHBPrescaler::DIV7 => sysclk / 7,
        AHBPrescaler::DIV8 => sysclk / 8,
        AHBPrescaler::DIV16 => sysclk / 16,
        AHBPrescaler::DIV32 => sysclk / 16,
        AHBPrescaler::DIV64 => sysclk / 64,
        AHBPrescaler::DIV128 => sysclk / 128,
        AHBPrescaler::DIV256 => sysclk / 256,
        _ => panic!(),
    };

    let pclk2 = Hertz(hclk) / config.apb2_pre;

    super::CLOCKS.sysclk = Hertz(sysclk);
    super::CLOCKS.hclk = Hertz(hclk);
    super::CLOCKS.pclk1 = Hertz(hclk);
    super::CLOCKS.pclk2 = pclk2;

    super::CLOCKS.pclk1_tim = Hertz(sysclk);
    super::CLOCKS.pclk2_tim = Hertz(sysclk);
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
