use crate::pac::rcc::vals::Hpre as AHBPrescaler;
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

const HSI_FREQUENCY: Hertz = Hertz(48_000_000);

// const DEFAULT_FREQUENCY: Hertz = Hertz(8_000_000);

pub struct Config {
    pub ahb_pre: AHBPrescaler,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ahb_pre: AHBPrescaler::DIV6,
        }
    }
}

impl Config {
    pub const SYSCLK_FREQ_48MHZ_HSI: Config = {
        Config {
            ahb_pre: AHBPrescaler::DIV1,
        }
    };
    pub const SYSCLK_FREQ_24MHZ_HSI: Config = {
        Config {
            ahb_pre: AHBPrescaler::DIV2,
        }
    };
    pub const SYSCLK_FREQ_16MHZ_HSI: Config = {
        Config {
            ahb_pre: AHBPrescaler::DIV3,
        }
    };
    pub const SYSCLK_FREQ_12MHZ_HSI: Config = {
        Config {
            ahb_pre: AHBPrescaler::DIV4,
        }
    };
}

#[allow(unused_variables)]
pub(crate) unsafe fn init(config: Config) {
    RCC.ctlr().modify(|w| w.set_hsion(true));
    //while !RCC.ctlr().read().hsirdy() {}

    let hclk = match config.ahb_pre {
        AHBPrescaler::DIV1 => HSI_FREQUENCY,
        AHBPrescaler::DIV2 | AHBPrescaler::DIV2_ALT => HSI_FREQUENCY / 2_u32,
        AHBPrescaler::DIV3 => HSI_FREQUENCY / 3_u32,
        AHBPrescaler::DIV4 | AHBPrescaler::DIV4_ALT => HSI_FREQUENCY / 4_u32,
        AHBPrescaler::DIV5 => HSI_FREQUENCY / 5_u32,
        AHBPrescaler::DIV6 => HSI_FREQUENCY / 6_u32,
        AHBPrescaler::DIV7 => HSI_FREQUENCY / 7_u32,
        AHBPrescaler::DIV8 | AHBPrescaler::DIV8_ALT => HSI_FREQUENCY / 8_u32,
        AHBPrescaler::DIV16 => HSI_FREQUENCY / 16_u32,
        AHBPrescaler::DIV32 => HSI_FREQUENCY / 32_u32,
        AHBPrescaler::DIV64 => HSI_FREQUENCY / 64_u32,
        AHBPrescaler::DIV128 => HSI_FREQUENCY / 128_u32,
        AHBPrescaler::DIV256 => HSI_FREQUENCY / 256_u32,
    };

    if hclk.0 <= 12_000_000 {
        FLASH.actlr().modify(|w| w.set_latency(0b00)); // 0 等待（0≤HCLK≤12MHz）
    } else if hclk.0 <= 24_000_000 {
        FLASH.actlr().modify(|w| w.set_latency(0b01)); // 1 等待（12MHz<HCLK≤24MHz）
    } else {
        FLASH.actlr().modify(|w| w.set_latency(0b10)); // 2 等待（24MHz<HCLK<=48MHz）
    }

    RCC.cfgr0().modify(|w| {
        w.set_hpre(config.ahb_pre);
    });

    super::CLOCKS.sysclk = HSI_FREQUENCY;
    super::CLOCKS.hclk = hclk;
    super::CLOCKS.pclk1 = hclk;
    super::CLOCKS.pclk2 = hclk;

    super::CLOCKS.pclk1_tim = hclk;
    super::CLOCKS.pclk2_tim = hclk;
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ResetReason {
    LowPower,
    WWDG,
    IWDG,
    Software,
    PowerOn,
    ResetPin,
    OPA,
    USBPD,
}

pub fn reset_resona() -> ResetReason {
    let csr = RCC.rstsckr().read();

    if csr.porrstf() {
        ResetReason::PowerOn
    } else if csr.pinrstf() {
        ResetReason::ResetPin
    } else if csr.lpwrrstf() {
        ResetReason::LowPower
    } else if csr.wwdgrstf() {
        ResetReason::WWDG
    } else if csr.iwdgrstf() {
        ResetReason::IWDG
    } else if csr.sftrstf() {
        ResetReason::Software
    } else if csr.oparstf() {
        ResetReason::OPA
    } else {
        panic!("Unknown reset reason")
    }
}
