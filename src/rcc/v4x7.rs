//! CH32V407/CH32V467 clocks.

use super::{apb_timer_clk, set_clocks, Clocks};
use crate::pac::{FLASH, RCC};
use crate::time::Hertz;

pub const HSI_FREQUENCY: Hertz = Hertz(20_000_000);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum AHBPrescaler {
    #[default]
    DIV1 = 0,
    DIV2 = 8,
    DIV4 = 9,
    DIV8 = 10,
    DIV16 = 11,
    DIV64 = 12,
    DIV128 = 13,
    DIV256 = 14,
    DIV512 = 15,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum APBPrescaler {
    #[default]
    DIV1 = 0,
    DIV2 = 4,
    DIV4 = 5,
    DIV8 = 6,
    DIV16 = 7,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum Sysclk {
    #[default]
    HSI = 0,
    HSE = 1,
    PLL = 2,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum PllSource {
    #[default]
    HSI = 0,
    HSE = 1,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum PllMul {
    MUL8 = 0,
    MUL9,
    MUL10,
    MUL11,
    MUL12,
    MUL12_5,
    MUL13,
    MUL13_5,
    MUL14,
    MUL14_5,
    MUL15,
    MUL15_5,
    MUL16,
    MUL16_5,
    MUL17,
    MUL17_5,
    MUL18,
    MUL18_5,
    MUL19,
    MUL19_5,
    MUL20,
    MUL21,
    MUL22,
    MUL24,
    MUL26,
    MUL28,
    MUL30,
    MUL32,
    MUL34,
    MUL36,
    MUL38,
    MUL40,
}

#[derive(Clone, Copy, Debug)]
pub struct Pll {
    pub mul: PllMul,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SysPllSource {
    UsbHs = 0,
    Eth = 1,
    UsbHsMul2Div3 = 2,
    EthMul2Div3 = 3,
    #[default]
    Pll = 6,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SysPllPrescaler {
    #[default]
    DIV1 = 0,
    DIV2,
    DIV3,
    DIV4,
    DIV5,
    DIV6,
    DIV7,
    DIV8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum HsPllSource {
    HSE = 0,
    HSI = 1,
    ETH = 2,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum RngClockSource {
    #[default]
    Sysclk,
    UsbHsPll,
    EthPll,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum AdcClockSource {
    #[default]
    Hclk,
    /// USBHS PLL divided by 1 through 32, before the APB2 and ADC prescalers.
    UsbHsPll { divisor: u8 },
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum AdcPrescaler {
    DIV2,
    DIV4,
    DIV6,
    #[default]
    DIV8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum AdcDutyCycle {
    Percent50,
    #[default]
    Percent62_5,
    Percent75,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    pub hse: Option<super::Hse>,
    pub sys: Sysclk,
    pub pll_src: PllSource,
    pub pll: Option<Pll>,
    pub syspll_src: SysPllSource,
    pub syspll_pre: SysPllPrescaler,
    pub hspll: Option<HsPllSource>,
    pub ethpll: bool,
    pub ahb_pre: AHBPrescaler,
    pub apb1_pre: APBPrescaler,
    pub apb2_pre: APBPrescaler,
    /// PLL selection shares I2SPLLSRC with the I2S2/I2S3 kernel clocks.
    pub rng_src: RngClockSource,
    pub adc_src: AdcClockSource,
    pub adc_pre: AdcPrescaler,
    pub adc_duty: AdcDutyCycle,
}

/// Selected ADC clock, rounded down to whole Hz, or None if its PLL is not ready.
pub fn adc_frequency() -> Option<Hertz> {
    adc_clock_ratio().map(|(source, divisor)| Hertz(source / divisor))
}

pub(crate) fn adc_clock_ratio() -> Option<(u32, u32)> {
    let selection = RCC.cfgr0().read();
    let (source, divisor) = if selection.adcsrc() {
        if !RCC.ctlr().read().usbhs_pllrdy() {
            return None;
        }
        (480_000_000, u32::from(RCC.cfgr3().read().adc_clkdiv()) + 1)
    } else {
        (super::clocks().sysclk.0, hclk_divisor())
    };
    Some((
        source,
        divisor * (1 << APB_SHIFTS[selection.ppre2() as usize]) * adc_divisor(selection.adcpre()),
    ))
}

const fn adc_divisor(prescaler: u8) -> u32 {
    (prescaler as u32 + 1) * 2
}

/// Selected RNG source clock, or None when its source PLL is not ready.
pub fn rng_source_frequency() -> Option<Hertz> {
    let selection = RCC.cfgr2().read();
    let control = RCC.ctlr().read();
    if !selection.rngsrc() {
        Some(super::clocks().sysclk)
    } else if selection.i2spllsrc() {
        control.eth_pllrdy().then_some(Hertz(500_000_000))
    } else {
        control.usbhs_pllrdy().then_some(Hertz(480_000_000))
    }
}

const AHB_SHIFTS: [u8; 16] = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9];
const APB_SHIFTS: [u8; 8] = [0, 0, 0, 0, 1, 2, 3, 4];
const PLL_MUL_X2: [u8; 32] = [
    16, 18, 20, 22, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 42, 44, 48, 52, 56, 60, 64, 68,
    72, 76, 80,
];

pub(crate) fn hclk_divisor() -> u32 {
    1 << AHB_SHIFTS[RCC.cfgr0().read().hpre() as usize]
}

fn bus_clocks(sysclk: Hertz, ahb: u8, apb1: u8, apb2: u8) -> Clocks {
    let hclk = Hertz(sysclk.0 >> AHB_SHIFTS[ahb as usize]);
    let pclk1 = Hertz(hclk.0 >> APB_SHIFTS[apb1 as usize]);
    let pclk2 = Hertz(hclk.0 >> APB_SHIFTS[apb2 as usize]);
    assert!(hclk.0 <= 200_000_000, "CH32V4x7 HCLK exceeds the 200 MHz rated limit");
    Clocks {
        sysclk,
        hclk,
        pclk1,
        pclk2,
        pclk1_tim: apb_timer_clk(hclk, pclk1, true),
        pclk2_tim: apb_timer_clk(hclk, pclk2, true),
    }
}

fn hse_frequency(hse: Option<Hertz>) -> Hertz {
    let frequency = hse.expect("CH32V4x7 HSE frequency is required");
    assert!((5_000_000..=32_000_000).contains(&frequency.0), "HSE must be 5..32 MHz");
    frequency
}

fn main_pll_frequency(hse: Option<Hertz>, source: bool, mul: u8) -> Hertz {
    let input = if source { hse_frequency(hse) } else { HSI_FREQUENCY };
    let output = Hertz((u64::from(input.0) * u64::from(PLL_MUL_X2[mul as usize]) / 2) as u32);
    assert!(
        (100_000_000..=400_000_000).contains(&output.0),
        "PLL output must be 100..400 MHz"
    );
    output
}

fn hspll_reference(frequency: Hertz) -> u8 {
    match frequency.0 {
        25_000_000 => 0,
        20_000_000 => 1,
        24_000_000 => 2,
        32_000_000 => 3,
        _ => panic!("USBHS PLL reference must be 20, 24, 25 or 32 MHz"),
    }
}

fn hspll_frequency(hse: Option<Hertz>) -> Hertz {
    let pll = RCC.cfgr2().read();
    let reference = match pll.usbhspllsrc() {
        0 => hse_frequency(hse),
        1 => HSI_FREQUENCY,
        2 => {
            assert_eq!(hse_frequency(hse).0, 25_000_000, "ETH PLL requires a 25 MHz HSE");
            HSI_FREQUENCY
        }
        _ => panic!("Reserved USBHS PLL reference source"),
    };
    assert_eq!(
        hspll_reference(reference),
        pll.usbhspllclk(),
        "USBHS PLL reference selection mismatch"
    );
    Hertz(480_000_000)
}

impl Config {
    pub fn clocks(&self) -> Clocks {
        match self.rng_src {
            RngClockSource::Sysclk => {}
            RngClockSource::UsbHsPll => assert!(self.hspll.is_some(), "RNG requires USBHS PLL"),
            RngClockSource::EthPll => assert!(self.ethpll, "RNG requires ETH PLL"),
        }
        let hse = self.hse.map(|clock| hse_frequency(Some(clock.freq)));
        let pll = self
            .pll
            .map(|pll| main_pll_frequency(hse, self.pll_src == PllSource::HSE, pll.mul as u8));
        if self.ethpll {
            assert_eq!(hse_frequency(hse).0, 25_000_000, "ETH PLL requires a 25 MHz HSE");
        }
        if let Some(source) = self.hspll {
            let reference = match source {
                HsPllSource::HSE => hse_frequency(hse),
                HsPllSource::HSI => HSI_FREQUENCY,
                HsPllSource::ETH => {
                    assert!(self.ethpll, "USBHS PLL ETH input requires ETH PLL");
                    Hertz(20_000_000)
                }
            };
            hspll_reference(reference);
        }
        let sysclk = match self.sys {
            Sysclk::HSI => HSI_FREQUENCY,
            Sysclk::HSE => hse_frequency(hse),
            Sysclk::PLL => {
                let input = match self.syspll_src {
                    SysPllSource::Pll => pll.expect("SYSPLL requires the main PLL"),
                    SysPllSource::UsbHs | SysPllSource::UsbHsMul2Div3 => {
                        assert!(self.hspll.is_some(), "SYSPLL requires USBHS PLL");
                        Hertz(if self.syspll_src == SysPllSource::UsbHs {
                            480_000_000
                        } else {
                            320_000_000
                        })
                    }
                    SysPllSource::Eth | SysPllSource::EthMul2Div3 => {
                        assert!(self.ethpll, "SYSPLL requires ETH PLL");
                        Hertz(if self.syspll_src == SysPllSource::Eth {
                            500_000_000
                        } else {
                            1_000_000_000 / 3
                        })
                    }
                };
                Hertz(input.0 / (self.syspll_pre as u32 + 1))
            }
        };
        let clocks = bus_clocks(sysclk, self.ahb_pre as u8, self.apb1_pre as u8, self.apb2_pre as u8);
        let (adc_source, adc_source_divisor) = match self.adc_src {
            AdcClockSource::Hclk => (sysclk.0, 1 << AHB_SHIFTS[self.ahb_pre as usize]),
            AdcClockSource::UsbHsPll { divisor } => {
                assert!(self.hspll.is_some(), "ADC requires USBHS PLL");
                assert!(
                    (1..=32).contains(&divisor),
                    "ADC USBHS PLL divisor must be 1 through 32"
                );
                (480_000_000, u32::from(divisor))
            }
        };
        let adc_divisor =
            adc_source_divisor * (1 << APB_SHIFTS[self.apb2_pre as usize]) * adc_divisor(self.adc_pre as u8);
        assert!(
            u64::from(adc_source) <= 30_000_000 * u64::from(adc_divisor),
            "ADC clock exceeds 30 MHz"
        );
        assert!(adc_source / adc_divisor != 0, "ADC clock is below 1 Hz");
        clocks
    }
}

fn wait_for(mut ready: impl FnMut() -> bool) {
    for _attempt in 0..1_000_000 {
        if ready() {
            return;
        }
    }
    panic!("CH32V4x7 clock startup or switch timed out");
}

pub(crate) unsafe fn init(config: Config) {
    let clocks = config.clocks();
    RCC.ctlr().modify(|w| w.set_hsion(true));
    wait_for(|| RCC.ctlr().read().hsirdy());
    RCC.cfgr0().modify(|w| w.set_sw(0));
    wait_for(|| RCC.cfgr0().read().sws() == 0);
    RCC.cfgr2().modify(|w| w.set_syspll_gate(false));
    RCC.ctlr().modify(|w| {
        w.set_csson(false);
        w.set_pllon(false);
        w.set_usbhs_pllon(false);
        w.set_eth_pllon(false);
        w.set_hseon(false);
    });
    wait_for(|| {
        let control = RCC.ctlr().read();
        !control.pllrdy() && !control.usbhs_pllrdy() && !control.eth_pllrdy() && !control.hserdy()
    });
    if let Some(hse) = config.hse {
        RCC.ctlr().modify(|w| w.set_hsebyp(hse.mode == super::HseMode::Bypass));
        RCC.ctlr().modify(|w| w.set_hseon(true));
        wait_for(|| RCC.ctlr().read().hserdy());
    }
    if config.ethpll {
        RCC.ctlr().modify(|w| w.set_eth_pllon(true));
        wait_for(|| RCC.ctlr().read().eth_pllrdy());
    }
    if let Some(source) = config.hspll {
        let reference = match source {
            HsPllSource::HSE => config.hse.unwrap().freq,
            HsPllSource::HSI | HsPllSource::ETH => HSI_FREQUENCY,
        };
        RCC.cfgr2().modify(|w| {
            w.set_usbhspllsrc(source as u8);
            w.set_usbhspllclk(hspll_reference(reference));
        });
        RCC.ctlr().modify(|w| w.set_usbhs_pllon(true));
        wait_for(|| RCC.ctlr().read().usbhs_pllrdy());
    }
    if let Some(pll) = config.pll {
        RCC.cfgr0().modify(|w| {
            w.set_pllsrc(config.pll_src == PllSource::HSE);
            w.set_pllmul(pll.mul as u8);
        });
        RCC.ctlr().modify(|w| w.set_pllon(true));
        wait_for(|| RCC.ctlr().read().pllrdy());
    }
    FLASH.keyr().write(|w| w.set_keyr(0x4567_0123));
    FLASH.keyr().write(|w| w.set_keyr(0xcdef_89ab));
    FLASH.modekeyr().write(|w| w.set_modekeyr(0x4567_0123));
    FLASH.modekeyr().write(|w| w.set_modekeyr(0xcdef_89ab));
    FLASH.ctlr().modify(|w| {
        w.set_sckmod(if clocks.hclk.0 <= 100_000_000 { 0 } else { 1 });
        w.set_lock(true);
        w.set_flock(true);
    });
    assert_eq!(
        FLASH.ctlr().read().sckmod(),
        if clocks.hclk.0 <= 100_000_000 { 0 } else { 1 }
    );
    RCC.cfgr2().modify(|w| {
        w.set_syspllsrc(config.syspll_src as u8);
        w.set_sysplldiv(config.syspll_pre as u8);
        w.set_rngsrc(config.rng_src != RngClockSource::Sysclk);
        if config.rng_src != RngClockSource::Sysclk {
            w.set_i2spllsrc(config.rng_src == RngClockSource::EthPll);
        }
    });
    RCC.cfgr0().modify(|w| {
        w.set_hpre(config.ahb_pre as u8);
        w.set_ppre1(config.apb1_pre as u8);
        w.set_ppre2(config.apb2_pre as u8);
        w.set_adcpre(config.adc_pre as u8);
        w.set_adcsrc(matches!(config.adc_src, AdcClockSource::UsbHsPll { .. }));
        w.set_adc_duty_sel(config.adc_duty as u8);
    });
    if let AdcClockSource::UsbHsPll { divisor } = config.adc_src {
        RCC.cfgr3().modify(|w| w.set_adc_clkdiv(divisor - 1));
    }
    if config.sys == Sysclk::PLL {
        RCC.cfgr2().modify(|w| w.set_syspll_gate(true));
    }
    RCC.cfgr0().modify(|w| w.set_sw(config.sys as u8));
    wait_for(|| RCC.cfgr0().read().sws() == config.sys as u8);
    refresh_clocks(config.hse.map(|hse| hse.freq));
    assert_eq!(*super::clocks(), clocks);
}

pub(crate) unsafe fn refresh_clocks(hse: Option<Hertz>) {
    let cfgr = RCC.cfgr0().read();
    let sysclk = match cfgr.sws() {
        0 => HSI_FREQUENCY,
        1 => hse_frequency(hse),
        2 => {
            let pll = RCC.cfgr2().read();
            let input = match pll.syspllsrc() {
                0 => hspll_frequency(hse).0,
                1 => {
                    assert_eq!(hse_frequency(hse).0, 25_000_000);
                    500_000_000
                }
                2 => hspll_frequency(hse).0 / 3 * 2,
                3 => {
                    assert_eq!(hse_frequency(hse).0, 25_000_000);
                    1_000_000_000 / 3
                }
                6 => main_pll_frequency(hse, cfgr.pllsrc(), cfgr.pllmul()).0,
                _ => panic!("Reserved SYSPLL clock source"),
            };
            Hertz(input / (u32::from(pll.sysplldiv()) + 1))
        }
        _ => panic!("Reserved system clock source"),
    };
    set_clocks(bus_clocks(sysclk, cfgr.hpre(), cfgr.ppre1(), cfgr.ppre2()));
}
