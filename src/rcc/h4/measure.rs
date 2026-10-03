//! Derive bus / core frequencies from live RCC registers (EVT `SystemAndCoreClockUpdate`).

use crate::pac::rcc::vals::{Pllmul, Pllsrc, Sw as SysclkSw, SyspllSel};
use crate::pac::RCC;
use crate::time::Hertz;

use super::prescale::{ahb_from_sysclk, hclk_from_ahb, pclk_from_hclk};
use super::{CoreClocks, HSI_FREQUENCY};

const PLL_MUL_TABLE: [u8; 32] =
    [4, 6, 7, 8, 17, 9, 19, 10, 21, 11, 23, 12, 25, 13, 14, 15, 16, 17, 18, 19, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 59];

const SERDES_MUL_TABLE: [u8; 16] = [25, 28, 30, 32, 35, 38, 40, 45, 50, 56, 60, 64, 70, 76, 80, 90];

pub(crate) fn measure(hse: Option<Hertz>) -> (super::super::Clocks, CoreClocks) {
    let sysclk = sysclk_frequency(hse);
    let cfgr = RCC.cfgr0().read();
    let ahb = ahb_from_sysclk(sysclk, cfgr.hpre());
    let hclk = hclk_from_ahb(ahb, cfgr.fpre());
    let (pclk1, pclk1_tim) = pclk_from_hclk(hclk, cfgr.ppre1());
    let (pclk2, pclk2_tim) = pclk_from_hclk(hclk, cfgr.ppre2());

    let bus = super::super::Clocks {
        sysclk,
        hclk,
        pclk1,
        pclk2,
        pclk1_tim,
        pclk2_tim,
    };
    let core = CoreClocks {
        sysclk,
        hclk,
        v5f: ahb,
        v3f: hclk,
    };
    (bus, core)
}

fn sysclk_frequency(hse: Option<Hertz>) -> Hertz {
    match RCC.cfgr0().read().sws() {
        SysclkSw::HSI => HSI_FREQUENCY,
        SysclkSw::HSE => hse.expect("RCC: HSE frequency required to measure clocks"),
        SysclkSw::PLL => syspll_frequency(hse),
        _ => HSI_FREQUENCY,
    }
}

fn syspll_frequency(hse: Option<Hertz>) -> Hertz {
    let sel = RCC.pllcfgr().read().syspll_sel();
    match sel {
        SyspllSel::PLL_CLK => main_pll_frequency(hse),
        SyspllSel::USBHS_PLL => Hertz(480_000_000),
        SyspllSel::ETH_PLL => Hertz(500_000_000),
        SyspllSel::SERDES_PLL_DIV2 => serdes_sysclk(hse),
        SyspllSel::USBSS_PLL => Hertz(125_000_000),
        _ => HSI_FREQUENCY,
    }
}

fn main_pll_frequency(hse: Option<Hertz>) -> Hertz {
    let pllcfgr = RCC.pllcfgr().read();
    let pllmull = pllcfgr.pllmul();
    let pllsrc = pllcfgr.pllsrc();
    let presc = (pllcfgr.pll_src_div() as u32) + 1;

    let ref_hz = match pllsrc {
        Pllsrc::HSI => HSI_FREQUENCY.0 / presc,
        Pllsrc::HSE => hse.expect("RCC: HSE frequency required for PLL").0 / presc,
        Pllsrc::USBHS_PLL => 480_000_000 / presc,
        Pllsrc::ETH_PLL => 500_000_000 / presc,
        Pllsrc::USBSS_PLL => 125_000_000 / presc,
        Pllsrc::SERDES_PLL_DIV2 => serdes_pll_hz(hse) / presc,
        _ => HSI_FREQUENCY.0 / presc,
    };

    let mul = pll_multiplier(pllmull);
    let half = matches!(
        pllmull,
        Pllmul::MUL4 | Pllmul::MUL6 | Pllmul::MUL8 | Pllmul::MUL10 | Pllmul::MUL12
    );
    let hz = if half {
        (ref_hz * mul) / 2
    } else {
        ref_hz * mul
    };
    Hertz(hz)
}

fn pll_multiplier(mul: Pllmul) -> u32 {
    let idx = mul as u8 as usize;
    PLL_MUL_TABLE.get(idx).copied().unwrap_or(16) as u32
}

fn serdes_pll_hz(hse: Option<Hertz>) -> u32 {
    let hse = hse.expect("RCC: HSE frequency required for SERDES PLL").0;
    let idx = RCC.pllcfgr2().read().serdespll_mul() as u8 as usize;
    let mul = SERDES_MUL_TABLE.get(idx).copied().unwrap_or(25) as u32;
    hse * mul
}

fn serdes_sysclk(hse: Option<Hertz>) -> Hertz {
    Hertz(serdes_pll_hz(hse) / 2)
}
