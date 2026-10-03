//! LSE / LSI / RTC clock setup (RCC_BDCTLR).

use crate::pac::rcc::vals::Rtcsel;
use crate::pac::RCC;
use crate::time::Hertz;

use super::LSI_FREQUENCY;
use crate::rcc::{LseMode, LsConfig, RtcClockSource};

pub(crate) unsafe fn init_ls(config: &LsConfig) -> Option<Hertz> {
    if config.lsi {
        RCC.rstsckr().modify(|w| w.set_lsion(true));
        while !RCC.rstsckr().read().lsirdy() {}
    }

    let lse_hz = match &config.lse {
        Some(lse) => {
            RCC.bdctlr().modify(|w| w.set_lsebyp(lse.mode == LseMode::Bypass));
            RCC.bdctlr().modify(|w| w.set_lseon(true));
            while !RCC.bdctlr().read().lserdy() {}
            Some(lse.frequency)
        }
        None => None,
    };

    let rtcsel = match config.rtc {
        RtcClockSource::DISABLE => Rtcsel::NO_CLK,
        RtcClockSource::LSE => {
            if lse_hz.is_none() {
                panic!("RCC: RTC clock source LSE requires `Config.ls.lse`");
            }
            Rtcsel::LSE
        }
        RtcClockSource::LSI => {
            if !config.lsi {
                panic!("RCC: RTC clock source LSI requires `Config.ls.lsi = true`");
            }
            Rtcsel::LSI
        }
        RtcClockSource::HSE => Rtcsel::HSE_DIV512,
    };

    RCC.bdctlr().modify(|w| w.set_rtcsel(rtcsel));

    match rtcsel {
        Rtcsel::LSE => lse_hz,
        Rtcsel::LSI => Some(LSI_FREQUENCY),
        Rtcsel::HSE_DIV512 => None,
        Rtcsel::NO_CLK => None,
    }
}
