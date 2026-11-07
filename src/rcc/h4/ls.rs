//! LSE / LSI / RTC clock setup (RCC_BDCTLR).

use super::LSI_FREQUENCY;
use crate::pac::rcc::vals::Rtcsel;
use crate::pac::RCC;
use crate::rcc::{LsConfig, LseMode, RtcClockSource};
use crate::time::Hertz;

pub(crate) unsafe fn init_ls(config: &LsConfig) -> Option<Hertz> {
    // LSEON, LSEBYP, RTCSEL and RTCEN live in the write-protected backup
    // domain: writes are dropped until `PWR.CTLR.DBP` is set, and PWR itself
    // needs its HB1 clock (RM §3.4.10).
    RCC.hb1pcenr().modify(|w| w.set_pwren(true));
    crate::pac::PWR.ctlr().modify(|w| w.set_dbp(true));

    if config.lsi {
        RCC.rstsckr().modify(|w| w.set_lsion(true));
        while !RCC.rstsckr().read().lsirdy() {}
    }

    let lse_hz = match &config.lse {
        Some(lse) => {
            // One write: a second read-modify-write can read back a stale
            // `BDCTLR` and drop `LSEBYP` again.
            RCC.bdctlr().modify(|w| {
                w.set_lsebyp(lse.mode == LseMode::Bypass);
                w.set_lseon(true);
            });
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

    // Never retarget a running RTC: its counter would silently change time
    // base while the prescaler keeps the old divider.
    if !RCC.bdctlr().read().rtcen() || rtcsel == Rtcsel::NO_CLK {
        RCC.bdctlr().modify(|w| w.set_rtcsel(rtcsel));
    }

    match rtcsel {
        Rtcsel::LSE => lse_hz,
        Rtcsel::LSI => Some(LSI_FREQUENCY),
        Rtcsel::HSE_DIV512 => None,
        Rtcsel::NO_CLK => None,
    }
}
