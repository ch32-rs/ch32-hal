//! LSE / LSI / RTC clock setup (RCC_BDCTLR).

use crate::pac::rcc::vals::Rtcsel;
use crate::pac::RCC;
use crate::rcc::{LsConfig, LseMode, RtcClockSource, LSI_FREQ};
use crate::time::Hertz;

/// `RTCSEL` value for the divided HSE: /512 on CH32H4; /128 or /512 on
/// V2/V3, depending on the part (RCC_BDCTLR).
#[cfg(rcc_h4)]
const RTCSEL_HSE: Rtcsel = Rtcsel::HSE_DIV512;
#[cfg(pwr_v3)]
const RTCSEL_HSE: Rtcsel = Rtcsel::HSE;

/// Allow writes to the backup domain (`RCC_BDCTLR`, RTC, BKP).
///
/// LSEON, LSEBYP, RTCSEL and RTCEN live in the write-protected backup
/// domain: writes are dropped until `PWR.CTLR.DBP` is set, and PWR itself
/// needs its bus clock (RCC_BDCTLR in the reference manuals).
pub(crate) fn enable_backup_domain_writes() {
    #[cfg(rcc_h4)]
    RCC.hb1pcenr().modify(|w| w.set_pwren(true));
    #[cfg(pwr_v3)]
    RCC.apb1pcenr().modify(|w| w.set_pwren(true));
    crate::pac::PWR.ctlr().modify(|w| w.set_dbp(true));
}

pub(crate) unsafe fn init_ls(config: &LsConfig) -> Option<Hertz> {
    enable_backup_domain_writes();

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
        RtcClockSource::HSE => RTCSEL_HSE,
    };

    // Never retarget a running RTC: its counter would silently change time
    // base while the prescaler keeps the old divider.
    if !RCC.bdctlr().read().rtcen() || rtcsel == Rtcsel::NO_CLK {
        RCC.bdctlr().modify(|w| w.set_rtcsel(rtcsel));
    }

    match rtcsel {
        Rtcsel::LSE => lse_hz,
        Rtcsel::LSI => Some(LSI_FREQ),
        _ => None,
    }
}
