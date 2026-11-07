//! Real-time clock (RTC).
//!
//! A 32-bit seconds counter with a 20-bit prescaler and one alarm compare
//! register (RM §6.1). The counter and prescaler sit in the backup domain, so
//! they keep running across system resets while VBAT is present and dropping
//! the handle does not stop the clock (RM §6.2.2).
//!
//! [`Rtc::new`] enables the requested oscillator, selects it as `RTCCLK` and
//! writes the prescaler for a 1 Hz time base (RM §3.3.5.3). If the RTC is
//! already running from the same source, the elapsed time is preserved.
//!
//! Register writes go through the configuration sequence of RM §6.2.3: poll
//! `CTLRL.RTOFF`, set `CNF`, write, clear `CNF`, poll `RTOFF`.

use crate::pac::rcc::vals::Rtcsel;
use crate::time::Hertz;
use crate::{pac, peripherals, Peri};

/// Source of the 1 Hz RTC time base.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ClockSource {
    /// Internal ~40 kHz RC oscillator. Always available, but its frequency is
    /// only nominal, so expect drift of a few percent.
    Lsi,
    /// External 32.768 kHz crystal on the OSC32 pins, or a square wave fed
    /// into OSC32_IN when `bypass` is set.
    Lse { frequency: Hertz, bypass: bool },
    /// HSE divided by 512. The HSE tree must already be running.
    HseDiv512 { frequency: Hertz },
}

/// RTC configuration error.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The RTC is already running from another source. Retargeting it would
    /// silently change the time base, so the constructor refuses to do so.
    ClockSourceMismatch,
    /// `frequency - 1` does not fit the 20-bit prescaler.
    InvalidFrequency(Hertz),
}

impl ClockSource {
    fn rtcsel(self) -> Rtcsel {
        match self {
            Self::Lsi => Rtcsel::LSI,
            Self::Lse { .. } => Rtcsel::LSE,
            Self::HseDiv512 { .. } => Rtcsel::HSE_DIV512,
        }
    }

    /// Clock frequency fed to the prescaler.
    fn frequency(self) -> Hertz {
        match self {
            Self::Lsi => crate::rcc::LSI_FREQ,
            Self::Lse { frequency, .. } => frequency,
            Self::HseDiv512 { frequency } => frequency / 512u32,
        }
    }

    /// Start the oscillator and wait until it is ready.
    fn enable(self) {
        let rcc = pac::RCC;
        match self {
            Self::Lsi => {
                // LSION lives in RSTSCKR, outside the write-protected backup
                // block (RM §3.4.11).
                rcc.rstsckr().modify(|w| w.set_lsion(true));
                while !rcc.rstsckr().read().lsirdy() {}
            }
            Self::Lse { bypass, .. } => {
                // `LSEBYP` may only change while `LSEON` is clear (RM §3.4.10).
                // Leave a running oscillator alone, including its bypass mode.
                if !rcc.bdctlr().read().lseon() {
                    // One write: a second read-modify-write can read back a
                    // stale `BDCTLR` and drop this bit again.
                    rcc.bdctlr().modify(|w| {
                        w.set_lsebyp(bypass);
                        w.set_lseon(true);
                    });
                }
                while !rcc.bdctlr().read().lserdy() {}
            }
            Self::HseDiv512 { .. } => {}
        }
    }
}

/// RTC handle. The clock keeps counting after this is dropped.
pub struct Rtc<'d> {
    _peri: Peri<'d, peripherals::RTC>,
    frequency: Hertz,
}

impl<'d> Rtc<'d> {
    /// Enable the RTC from `source` with a 1 Hz time base.
    ///
    /// Returns an error before changing `RTCCLK` when the divider does not
    /// fit or the RTC already runs from another source.
    pub fn new(peri: Peri<'d, peripherals::RTC>, source: ClockSource) -> Result<Self, Error> {
        let frequency = source.frequency();
        let divider = match frequency.0.checked_sub(1) {
            Some(divider) if divider <= 0x000F_FFFF => divider,
            _ => return Err(Error::InvalidFrequency(frequency)),
        };

        // PWR is clocked from HB1, and the backup domain ignores writes until
        // `PWR.CTLR.DBP` is set (RM §3.4.10).
        pac::RCC.hb1pcenr().modify(|w| w.set_pwren(true));
        pac::PWR.ctlr().modify(|w| w.set_dbp(true));

        let bdctlr = pac::RCC.bdctlr();
        let rtcsel = source.rtcsel();
        let current = bdctlr.read().rtcsel();
        // `RTCEN` can be left set with `RTCSEL = NO_CLK`; that is not a
        // running clock, so it is safe to pick a source again.
        let running = bdctlr.read().rtcen() && current != Rtcsel::NO_CLK;
        if running && current != rtcsel {
            return Err(Error::ClockSourceMismatch);
        }

        source.enable();

        if !running {
            // Select the source and enable the clock in one write. Split
            // read-modify-writes race the backup-domain update: the second
            // read can return the old `RTCSEL` and clear it again.
            bdctlr.modify(|w| {
                w.set_rtcsel(rtcsel);
                w.set_rtcen(true);
            });
        }

        let rtc = Self { _peri: peri, frequency };
        // First access after a reset must re-acquire the register shadow
        // (RM §6.3.2). Then program the divider; the counter itself is kept.
        rtc.synchronize();
        rtc.write_prescaler(divider);
        Ok(rtc)
    }

    /// Clock frequency fed into the prescaler.
    pub fn frequency(&self) -> Hertz {
        self.frequency
    }

    /// Re-acquire the register shadow after Stop-mode wake-up or an HB reset.
    ///
    /// Clearing `RSF` and waiting for hardware to set it again makes the
    /// prescaler, alarm and counter reads current (RM §6.3.2).
    pub fn synchronize(&self) {
        let ctlrl = pac::RTC.ctlrl();
        ctlrl.modify(|w| w.set_rsf(false));
        while !ctlrl.read().rsf() {}
    }

    /// Current counter value in seconds. The epoch is whatever the user set.
    pub fn get_time(&self) -> u32 {
        let rtc = pac::RTC;
        // CNTH changes once per 65536 s, but re-read it so a wrap between the
        // two accesses cannot produce a torn value.
        loop {
            let high = rtc.cnth().read().cnth();
            let low = rtc.cntl().read().cntl();
            if rtc.cnth().read().cnth() == high {
                return (u32::from(high) << 16) | u32::from(low);
            }
        }
    }

    /// Write the counter. Takes effect on the next prescaler tick.
    pub fn set_time(&self, seconds: u32) {
        let rtc = pac::RTC;
        self.enter_config();
        // The low word write commits the pair.
        rtc.cnth().write(|w| w.set_cnth((seconds >> 16) as u16));
        rtc.cntl().write(|w| w.set_cntl(seconds as u16));
        self.exit_config();
    }

    /// Set the alarm compare value.
    ///
    /// A pending `ALRF` is cleared first so a stale match cannot fire the
    /// interrupt for the new value.
    pub fn set_alarm(&self, seconds: u32) {
        let rtc = pac::RTC;
        self.clear_alarm_flag();
        self.enter_config();
        rtc.alrmh().write(|w| w.set_alrh((seconds >> 16) as u16));
        rtc.alrml().write(|w| w.set_alrl(seconds as u16));
        self.exit_config();
    }

    /// Current prescaler counter. It counts down from `frequency - 1` to 0;
    /// two reads one tick apart differ while `RTCCLK` is alive.
    pub fn read_prescaler(&self) -> u32 {
        let rtc = pac::RTC;
        (u32::from(rtc.divh().read().divh()) << 16) | u32::from(rtc.divl().read().divl())
    }

    /// Second-event interrupt on the `RTC` vector.
    pub fn enable_second_interrupt(&self, enable: bool) {
        pac::RTC.ctlrh().modify(|w| w.set_secie(enable));
    }

    /// Alarm interrupt on the `RTCALARM` vector.
    pub fn enable_alarm_interrupt(&self, enable: bool) {
        pac::RTC.ctlrh().modify(|w| w.set_alrie(enable));
    }

    /// Overflow interrupt on the `RTC` vector.
    pub fn enable_overflow_interrupt(&self, enable: bool) {
        pac::RTC.ctlrh().modify(|w| w.set_owie(enable));
    }

    /// A second elapsed since the last clear.
    pub fn second_flag(&self) -> bool {
        pac::RTC.ctlrl().read().secf()
    }

    /// The counter reached the alarm value.
    pub fn alarm_flag(&self) -> bool {
        pac::RTC.ctlrl().read().alrf()
    }

    /// The 32-bit counter wrapped.
    pub fn overflow_flag(&self) -> bool {
        pac::RTC.ctlrl().read().owf()
    }

    /// Clear the second-event flag.
    pub fn clear_second_flag(&self) {
        pac::RTC.ctlrl().modify(|w| w.set_secf(false));
    }

    /// Clear the alarm flag.
    pub fn clear_alarm_flag(&self) {
        pac::RTC.ctlrl().modify(|w| w.set_alrf(false));
    }

    /// Clear the overflow flag.
    pub fn clear_overflow_flag(&self) {
        pac::RTC.ctlrl().modify(|w| w.set_owf(false));
    }

    /// Clear `SECF`, `ALRF` and `OWF` in one write.
    pub fn clear_flags(&self) {
        pac::RTC.ctlrl().modify(|w| {
            w.set_secf(false);
            w.set_alrf(false);
            w.set_owf(false);
        });
    }

    fn write_prescaler(&self, divider: u32) {
        self.enter_config();
        // High word first; the low word commits the divider.
        pac::RTC.pscrh().write(|w| w.set_prlh((divider >> 16) as u8));
        pac::RTC.pscrl().write(|w| w.set_prll(divider as u16));
        self.exit_config();
    }

    fn enter_config(&self) {
        let ctlrl = pac::RTC.ctlrl();
        while !ctlrl.read().rtoff() {}
        ctlrl.modify(|w| w.set_cnf(true));
    }

    fn exit_config(&self) {
        let ctlrl = pac::RTC.ctlrl();
        ctlrl.modify(|w| w.set_cnf(false));
        while !ctlrl.read().rtoff() {}
    }
}
