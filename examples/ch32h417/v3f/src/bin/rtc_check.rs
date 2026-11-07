//! RTC self-check: LSI time base, second and alarm interrupts (V3F only).
//!
//! Builds the RTC from LSI, sets the counter to zero and the alarm to four
//! seconds, then waits six seconds on the SysTick delay. The `RTC` handler
//! counts second events and the `RTCALARM` handler counts alarm matches, so a
//! vector wired to the wrong side, a missing flag clear or a prescaler that
//! does not divide to 1 Hz all show up in the mailbox.
//!
//! The driver also refuses two invalid constructions: a running RTC cannot be
//! retargeted to LSE, and a divider that does not fit 20 bits is rejected.
//! Re-opening the RTC from the same source must preserve the elapsed time.
//!
//! Finally the LSE oscillator is probed with a two-second bound. If the board
//! has the 32.768 kHz crystal, the backup domain is reset and the RTC is run
//! from LSE for another two seconds; without it, `lse = 0` is reported and the
//! oscillator is switched back off.

#![no_std]
#![no_main]

use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::RtcCheck;
use hal::bind_interrupts;
use hal::delay::Delay;
use hal::interrupt::typelevel::{Handler, Interrupt};
use hal::rtc::{ClockSource, Error, Rtc};
use hal::time::Hertz;

ipc::mailbox!(RtcCheck);
use panic_halt as _;

static SECONDS: AtomicU32 = AtomicU32::new(0);
static ALARMS: AtomicU32 = AtomicU32::new(0);

struct RtcHandler;

/// Service every pending event, whichever vector delivered it. Clearing both
/// flags in both handlers avoids an interrupt storm if the H4 routes one of
/// the events to the other vector than the YAML says.
unsafe fn service() {
    let ctlrl = hal::pac::RTC.ctlrl();
    if ctlrl.read().secf() {
        ctlrl.modify(|w| w.set_secf(false));
        SECONDS.fetch_add(1, Relaxed);
    }
    if ctlrl.read().alrf() {
        ctlrl.modify(|w| w.set_alrf(false));
        ALARMS.fetch_add(1, Relaxed);
    }
    if ctlrl.read().owf() {
        ctlrl.modify(|w| w.set_owf(false));
    }
}

impl Handler<hal::interrupt::typelevel::RTC> for RtcHandler {
    unsafe fn on_interrupt() {
        service();
    }
}

impl Handler<hal::interrupt::typelevel::RTCALARM> for RtcHandler {
    unsafe fn on_interrupt() {
        service();
    }
}

bind_interrupts!(struct Irqs {
    RTC => RtcHandler;
    RTCALARM => RtcHandler;
});

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut p = hal::init(hal::Config::default());
    let mut delay = Delay;
    let mut checks = 0u32;
    let mut failures = 0u32;

    // Start from a known source, so a retained run from another one does not
    // trip the driver's retargeting guard and the check stays re-runnable.
    hal::pac::RCC.hb1pcenr().modify(|w| w.set_pwren(true));
    hal::pac::PWR.ctlr().modify(|w| w.set_dbp(true));
    let bdctlr = hal::pac::RCC.bdctlr();
    bdctlr.modify(|w| w.set_bdrst(true));
    bdctlr.modify(|w| w.set_bdrst(false));
    delay.delay_ms(1);

    let rtc = match Rtc::new(p.RTC.reborrow(), ClockSource::Lsi) {
        Ok(rtc) => rtc,
        Err(_) => {
            mailbox.payload.stage.store(2, Relaxed);
            mailbox.payload.failures.store(1, Relaxed);
            loop {
                delay.delay_ms(1000);
            }
        }
    };
    let _irqs = Irqs;
    mailbox.payload.frequency.store(rtc.frequency().0, Relaxed);
    check(&mut failures, &mut checks, rtc.frequency() == hal::rcc::LSI_FREQ);

    rtc.clear_flags();
    rtc.set_time(0);
    rtc.set_alarm(4);
    rtc.enable_second_interrupt(true);
    rtc.enable_alarm_interrupt(true);
    unsafe {
        hal::interrupt::typelevel::RTC::unpend();
        hal::interrupt::typelevel::RTC::enable();
        hal::interrupt::typelevel::RTCALARM::unpend();
        hal::interrupt::typelevel::RTCALARM::enable();
    }

    // The divider counts down from `frequency - 1`; two reads a few
    // milliseconds apart differ while RTCCLK is alive.
    let before = rtc.read_prescaler();
    delay.delay_ms(5);
    let after = rtc.read_prescaler();
    mailbox.payload.prescaler.store(after, Relaxed);
    check(&mut failures, &mut checks, before < rtc.frequency().0);
    check(&mut failures, &mut checks, after < rtc.frequency().0);
    check(&mut failures, &mut checks, before != after);

    // Six seconds at 1 Hz: the alarm at 4 s fires once, the counter reaches
    // 5 or 6 depending on where the prescaler was when it was written.
    delay.delay_ms(6000);

    let time = rtc.get_time();
    let seconds = SECONDS.load(Relaxed);
    let alarms = ALARMS.load(Relaxed);
    let elapsed = time;
    check(&mut failures, &mut checks, (5..=7).contains(&elapsed));
    check(&mut failures, &mut checks, seconds.abs_diff(elapsed) <= 1);
    check(&mut failures, &mut checks, alarms == 1);
    check(&mut failures, &mut checks, !rtc.alarm_flag());

    rtc.enable_second_interrupt(false);
    rtc.enable_alarm_interrupt(false);

    // A running RTC must not be retargeted, and an impossible divider must be
    // rejected before any hardware is touched.
    match Rtc::new(
        p.RTC.reborrow(),
        ClockSource::Lse {
            frequency: Hertz(32_768),
            bypass: false,
        },
    ) {
        Err(Error::ClockSourceMismatch) => check(&mut failures, &mut checks, true),
        _ => check(&mut failures, &mut checks, false),
    }
    match Rtc::new(p.RTC.reborrow(), ClockSource::HseDiv512 { frequency: Hertz(0) }) {
        Err(Error::InvalidFrequency(_)) => check(&mut failures, &mut checks, true),
        _ => check(&mut failures, &mut checks, false),
    }

    // Reusing the same source keeps the counter; only the divider restarts.
    match Rtc::new(p.RTC.reborrow(), ClockSource::Lsi) {
        Ok(rtc) => {
            let reopened = rtc.get_time();
            check(&mut failures, &mut checks, reopened >= time && reopened <= time + 2);
            rtc.enable_second_interrupt(false);
            rtc.enable_alarm_interrupt(false);
        }
        Err(_) => check(&mut failures, &mut checks, false),
    }

    // Second time base: probe the LSE oscillator with a bounded wait, so a
    // board without the 32.768 kHz crystal still reports. Switching source
    // needs a backup-domain reset, which the driver deliberately refuses to
    // do on its own.
    let bdctlr = hal::pac::RCC.bdctlr();
    bdctlr.modify(|w| {
        w.set_lsebyp(false);
        w.set_lseon(true);
    });
    let mut lse_wait_ms = 0u32;
    while !bdctlr.read().lserdy() {
        delay.delay_ms(1);
        lse_wait_ms += 1;
        if lse_wait_ms >= 2000 {
            break;
        }
    }
    let mut lse = 0u32;
    if bdctlr.read().lserdy() {
        check(&mut failures, &mut checks, true);
        bdctlr.modify(|w| w.set_bdrst(true));
        bdctlr.modify(|w| w.set_bdrst(false));
        // Let the backup-domain reset settle before the driver reads it.
        delay.delay_ms(1);
        match Rtc::new(
            p.RTC.reborrow(),
            ClockSource::Lse {
                frequency: Hertz(32_768),
                bypass: false,
            },
        ) {
            Ok(rtc) => {
                rtc.set_time(0);
                delay.delay_ms(2000);
                let elapsed = rtc.get_time();
                check(&mut failures, &mut checks, (1..=3).contains(&elapsed));
                if elapsed != 0 {
                    lse = 1;
                }
                rtc.enable_second_interrupt(false);
                rtc.enable_alarm_interrupt(false);
            }
            Err(_) => check(&mut failures, &mut checks, false),
        }
    } else {
        // Leave the oscillator off when it never started.
        bdctlr.modify(|w| w.set_lseon(false));
    }

    mailbox.payload.time.store(time, Relaxed);
    mailbox.payload.seconds.store(seconds, Relaxed);
    mailbox.payload.alarms.store(alarms, Relaxed);
    mailbox.payload.lse.store(lse, Relaxed);
    mailbox.payload.lse_wait_ms.store(lse_wait_ms, Relaxed);
    mailbox
        .payload
        .rtcsel
        .store(u32::from(bdctlr.read().rtcsel().to_bits()), Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    loop {
        delay.delay_ms(1000);
    }
}
