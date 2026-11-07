//! RTC self-check on the HSE/512 time base (V3F only).
//!
//! `cargo xtask flash --example rtc_hse_check && cargo xtask report --example rtc_hse_check`
//!
//! `rtc_check` covers LSI and LSE with the default HSI clock setup. This boots
//! the HSE recipe and selects `RTCCLK = HSE / 512`, which is the last of the
//! three sources and the only one that depends on the main oscillator tree
//! being up before the RTC constructor runs.
//!
//! The backup domain is reset first so a previous run from another source
//! does not trip the driver's retargeting guard; the check is re-runnable
//! without a power cycle.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::RtcHseCheck;
use hal::delay::Delay;
use hal::rtc::{ClockSource, Rtc};
use hal::time::Hertz;

ipc::mailbox!(RtcHseCheck);
use panic_halt as _;

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });
    let mut delay = Delay;
    let mut checks = 0u32;
    let mut failures = 0u32;

    // Clear the backup domain so a retained LSI/LSE configuration cannot
    // block the source switch. `DBP` must be set first (RM §3.4.10).
    hal::pac::RCC.hb1pcenr().modify(|w| w.set_pwren(true));
    hal::pac::PWR.ctlr().modify(|w| w.set_dbp(true));
    let bdctlr = hal::pac::RCC.bdctlr();
    bdctlr.modify(|w| w.set_bdrst(true));
    bdctlr.modify(|w| w.set_bdrst(false));
    // Let the backup-domain reset settle before the driver reads it.
    delay.delay_ms(1);

    let rtc = match Rtc::new(
        p.RTC.reborrow(),
        ClockSource::HseDiv512 {
            frequency: hal::rcc::HSE_FREQUENCY_25M,
        },
    ) {
        Ok(rtc) => rtc,
        Err(_) => {
            mailbox.payload.stage.store(2, Relaxed);
            mailbox.payload.failures.store(1, Relaxed);
            loop {
                delay.delay_ms(1000);
            }
        }
    };

    // 25 MHz / 512 = 48828.125 Hz, truncated to 48828 by the divider.
    let expected = Hertz(48_828);
    check(&mut failures, &mut checks, rtc.frequency() == expected);

    rtc.clear_flags();
    rtc.set_time(0);
    delay.delay_ms(3000);

    let time = rtc.get_time();
    let prescaler = rtc.read_prescaler();
    mailbox.payload.frequency.store(rtc.frequency().0, Relaxed);
    mailbox.payload.prescaler.store(prescaler, Relaxed);
    check(&mut failures, &mut checks, (2..=4).contains(&time));
    check(&mut failures, &mut checks, prescaler < expected.0);

    mailbox.payload.time.store(time, Relaxed);
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
