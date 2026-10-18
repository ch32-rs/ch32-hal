//! Default-HSE RCC and SysTick rounding regression check (V3F only).
//!
//! `cargo xtask flash --example rcc_delay_check && cargo xtask report --example rcc_delay_check`
//! Mailbox (`RccDelayCheck`): stage 1 = entered init, 2 = init returned, 3 =
//! checks passed; failures = failed checks, v3f_hz/v5f_hz = the reported core
//! frequencies, heartbeat = one-second ticks. No hart 1 or SDI console is
//! needed.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::RccDelayCheck;
use embedded_hal::delay::DelayNs;

ipc::mailbox!(RccDelayCheck);
use panic_halt as _;

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });
    mailbox.payload.stage.store(2, Relaxed);
    let clocks = *hal::rcc::clocks();
    mailbox.payload.v3f_hz.store(clocks.v3f.0, Relaxed);
    mailbox.payload.v5f_hz.store(clocks.v5f.0, Relaxed);

    let mut failures = u32::from(clocks.v3f.0 != 100_000_000) + u32::from(clocks.v5f.0 != 400_000_000);
    let mut delay = hal::delay::Delay;
    delay.delay_ns(11);
    failures += u32::from(hal::pac::SYSTICK.cmp_0().read().cmp() != 2);
    delay.delay_ns(0);
    failures += u32::from(hal::pac::SYSTICK.cmp_0().read().cmp() != 2);
    delay.delay_us(1);
    failures += u32::from(hal::pac::SYSTICK.cmp_0().read().cmp() != 100);
    delay.delay_ms(1);
    failures += u32::from(hal::pac::SYSTICK.cmp_0().read().cmp() != 100_000);

    // Refresh reads RCC without changing the PLL or the effective frequencies.
    unsafe { hal::rcc::refresh(Some(hal::time::Hertz(25_000_000))) };
    failures += u32::from(*hal::rcc::clocks() != clocks);
    mailbox.payload.failures.store(failures, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    let mut ticks = 0u32;
    loop {
        delay.delay_ms(1000);
        ticks = ticks.wrapping_add(1);
        mailbox.payload.heartbeat.store(ticks, Relaxed);
    }
}
