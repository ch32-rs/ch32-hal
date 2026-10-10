//! Default-HSE RCC and SysTick rounding regression check (V3F only).
//!
//! `cargo xtask flash --example rcc_delay_check && cargo xtask report`
//! Mailbox: marker 1 = entered init, 2 = init returned, 3 = checks passed;
//! cas_total = failed checks, ping/pong = V3F/V5F Hz, dualcore_counter =
//! one-second heartbeat. No hart 1 or SDI console is needed.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use embedded_hal::delay::DelayNs;
use panic_halt as _;

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = ipc::mailbox();
    mailbox.clear();
    mailbox.dualcore_marker.store(1, Relaxed);

    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });
    mailbox.dualcore_marker.store(2, Relaxed);
    let clocks = *hal::rcc::clocks();
    mailbox.ping.store(clocks.v3f.0, Relaxed);
    mailbox.pong.store(clocks.v5f.0, Relaxed);

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
    mailbox.cas_total.store(failures, Relaxed);
    if failures == 0 {
        mailbox.dualcore_marker.store(3, Relaxed);
    }

    let mut ticks = 0u32;
    loop {
        delay.delay_ms(1000);
        ticks = ticks.wrapping_add(1);
        mailbox.dualcore_counter.store(ticks, Relaxed);
    }
}
