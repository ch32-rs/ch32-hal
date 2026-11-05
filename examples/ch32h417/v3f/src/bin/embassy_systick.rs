//! Embassy SysTick time driver against a free-running TIM2, V3F only.
//!
//! `cargo xtask flash --example embassy_systick && cargo xtask report --example embassy_systick`
//!
//! `stage` reaches 2 once an Embassy 100 ms wait measures within 1 ms of
//! 100 ms on TIM2, which runs from the timer kernel clock and not from
//! SysTick. `error_us` is that measurement minus 100 ms, and `ticks` then
//! advances once a second. No hart 1 or console is needed.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::EmbassySystick;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};

ipc::mailbox!(EmbassySystick);
use panic_halt as _;

/// TIM2 counts at 1 MHz, so one count is one microsecond.
fn start_reference(timer_hz: u32) {
    let tim = hal::pac::TIM2;
    let rcc = hal::pac::RCC;
    rcc.hb1pcenr().modify(|w| w.set_tim2en(true));
    rcc.hb1prstr().modify(|w| w.set_tim2rst(true));
    rcc.hb1prstr().modify(|w| w.set_tim2rst(false));
    tim.ctlr1().modify(|w| w.set_cen(false));
    tim.psc().write_value((timer_hz / 1_000_000 - 1) as u16);
    tim.atrlr().write_value(u16::MAX);
    tim.swevgr().write(|w| w.set_ug(true));
    tim.intfr().modify(|w| w.set_uif(false));
    tim.cnt().write_value(0);
    tim.ctlr1().modify(|w| w.set_cen(true));
}

/// Microseconds since `start_reference`, across TIM2's 16-bit wraps.
fn reference_us() -> u64 {
    let tim = hal::pac::TIM2;
    let mut periods = 0u64;
    loop {
        let high = tim.intfr().read().uif();
        let count = tim.cnt().read();
        if tim.intfr().read().uif() == high {
            if high {
                tim.intfr().modify(|w| w.set_uif(false));
                periods += 1;
            }
            return periods * 65_536 + u64::from(count);
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mailbox = mailbox();
    mailbox.clear();

    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });
    // TIM2 is on the V3F/HCLK domain. The 400/400/100 recipe leaves that at
    // 100 MHz, which divides to the 1 MHz reference exactly.
    start_reference(hal::rcc::clocks().hclk.0);
    mailbox.payload.stage.store(1, Relaxed);

    let start = Instant::now();
    let before = reference_us();
    Timer::at(start + embassy_time::Duration::from_millis(100)).await;
    let elapsed = reference_us() - before;
    let error = elapsed.abs_diff(100_000);
    mailbox.payload.error_us.store(error as u32, Relaxed);
    if error <= 1_000 {
        mailbox.payload.stage.store(2, Relaxed);
    }

    let mut ticks = 0u32;
    loop {
        Timer::after_millis(1000).await;
        ticks = ticks.wrapping_add(1);
        mailbox.payload.ticks.store(ticks, Relaxed);
    }
}
