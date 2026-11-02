#![no_std]
#![no_main]

use core::cell::RefCell;
use core::future::{poll_fn, Future};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Poll, Waker};

use ch32_hal::pac;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer, TICK_HZ};
use panic_halt as _;

#[no_mangle]
pub static mut SYSTICK_STRESS_RESULT: u32 = 0;
#[no_mangle]
pub static mut SYSTICK_STRESS_STATS: [u32; 5] = [0; 5];
#[no_mangle]
pub static mut NATURAL_WRAP_STATS: [u32; 4] = [0; 4];
static REFERENCE_PERIODS: AtomicU32 = AtomicU32::new(0);
static WATCHDOG: critical_section::Mutex<RefCell<Option<Waker>>> = critical_section::Mutex::new(RefCell::new(None));

fn check(condition: bool, code: u32) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SYSTICK_STRESS_RESULT), code) };
        defmt::panic!("SYSTICK_STRESS_RESULT=FAIL code={}", code);
    }
}

#[qingke_rt::interrupt]
fn TIM2() {
    pac::TIM2.intfr().write(|register| register.set_uif(false));
    REFERENCE_PERIODS.store(REFERENCE_PERIODS.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
    critical_section::with(|section| {
        if let Some(waker) = WATCHDOG.borrow(section).borrow().as_ref() {
            waker.wake_by_ref();
        }
    });
}

fn reference_us() -> u64 {
    critical_section::with(|_| {
        let mut periods = REFERENCE_PERIODS.load(Ordering::Relaxed);
        let mut count = pac::TIM2.cnt().read();
        if pac::TIM2.intfr().read().uif() {
            periods += 1;
            count = pac::TIM2.cnt().read();
        }
        u64::from(periods) * 65_536 + u64::from(count)
    })
}

fn start_reference() {
    let timer_hz = ch32_hal::rcc::clocks().pclk1.0 * if cfg!(feature = "clock-dividers") { 2 } else { 1 };
    check(timer_hz % 1_000_000 == 0, 10);
    pac::RCC.pb1pcenr().modify(|register| register.set_tim2en(true));
    pac::RCC.pb1prstr().modify(|register| register.set_tim2rst(true));
    pac::RCC.pb1prstr().modify(|register| register.set_tim2rst(false));
    pac::TIM2.ctlr1().write(|register| register.set_cen(false));
    pac::TIM2.psc().write_value((timer_hz / 1_000_000 - 1) as u16);
    pac::TIM2.atrlr().write_value(u16::MAX);
    pac::TIM2.swevgr().write(|register| register.set_ug(true));
    pac::TIM2.intfr().write(|register| register.set_uif(false));
    pac::TIM2.dmaintenr().write(|register| register.set_uie(true));
    unsafe {
        qingke::pfic::unpend_interrupt(pac::Interrupt::TIM2 as u8);
        qingke::pfic::enable_interrupt(pac::Interrupt::TIM2 as u8);
    }
    pac::TIM2.ctlr1().modify(|register| register.set_cen(true));
    defmt::info!("TIM2 reference: timer_hz={} counter_hz=1000000", timer_hz);
}

async fn natural_wrap(counter_hz: u64) {
    let reference_start = reference_us();
    let start = Instant::now();
    let wait_us = ((1u64 << 32) * 1_000_000).div_ceil(counter_hz) + 100_000;
    let mut timer = pin!(Timer::at(start + Duration::from_micros(wait_us)));
    let mut armed = false;
    let mut saw_end = false;
    let mut rearmed = false;
    defmt::info!("Natural wrap starting: wait_us={}", wait_us);
    poll_fn(|context| {
        if !armed {
            check(timer.as_mut().poll(context).is_pending(), 18);
            armed = true;
            return Poll::Pending;
        }
        let compare = pac::SYSTICK.cmpl().read();
        saw_end |= compare == u32::MAX;
        rearmed |= saw_end && compare != u32::MAX;
        let elapsed = reference_us() - reference_start;
        check(elapsed < wait_us + 1_000_000, 18);
        if elapsed < wait_us {
            return Poll::Pending;
        }
        check(rearmed, 18);
        timer.as_mut().poll(context)
    })
    .await;
    critical_section::with(|_| {
        let epoch = 513u64 << 32;
        let before = (epoch | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
        let now = Instant::now().as_ticks();
        let after = (epoch | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
        check(before <= now && now <= after, 18);
    });
    let elapsed = reference_us() - reference_start;
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(NATURAL_WRAP_STATS),
            [wait_us as u32, elapsed as u32, 513, rearmed as u32],
        );
    }
    defmt::info!("Natural wrap PASS reference_us={}", elapsed);
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SYSTICK_STRESS_RESULT), 1) };
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = ch32_hal::rcc::Sysclk::PLL;
        config.rcc.pll = Some(ch32_hal::rcc::Pll {
            mul: ch32_hal::rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = ch32_hal::rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.ahb_pre = ch32_hal::rcc::AHBPrescaler::DIV2;
        config.rcc.apb1_pre = ch32_hal::rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = ch32_hal::rcc::APBPrescaler::DIV2;
    }
    let _peripherals = ch32_hal::init(config);
    let counter_hz = u64::from(ch32_hal::rcc::clocks().hclk.0 / 8);
    start_reference();
    Timer::after_millis(100).await;
    let mut min_elapsed = u64::MAX;
    let mut max_elapsed = 0;
    for round in 0..512u32 {
        let masked_us = [0, 50, 500, 2_000][round as usize % 4];
        let remaining = (counter_hz / 1_000_000 * 10 + u64::from(round % 17)) as u32;
        let start = critical_section::with(|_| {
            pac::SYSTICK.ctlr().modify(|register| {
                register.set_ste(false);
                register.set_stie(false);
            });
            pac::SYSTICK.cntl().write_value(u32::MAX - remaining);
            Instant::now()
        });
        let deadline = start + Duration::from_millis(3);
        let mut timer = pin!(Timer::at(deadline));
        let mut armed = false;
        let mut reference_start = 0;
        poll_fn(|context| {
            if !armed {
                critical_section::with(|section| {
                    *WATCHDOG.borrow(section).borrow_mut() = Some(context.waker().clone());
                });
                check(timer.as_mut().poll(context).is_pending(), 11);
                check(pac::SYSTICK.cmpl().read() == u32::MAX, 12);
                reference_start = reference_us();
                critical_section::with(|_| {
                    pac::SYSTICK.ctlr().modify(|register| register.set_ste(true));
                    if cfg!(feature = "stress-disable-systick-irq") {
                        pac::SYSTICK.ctlr().modify(|register| register.set_stie(false));
                    }
                    while reference_us() - reference_start < masked_us {
                        core::hint::spin_loop();
                    }
                });
                armed = true;
                return Poll::Pending;
            }
            let elapsed = reference_us() - reference_start;
            if elapsed > 200_000 {
                defmt::error!(
                    "Timeout round={} masked_us={} cnt={} cmp={} pending={}",
                    round,
                    masked_us,
                    pac::SYSTICK.cntl().read(),
                    pac::SYSTICK.cmpl().read(),
                    pac::SYSTICK.sr().read().cntif()
                );
                check(false, 13);
            }
            if pac::SYSTICK.cmpl().read() == u32::MAX {
                return Poll::Pending;
            }
            timer.as_mut().poll(context)
        })
        .await;
        let elapsed = reference_us() - reference_start;
        min_elapsed = min_elapsed.min(elapsed);
        max_elapsed = max_elapsed.max(elapsed);
        check((2_980..=200_000).contains(&elapsed), 14);
        critical_section::with(|_| {
            let epoch = (u64::from(round) + 1) << 32;
            let before = (epoch | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
            let now = Instant::now().as_ticks();
            let after = (epoch | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
            if !(before <= now && now <= after) {
                defmt::error!("Epoch round={} expected={}..{} actual={}", round, before, after, now);
                check(false, 15);
            }
            check(now >= deadline.as_ticks(), 16);
        });
        if round % 64 == 63 {
            defmt::info!(
                "Rollover rounds={} elapsed_us={}..{}",
                round + 1,
                min_elapsed,
                max_elapsed
            );
        }
    }
    let reference_start = reference_us();
    let start = Instant::now();
    Timer::after_secs(2).await;
    let elapsed = reference_us() - reference_start;
    let embassy_elapsed = Instant::now().duration_since(start).as_micros();
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(SYSTICK_STRESS_STATS),
            [
                512,
                min_elapsed as u32,
                max_elapsed as u32,
                elapsed as u32,
                embassy_elapsed as u32,
            ],
        );
    }
    defmt::info!("Scale reference_us={} embassy_us={}", elapsed, embassy_elapsed);
    check(elapsed.abs_diff(embassy_elapsed) < 100, 17);
    if cfg!(feature = "stress-natural-wrap") {
        natural_wrap(counter_hz).await;
    }
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SYSTICK_STRESS_RESULT), 2) };
    defmt::info!("SYSTICK_STRESS_RESULT=PASS rounds=512");
    loop {
        Timer::after_secs(1).await;
    }
}
