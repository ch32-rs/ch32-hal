#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use ch32_hal::gpio::{Level, Output, Speed};
use ch32_hal::pac;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer, TICK_HZ};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

report_static!(EMBASSY_RESULT);
static COMPLETED: AtomicU32 = AtomicU32::new(0);

fn check(condition: bool) {
    report_check!(EMBASSY_RESULT, condition, 3, "EMBASSY_RESULT check failed");
}

fn set_boundary(remaining: u32) {
    critical_section::with(|_| {
        pac::SYSTICK.ctlr().modify(|w| w.set_ste(false));
        pac::SYSTICK.cntl().write_value(u32::MAX - remaining);
        let _now = Instant::now();
        pac::SYSTICK.cmpl().write_value(u32::MAX);
        pac::SYSTICK.sr().write(|w| w.set_cntif(false));
        pac::SYSTICK.ctlr().modify(|w| w.set_ste(true));
    });
}

fn check_time_scale(counter_hz: u64, epoch: u64) {
    let before = ((epoch << 32) | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
    let now = Instant::now().as_ticks();
    let after = ((epoch << 32) | u64::from(pac::SYSTICK.cntl().read())) * TICK_HZ / counter_hz;
    defmt::info!(
        "Time scale: expected={}..{} actual={} tick_hz={} counter_hz={}",
        before,
        after,
        now,
        TICK_HZ,
        counter_hz
    );
    check(before <= now && now <= after);
}

#[embassy_executor::task(pool_size = 3)]
async fn worker(id: u32, period_ms: u64, mut led: Option<Output<'static>>) {
    for iteration in 0..16 {
        let start = Instant::now();
        let deadline = start + Duration::from_millis(period_ms);
        Timer::at(deadline).await;
        let now = Instant::now();
        check(now >= deadline);
        check(now.duration_since(start).as_millis() < period_ms + 250);
        if let Some(led) = led.as_mut() {
            led.toggle();
        }
        if iteration % 4 == 3 {
            defmt::info!(
                "Timer worker={} iteration={} elapsed_ms={}",
                id,
                iteration + 1,
                now.duration_since(start).as_millis()
            );
        }
    }
    COMPLETED.store(COMPLETED.load(Ordering::Relaxed) | (1 << id), Ordering::Relaxed);
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    report_set!(EMBASSY_RESULT, 1);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-dividers") {
        config.rcc.ahb_pre = ch32_hal::rcc::AHBPrescaler::DIV2;
        config.rcc.apb1_pre = ch32_hal::rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = ch32_hal::rcc::APBPrescaler::DIV2;
    }
    let peripherals = ch32_hal::init(config);
    let counter_hz = u64::from(ch32_hal::rcc::clocks().hclk.0 / 8);
    check(pac::PFIC.sctlr().read().sevonpend());
    defmt::info!(
        "Embassy starting: hclk={} counter_hz={}",
        ch32_hal::rcc::clocks().hclk.0,
        counter_hz
    );
    set_boundary(20_000);
    let start = Instant::now();
    let mut previous = start;
    let mut crossed = false;
    for _ in 0..100_000 {
        let now = Instant::now();
        check(now >= previous);
        check(now.duration_since(previous).as_micros() < 1_000);
        previous = now;
        if pac::SYSTICK.cntl().read() < 20_000 {
            crossed = true;
            break;
        }
    }
    check(crossed);
    check_time_scale(counter_hz, 1);
    defmt::info!("Counter rollover: monotonic PASS");
    set_boundary((counter_hz / 20) as u32);
    check_time_scale(counter_hz, 1);
    let led8 = Output::new(peripherals.PE8, Level::Low, Speed::Low);
    let led9 = Output::new(peripherals.PE9, Level::High, Speed::Low);
    spawner.spawn(worker(0, 100, Some(led8)).unwrap());
    spawner.spawn(worker(1, 150, Some(led9)).unwrap());
    spawner.spawn(worker(2, 230, None).unwrap());
    let start = Instant::now();
    for heartbeat in 1..=6 {
        Timer::after_secs(1).await;
        defmt::info!(
            "Embassy heartbeat={} elapsed_ms={} completed={}",
            heartbeat,
            Instant::now().duration_since(start).as_millis(),
            COMPLETED.load(Ordering::Relaxed)
        );
    }
    check(COMPLETED.load(Ordering::Relaxed) == 7);
    check_time_scale(counter_hz, 2);
    check(matches!(
        select(Timer::after_millis(5), Timer::after_secs(2)).await,
        Either::First(_)
    ));
    Timer::after_millis(10).await;
    Timer::at(Instant::now() - Duration::from_millis(1)).await;
    for _ in 0..16 {
        Timer::after_micros(1).await;
    }
    defmt::info!("Alarm rollover, cancellation, past/near deadlines: PASS");
    let mut slept = 0u64;
    for _ in 0..8 {
        let mut timer = pin!(Timer::after_millis(20));
        poll_fn(|context| {
            if timer.as_mut().poll(context).is_ready() {
                Poll::Ready(())
            } else {
                let before = pac::SYSTICK.cntl().read();
                critical_section::with(|_| {
                    while !pac::SYSTICK.sr().read().cntif() {
                        unsafe { core::arch::asm!("wfi") };
                    }
                });
                slept += u64::from(pac::SYSTICK.cntl().read().wrapping_sub(before));
                Poll::Pending
            }
        })
        .await;
    }
    let sleep_us = slept * 1_000_000 / counter_hz;
    defmt::info!("WFI measured_sleep_us={}", sleep_us);
    check(sleep_us >= 80_000);
    defmt::info!("WFI with interrupts masked: PASS measured_sleep_us={}", sleep_us);
    report_set!(EMBASSY_RESULT, 2);
    defmt::info!("EMBASSY_RESULT=PASS");
    loop {
        Timer::after_secs(1).await;
        defmt::info!(
            "Embassy alive elapsed_ms={}",
            Instant::now().duration_since(start).as_millis()
        );
    }
}
