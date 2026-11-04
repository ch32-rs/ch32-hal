#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::rng::{InterruptHandler, Rng};
use ch32_hal::{bind_interrupts, pac, rcc};
use ch32v467_examples::{report_check, report_set, report_static};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    RNG => InterruptHandler<ch32_hal::peripherals::RNG>;
});

report_static!(RNG_RESULT);
#[no_mangle]
pub static mut RNG_SAMPLES: [u32; 32] = [0; 32];
#[no_mangle]
pub static mut RNG_SOURCE_HZ: u32 = 0;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(RNG_RESULT, 1);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "rng-usbhs-pll") {
        config.rcc.hspll = Some(rcc::HsPllSource::HSI);
        config.rcc.rng_src = rcc::RngClockSource::UsbHsPll;
    }
    let mut peripherals = ch32_hal::init(config);
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(RNG_SOURCE_HZ),
            rcc::rng_source_frequency().unwrap().0,
        )
    };
    let mut random = Rng::new(peripherals.RNG.reborrow(), Irqs);
    for index in 0..32 {
        let result = with_timeout(Duration::from_millis(100), random.async_next_u32()).await;
        report_check!(RNG_RESULT, matches!(result, Ok(Ok(_))), 10, "RNG single-sample acquisition failed");
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(RNG_SAMPLES[index]), result.unwrap().unwrap()) };
    }
    let mut bytes = [0u8; 31];
    report_check!(
        RNG_RESULT,
        matches!(
            with_timeout(Duration::from_millis(100), random.async_fill_bytes(&mut bytes)).await,
            Ok(Ok(()))
        ),
        11,
        "RNG fill_bytes failed"
    );
    random.reset();
    {
        let mut read = pin!(random.async_next_u32());
        poll_fn(|context| {
            let _ = read.as_mut().poll(context);
            Poll::Ready(())
        })
        .await;
    }
    report_check!(RNG_RESULT, !pac::RNG.cr().read().ie(), 12, "RNG interrupt still enabled after poll");
    report_check!(
        RNG_RESULT,
        matches!(
            with_timeout(Duration::from_millis(100), random.async_next_u32()).await,
            Ok(Ok(_))
        ),
        13,
        "RNG acquisition after reset failed"
    );
    drop(random);
    report_check!(
        RNG_RESULT,
        !pac::RNG.cr().read().rngen() && !pac::RNG.cr().read().ie(),
        14,
        "RNG not disabled after drop"
    );
    report_check!(RNG_RESULT, !pac::RCC.hbpcenr().read().trngen(), 15, "RNG clock not gated off after drop");
    let mut random = Rng::new(peripherals.RNG.reborrow(), Irqs);
    report_check!(
        RNG_RESULT,
        matches!(
            with_timeout(Duration::from_millis(100), random.async_next_u32()).await,
            Ok(Ok(_))
        ),
        16,
        "RNG re-acquisition failed"
    );
    report_set!(RNG_RESULT, 2);
    defmt::info!("RNG_RESULT=PASS acquisition and lifecycle; entropy quality requires separate assessment");
    loop {
        Timer::after_secs(1).await;
    }
}
