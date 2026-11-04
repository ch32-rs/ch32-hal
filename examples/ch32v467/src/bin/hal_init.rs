#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32v467_examples as board;
use ch32v467_examples::{report_check, report_set, report_static};
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

report_static!(HAL_INIT_RESULT);

#[entry]
fn main() -> ! {
    report_set!(HAL_INIT_RESULT, 1);
    qingke::riscv::asm::delay(2_000_000);
    defmt::info!("HAL init starting");

    let mut config = hal::Config::default();
    if cfg!(feature = "clock-dividers") {
        config.rcc.ahb_pre = hal::rcc::AHBPrescaler::DIV2;
        config.rcc.apb1_pre = hal::rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = hal::rcc::APBPrescaler::DIV2;
    }
    let _peripherals = hal::init(config);

    let expected_hclk = if cfg!(feature = "clock-dividers") {
        10_000_000
    } else {
        20_000_000
    };
    let expected_pclk1 = if cfg!(feature = "clock-dividers") {
        2_500_000
    } else {
        20_000_000
    };
    let expected_pclk2 = if cfg!(feature = "clock-dividers") {
        5_000_000
    } else {
        20_000_000
    };
    check(hal::rcc::clocks().sysclk.0 == 20_000_000);
    check(hal::rcc::clocks().hclk.0 == expected_hclk);
    check(hal::rcc::clocks().pclk1.0 == expected_pclk1);
    check(hal::rcc::clocks().pclk2.0 == expected_pclk2);
    check(hal::pac::RCC.ctlr().read().hsirdy());
    check(hal::pac::RCC.cfgr0().read().sws() == 0);
    let enabled = hal::pac::RCC.pb2pcenr().read();
    check(enabled.afioen() && enabled.iopaen() && enabled.iopeen());
    unsafe { hal::rcc::refresh(None) };
    check(hal::rcc::clocks().hclk.0 == expected_hclk);
    check(hal::rcc::clocks().pclk1.0 == expected_pclk1);
    check(hal::rcc::clocks().pclk2.0 == expected_pclk2);

    #[cfg(not(feature = "embassy"))]
    {
        hal::delay::Delay.delay_us(0);
        hal::delay::Delay.delay_us(1);
        hal::delay::Delay.delay_ms(0);
    }
    #[cfg(feature = "embassy")]
    {
        let systick = hal::pac::SYSTICK.ctlr().read();
        check(systick.ste());
        check(systick.stclk() == hal::pac::systick::vals::Stclk::HCLK_DIV8);
        check(systick.mode() == hal::pac::systick::vals::Mode::UPCOUNT);
        check(hal::pac::PFIC.sctlr().read().sevonpend());
    }

    report_set!(HAL_INIT_RESULT, 2);
    defmt::info!(
        "HAL_INIT_RESULT=PASS sysclk={} hclk={} pclk1={} pclk2={} cfgr0={=u32:08x}",
        hal::rcc::clocks().sysclk.0,
        hal::rcc::clocks().hclk.0,
        hal::rcc::clocks().pclk1.0,
        hal::rcc::clocks().pclk2.0,
        hal::pac::RCC.cfgr0().read().0,
    );

    let mut heartbeat = 0u32;
    loop {
        heartbeat = heartbeat.wrapping_add(1);
        board::heartbeat(heartbeat);
        defmt::info!("HAL heartbeat={}", heartbeat);
        #[cfg(not(feature = "embassy"))]
        hal::delay::Delay.delay_ms(1_000);
        #[cfg(feature = "embassy")]
        qingke::riscv::asm::delay(expected_hclk);
    }
}

fn check(condition: bool) {
    report_check!(HAL_INIT_RESULT, condition, 3, "HAL init check failed");
}
