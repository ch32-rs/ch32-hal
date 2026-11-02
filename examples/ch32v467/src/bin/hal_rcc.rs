#![no_std]
#![no_main]

use ch32_hal::pac;
use ch32_hal::rcc::{
    self, AHBPrescaler, APBPrescaler, HsPllSource, Pll, PllMul, PllSource, SysPllPrescaler, SysPllSource, Sysclk,
};
use ch32_hal::time::Hertz;
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

#[no_mangle]
pub static mut RCC_RESULT: u32 = 0;
#[no_mangle]
pub static mut RCC_CASES: u32 = 0;
#[no_mangle]
pub static mut RCC_SNAPSHOTS: [[u32; 8]; 14] = [[0; 8]; 14];

fn check(condition: bool) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(RCC_RESULT), 3) };
        defmt::panic!("RCC_RESULT=FAIL");
    }
}

fn check_clock(config: rcc::Config, sysclk: u32, hclk: u32) {
    defmt::info!("RCC switching: sysclk={} hclk={}", sysclk, hclk);
    let expected = config.clocks();
    check(expected.sysclk.0 == sysclk && expected.hclk.0 == hclk);
    unsafe { rcc::init(config) };
    check(*rcc::clocks() == expected);
    check(pac::RCC.cfgr0().read().sws() == config.sys as u8);
    check(pac::FLASH.ctlr().read().sckmod() == if hclk <= 100_000_000 { 0 } else { 1 });
    check(pac::FLASH.ctlr().read().lock() && pac::FLASH.ctlr().read().flock());
    unsafe { rcc::refresh(config.hse.map(|clock| clock.freq)) };
    check(*rcc::clocks() == expected);

    let timer_hz = rcc::clocks().pclk1.0 * if config.apb1_pre == APBPrescaler::DIV1 { 1 } else { 2 };
    let divisor = timer_hz.div_ceil(1_000_000);
    pac::RCC.pb1pcenr().modify(|register| register.set_tim2en(true));
    pac::RCC.pb1prstr().modify(|register| register.set_tim2rst(true));
    pac::RCC.pb1prstr().modify(|register| register.set_tim2rst(false));
    pac::TIM2.psc().write_value((divisor - 1) as u16);
    pac::TIM2.atrlr().write_value(9_999);
    pac::TIM2.swevgr().write(|register| register.set_ug(true));
    pac::TIM2.intfr().write(|register| register.set_uif(false));
    pac::SYSTICK.ctlr().write(|register| {
        register.set_mode(pac::systick::vals::Mode::UPCOUNT);
        register.set_stclk(pac::systick::vals::Stclk::HCLK);
    });
    pac::SYSTICK.cntl().write_value(0);
    pac::SYSTICK.cmpl().write_value(u32::MAX);
    pac::SYSTICK.sr().write(|register| register.set_cntif(false));
    pac::SYSTICK.ctlr().modify(|register| register.set_ste(true));
    let before = pac::SYSTICK.cntl().read();
    pac::TIM2.ctlr1().modify(|register| register.set_cen(true));
    while !pac::TIM2.intfr().read().uif() {
        check(pac::SYSTICK.cntl().read().wrapping_sub(before) < hclk);
    }
    let cycles = pac::SYSTICK.cntl().read().wrapping_sub(before);
    pac::TIM2.ctlr1().modify(|register| register.set_cen(false));
    pac::SYSTICK.ctlr().modify(|register| register.set_ste(false));
    let expected_cycles = (u64::from(hclk) * u64::from(divisor) * 10_000 / u64::from(timer_hz)) as u32;
    defmt::info!(
        "RCC measurement sysclk={} hclk={} pclk1={} pclk2={} timer_cycles={} expected={}",
        sysclk,
        hclk,
        expected.pclk1.0,
        expected.pclk2.0,
        cycles,
        expected_cycles
    );
    check(cycles.abs_diff(expected_cycles) < expected_cycles / 100);
    unsafe {
        let index = core::ptr::read_volatile(core::ptr::addr_of!(RCC_CASES)) as usize;
        check(index < 14);
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(RCC_SNAPSHOTS[index]),
            [
                sysclk,
                hclk,
                expected.pclk1.0,
                expected.pclk2.0,
                pac::RCC.cfgr0().read().0,
                pac::RCC.cfgr2().read().0,
                pac::FLASH.ctlr().read().0,
                cycles,
            ],
        );
        core::ptr::write_volatile(core::ptr::addr_of_mut!(RCC_CASES), index as u32 + 1);
    }
}

#[entry]
fn main() -> ! {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(RCC_RESULT), 1) };
    qingke::riscv::asm::delay(2_000_000);
    check_clock(rcc::Config::default(), 20_000_000, 20_000_000);
    for (mul, sysclk, hclk) in [
        (PllMul::MUL8, 160_000_000, 80_000_000),
        (PllMul::MUL12_5, 250_000_000, 125_000_000),
        (PllMul::MUL17_5, 350_000_000, 175_000_000),
        (PllMul::MUL20, 400_000_000, 200_000_000),
    ] {
        check_clock(
            rcc::Config {
                sys: Sysclk::PLL,
                pll: Some(Pll { mul }),
                ahb_pre: AHBPrescaler::DIV2,
                apb1_pre: APBPrescaler::DIV4,
                apb2_pre: APBPrescaler::DIV2,
                ..Default::default()
            },
            sysclk,
            hclk,
        );
    }
    for (pre, sysclk, hclk) in [
        (SysPllPrescaler::DIV4, 120_000_000, 60_000_000),
        (SysPllPrescaler::DIV2, 240_000_000, 120_000_000),
    ] {
        check_clock(
            rcc::Config {
                sys: Sysclk::PLL,
                hspll: Some(HsPllSource::HSI),
                syspll_src: SysPllSource::UsbHs,
                syspll_pre: pre,
                ahb_pre: AHBPrescaler::DIV2,
                ..Default::default()
            },
            sysclk,
            hclk,
        );
    }
    check_clock(
        rcc::Config {
            sys: Sysclk::PLL,
            hspll: Some(HsPllSource::HSI),
            syspll_src: SysPllSource::UsbHsMul2Div3,
            ahb_pre: AHBPrescaler::DIV2,
            ..Default::default()
        },
        320_000_000,
        160_000_000,
    );
    if cfg!(feature = "clock-hse") {
        let hse = Some(rcc::Hse {
            freq: Hertz(25_000_000),
            mode: rcc::HseMode::Oscillator,
        });
        check_clock(
            rcc::Config {
                hse,
                sys: Sysclk::HSE,
                ..Default::default()
            },
            25_000_000,
            25_000_000,
        );
        check_clock(
            rcc::Config {
                hse,
                sys: Sysclk::PLL,
                pll_src: PllSource::HSE,
                pll: Some(Pll { mul: PllMul::MUL16 }),
                ahb_pre: AHBPrescaler::DIV2,
                ..Default::default()
            },
            400_000_000,
            200_000_000,
        );
        for (source, sysclk, hclk) in [
            (SysPllSource::Eth, 250_000_000, 125_000_000),
            (SysPllSource::EthMul2Div3, 166_666_666, 83_333_333),
        ] {
            check_clock(
                rcc::Config {
                    hse,
                    sys: Sysclk::PLL,
                    ethpll: true,
                    syspll_src: source,
                    syspll_pre: SysPllPrescaler::DIV2,
                    ahb_pre: AHBPrescaler::DIV2,
                    ..Default::default()
                },
                sysclk,
                hclk,
            );
        }
        check_clock(
            rcc::Config {
                hse,
                sys: Sysclk::PLL,
                ethpll: true,
                hspll: Some(HsPllSource::ETH),
                syspll_src: SysPllSource::UsbHs,
                syspll_pre: SysPllPrescaler::DIV2,
                ahb_pre: AHBPrescaler::DIV2,
                ..Default::default()
            },
            240_000_000,
            120_000_000,
        );
    }
    check_clock(rcc::Config::default(), 20_000_000, 20_000_000);
    check(
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(RCC_CASES)) }
            == if cfg!(feature = "clock-hse") { 14 } else { 9 },
    );
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(RCC_RESULT), 2) };
    defmt::info!("RCC_RESULT=PASS");
    loop {
        qingke::riscv::asm::delay(20_000_000);
    }
}
