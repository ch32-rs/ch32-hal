#![no_std]
#![no_main]

use ch32_hal::{pac, rcc};
use defmt_rtt as _;
use embassy_time::{Instant, TICK_HZ};
use panic_halt as _;
use qingke_rt::entry;

#[no_mangle]
pub static mut TIME_SCALE_RESULT: u32 = 0;
#[no_mangle]
pub static mut TIME_SCALE_SAMPLES: [[u32; 4]; 5] = [[0; 4]; 5];

fn check(condition: bool) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(TIME_SCALE_RESULT), 3) };
        defmt::panic!("TIME_SCALE_RESULT=FAIL");
    }
}

#[entry]
fn main() -> ! {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(TIME_SCALE_RESULT), 1) };
    qingke::riscv::asm::delay(2_000_000);
    let mut config = ch32_hal::Config::default();
    let mut ahb_divisor = 1u64;
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
        ahb_divisor = 2;
    }
    if cfg!(feature = "clock-slow") {
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV512;
        ahb_divisor = 512;
    }
    let _peripherals = ch32_hal::init(config);
    let source_hz = rcc::clocks().sysclk.0;
    critical_section::with(|_| {
        pac::SYSTICK.ctlr().modify(|register| {
            register.set_ste(false);
            register.set_stie(false);
        });
        for (index, count) in [0, 1, 10_000, 3_000_000_000, u32::MAX - 1].into_iter().enumerate() {
            pac::SYSTICK.cntl().write_value(count);
            let now = Instant::now().as_ticks();
            let expected = (u128::from(count) * u128::from(TICK_HZ) * u128::from(ahb_divisor) * 8
                / u128::from(rcc::clocks().sysclk.0)) as u64;
            unsafe {
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(TIME_SCALE_SAMPLES[index]),
                    [count, now as u32, (now >> 32) as u32, (expected == now) as u32],
                );
            }
            check(now == expected);
        }
        pac::SYSTICK.sr().write(|register| register.set_cntif(false));
        unsafe { qingke::pfic::unpend_interrupt(qingke_rt::CoreInterrupt::SysTick as u8) };
    });
    unsafe { rcc::init(rcc::Config::default()) };
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(TIME_SCALE_RESULT), 2) };
    defmt::info!(
        "TIME_SCALE_RESULT=PASS sysclk={} ahb_divisor={}",
        source_hz,
        ahb_divisor
    );
    loop {
        core::hint::spin_loop();
    }
}
