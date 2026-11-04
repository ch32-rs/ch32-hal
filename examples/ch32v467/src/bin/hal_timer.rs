#![no_std]
#![no_main]

use core::marker::PhantomData;
use core::sync::atomic::{AtomicU32, Ordering};

use ch32_hal::interrupt::typelevel::{Binding, Handler, Interrupt};
use ch32_hal::time::Hertz;
use ch32_hal::timer::complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin};
use ch32_hal::timer::low_level::{CountingMode, Timer};
use ch32_hal::timer::simple_pwm::{PwmPin, SimplePwm};
use ch32_hal::timer::{BasicInstance, Channel, GeneralInstance16bit};
use ch32_hal::{bind_interrupts, pac, rcc, Peri};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer as EmbassyTimer};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

static UPDATES: AtomicU32 = AtomicU32::new(0);

struct UpdateHandler<T: BasicInstance>(PhantomData<T>);

impl<T: BasicInstance> Handler<T::UpdateInterrupt> for UpdateHandler<T> {
    unsafe fn on_interrupt() {
        let registers = pac::timer::Bctm::from_ptr(T::regs());
        if registers.intfr().read().uif() {
            registers.intfr().write_value(pac::timer::regs::Intfr(0xfffe));
            UPDATES.store(UPDATES.load(Ordering::Relaxed) + 1, Ordering::Relaxed);
        }
    }
}

bind_interrupts!(struct Irqs {
    TIM1_UP => UpdateHandler<ch32_hal::peripherals::TIM1>;
    TIM2 => UpdateHandler<ch32_hal::peripherals::TIM2>;
    TIM3 => UpdateHandler<ch32_hal::peripherals::TIM3>;
    TIM4 => UpdateHandler<ch32_hal::peripherals::TIM4>;
    TIM5 => UpdateHandler<ch32_hal::peripherals::TIM5>;
    TIM6 => UpdateHandler<ch32_hal::peripherals::TIM6>;
    TIM7 => UpdateHandler<ch32_hal::peripherals::TIM7>;
    TIM8_UP => UpdateHandler<ch32_hal::peripherals::TIM8>;
});

report_static!(TIMER_RESULT);
#[no_mangle]
pub static mut TIMER_CASES: u32 = 0;
#[no_mangle]
pub static mut TIMER_SNAPSHOTS: [[u32; 4]; 8] = [[0; 4]; 8];

fn check(condition: bool, code: u32) {
    report_check!(TIMER_RESULT, condition, code, "TIMER_RESULT check failed");
}

async fn exercise<T: BasicInstance>(
    mut peripheral: Peri<'_, T>,
    irq: impl Binding<T::UpdateInterrupt, UpdateHandler<T>>,
    index: usize,
) {
    let _ = irq;
    for frequency in [50, 1000, 10_000] {
        let timer = Timer::new(peripheral.reborrow());
        timer.set_frequency(Hertz(frequency));
        check(timer.get_frequency().0.abs_diff(frequency) <= frequency / 100 + 1, 10);
        timer.clear_update_interrupt();
        UPDATES.store(0, Ordering::Relaxed);
        T::UpdateInterrupt::unpend();
        timer.enable_update_interrupt(true);
        unsafe { T::UpdateInterrupt::enable() };
        let started = Instant::now();
        timer.start();
        EmbassyTimer::after_millis(100).await;
        timer.stop();
        T::UpdateInterrupt::disable();
        timer.enable_update_interrupt(false);
        let count = UPDATES.load(Ordering::Relaxed);
        let expected = (started.elapsed().as_micros() * u64::from(frequency) / 1_000_000) as u32;
        check(count.abs_diff(expected) <= expected / 20 + 2, 11);
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(TIMER_SNAPSHOTS[index]),
                [
                    timer.get_frequency().0,
                    u32::from(timer.regs_basic().psc().read()),
                    u32::from(timer.regs_basic().atrlr().read()),
                    count,
                ],
            );
            let cases = core::ptr::addr_of_mut!(TIMER_CASES);
            cases.write_volatile(cases.read_volatile() + 1);
        }
        drop(timer);
    }
}

fn exercise_pwm<T: GeneralInstance16bit>(mut peripheral: Peri<'_, T>) {
    for mode in [
        CountingMode::EdgeAlignedUp,
        CountingMode::EdgeAlignedDown,
        CountingMode::CenterAlignedBothInterrupts,
    ] {
        let mut pwm = SimplePwm::new(peripheral.reborrow(), None, None, None, None, Hertz(1000), mode);
        for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
            pwm.enable(channel);
            check(pwm.is_enabled(channel), 12);
            for duty in [0, pwm.get_max_duty() / 2, pwm.get_max_duty()] {
                pwm.set_duty(channel, duty);
                check(pwm.get_duty(channel) == duty, 13);
            }
            pwm.disable(channel);
            check(!pwm.is_enabled(channel), 14);
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(TIMER_RESULT, 1);
    qingke::riscv::asm::delay(2_000_000);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    let mut peripherals = ch32_hal::init(config);
    macro_rules! test_timer {
        ($instance:ident, $index:expr) => {
            exercise(peripherals.$instance.reborrow(), Irqs, $index).await;
        };
    }
    test_timer!(TIM1, 0);
    test_timer!(TIM2, 1);
    test_timer!(TIM3, 2);
    test_timer!(TIM4, 3);
    test_timer!(TIM5, 4);
    test_timer!(TIM6, 5);
    test_timer!(TIM7, 6);
    test_timer!(TIM8, 7);
    exercise_pwm(peripherals.TIM1.reborrow());
    exercise_pwm(peripherals.TIM2.reborrow());
    exercise_pwm(peripherals.TIM3.reborrow());
    exercise_pwm(peripherals.TIM4.reborrow());
    exercise_pwm(peripherals.TIM5.reborrow());
    exercise_pwm(peripherals.TIM8.reborrow());
    let channel = PwmPin::new_ch1::<3>(peripherals.PE9);
    let complementary = ComplementaryPwmPin::new_ch1::<3>(peripherals.PE8);
    let mut pwm = ComplementaryPwm::new(
        peripherals.TIM1,
        Some(channel),
        Some(complementary),
        None,
        None,
        None,
        None,
        None,
        Hertz(1000),
        CountingMode::EdgeAlignedUp,
    );
    pwm.set_dead_time(20);
    pwm.set_duty(Channel::Ch1, pwm.get_max_duty() / 2);
    pwm.enable(Channel::Ch4);
    pwm.disable(Channel::Ch4);
    check(!pac::TIM1.ccer().read().cce(3), 15);
    pwm.enable(Channel::Ch1);
    check(pac::TIM1.bdtr().read().moe() && pac::TIM1.ccer().read().ccne(0), 16);
    report_set!(TIMER_RESULT, 2);
    defmt::info!("TIMER_RESULT=PASS; TIM1 CH1/CH1N on PE9/PE8 at 1kHz");
    loop {
        EmbassyTimer::after(Duration::from_secs(1)).await;
    }
}
