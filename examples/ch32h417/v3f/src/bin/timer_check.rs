//! Timer self-check across the four H4 timer classes (V3F only).
//!
//! `cargo xtask flash --example timer_check && cargo xtask report --example timer_check`
//!
//! Runs one timer of each register layout — TIM1 (ADTM), TIM2 (16-bit GPTM),
//! TIM9 (32-bit GPTM) and TIM6 (BCTM) — through the HAL driver: frequency
//! readback at 50 Hz / 1 kHz / 10 kHz, the update interrupt counted for 100 ms
//! against the SysTick delay, and PWM compare values on the general-purpose
//! timers. TIM9 additionally proves the 32-bit autoreload path, since 1 Hz at
//! PCLK1 needs an ARR that does not fit in 16 bits.
//!
//! `stage` reaches 2 once every class ran, `timers` counts the classes and
//! `checks`/`failures` the assertions. `updates`/`frequency` are the last
//! measured values and `wide_arr` TIM9's autoreload.

#![no_std]
#![no_main]

use core::marker::PhantomData;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::TimerCheck;
use hal::delay::Delay;
use hal::interrupt::typelevel::{Binding, Handler, Interrupt};
use hal::time::Hertz;
use hal::timer::low_level::{CountingMode, Timer};
use hal::timer::simple_pwm::SimplePwm;
use hal::timer::{BasicInstance, Channel, CoreInstance, GeneralInstance16bit};
use hal::{bind_interrupts, pac, Peri};

ipc::mailbox!(TimerCheck);
use panic_halt as _;

static UPDATES: AtomicU32 = AtomicU32::new(0);

struct UpdateHandler<T: BasicInstance>(PhantomData<T>);

impl<T: BasicInstance> Handler<T::UpdateInterrupt> for UpdateHandler<T> {
    unsafe fn on_interrupt() {
        let regs = pac::timer::Bctm::from_ptr(T::regs());
        if regs.intfr().read().uif() {
            regs.intfr().write_value(pac::timer::regs::Intfr(0xfffe));
            UPDATES.fetch_add(1, Relaxed);
        }
    }
}

// One handler per H4 interrupt shape: ADTM keeps the V3 `_UP` vector, while
// GPTM/GPTM32/BCTM collapse everything into a single `GLOBAL` vector.
bind_interrupts!(struct Irqs {
    TIM1_UP => UpdateHandler<hal::peripherals::TIM1>;
    TIM2 => UpdateHandler<hal::peripherals::TIM2>;
    TIM9 => UpdateHandler<hal::peripherals::TIM9>;
    TIM6 => UpdateHandler<hal::peripherals::TIM6>;
});

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

/// Frequency readback plus update-interrupt rate for one timer class.
fn exercise<T: BasicInstance>(
    mut peri: Peri<'_, T>,
    _irq: impl Binding<T::UpdateInterrupt, UpdateHandler<T>>,
    delay: &mut Delay,
    mailbox: &'static ipc::Mailbox<TimerCheck>,
    checks: &mut u32,
    failures: &mut u32,
) {
    let timer = Timer::new(peri.reborrow());

    for frequency in [50u32, 1_000, 10_000] {
        timer.set_frequency(Hertz(frequency));
        let readback = timer.get_frequency().0;
        check(failures, checks, readback.abs_diff(frequency) <= frequency / 100 + 1);

        timer.clear_update_interrupt();
        UPDATES.store(0, Relaxed);
        T::UpdateInterrupt::unpend();
        timer.enable_update_interrupt(true);
        unsafe { T::UpdateInterrupt::enable() };
        timer.start();
        delay.delay_ms(100);
        timer.stop();
        timer.enable_update_interrupt(false);
        T::UpdateInterrupt::disable();

        let count = UPDATES.load(Relaxed);
        let expected = frequency / 10;
        check(failures, checks, count.abs_diff(expected) <= expected / 5 + 2);
        mailbox.payload.updates.store(count, Relaxed);
        mailbox.payload.frequency.store(readback, Relaxed);
    }
}

/// PWM compare/enable path on a general-purpose timer, without pins: only the
/// channel registers are exercised, no waveform leaves the chip.
fn exercise_pwm<T: GeneralInstance16bit>(mut peri: Peri<'_, T>, checks: &mut u32, failures: &mut u32) {
    let mut pwm = SimplePwm::new(
        peri.reborrow(),
        None,
        None,
        None,
        None,
        Hertz(1_000),
        CountingMode::EdgeAlignedUp,
    );
    let max = pwm.get_max_duty();
    for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3, Channel::Ch4] {
        pwm.enable(channel);
        check(failures, checks, pwm.is_enabled(channel));
        for duty in [0, max / 2, max] {
            pwm.set_duty(channel, duty);
            check(failures, checks, pwm.get_duty(channel) == duty);
        }
        pwm.disable(channel);
        check(failures, checks, !pwm.is_enabled(channel));
    }
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut p = hal::init(hal::Config::default());
    let mut delay = Delay;
    let mut checks = 0u32;
    let mut failures = 0u32;

    exercise(p.TIM1.reborrow(), Irqs, &mut delay, mailbox, &mut checks, &mut failures);
    exercise(p.TIM2.reborrow(), Irqs, &mut delay, mailbox, &mut checks, &mut failures);
    exercise(p.TIM9.reborrow(), Irqs, &mut delay, mailbox, &mut checks, &mut failures);
    exercise(p.TIM6.reborrow(), Irqs, &mut delay, mailbox, &mut checks, &mut failures);

    // 25 MHz PCLK1 cannot express 1 Hz with a 16-bit ARR, so this readback and
    // the register width are the 32-bit path's own evidence.
    {
        let timer = Timer::new(p.TIM9.reborrow());
        timer.set_frequency(Hertz(1));
        let readback = timer.get_frequency().0;
        check(&mut failures, &mut checks, readback == 1);
        let wide = unsafe { pac::timer::Gptm32::from_ptr(hal::peripherals::TIM9::regs()) };
        let arr = wide.atrlr().read();
        mailbox.payload.wide_arr.store(arr, Relaxed);
        check(&mut failures, &mut checks, arr > 0xFFFF);
    }

    exercise_pwm(p.TIM1.reborrow(), &mut checks, &mut failures);
    exercise_pwm(p.TIM3.reborrow(), &mut checks, &mut failures);
    exercise_pwm(p.TIM9.reborrow(), &mut checks, &mut failures);

    mailbox.payload.timers.store(4, Relaxed);
    mailbox.payload.pclk1.store(hal::rcc::clocks().pclk1.0, Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    loop {
        delay.delay_ms(1000);
    }
}
