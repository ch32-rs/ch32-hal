#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let p = hal::init(Default::default());
    hal::embassy::init();

    // use remap 1, or 3
    let pin = PwmPin::new_ch1::<1>(p.PA15);
    let mut pwm = SimplePwm::new(
        p.TIM2,
        Some(pin),
        None,
        None,
        None,
        Hertz::khz(1),
        CountingMode::default(),
    );
    let ch = hal::timer::Channel::Ch1;

    let max_duty = pwm.get_max_duty();
    println!("max duty: {}", max_duty);
    pwm.set_duty(ch, 7000);
    pwm.enable(ch);

    loop {
        for i in 0..100 {
            pwm.set_duty(ch, i * 80);
            Timer::after_millis(5).await;
        }
        for i in (0..100).rev() {
            pwm.set_duty(ch, i * 80);
            Timer::after_millis(5).await;
        }
    }
}
