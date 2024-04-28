#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    hal::embassy::init();

    let pin = PwmPin::new_ch4(p.PB12);
    let mut pwm = SimplePwm::new(
        p.TIM1,
        None,
        None,
        None,
        Some(pin),
        Hertz::khz(1),
        CountingMode::default(),
    );

    let max_duty = pwm.get_max_duty();
    println!("max duty: {}", max_duty);
    pwm.set_duty(hal::timer::Channel::Ch4, 7000);
    pwm.enable(hal::timer::Channel::Ch4);

    loop {
        for i in 0..100 {
            pwm.set_duty(hal::timer::Channel::Ch4, i * 80);
            Timer::after_millis(5).await;
        }
        for i in (0..100).rev() {
            pwm.set_duty(hal::timer::Channel::Ch4, i * 80);
            Timer::after_millis(5).await;
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
