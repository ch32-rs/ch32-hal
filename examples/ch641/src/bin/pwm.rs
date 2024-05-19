#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use hal::delay::Delay;
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let pin = PwmPin::new_ch2::<0>(p.PB3);
    // let pin = PwmPin::new_ch3::<0>(p.PB4);
    // let pin = PwmPin::new_ch3::<1>(p.PA2);
    // let pin = PwmPin::new_ch2::<1>(p.PA4);
    let ch = hal::timer::Channel::Ch2;
    let mut pwm = SimplePwm::new(
        p.TIM1,
        None,
        Some(pin),
        None,
        None,
        Hertz::khz(1),
        CountingMode::default(),
    );

    let _max_duty = pwm.get_max_duty();
    pwm.set_duty(ch, 2000);
    pwm.enable(ch);

    loop {
        for i in 0..100 {
            pwm.set_duty(ch, i * 80);
            Delay.delay_ms(1_0);
        }
        for i in (0..100).rev() {
            pwm.set_duty(ch, i * 80);
            Delay.delay_ms(1_0);
        }
    }
}
