#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use hal::delay::Delay;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let pin = PwmPin::new_ch4::<0>(p.PC4);
    let mut pwm = SimplePwm::new(
        p.TIM1,
        None,
        None,
        None,
        Some(pin),
        Hertz::khz(1),
        CountingMode::default(),
    );
    let ch = hal::timer::Channel::Ch4;

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
