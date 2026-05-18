#![no_std]
#![no_main]



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

    // TIM1_RM is a 1-bit field on ch641, so the marker is `AfioRemapBool`.
    let pin = PwmPin::new_ch2::<hal::gpio::AfioRemapBool<false>>(p.PB3);
    // let pin = PwmPin::new_ch3::<hal::gpio::AfioRemapBool<false>>(p.PB4);
    // let pin = PwmPin::new_ch3::<hal::gpio::AfioRemapBool<true>>(p.PA2);
    // let pin = PwmPin::new_ch2::<hal::gpio::AfioRemapBool<true>>(p.PA4);
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
