#![no_std]
#![no_main]



use hal::delay::Delay;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    let mut delay = Delay;

    loop {
        led.toggle();
        delay.delay_ms(1000);
    }
}
