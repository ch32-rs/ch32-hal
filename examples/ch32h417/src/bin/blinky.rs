#![no_std]
#![no_main]

use hal::delay::Delay;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();
        unsafe { core::ptr::write_volatile(0x200a0000 as *mut u32, counter); }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
