#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use hal::gpio::{Level, Output};
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.clock = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSE;
    let p = hal::init(config);

    let mut led = Output::new(p.PD6, Level::Low, Default::default());
    loop {
        led.toggle();

        riscv::asm::delay(1000000);
    }
}
