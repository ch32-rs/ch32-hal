#![no_std]
#![no_main]



use hal::gpio::{Level, Output};
use qingke::riscv;
use {ch32_hal as hal, panic_halt as _};

#[qingke_rt::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PB8, Level::Low, Default::default());
    loop {
        led.toggle();

        unsafe {
            riscv::asm::delay(1000000);
        }
    }
}
