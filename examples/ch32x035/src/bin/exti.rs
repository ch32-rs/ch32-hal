#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use hal::exti::ExtiInput;
use hal::gpio::{Level, Output};
use hal::println;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SdiPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    let button = p.PC3;
    let mut button = ExtiInput::new(button, p.EXTI3, hal::gpio::Pull::None);

    // GPIO
    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    loop {
        button.wait_for_any_edge().await;
        led.toggle()
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
