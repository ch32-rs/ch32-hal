#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{Input, Level, Output, Pull, Speed};
use hal::println;
use {ch32_hal as hal, panic_halt as _};

static BLINK_INTERVAL: AtomicU32 = AtomicU32::new(500);

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        let interval = BLINK_INTERVAL.load(Ordering::Relaxed) as u64;
        led.set_high();
        Timer::after(Duration::from_millis(interval)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());
    hal::embassy::init();

    let button = Input::new(p.PB3, Pull::Up);

    let led = Output::new(p.PA15, Level::Low, Speed::default());

    spawner.spawn(blink(led)).unwrap();

    loop {
        if button.is_low() {
            BLINK_INTERVAL.store(50, Ordering::Relaxed);
            println!("Button pressed");
        } else {
            BLINK_INTERVAL.store(500, Ordering::Relaxed);
        }
        // give CPU to other tasks
        Timer::after(Duration::from_millis(1)).await;
    }
}
