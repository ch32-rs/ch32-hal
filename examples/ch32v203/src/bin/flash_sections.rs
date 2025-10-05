#![no_std]
#![no_main]



use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::{pac, println};

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[link_section = ".coldtext"]
#[inline(never)]
fn hello() {
    println!("hello world! from .coldtext");
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let p = hal::init(config);

    // GPIO
    spawner.spawn(blink(p.PB8.degrade(), 500)).unwrap();
    let mut last = pac::SYSTICK.cnt().read();
    loop {
        Timer::after_millis(1000).await;
        let cnt = pac::SYSTICK.cnt().read();
        let elapsed = cnt.wrapping_sub(last);
        last = cnt;
        println!("tick");
        println!("systick: {}", elapsed);

        hello();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
