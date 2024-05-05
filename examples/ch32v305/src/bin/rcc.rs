#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::println;
use hal::time::Hertz;
use {ch32_hal as hal, panic_halt as _};

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

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    {
        use hal::rcc::*;

        config.rcc = Config {
            hse: Some(Hse {
                freq: Hertz::mhz(12),
                mode: HseMode::Oscillator,
            }),
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                // 12 * 12 = max 144MHz
                // use MUL16 to overclock to 192MHz
                mul: PllMul::MUL12,
            }),
            sys: Sysclk::PLL,
            ..Default::default()
        }
    }
    let p = hal::init(config);
    hal::embassy::init();

    println!("SYS:   {}Hz", hal::rcc::clocks().sysclk.0);
    println!("HCLK:  {}Hz", hal::rcc::clocks().hclk.0);
    println!("PCLK1: {}Hz", hal::rcc::clocks().pclk1.0);
    println!("PCLK2: {}Hz", hal::rcc::clocks().pclk2.0);

    // GPIO
    spawner.spawn(blink(p.PC9.degrade(), 1000)).unwrap();
    spawner.spawn(blink(p.PB4.degrade(), 100)).unwrap();
    spawner.spawn(blink(p.PB8.degrade(), 100)).unwrap();
    let mut tick = 0;
    loop {
        Timer::after_millis(1000).await;

        let t = hal::pac::SYSTICK.cnt().read();
        let dt = t.wrapping_sub(tick);
        println!("systick: {}", dt);
        tick = t;
    }
}
