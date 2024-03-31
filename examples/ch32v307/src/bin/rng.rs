#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::rng::Rng;
use hal::{bind_interrupts, peripherals, println, rng};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.clock = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    println!("Hello World!");

    let mut rng = Rng::new(p.RNG, Irqs);

    let mut buf = [0u8; 16];
    let _ = rng.async_fill_bytes(&mut buf).await.unwrap();

    println!("random bytes: {:?}", &buf);

    loop {
        Timer::after_secs(1).await;
        let _ = rng.async_fill_bytes(&mut buf).await.unwrap();

        println!("random bytes: {:?}", &buf);
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
