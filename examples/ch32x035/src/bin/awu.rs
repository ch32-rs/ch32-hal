//! Auto wakeup timer (AWU) demo for CH32X035.
//!
//! The AWU counts from HSI/1024 and raises an interrupt when the counter
//! reaches the window value, independent of every other timer. This example
//! wakes every 2 s and logs the interval measured by `embassy-time`, which
//! runs on TIM2 here.
//!
//! ```text
//! cd examples/ch32x035 && cargo run --release --bin awu
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use panic_halt as _;

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    // Give the debugger a beat to attach and start the RTT session.
    Timer::after_millis(500).await;

    let mut awu = hal::awu::Awu::new(p.AWU);
    let period = awu.start(2_000_000);
    defmt::info!("AWU started, period {} us (requested 2000000)", period);

    let mut last = Instant::now();
    loop {
        awu.wait().await;

        let now = Instant::now();
        defmt::info!("AWU wakeup after {} ms", (now - last).as_millis());
        last = now;
    }
}
