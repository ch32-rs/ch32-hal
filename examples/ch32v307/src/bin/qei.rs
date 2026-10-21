#![no_std]
#![no_main]

//! Quadrature encoder on TIM5: A on PA0, B on PA1 (to GND through the encoder's switches,
//! pulled up here). Prints the count over SDI whenever it changes.

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::Pull;
use hal::pac::timer::vals::FilterValue;
use hal::println;
use hal::timer::qei::{Qei, QeiPin};
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    hal::debug::SDIPrint::enable();

    // the filter ignores pulses shorter than ~2 us: contact bounce
    let qei = Qei::new(
        p.TIM5,
        QeiPin::new_ch1(p.PA0, Pull::Up),
        QeiPin::new_ch2(p.PA1, Pull::Up),
        FilterValue::FDTS_DIV32_N8,
    );

    let mut last = qei.count();
    println!("encoder: {}", last);
    loop {
        let now = qei.count();
        if now != last {
            // the counter wraps: the signed difference is the movement
            println!("encoder: {} ({:+})", now, now.wrapping_sub(last) as i16);
            last = now;
        }
        Timer::after_millis(10).await;
    }
}
