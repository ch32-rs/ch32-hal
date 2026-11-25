// Example of TKEY (touch key detection) that lights an LED when a touch is detected.
//
// This example demonstrates:
// 1. Configuring a single touch key
// 2. Using the async TKEY interface
// 3. Waiting for acquisition completion using `pend_for_acquisition`
// 4. Reading touch values and controlling an LED based on the results
//
// Physical setup: a touch pad, or a loose wire, on PA6 (ADC channel 6) and an LED on PB12.
//
// The program takes the untouched level of the key at start-up, then keeps reading the key.
// A touch adds capacitance, which lowers the value: the LED is on while the value is more
// than 1/16 below the untouched level. Touch values are printed over SDI.
//
// Troubleshooting:
// - A value at the top of the range (4095) leaves no room to detect a touch: shorten the
//   charge with `charge_time` and `charge_offset`. If the value hardly moves, lengthen it.

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::tkey::{Config, InterruptHandler, Tkey};
use hal::{bind_interrupts, peripherals, println};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ADC1 => InterruptHandler<peripherals::ADC1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    let mut config = Config::default();
    // An ADC clock of 8 MHz, the fastest the ADC takes: 48 MHz / 6
    config.adc.clkdiv = 5;

    let mut touch_controller = Tkey::new_async(p.ADC1, Irqs, config);
    let mut key = p.PA6;
    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    touch_controller.set_active_channel(&mut key);

    // The untouched level: keep away from the key during start-up
    touch_controller.start();
    touch_controller.pend_for_acquisition().await;
    let untouched = touch_controller.get_value();
    println!("untouched level {}", untouched);

    loop {
        touch_controller.start();
        touch_controller.pend_for_acquisition().await;
        let value = touch_controller.get_value();
        println!("key value {}", value);

        if value < untouched - untouched / 16 {
            led.set_high();
        } else {
            led.set_low();
        }

        Timer::after_millis(100).await;
    }
}
