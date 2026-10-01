#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

//! PWM LED breathing effect example for CH32V208
//!
//! This example demonstrates how to use PWM to create a smooth LED breathing effect.
//! The LED gradually fades in and out using a sine-wave-like pattern for a natural look.
//!
//! Hardware:
//! - LED connected to PA15 (TIM2_CH1, remap=1)
//! - Other TIM2 options: PA0 (CH1, remap=0), PB3 (CH2, remap=1)

use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use hal::timer::Channel;
use {ch32_hal as hal, panic_halt as _};

// Gamma correction table for smoother LED fading (8-bit to perceived brightness)
// This makes the LED brightness appear more linear to human eyes
const GAMMA_TABLE: [u16; 64] = [
    0, 1, 2, 3, 4, 5, 7, 9, 12, 15, 18, 22, 27, 32, 38, 44, 51, 58, 67, 76, 86, 96, 108, 120, 134,
    148, 163, 180, 197, 216, 235, 256, 278, 301, 326, 352, 379, 408, 438, 470, 504, 539, 575, 614,
    654, 696, 740, 786, 833, 883, 934, 988, 1044, 1102, 1162, 1224, 1289, 1356, 1425, 1497, 1572,
    1649, 1729, 1811,
];

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("PWM LED Breathing Example");
    println!("CHIP: {}", hal::signature::chip_id().name());

    // Setup PWM on TIM2 CH1 (PA15)
    // TIM2 is a general-purpose 16-bit timer
    let ch1_pin = PwmPin::new_ch1::<1>(p.PA15); // remap=1: PA15 is TIM2_CH1

    let mut pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        None,
        None,
        None,
        Hertz::khz(1), // 1kHz PWM frequency - good for LED dimming
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.get_max_duty();
    println!("PWM initialized, max duty: {}", max_duty);

    // Enable the channel
    pwm.enable(Channel::Ch1);

    // Calculate scaling factor for gamma table
    // GAMMA_TABLE max is 1811, we need to scale to max_duty
    let scale = max_duty / 1811;

    println!("Starting breathing effect...");

    let mut cycle = 0u32;
    loop {
        // Fade in (0 -> 63)
        for i in 0..64 {
            let duty = GAMMA_TABLE[i] as u32 * scale;
            pwm.set_duty(Channel::Ch1, duty);
            Timer::after_millis(20).await;
        }

        // Brief pause at full brightness
        Timer::after_millis(100).await;

        // Fade out (63 -> 0)
        for i in (0..64).rev() {
            let duty = GAMMA_TABLE[i] as u32 * scale;
            pwm.set_duty(Channel::Ch1, duty);
            Timer::after_millis(20).await;
        }

        // Brief pause at minimum brightness
        Timer::after_millis(200).await;

        cycle += 1;
        println!("Breathing cycle: {}", cycle);
    }
}

