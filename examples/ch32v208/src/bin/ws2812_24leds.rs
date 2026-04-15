//! WS2812 LED strip control - 24 LEDs
//!
//! Demonstrates rainbow animation on a 24-LED WS2812 strip using PWM + DMA.
//!
//! Hardware setup:
//! - WS2812 DIN connected to PA15 (TIM2_CH1, remap=1)
//! - PA8 optional for blinky indicator

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use hal::timer::Channel;

const NUM_LEDS: usize = 24;
const RESET_LEN: usize = 50;
const BUFFER_LEN: usize = NUM_LEDS * 24 + RESET_LEN;

static mut DMA_BUFFER: [u16; BUFFER_LEN] = [0u16; BUFFER_LEN];

// Blinky task to show system is alive
#[embassy_executor::task]
async fn blinky(pin: hal::Peri<'static, hal::peripherals::PA8>) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut p = hal::init(Default::default());

    println!("WS2812 24-LED Example");

    // Start blinky
    spawner.spawn(blinky(p.PA8)).unwrap();

    let ch1_pin = PwmPin::new_ch1::<1>(p.PA15);
    let mut pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        None,
        None,
        None,
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.get_max_duty() as u16;
    let n0 = (3 * max_duty / 10) as u16;
    let n1 = (7 * max_duty / 10) as u16;
    println!("n0={}, n1={}, max_duty={}", n0, n1, max_duty);

    let pwm_channel = Channel::Ch1;
    pwm.set_duty(pwm_channel, 0);
    pwm.enable(pwm_channel);

    let buffer = unsafe { &mut DMA_BUFFER };

    // Initialize reset signal
    for i in (NUM_LEDS * 24)..BUFFER_LEN {
        buffer[i] = 0;
    }

    println!("Starting rainbow animation...");

    let mut offset = 0u8;
    loop {
        // Build rainbow pattern
        for led in 0..NUM_LEDS {
            let hue = offset.wrapping_add((led as u8) * (255 / NUM_LEDS as u8));
            let (r, g, b) = hsv_to_rgb(hue, 255, 32); // Low brightness
            set_led(buffer, led, n0, n1, g, r, b);
        }

        // Send to LEDs
        pwm.waveform_up(p.DMA1_CH2.reborrow(), pwm_channel, buffer).await;

        offset = offset.wrapping_add(2);
        Timer::after_millis(30).await;
    }
}

fn set_led(buffer: &mut [u16], led: usize, n0: u16, n1: u16, g: u8, r: u8, b: u8) {
    let base = led * 24;
    // Green
    for i in 0..8 {
        buffer[base + i] = if (g >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
    // Red
    for i in 0..8 {
        buffer[base + 8 + i] = if (r >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
    // Blue
    for i in 0..8 {
        buffer[base + 16 + i] = if (b >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
}

fn hsv_to_rgb(h: u8, s: u8, v: u8) -> (u8, u8, u8) {
    if s == 0 {
        return (v, v, v);
    }

    let region = h / 43;
    let remainder = (h as u16 - (region as u16 * 43)) * 6;

    let p = ((v as u16) * (255 - s as u16) / 255) as u8;
    let q = ((v as u16) * (255 - ((s as u16 * remainder) / 255)) / 255) as u8;
    let t = ((v as u16) * (255 - ((s as u16 * (255 - remainder)) / 255)) / 255) as u8;

    match region {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}








