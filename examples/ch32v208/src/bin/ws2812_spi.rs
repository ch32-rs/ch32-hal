//! WS2812 LED control using SPI
//!
//! This method is more stable than PWM+DMA because SPI hardware
//! provides precise timing.
//!
//! Hardware setup:
//! - WS2812 DIN connected to PA7 (SPI1_MOSI)
//! - PA8 optional for blinky indicator
//!
//! SPI encoding:
//! - SPI clock: 6MHz (each bit = 0.167μs)
//! - WS2812 bit 0: 0x8 (1000) = ~0.33μs high, ~0.5μs low
//! - WS2812 bit 1: 0xE (1110) = ~0.67μs high, ~0.17μs low
//! - Each WS2812 byte needs 4 SPI bytes (8 bits * 4 bits/bit / 8)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::println;
use hal::spi::{Config, Spi};
use hal::time::Hertz;

const NUM_LEDS: usize = 64;
// Each LED = 24 bits, each bit = 4 SPI bits = 0.5 byte
// So each LED = 24 * 4 / 8 = 12 SPI bytes
const BYTES_PER_LED: usize = 12;
const RESET_BYTES: usize = 25; // ~50μs reset at 6MHz = 300 bits = 37.5 bytes, use 25 for safety
const BUFFER_LEN: usize = NUM_LEDS * BYTES_PER_LED + RESET_BYTES;

static mut SPI_BUFFER: [u8; BUFFER_LEN] = [0u8; BUFFER_LEN];

// WS2812 bit encoding for SPI
// At 6MHz SPI: each SPI bit = 0.167μs
// WS2812 bit 0: T0H=0.4μs, T0L=0.85μs → need ~0.4μs high
// WS2812 bit 1: T1H=0.8μs, T1L=0.45μs → need ~0.8μs high
// Using 4 SPI bits per WS2812 bit:
// - 0 → 0b1000 = 0x8 → 0.167μs high, 0.5μs low
// - 1 → 0b1110 = 0xE → 0.5μs high, 0.167μs low
const BIT_0: u8 = 0x8;
const BIT_1: u8 = 0xE;

// Blinky task
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
    let p = hal::init(Default::default());

    println!("WS2812 SPI Example ({} LEDs)", NUM_LEDS);
    println!("Buffer size: {} bytes", BUFFER_LEN);

    // Start blinky
    spawner.spawn(blinky(p.PA8)).unwrap();

    // Configure SPI1
    // PA7 = MOSI (data out to WS2812)
    let mut spi_config = Config::default();
    spi_config.frequency = Hertz::mhz(6); // 6MHz for WS2812 timing

    let mut spi = Spi::new_blocking_txonly::<0>(
        p.SPI1,
        p.PA5, // SCK (directly connected, but we only care about MOSI)
        p.PA7, // MOSI -> WS2812 DIN
        spi_config,
    );

    println!("SPI initialized at 6MHz");

    let buffer = unsafe { &mut SPI_BUFFER };

    // Initialize reset bytes at end
    for i in (NUM_LEDS * BYTES_PER_LED)..BUFFER_LEN {
        buffer[i] = 0x00;
    }

    println!("Starting rainbow animation...");

    let mut offset = 0u8;
    let mut frame = 0u32;

    loop {
        // Build rainbow pattern
        for led in 0..NUM_LEDS {
            let hue = offset.wrapping_add((led as u8) * (255 / NUM_LEDS as u8));
            let (r, g, b) = hsv_to_rgb(hue, 255, 32); // Low brightness
            set_led_spi(buffer, led, g, r, b);
        }

        // Send via SPI
        match spi.blocking_write(buffer) {
            Ok(_) => {}
            Err(e) => {
                println!("SPI error: {:?}", e);
            }
        }

        if frame % 100 == 0 {
            println!("Frame {}", frame);
        }

        offset = offset.wrapping_add(3);
        frame += 1;
        Timer::after_millis(30).await;
    }
}

/// Convert a byte to SPI encoding (4 SPI bytes for 8 WS2812 bits)
/// Each WS2812 bit becomes 4 SPI bits (high nibble of one byte + low nibble of next)
fn byte_to_spi(val: u8, out: &mut [u8]) {
    // Process 2 WS2812 bits at a time (= 1 SPI byte)
    // Bit 7,6 -> out[0]
    // Bit 5,4 -> out[1]
    // Bit 3,2 -> out[2]
    // Bit 1,0 -> out[3]
    for i in 0..4 {
        let bit_high = (val >> (7 - i * 2)) & 1;
        let bit_low = (val >> (6 - i * 2)) & 1;
        let high_nibble = if bit_high == 1 { BIT_1 } else { BIT_0 };
        let low_nibble = if bit_low == 1 { BIT_1 } else { BIT_0 };
        out[i] = (high_nibble << 4) | low_nibble;
    }
}

fn set_led_spi(buffer: &mut [u8], led: usize, g: u8, r: u8, b: u8) {
    let base = led * BYTES_PER_LED;
    // GRB order
    byte_to_spi(g, &mut buffer[base..base + 4]);
    byte_to_spi(r, &mut buffer[base + 4..base + 8]);
    byte_to_spi(b, &mut buffer[base + 8..base + 12]);
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

