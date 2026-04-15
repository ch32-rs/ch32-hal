//! WS2812 LED control using Async SPI + DMA
//!
//! This uses the async SPI driver with DMA for non-blocking transfers.
//!
//! Hardware: PA7 (SPI1_MOSI) -> WS2812 DIN
//! DMA: SPI1_TX uses DMA1_CH3

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use embedded_hal_async::spi::SpiBus;
use hal::gpio::{Level, Output};
use hal::println;
use hal::spi::{Config, Spi};
use hal::time::Hertz;

const NUM_LEDS: usize = 64;
const BYTES_PER_LED: usize = 12;
const RESET_BYTES: usize = 32; // Multiple of 4 for alignment
const BUFFER_LEN: usize = NUM_LEDS * BYTES_PER_LED + RESET_BYTES;

// Ensure 4-byte alignment for DMA
#[repr(C, align(4))]
struct AlignedBuffer([u8; BUFFER_LEN]);

static mut SPI_BUFFER: AlignedBuffer = AlignedBuffer([0u8; BUFFER_LEN]);

const BIT_0: u8 = 0x8;
const BIT_1: u8 = 0xE;

#[embassy_executor::task]
async fn blinky(pin: hal::Peri<'static, hal::peripherals::PA8>) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("WS2812 Async SPI + DMA Example ({} LEDs)", NUM_LEDS);

    // Start blinky task
    spawner.spawn(blinky(p.PA8).unwrap());
    println!("Blinky task started (200ms interval)");

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz::mhz(6);

    // Async SPI with DMA - SPI1_TX uses DMA1_CH3
    let mut spi = Spi::new_txonly(
        p.SPI1,
        p.PA5,      // SCK
        p.PA7,      // MOSI -> WS2812 DIN
        p.DMA1_CH3, // TX DMA channel
        spi_config,
    );

    println!("Async SPI + DMA initialized at 6MHz");

    let buffer = unsafe { &mut SPI_BUFFER.0 };

    // Initialize reset bytes
    for i in (NUM_LEDS * BYTES_PER_LED)..BUFFER_LEN {
        buffer[i] = 0x00;
    }

    println!("Buffer addr: {:p}, len: {}", buffer.as_ptr(), buffer.len());
    println!("Starting rainbow animation with async DMA...");

    let mut offset = 0u8;
    let mut frame = 0u32;

    loop {
        // Build rainbow pattern
        for led in 0..NUM_LEDS {
            let hue = offset.wrapping_add((led as u8) * 4);
            let (r, g, b) = hsv_to_rgb(hue, 255, 32);
            set_led_spi(buffer, led, g, r, b);
        }

        // Async DMA write
        let _ = spi.write(buffer).await;
        
        // WS2812 reset time - use spin delay instead of timer
        for _ in 0..10000 { core::hint::spin_loop(); }

        if frame % 100 == 0 {
            println!("Frame {}", frame);
        }

        offset = offset.wrapping_add(2);
        frame += 1;
        
        // ~30fps animation - use spin delay (~33ms at 144MHz)
        for _ in 0..40000 { core::hint::spin_loop(); }
    }
}

fn byte_to_spi(val: u8, out: &mut [u8]) {
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
    byte_to_spi(g, &mut buffer[base..base + 4]);
    byte_to_spi(r, &mut buffer[base + 4..base + 8]);
    byte_to_spi(b, &mut buffer[base + 8..base + 12]);
}

fn hsv_to_rgb(h: u8, s: u8, v: u8) -> (u8, u8, u8) {
    if s == 0 { return (v, v, v); }
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

