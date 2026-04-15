//! WS2812 8x8 Matrix Effects
//!
//! 8x8 LED matrix with zig-zag layout using SPI
//!
//! Hardware: PA7 (SPI1_MOSI) -> WS2812 DIN

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

const WIDTH: usize = 8;
const HEIGHT: usize = 8;
const NUM_LEDS: usize = WIDTH * HEIGHT;
const BYTES_PER_LED: usize = 12;
const RESET_BYTES: usize = 25;
const BUFFER_LEN: usize = NUM_LEDS * BYTES_PER_LED + RESET_BYTES;

static mut SPI_BUFFER: [u8; BUFFER_LEN] = [0u8; BUFFER_LEN];

const BIT_0: u8 = 0x8;
const BIT_1: u8 = 0xE;

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

    println!("WS2812 8x8 Matrix Effects");

    spawner.spawn(blinky(p.PA8)).unwrap();

    let mut spi_config = Config::default();
    spi_config.frequency = Hertz::mhz(6);

    let mut spi = Spi::new_blocking_txonly::<0>(
        p.SPI1,
        p.PA5,
        p.PA7,
        spi_config,
    );

    let buffer = unsafe { &mut SPI_BUFFER };
    
    // Initialize reset bytes
    for i in (NUM_LEDS * BYTES_PER_LED)..BUFFER_LEN {
        buffer[i] = 0x00;
    }

    // Framebuffer: RGB values for each LED
    let mut fb = [[0u8; 3]; NUM_LEDS]; // [G, R, B]

    let mut effect = 0u8;
    let mut frame = 0u32;

    println!("Press reset to cycle effects");

    loop {
        // Clear framebuffer
        for led in fb.iter_mut() {
            *led = [0, 0, 0];
        }

        // Apply current effect
        match effect {
            0 => effect_rainbow_wave(&mut fb, frame),
            1 => effect_plasma(&mut fb, frame),
            2 => effect_spiral(&mut fb, frame),
            3 => effect_rain(&mut fb, frame),
            4 => effect_breathing(&mut fb, frame),
            _ => effect = 0,
        }

        // Copy framebuffer to SPI buffer
        for (i, led) in fb.iter().enumerate() {
            set_led_spi(buffer, i, led[0], led[1], led[2]);
        }

        // Send to LEDs
        let _ = spi.blocking_write(buffer);

        frame = frame.wrapping_add(1);
        
        // Change effect every 10 seconds
        if frame % 333 == 0 {
            effect = (effect + 1) % 5;
            println!("Effect: {}", effect);
        }

        Timer::after_millis(30).await;
    }
}

/// Convert (x, y) to LED index for row-by-row layout
fn xy_to_index(x: usize, y: usize) -> usize {
    y * WIDTH + x
}

/// Effect 1: Rainbow wave
fn effect_rainbow_wave(fb: &mut [[u8; 3]; NUM_LEDS], frame: u32) {
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let hue = ((x * 32 + y * 32) as u32 + frame * 3) as u8;
            let (r, g, b) = hsv_to_rgb(hue, 255, 40);
            let idx = xy_to_index(x, y);
            fb[idx] = [g, r, b];
        }
    }
}

/// Effect 2: Plasma effect
fn effect_plasma(fb: &mut [[u8; 3]; NUM_LEDS], frame: u32) {
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let v1 = sin8((x as u32 * 32 + frame * 2) as u8);
            let v2 = sin8((y as u32 * 32 + frame * 3) as u8);
            let v3 = sin8(((x + y) as u32 * 16 + frame) as u8);
            let hue = ((v1 as u16 + v2 as u16 + v3 as u16) / 3) as u8;
            let (r, g, b) = hsv_to_rgb(hue, 255, 35);
            let idx = xy_to_index(x, y);
            fb[idx] = [g, r, b];
        }
    }
}

/// Effect 3: Expanding rings from center
fn effect_spiral(fb: &mut [[u8; 3]; NUM_LEDS], frame: u32) {
    // Center at (3.5, 3.5) - use doubled coordinates for integer math
    // Real center = 7 in doubled space (0-14 range for 0-7 pixels)
    const CENTER: i32 = 7; // 3.5 * 2
    
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            // Double the coordinates, add 1 to get pixel center
            let dx = (x as i32 * 2 + 1) - CENTER;
            let dy = (y as i32 * 2 + 1) - CENTER;
            // Manhattan distance (in doubled space)
            let dist = dx.abs() + dy.abs();
            
            let hue = ((dist * 15) as u32 + frame * 4) as u8;
            let (r, g, b) = hsv_to_rgb(hue, 255, 40);
            let idx = xy_to_index(x, y);
            fb[idx] = [g, r, b];
        }
    }
}

/// Effect 4: Rain drops
fn effect_rain(fb: &mut [[u8; 3]; NUM_LEDS], frame: u32) {
    // Simple pseudo-random rain using frame and position
    for x in 0..WIDTH {
        let drop_y = ((frame + x as u32 * 17) % 16) as usize;
        if drop_y < HEIGHT {
            let idx = xy_to_index(x, drop_y);
            fb[idx] = [0, 0, 60]; // Blue drop
        }
        // Trail
        if drop_y > 0 && drop_y <= HEIGHT {
            let idx = xy_to_index(x, drop_y - 1);
            fb[idx] = [0, 0, 20];
        }
    }
}

/// Effect 5: Breathing
fn effect_breathing(fb: &mut [[u8; 3]; NUM_LEDS], frame: u32) {
    let brightness = sin8((frame * 2) as u8) / 6;
    let hue = (frame / 4) as u8;
    let (r, g, b) = hsv_to_rgb(hue, 255, brightness);
    
    for led in fb.iter_mut() {
        *led = [g, r, b];
    }
}

// === Helper functions ===

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

/// Simple sine approximation (0-255 input, 0-255 output)
fn sin8(x: u8) -> u8 {
    // Approximate sine using parabola
    let x = x as i16;
    let y = if x < 128 {
        // Rising half
        let t = x - 64;
        128 + (t * (128 - t.abs())) / 32
    } else {
        // Falling half
        let t = x - 192;
        128 - (t * (128 - t.abs())) / 32
    };
    y.clamp(0, 255) as u8
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}

