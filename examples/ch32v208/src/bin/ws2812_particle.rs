//! WS2812 8x8 Matrix Particle Effects
//!
//! Focus on small colorful particles instead of large color blocks
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

// Maximum particles for each effect
const MAX_METEORS: usize = 4;
const MAX_FIREFLIES: usize = 8;
const MAX_BALLS: usize = 3;
const MAX_SPARKS: usize = 16;

// Particle structures
#[derive(Copy, Clone)]
struct Meteor {
    x: i8,
    y: i8,
    speed: u8,
    hue: u8,
    active: bool,
}

#[derive(Copy, Clone)]
struct Firefly {
    x: u8,
    y: u8,
    brightness: u8,
    phase: u8,
    hue: u8,
}

#[derive(Copy, Clone)]
struct Ball {
    x: i16,  // Fixed point 8.8
    y: i16,
    vx: i16,
    vy: i16,
    hue: u8,
}

#[derive(Copy, Clone)]
struct Spark {
    x: i8,
    y: i8,
    vx: i8,
    vy: i8,
    life: u8,
    hue: u8,
}

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

    println!("WS2812 Particle Effects");

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
    for i in (NUM_LEDS * BYTES_PER_LED)..BUFFER_LEN {
        buffer[i] = 0x00;
    }

    // Framebuffer
    let mut fb = [[0u8; 3]; NUM_LEDS];

    // Particle states
    let mut meteors = [Meteor { x: 0, y: -1, speed: 1, hue: 0, active: false }; MAX_METEORS];
    let mut fireflies = [Firefly { x: 0, y: 0, brightness: 0, phase: 0, hue: 0 }; MAX_FIREFLIES];
    let mut balls = [Ball { x: 0, y: 0, vx: 0, vy: 0, hue: 0 }; MAX_BALLS];
    let mut sparks = [Spark { x: 0, y: 0, vx: 0, vy: 0, life: 0, hue: 0 }; MAX_SPARKS];

    // Initialize fireflies
    for (i, f) in fireflies.iter_mut().enumerate() {
        f.x = (i as u8 * 37) % 8;
        f.y = (i as u8 * 23) % 8;
        f.phase = (i as u8 * 31) % 255;
        f.hue = (i as u8 * 32) % 255;
    }

    // Initialize balls
    for (i, b) in balls.iter_mut().enumerate() {
        b.x = ((i as i16 + 1) * 200) % (8 * 256);
        b.y = ((i as i16 + 2) * 150) % (8 * 256);
        b.vx = (i as i16 + 1) * 20 - 30;
        b.vy = (i as i16 + 1) * 15 - 20;
        b.hue = (i as u8 * 85) % 255;
    }

    let mut effect = 0u8;
    let mut frame = 0u32;
    let mut rng_state = 12345u32;

    println!("Effects: 0=Meteors, 1=Fireflies, 2=Balls, 3=Fireworks, 4=Twinkle");

    loop {
        // Fade framebuffer
        for led in fb.iter_mut() {
            led[0] = led[0].saturating_sub(20);
            led[1] = led[1].saturating_sub(20);
            led[2] = led[2].saturating_sub(20);
        }

        // Update current effect
        match effect {
            0 => update_meteors(&mut fb, &mut meteors, frame, &mut rng_state),
            1 => update_fireflies(&mut fb, &mut fireflies, frame),
            2 => update_balls(&mut fb, &mut balls),
            3 => update_fireworks(&mut fb, &mut sparks, frame, &mut rng_state),
            4 => update_twinkle(&mut fb, frame, &mut rng_state),
            _ => effect = 0,
        }

        // Copy to SPI buffer
        for (i, led) in fb.iter().enumerate() {
            set_led_spi(buffer, i, led[0], led[1], led[2]);
        }

        let _ = spi.blocking_write(buffer);

        frame = frame.wrapping_add(1);

        // Change effect every 12 seconds
        if frame % 400 == 0 {
            effect = (effect + 1) % 5;
            println!("Effect: {}", effect);
            // Reset particles
            for m in meteors.iter_mut() { m.active = false; }
            for s in sparks.iter_mut() { s.life = 0; }
        }

        Timer::after_millis(30).await;
    }
}

// Simple pseudo-random number generator
fn next_random(state: &mut u32) -> u32 {
    *state = state.wrapping_mul(1103515245).wrapping_add(12345);
    *state
}

fn xy_to_index(x: usize, y: usize) -> usize {
    y * WIDTH + x
}

// Effect 0: Meteors falling down
fn update_meteors(fb: &mut [[u8; 3]; NUM_LEDS], meteors: &mut [Meteor; MAX_METEORS], frame: u32, rng: &mut u32) {
    // Spawn new meteor occasionally
    if frame % 15 == 0 {
        for m in meteors.iter_mut() {
            if !m.active {
                m.x = (next_random(rng) % 8) as i8;
                m.y = -1;
                m.speed = 1 + (next_random(rng) % 2) as u8;
                m.hue = (next_random(rng) % 256) as u8;
                m.active = true;
                break;
            }
        }
    }

    // Update and draw meteors
    for m in meteors.iter_mut() {
        if !m.active { continue; }

        m.y += m.speed as i8;

        // Draw meteor head
        if m.y >= 0 && m.y < 8 {
            let idx = xy_to_index(m.x as usize, m.y as usize);
            let (r, g, b) = hsv_to_rgb(m.hue, 255, 80);
            fb[idx] = [g, r, b];
        }

        // Draw tail (3 pixels)
        for t in 1..4i8 {
            let ty = m.y - t;
            if ty >= 0 && ty < 8 {
                let idx = xy_to_index(m.x as usize, ty as usize);
                let brightness = 60 - t as u8 * 15;
                let (r, g, b) = hsv_to_rgb(m.hue, 255, brightness);
                fb[idx][0] = fb[idx][0].saturating_add(g);
                fb[idx][1] = fb[idx][1].saturating_add(r);
                fb[idx][2] = fb[idx][2].saturating_add(b);
            }
        }

        // Deactivate when off screen
        if m.y > 10 {
            m.active = false;
        }
    }
}

// Effect 1: Fireflies blinking
fn update_fireflies(fb: &mut [[u8; 3]; NUM_LEDS], fireflies: &mut [Firefly; MAX_FIREFLIES], frame: u32) {
    for f in fireflies.iter_mut() {
        // Update phase
        f.phase = f.phase.wrapping_add(5);
        
        // Calculate brightness from sine-like curve
        f.brightness = sin8(f.phase) / 4;

        // Move occasionally
        if frame % 30 == 0 {
            f.x = (f.x + 1) % 8;
            if f.x == 0 {
                f.y = (f.y + 1) % 8;
            }
            f.hue = f.hue.wrapping_add(10);
        }

        // Draw
        if f.brightness > 10 {
            let idx = xy_to_index(f.x as usize, f.y as usize);
            let (r, g, b) = hsv_to_rgb(f.hue, 200, f.brightness);
            fb[idx][0] = fb[idx][0].saturating_add(g);
            fb[idx][1] = fb[idx][1].saturating_add(r);
            fb[idx][2] = fb[idx][2].saturating_add(b);
        }
    }
}

// Effect 2: Bouncing balls
fn update_balls(fb: &mut [[u8; 3]; NUM_LEDS], balls: &mut [Ball; MAX_BALLS]) {
    for b in balls.iter_mut() {
        // Update position (fixed point)
        b.x += b.vx;
        b.y += b.vy;

        // Bounce off walls
        if b.x < 0 { b.x = 0; b.vx = -b.vx; }
        if b.x >= 8 * 256 { b.x = 8 * 256 - 1; b.vx = -b.vx; }
        if b.y < 0 { b.y = 0; b.vy = -b.vy; }
        if b.y >= 8 * 256 { b.y = 8 * 256 - 1; b.vy = -b.vy; }

        // Draw ball
        let px = (b.x / 256) as usize;
        let py = (b.y / 256) as usize;
        if px < 8 && py < 8 {
            let idx = xy_to_index(px, py);
            let (r, g, bb) = hsv_to_rgb(b.hue, 255, 60);
            fb[idx] = [g, r, bb];
        }

        // Update hue slowly
        b.hue = b.hue.wrapping_add(1);
    }
}

// Effect 3: Fireworks
fn update_fireworks(fb: &mut [[u8; 3]; NUM_LEDS], sparks: &mut [Spark; MAX_SPARKS], frame: u32, rng: &mut u32) {
    // Launch new firework occasionally
    if frame % 40 == 0 {
        let cx = (next_random(rng) % 6 + 1) as i8;
        let cy = (next_random(rng) % 4 + 1) as i8;
        let hue = (next_random(rng) % 256) as u8;

        // Create sparks
        for (i, s) in sparks.iter_mut().enumerate() {
            if s.life == 0 {
                s.x = cx * 8; // Fixed point
                s.y = cy * 8;
                // Spread in all directions
                let angle = i as i8 * 2;
                s.vx = (angle % 5) - 2;
                s.vy = (angle / 5) - 2;
                if s.vx == 0 && s.vy == 0 { s.vy = 1; }
                s.life = 20 + (next_random(rng) % 10) as u8;
                s.hue = hue.wrapping_add((i as u8) * 5);
            }
        }
    }

    // Update sparks
    for s in sparks.iter_mut() {
        if s.life == 0 { continue; }

        s.x += s.vx;
        s.y += s.vy;
        s.life = s.life.saturating_sub(1);

        // Draw spark
        let px = s.x / 8;
        let py = s.y / 8;
        if px >= 0 && px < 8 && py >= 0 && py < 8 {
            let idx = xy_to_index(px as usize, py as usize);
            let brightness = (s.life as u16 * 4).min(80) as u8;
            let (r, g, b) = hsv_to_rgb(s.hue, 255, brightness);
            fb[idx][0] = fb[idx][0].saturating_add(g);
            fb[idx][1] = fb[idx][1].saturating_add(r);
            fb[idx][2] = fb[idx][2].saturating_add(b);
        }
    }
}

// Effect 4: Random twinkling stars
fn update_twinkle(fb: &mut [[u8; 3]; NUM_LEDS], _frame: u32, rng: &mut u32) {
    // Light up random pixels
    for _ in 0..2 {
        let x = (next_random(rng) % 8) as usize;
        let y = (next_random(rng) % 8) as usize;
        let hue = (next_random(rng) % 256) as u8;
        let brightness = 30 + (next_random(rng) % 40) as u8;

        let idx = xy_to_index(x, y);
        let (r, g, b) = hsv_to_rgb(hue, 255, brightness);
        fb[idx] = [g, r, b];
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

fn sin8(x: u8) -> u8 {
    let x = x as i16;
    let y = if x < 128 {
        let t = x - 64;
        128 + (t * (128 - t.abs())) / 32
    } else {
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

