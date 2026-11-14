//! WS2812 panel over the PIOC on CH32X035 (`PC18`, IO0).
//!
//! The first PIOC target outside CH32H4. X0 releases the block with a reset
//! pulse instead of an RCC clock enable, and the core clock is
//! `SYS_CFG.MST_CLK_GATE`, which the driver drives.
//!
//! The effects are deliberately independent of how the panel chains its pixels,
//! because this board is wired differently from the H417 one: a single LED
//! through the SFR path, then the whole panel in one cycling colour, then one
//! pixel walking the chain. Timing and "every pixel is reachable" are the
//! questions here, not the picture.
//!
//! ```text
//! cargo build --release --bin pioc_rainbow
//! ```
//!
//! `Config::release_swd_pins` frees the RVSWD pins (RM §8.3.2.1). Afterwards the
//! debugger cannot attach; reset into the system bootloader to program again.

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::pioc::Pioc;
use panic_halt as _;
use pioc_programs::ws2812::{
    bit_cycles_for_hz, ColorOrder, Config, OutputPin, Rgb8, Storage, Ws2812,
};
use pioc_programs::Firmware;

/// WCH RGB1W in the canonical dialect, assembled for a 48 MHz PIOC clock: the
/// SFR damage loops are `nop`s of a fixed length, so the image is tied to the
/// clock it was built for. CH32X035 runs HCLK at 48 MHz out of reset.
const FIRMWARE: Firmware = Firmware {
    words: pioc_proc::pioc_file_canonical!("firmware/rgb1w-48m.pio"),
    sfr_bit_cycles: 56,
};

const PIXELS: usize = 64;

/// Brightness cap for a panel that may be bus powered.
const LEVEL: u8 = 0x10;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // X0's default divides HSI by 6 (8 MHz HCLK); RGB1W's 1.17 us bit and the
    // 48 MHz image both assume the undivided clock.
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        // `PC18` is RVSWD's SWDIO pin, so the PIOC needs it back.
        release_swd_pins: true,
        ..Default::default()
    });

    let bit_cycles = bit_cycles_for_hz(hal::rcc::clocks().hclk.0)
        .expect("48 MHz HCLK must give a representable WS2812 bit");
    assert_eq!(bit_cycles, 56, "RGB1W's 1.17 us bit at 48 MHz");

    // `PC18` is PIOC IO0 without a remap; `PC7` would need `PCFR1.PIOC_RM = 1`.
    let mut pioc: Pioc<'_, _> = Pioc::new_io0(p.PIOC, p.PC18);
    pioc.load_image(FIRMWARE.words)
        .expect("RGB1W is 576 words, well inside the 4 KiB program SRAM");

    let mut ws = Ws2812::new(
        pioc,
        FIRMWARE,
        Config {
            output: OutputPin::Io0,
            storage: Storage::Auto,
            bit_cycles,
        },
    );

    // The SFR path: one pixel, the vendor's short command. Best effort: with
    // nothing wired to the pin it floats high and RGB1W answers status 6
    // (`InputHigh`), which is a wiring fact, not a driver failure.
    let _ = ws.write_rgb(&[Rgb8::new(0x40, 0, 0)], ColorOrder::Grb).await;
    Timer::after_millis(800).await;

    let mut frames = 0u32;
    loop {
        // One colour for the whole panel, stepping through the wheel. Wiring
        // order cannot hide a failure here.
        let mut frame = [Rgb8::default(); PIXELS];
        let hue = frames % 96;
        let colour = wheel(hue);
        frame.fill(colour);
        if ws.write_rgb(&frame, ColorOrder::Grb).await.is_err() {
            // The line reads high (nothing attached, or a dead pixel); keep
            // cycling instead of stopping the test.
            Timer::after_millis(100).await;
            continue;
        }
        frames = frames.wrapping_add(1);
        Timer::after_millis(60).await;

        // Every 32 frames, walk one pixel along the chain in raw order.
        if frames % 32 == 0 {
            for pixel in 0..PIXELS {
                let mut frame = [Rgb8::default(); PIXELS];
                frame[pixel] = Rgb8::new(LEVEL, LEVEL, LEVEL);
                let _ = ws.write_rgb(&frame, ColorOrder::Grb).await;
                Timer::after_millis(40).await;
            }
        }
    }
}

/// 96-step colour wheel, scaled to [`LEVEL`].
fn wheel(step: u32) -> Rgb8 {
    let step = step % 96;
    let level = |value: u32| (value * u32::from(LEVEL) / 32) as u8;
    match step / 16 {
        0 => Rgb8::new(level(32), level(step % 16 * 2), 0),
        1 => Rgb8::new(level(32 - step % 16 * 2), level(32), 0),
        2 => Rgb8::new(0, level(32), level(step % 16 * 2)),
        3 => Rgb8::new(0, level(32 - step % 16 * 2), level(32)),
        4 => Rgb8::new(level(step % 16 * 2), 0, level(32)),
        _ => Rgb8::new(level(32), 0, level(32 - step % 16 * 2)),
    }
}
