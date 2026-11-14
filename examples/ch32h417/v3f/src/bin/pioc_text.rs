//! Scrolling text on an 8x8 WS2812 panel, driven through the PIOC firmware.
//!
//! Same transport as `pioc_ws2812`: WCH's RGB1W image runs in the PIOC program
//! SRAM, the HAL does the host handshake, and every frame is 64 pixels = 192
//! payload bytes, which is a program-RAM transfer.
//!
//! The font is a 5x7 bitmap per glyph with a one-column gap, drawn into an 8-row
//! panel one column at a time and shifted left one column per frame. The colour
//! follows the column, so the letters carry a rainbow as they travel.
//!
//! ```text
//! cargo xtask flash --example pioc_text --v3f-only --features pioc-io0-pe3 \
//!     --flasher probe-rs
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::pioc::Pioc;
use panic_halt as _;
use pioc_programs::ws2812::{bit_cycles_for_hz, ColorOrder, Config, OutputPin, Rgb8, Storage, Ws2812};
use pioc_programs::Firmware;

#[path = "../panel.rs"]
mod panel;

use panel::{clear, index, wheel, PIXELS, WIDTH};

/// WCH RGB1W, translated from the CH32H417 EVT
/// (`EXAM/PIOC/PIOC_1_Wire/Common/Asm/RGB1W.ASM`) into the canonical dialect.
/// The delay loops are assembled for 100 MHz, so the SFR path may only be used
/// when the PIOC clock really is 100 MHz.
const FIRMWARE: Firmware = Firmware {
    words: pioc_proc::pioc_file_canonical!("firmware/rgb1w-100m.pio"),
    sfr_bit_cycles: 117,
};

/// Glyph size and the gap between glyphs, in columns. The glyph is 5x7, so a
/// frame uses seven of the panel's eight rows.
const GLYPH_W: usize = 5;
const GLYPH_H: usize = 7;
const ADVANCE: usize = GLYPH_W + 1;

/// Time per scrolled column.
const STEP_MS: u64 = 100;

/// Brightness cap; 64 pixels at full white would dip a bus-powered rail.
const LEVEL: u8 = 0x10;

/// Text to scroll.
const MESSAGE: &[u8] = b"HELLO WORLD";

/// One vertical slice of a glyph, bit 0 = top row.
fn column(message: &[u8], col: usize) -> u8 {
    let cell = col / ADVANCE;
    let inner = col % ADVANCE;
    if inner >= GLYPH_W {
        return 0;
    }
    let Some(&glyph) = message.get(cell) else {
        return 0;
    };
    let rows = glyph_rows(glyph);
    let mut mask = 0;
    for (y, row) in rows.iter().enumerate() {
        let bit = (row >> (GLYPH_W - 1 - inner)) & 1;
        mask |= bit << y;
    }
    mask
}

/// 5x7 bitmap of one glyph, one byte per row, top row first.
fn glyph_rows(glyph: u8) -> [u8; GLYPH_H] {
    match glyph {
        b'H' => [0x11, 0x11, 0x11, 0x1f, 0x11, 0x11, 0x11],
        b'E' => [0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x1f],
        b'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f],
        b'O' => [0x0e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0e],
        b'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0a],
        b'R' => [0x1e, 0x11, 0x11, 0x1e, 0x14, 0x12, 0x11],
        b'D' => [0x1e, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1e],
        _ => [0x00; GLYPH_H],
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });

    let bit_cycles = bit_cycles_for_hz(hal::rcc::clocks().hclk.0)
        .expect("a 100 MHz PIOC clock must give a representable WS2812 bit");
    assert_eq!(bit_cycles, 117, "RGB1W's 1.17 us bit at 100 MHz");

    #[cfg(not(any(feature = "pioc-io0-pb8", feature = "pioc-io0-pc1", feature = "pioc-io0-pf12")))]
    let io0 = p.PE3;
    #[cfg(feature = "pioc-io0-pb8")]
    let io0 = p.PB8;
    #[cfg(feature = "pioc-io0-pc1")]
    let io0 = p.PC1;
    #[cfg(feature = "pioc-io0-pf12")]
    let io0 = p.PF12;

    let mut pioc = Pioc::new_io0(p.PIOC, io0);
    pioc.load_image(FIRMWARE.words)
        .expect("RGB1W is 1152 bytes, well inside the 4 KiB program SRAM");

    let mut ws = Ws2812::new(
        pioc,
        FIRMWARE,
        Config {
            output: OutputPin::Io0,
            storage: Storage::Auto,
            bit_cycles,
        },
    );

    // The message starts just past the right edge and leaves on the left, so
    // the first glyph slides in like every later one. After `shift` steps the
    // message column `c` sits at `x = WIDTH + c - shift`, which makes `c` for a
    // given column `shift + x - WIDTH` — negative while it is still off-screen.
    let span = MESSAGE.len() * ADVANCE + WIDTH;
    loop {
        for shift in 0..span {
            let mut frame = clear();
            for x in 0..WIDTH {
                let Some(col) = (shift + x).checked_sub(WIDTH) else {
                    continue;
                };
                let mask = column(MESSAGE, col);
                if mask == 0 {
                    continue;
                }
                for y in 0..GLYPH_H {
                    if mask & (1 << y) != 0 {
                        // The hue travels with the column, so the letters keep
                        // their colours while moving.
                        let hue = (col * 4) as u32 % 96;
                        frame[index(x, y)] = wheel(hue);
                    }
                }
            }
            ws.write_rgb(&frame, ColorOrder::Grb).await.expect("text frame");
            Timer::after_millis(STEP_MS).await;
        }
    }
}
