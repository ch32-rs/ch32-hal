//! Panel mapping shared by the PIOC matrix examples.
//!
//! An 8x8 WS2812 module chains its 64 LEDs in some order, and the picture only
//! comes out upright once the code knows that order. Two things vary: whether
//! the chain runs along rows or columns and reverses every other line, and
//! where chain position 0 sits on the module.

#![allow(dead_code)]

use pioc_programs::ws2812::Rgb8;

/// Panel geometry.
pub const WIDTH: usize = 8;
pub const HEIGHT: usize = 8;
pub const PIXELS: usize = WIDTH * HEIGHT;

/// Brightness cap. 64 pixels at full white would draw close to 4 A, which most
/// USB supplies will not deliver; even this cap is enough to dip the rail on a
/// bus-powered board.
pub const LEVEL: u8 = 0x10;

/// How the panel chains its LEDs.
///
/// Getting this wrong does not corrupt anything, it mirrors or shuffles parts
/// of the picture. A single pixel walked through the chain, and a letter drawn
/// on the panel, tell the two apart: the walk shows the snake, the letter shows
/// the mirror.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Wiring {
    /// Left to right, top to bottom.
    RowStraight,
    /// Left to right, then right to left on the next row.
    RowZigzag,
    /// Top to bottom, left to right.
    ColumnStraight,
    /// Top to bottom, then bottom to top in the next column.
    ColumnZigzag,
}

/// Chain layout of the panel at hand: rows, every other one reversed.
pub const WIRING: Wiring = Wiring::RowZigzag;

/// Whether the module mounts chain position 0 at the top *right*.
///
/// Text drawn without this reads as a mirror image, while a walking pixel looks
/// the same either way, so a mirrored glyph is the signal to flip it.
pub const MIRROR_X: bool = true;

/// Linear pixel index for a visual `(x, y)`.
pub const fn index(x: usize, y: usize) -> usize {
    let x = if MIRROR_X { WIDTH - 1 - x } else { x };
    match WIRING {
        Wiring::RowStraight => y * WIDTH + x,
        Wiring::RowZigzag => y * WIDTH + if y % 2 == 0 { x } else { WIDTH - 1 - x },
        Wiring::ColumnStraight => x * HEIGHT + y,
        Wiring::ColumnZigzag => x * HEIGHT + if x % 2 == 0 { y } else { HEIGHT - 1 - y },
    }
}

/// A blank frame.
pub const fn clear() -> [Rgb8; PIXELS] {
    [Rgb8::new(0, 0, 0); PIXELS]
}

/// 96-step colour wheel.
///
/// `LEVEL` scales the result, so a full-intensity entry still stays inside the
/// panel's current budget.
pub fn wheel(step: u32) -> Rgb8 {
    let step = step % 96;
    let level = |value: u32| (value * LEVEL as u32 / 32) as u8;
    match step / 16 {
        0 => Rgb8::new(level(32), level(step % 16 * 2), 0),
        1 => Rgb8::new(level(32 - step % 16 * 2), level(32), 0),
        2 => Rgb8::new(0, level(32), level(step % 16 * 2)),
        3 => Rgb8::new(0, level(32 - step % 16 * 2), level(32)),
        4 => Rgb8::new(level(step % 16 * 2), 0, level(32)),
        _ => Rgb8::new(level(32), 0, level(32 - step % 16 * 2)),
    }
}

/// Green level meter `level` pixels tall, filled in visual order, with a white
/// head so the tip is easy to follow.
pub fn meter(level: usize) -> [Rgb8; PIXELS] {
    let mut frame = clear();
    for step in 0..level {
        let (x, y) = (step % WIDTH, step / WIDTH);
        frame[index(x, y)] = if step + 1 == level {
            Rgb8::new(LEVEL, LEVEL, LEVEL)
        } else {
            Rgb8::new(0, LEVEL, 0)
        };
    }
    frame
}
