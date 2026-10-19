//! HCLK / PCLK / V5F prescaler math (RM §3.4).

use core::ops::Div;

use crate::pac::rcc::vals::{Fpre, Hpre, Pllmul, Ppre};
use crate::time::Hertz;

/// AHB domain after `HPRE` (V5F core clock on CH32H4).
pub(crate) fn ahb_from_sysclk(sysclk: Hertz, hpre: Hpre) -> Hertz {
    sysclk / hpre
}

/// HCLK / V3F control-core clock after `FPRE`.
pub(crate) fn hclk_from_ahb(ahb: Hertz, fpre: Fpre) -> Hertz {
    ahb / fpre
}

pub(crate) fn pclk_from_hclk(hclk: Hertz, ppre: Ppre) -> (Hertz, Hertz) {
    let pclk = hclk / ppre;
    // H4 RM table 4.2: when PPRE≠DIV1 the timer kernel clock equals PCLK (no ×2).
    (pclk, pclk)
}

impl Div<Hpre> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: Hpre) -> Hertz {
        Hertz(ahb_hz(self.0, rhs as u32))
    }
}

/// AHB clock from `SYSCLK` and the raw `HPRE` code (RM §3.4.2).
///
/// The encoding has a hole: `0xxx` is ÷1, `1000`–`1011` step by powers of two
/// from ÷2 to ÷16, and `1100` jumps straight to ÷64. A plain
/// `raw - 0b1000 + 1` shift decodes `1100` as ÷32.
pub fn ahb_hz(sysclk_hz: u32, hpre: u32) -> u32 {
    if hpre >= 0b1100 {
        sysclk_hz >> (hpre - 0b0110)
    } else if hpre >= 0b1000 {
        sysclk_hz >> (hpre - 0b0111)
    } else {
        sysclk_hz
    }
}

/// HCLK from the AHB clock and the raw `FPRE` code (RM §3.4.2).
///
/// `00` is ÷1, `01` is ÷2, and `1x` is ÷4: the top bit is don't-care, so
/// `0b11` divides by 4 too. The field is only two bits wide.
pub fn hclk_hz(ahb_hz: u32, fpre: u32) -> u32 {
    match fpre & 0b11 {
        0b01 => ahb_hz / 2,
        0b10 | 0b11 => ahb_hz / 4,
        _ => ahb_hz,
    }
}

/// PCLK from HCLK and the raw `PPRE` code: `0xx` = ÷1, then ÷2, ÷4, ÷8, ÷16.
pub fn pclk_hz(hclk_hz: u32, ppre: u32) -> u32 {
    if ppre >= 0b100 {
        hclk_hz >> (ppre - 0b011)
    } else {
        hclk_hz
    }
}

/// `PLLMUL` as a multiplier doubled, so the .5 codes stay integral (RM §3.4.3).
///
/// The five half-step codes (x8.5, x9.5, x10.5, x11.5, x12.5) sit between the
/// integers and the table has holes (no x5, no x21, x23, ...), so the factor
/// comes from the enum rather than from `code + n`.
pub fn pll_mul_x2(mul: Pllmul) -> u32 {
    use Pllmul::*;
    match mul {
        MUL4 => 8,
        MUL6 => 12,
        MUL7 => 14,
        MUL8 => 16,
        MUL8_5 => 17,
        MUL9 => 18,
        MUL9_5 => 19,
        MUL10 => 20,
        MUL10_5 => 21,
        MUL11 => 22,
        MUL11_5 => 23,
        MUL12 => 24,
        MUL12_5 => 25,
        MUL13 => 26,
        MUL14 => 28,
        MUL15 => 30,
        MUL16 => 32,
        MUL17 => 34,
        MUL18 => 36,
        MUL19 => 38,
        MUL20 => 40,
        MUL22 => 44,
        MUL24 => 48,
        MUL26 => 52,
        MUL28 => 56,
        MUL30 => 60,
        MUL32 => 64,
        MUL34 => 68,
        MUL36 => 72,
        MUL38 => 76,
        MUL40 => 80,
        MUL59 => 118,
    }
}

impl Div<Ppre> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: Ppre) -> Hertz {
        let raw = rhs as u32;
        if raw >= 0b100 {
            let d = raw - 0b100 + 1;
            Hertz(self.0 >> d)
        } else {
            self
        }
    }
}

impl Div<Fpre> for Hertz {
    type Output = Hertz;
    fn div(self, rhs: Fpre) -> Hertz {
        // `Fpre::_RESERVED_3` (0b11) is not reserved: RM §3.4.2 encodes ÷4 as
        // `1x`, so both 0b10 and 0b11 divide by 4. `from_bits` keeps 0b11.
        Hertz(hclk_hz(self.0, rhs as u32))
    }
}
