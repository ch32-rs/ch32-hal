//! HCLK / PCLK / V5F prescaler math (RM §3.4).

use core::ops::Div;

use crate::pac::rcc::vals::{Fpre, Hpre, Ppre};
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
        let raw = rhs as u32;
        if raw >= 0b1000 {
            let d = raw - 0b1000 + 1;
            Hertz(self.0 >> d)
        } else {
            self
        }
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
        match rhs {
            Fpre::DIV1 => self,
            Fpre::DIV2 => Hertz(self.0 / 2),
            Fpre::DIV4 => Hertz(self.0 / 4),
            _ => self,
        }
    }
}
