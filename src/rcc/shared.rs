//! Shared RCC helpers used by every family implementation.

use crate::time::Hertz;

use super::Clocks;

/// Update the global [`clocks()`] cache (called from `init` / `refresh`).
pub(crate) unsafe fn set_clocks(clocks: Clocks) {
    super::CLOCKS = clocks;
}

/// APB timer kernel clock (STM32-style ×2 when PPRE ≠ 1).
pub(crate) fn apb_timer_clk(hclk: Hertz, pclk: Hertz, double_when_divided: bool) -> Hertz {
    if double_when_divided && hclk != pclk {
        pclk * 2u32
    } else {
        pclk
    }
}
