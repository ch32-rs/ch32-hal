//! Watchdog Timer (IWDG, WWDG)

#[cfg(iwdg)]
mod iwdg;
#[cfg(iwdg)]
pub use iwdg::*;

#[cfg(wwdg)]
mod wwdg;
#[cfg(wwdg)]
pub use wwdg::*;
