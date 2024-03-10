//! CH32V305/307, CH32F205/207
use crate::pac::rcc::vals::{
    Hpre as AHBPrescaler, PllMul, PllxMul, Ppre as APBPrescaler, Prediv as PllPreDiv, Sw as Sysclk,
};
use crate::time::Hertz;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum HseMode {
    /// crystal/ceramic oscillator (HSEBYP=0)
    Oscillator,
    /// external analog clock (low swing) (HSEBYP=1)
    Bypass,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Hse {
    /// HSE frequency.
    pub freq: Hertz,
    /// HSE mode.
    pub mode: HseMode,
}

#[derive(Clone, Copy)]
pub struct Pll {
    /// PLL pre-divider
    pub prediv: PllPreDiv,

    /// PLL multiplication factor.
    pub mul: PllMul,
}

// PLLSRC
pub enum PllSource {
    /// HSI or HSI div 2
    HSI,
    // HSE or HSE div (2 or Prediv)
    HSE,
}

pub struct Pll2 {
    /// PLL pre-divider
    pub prediv: PllPreDiv,

    /// PLL multiplication factor.
    pub mul: PllxMul,
}

pub struct Pllx {
    pub prediv2: PllPreDiv,
    pub pll2: Option<PllxMul>,
    pub pll3: Option<PllxMul>,
}

pub struct Config {
    pub hsi: bool,
    pub hse: Option<Hse>,
    pub sys: Sysclk,

    pub pll_src: PllSource,
    pub pll: Option<Pll>,

    // TODO: optional
    pub pllx : Option<Pllx>,

    pub ahb_pre: AHBPrescaler,
    pub apb1_pre: APBPrescaler,
    pub apb2_pre: APBPrescaler,

    pub ls: super::LsConfig,
    // /// Per-peripheral kernel clock selection muxes
    // pub mux: super::mux::ClockMux,
}


