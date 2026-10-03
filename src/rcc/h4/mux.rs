//! RCC_CFGR2 peripheral kernel-clock muxes (RM §3.4.13).

use crate::pac::rcc::vals::{
    ClkSrcPll, Eth1gsrc, Hsadcsrc, Ltdcsrc, Uhsifsrc, Usbfsdiv, Usbfssrc,
};
use crate::pac::RCC;

/// CFGR2 mux defaults aligned with WCH EVT `system_ch32h417.c` power-on values.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct KernelMux {
    pub uhsifdiv: u8,
    pub uhsifsrc: Uhsifsrc,
    pub ltdcdiv: u8,
    pub ltdcsrc: Ltdcsrc,
    pub usbfsdiv: Usbfsdiv,
    pub usbfssrc: Usbfssrc,
    pub rngsrc: ClkSrcPll,
    pub i2s2src: ClkSrcPll,
    pub i2s3src: ClkSrcPll,
    pub hsadcsrc: Hsadcsrc,
    pub eth1gsrc: Eth1gsrc,
}

impl Default for KernelMux {
    fn default() -> Self {
        Self {
            uhsifdiv: 1,
            uhsifsrc: Uhsifsrc::SYSCLK,
            ltdcdiv: 1,
            ltdcsrc: Ltdcsrc::PLL_CLK,
            usbfsdiv: Usbfsdiv::DIV10,
            usbfssrc: Usbfssrc::PLL,
            rngsrc: ClkSrcPll::PLL_CLK,
            i2s2src: ClkSrcPll::PLL_CLK,
            i2s3src: ClkSrcPll::PLL_CLK,
            hsadcsrc: Hsadcsrc::PLL_CLK,
            eth1gsrc: Eth1gsrc::PLL_CLK,
        }
    }
}

pub(crate) unsafe fn apply_mux(mux: &KernelMux) {
    RCC.cfgr2().modify(|w| {
        w.set_uhsifdiv(mux.uhsifdiv);
        w.set_uhsifsrc(mux.uhsifsrc);
        w.set_ltdcdiv(mux.ltdcdiv);
        w.set_ltdcsrc(mux.ltdcsrc);
        w.set_usbfsdiv(mux.usbfsdiv);
        w.set_usbfssrc(mux.usbfssrc);
        w.set_rngsrc(mux.rngsrc);
        w.set_i2s2src(mux.i2s2src);
        w.set_i2s3src(mux.i2s3src);
        w.set_hsadcsrc(mux.hsadcsrc);
        w.set_eth1gsrc(mux.eth1gsrc);
    });
}
