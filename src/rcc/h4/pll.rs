//! Secondary PLLs (USBHS / USBSS / ETH / SERDES) — RM §3.4.6–3.4.9.

use crate::pac::rcc::vals::{SerdespllMul, UsbhspllRefsel, Usbhspllsrc, UsbsspllRefsel};
use crate::pac::RCC;

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct UsbhsPll {
    pub src: Usbhspllsrc,
    pub refsel: UsbhspllRefsel,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct UsbssPll {
    pub refsel: UsbsspllRefsel,
}

/// Optional auxiliary PLLs applied after the SYSCLK recipe runs.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct AuxPlls {
    pub usbhs: Option<UsbhsPll>,
    pub usbss: Option<UsbssPll>,
    /// Fixed 500 MHz ETH PLL (enable only; ref path is chip-defined).
    pub eth: bool,
    pub serdes: Option<SerdespllMul>,
}

pub unsafe fn enable_usbhs_pll(cfg: UsbhsPll) {
    RCC.pllcfgr2().modify(|w| {
        w.set_usbhspll_refsel(cfg.refsel);
        w.set_usbhspllsrc(cfg.src);
    });
    RCC.ctlr().modify(|w| w.set_usbhs_pllon(true));
    while !RCC.ctlr().read().usbhs_pllrdy() {}
}

pub unsafe fn enable_usbss_pll(refsel: UsbsspllRefsel) {
    RCC.pllcfgr2().modify(|w| w.set_usbsspll_refsel(refsel));
    RCC.ctlr().modify(|w| w.set_usbss_pllon(true));
    while !RCC.ctlr().read().usbss_pllrdy() {}
}

pub unsafe fn enable_eth_pll() {
    RCC.ctlr().modify(|w| w.set_eth_pllon(true));
    while !RCC.ctlr().read().eth_pllrdy() {}
}

pub unsafe fn enable_serdes_pll(mul: SerdespllMul) {
    RCC.pllcfgr2().modify(|w| w.set_serdespll_mul(mul));
    RCC.ctlr().modify(|w| w.set_serdes_pllon(true));
    while !RCC.ctlr().read().serdes_pllrdy() {}
}

pub(crate) unsafe fn apply_aux(aux: &AuxPlls) {
    if let Some(usbhs) = aux.usbhs {
        enable_usbhs_pll(usbhs);
    }
    if let Some(usbss) = aux.usbss {
        enable_usbss_pll(usbss.refsel);
    }
    if aux.eth {
        enable_eth_pll();
    }
    if let Some(mul) = aux.serdes {
        enable_serdes_pll(mul);
    }
}
