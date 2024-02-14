use fugit::HertzU32 as Hertz;

use crate::pac;

const HSI_FREQUENCY: Hertz = Hertz::from_raw(8_000_000);

// Power on default: No division/multiplication of the system bus takes place
const DEFAULT_FREQUENCY: Hertz = Hertz::from_raw(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    /// Clock of AHB
    pub hclk: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}

pub(crate) fn init() {
    let rcc = unsafe { &*pac::RCC::PTR };
    let exten = unsafe { &*pac::EXTEND::PTR };

    // Flash is by default divided by 2, maximum flash clock is 60 MHz, should be fine

    // set hckl = sysclk = APB1
    rcc.cfgr0().modify(|_, w| {
        w.pllmul().variant(0b1010); // Multiply by 12
        w.usbpre().variant(0b01); // Divided by 2, USB requires 48 MHz
        w.hpre().variant(0); // Don't divide the AHB clock source
        w
    });

    exten.extend_ctr().modify(|_, w| {
        w.pll_hsi_pre().set_bit() // Disable HSI clock divide by 2
    });

    rcc.ctlr().modify(|_, w| w.pllon().set_bit()); // Turn on the PLL

    while rcc.ctlr().read().pllrdy().bit_is_clear() {} // Wait till it is ready

    rcc.cfgr0().modify(|_, w| w.sw().variant(0b10)); // Use PLL as system clock source

    unsafe {
        CLOCKS = Clocks {
            sysclk: HSI_FREQUENCY * 12,
            hclk: HSI_FREQUENCY * 12,
        };
    }
}
