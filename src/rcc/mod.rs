use fugit::HertzU32 as Hertz;

// const HSI_FREQUENCY: Hertz = Hertz::from_raw(48_000_000);

const LSI_FREQUENCY: Hertz = Hertz::from_raw(40_000);
// CH32FV208 use 32.768KHz LSI

// Power on default: HPRE = 0b0101 = Div6
const DEFAULT_FREQUENCY: Hertz = Hertz::from_raw(8_000_000);

static mut CLOCKS: Clocks = Clocks {
    // Power on default
    sysclk: DEFAULT_FREQUENCY,
    hclk: DEFAULT_FREQUENCY,
    pclk1: DEFAULT_FREQUENCY,
    pclk2: DEFAULT_FREQUENCY,
};

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct Clocks {
    pub sysclk: Hertz,
    /// AHB clock
    pub hclk: Hertz,
    /// APB1 clock
    pub pclk1: Hertz,
    /// APB2 clock
    pub pclk2: Hertz,
}

#[inline]
pub fn clocks() -> &'static Clocks {
    unsafe { &CLOCKS }
}
