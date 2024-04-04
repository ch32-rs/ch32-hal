use crate::time::Hertz;

const HSI_FREQUENCY: Hertz = Hertz(48_000_000);

const DEFAULT_FREQUENCY: Hertz = Hertz(8_000_000);

#[derive(Debug, Default)]
pub struct Config {}

#[allow(unused_variables)]
pub(crate) unsafe fn init(config: Config) {
    // TODO: handle AHBPrescaler
    super::CLOCKS.sysclk = DEFAULT_FREQUENCY;
    super::CLOCKS.hclk = DEFAULT_FREQUENCY;
    super::CLOCKS.pclk1 = DEFAULT_FREQUENCY;
    super::CLOCKS.pclk2 = DEFAULT_FREQUENCY;
}
