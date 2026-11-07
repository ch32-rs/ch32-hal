#![no_std]
#![no_main]

use ch32_hal::rcc::{Mco, McoSource};
use ch32v467_examples::{report_set, report_static};
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

report_static!(MCO_RESULT);

#[entry]
fn main() -> ! {
    report_set!(MCO_RESULT, 1);
    let p = ch32_hal::init(Default::default());

    // Route the internal HSI onto PA8 as an alternate-function push-pull.
    let _mco = Mco::new(p.PA8, McoSource::HSI);

    // Confirm the source selection landed in RCC_CFGR0. The pin itself can
    // only be verified on a scope, but Mco::new has already switched it to
    // AF push-pull by the time we get here.
    let mco = ch32_hal::pac::RCC.cfgr0().read().mco();
    if mco == McoSource::HSI {
        report_set!(MCO_RESULT, 2);
        defmt::info!("MCO_RESULT=PASS: HSI selected on PA8 (CFGR0.MCO={=u8})", mco.to_bits());
    } else {
        report_set!(MCO_RESULT, 3);
        defmt::panic!("MCO selection mismatch: CFGR0.MCO={=u8}", mco.to_bits());
    }

    loop {}
}
