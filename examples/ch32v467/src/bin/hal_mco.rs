#![no_std]
#![no_main]

use ch32_hal::rcc::{Mco, McoSource};
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

#[no_mangle]
pub static mut MCO_RESULT: u32 = 0;

#[entry]
fn main() -> ! {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(MCO_RESULT), 1) };
    let p = ch32_hal::init(Default::default());

    // Route the internal HSI onto PA8 as an alternate-function push-pull.
    let _mco = Mco::new(p.PA8, McoSource::HSI);

    // Confirm the source selection landed in RCC_CFGR0. The pin itself can
    // only be verified on a scope, but Mco::new has already switched it to
    // AF push-pull by the time we get here.
    let mco = ch32_hal::pac::RCC.cfgr0().read().mco();
    if mco == 0b0101 {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(MCO_RESULT), 2) };
        defmt::info!("MCO_RESULT=PASS: HSI selected on PA8 (CFGR0.MCO={=u8})", mco);
    } else {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(MCO_RESULT), 3) };
        defmt::panic!("MCO selection mismatch: CFGR0.MCO={=u8}", mco);
    }

    loop {}
}
