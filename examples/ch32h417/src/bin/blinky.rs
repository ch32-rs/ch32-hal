//! Blinky + atomic counter test for nanoCH32H417.
//!
//! Writes a counter via AtomicU32 to both ITCM (0x200a0004) and
//! DTCM (0x200c0000) to test which region supports atomics.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicU32, Ordering};
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use ch32_hal as hal;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    let itcm = unsafe { &*(0x200a0004 as *const AtomicU32) };
    let dtcm = unsafe { &*(0x200c0000 as *const AtomicU32) };

    loop {
        led.toggle();
        // Write counter via AtomicU32 to both ITCM and DTCM
        itcm.store(counter, Ordering::Relaxed);
        dtcm.store(counter, Ordering::Relaxed);
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
