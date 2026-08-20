//! Read+write probe of shared SRAM after PLL switch.
//! Results stored in ITCM for wlink inspection:
//!   0x200A0000: loop counter
//!   0x200A0004: readback of 0x20100000 (should be 0xAAAA0001 if write+read works)
//!   0x200A0008: readback after write of counter (0xBBBB0000 | counter if works)

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use ch32_hal as hal;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[ch32_hal::entry]
fn main() -> ! {
    // Pre-init: write marker to shared SRAM
    unsafe {
        core::ptr::write_volatile(0x20100000 as *mut u32, 0xAAAA_0001);
    }

    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHsi;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();
        unsafe {
            core::ptr::write_volatile(0x200a0000 as *mut u32, counter);

            // READ the pre-init marker back
            let readback = core::ptr::read_volatile(0x20100000 as *const u32);
            core::ptr::write_volatile(0x200a0004 as *mut u32, readback);

            // WRITE then READ back
            core::ptr::write_volatile(0x20100000 as *mut u32, 0xBBBB_0000 | counter);
            let readback2 = core::ptr::read_volatile(0x20100000 as *const u32);
            core::ptr::write_volatile(0x200a0008 as *mut u32, readback2);
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
