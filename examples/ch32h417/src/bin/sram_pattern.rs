//! Write different patterns to shared SRAM after PLL, read back each.
//! ITCM log: [0x200A0000]=counter, [0x200A0004..]=pattern readbacks

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
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHsi;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;

    let patterns = [0xFFFF_FFFFu32, 0x1234_5678, 0xDEAD_BEEF, 0x0000_FFFF, 0xFFFF_0000, 0xA5A5_A5A5];

    let mut counter: u32 = 0;
    loop {
        led.toggle();
        unsafe {
            core::ptr::write_volatile(0x200a0000 as *mut u32, counter);
            let pat = patterns[(counter as usize) % patterns.len()];
            core::ptr::write_volatile(0x20100000 as *mut u32, pat);
            let rb = core::ptr::read_volatile(0x20100000 as *const u32);
            core::ptr::write_volatile(0x200a0004 as *mut u32, pat);
            core::ptr::write_volatile(0x200a0008 as *mut u32, rb);
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(1000);
    }
}
