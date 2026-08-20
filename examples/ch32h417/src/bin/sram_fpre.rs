//! Test: PLL 400MHz with FPRE=DIV1 (V3F core at full 400MHz instead of 100MHz)
//! to isolate whether the FPRE prescaler breaks shared SRAM access.

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

const RCC_BASE: u32 = 0x4002_1000;
const RCC_CFGR0: *mut u32 = (RCC_BASE + 0x08) as *mut u32;

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHsi;
    let p = hal::init(config);

    // Override FPRE back to DIV1 (V3F = 400MHz, like V5F)
    unsafe {
        let mut cfgr0 = core::ptr::read_volatile(RCC_CFGR0);
        cfgr0 &= !(0xF << 8); // clear FPRE
        core::ptr::write_volatile(RCC_CFGR0, cfgr0);
    }

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();
        unsafe {
            core::ptr::write_volatile(0x200a0000 as *mut u32, counter);
            core::ptr::write_volatile(0x20100004 as *mut u32, counter);
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
