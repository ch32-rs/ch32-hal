//! Test shared SRAM write BEFORE any RCC configuration.

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
    // Write to shared SRAM immediately, before hal::init()
    unsafe {
        core::ptr::write_volatile(0x20100000 as *mut u32, 0xAAAA_0001);
        core::ptr::write_volatile(0x20178000 as *mut u32, 0xAAAA_0002);
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
            core::ptr::write_volatile(0x20100004 as *mut u32, counter);
            core::ptr::write_volatile(0x20178000 as *mut u32, counter);
            core::arch::asm!("fence");
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
