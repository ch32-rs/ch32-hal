//! Direct SRAM write test — verify 0x20100000 is writable from V3F.

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
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    loop {
        led.toggle();

        // Write to ITCM (known working)
        unsafe { core::ptr::write_volatile(0x200a0000 as *mut u32, counter); }

        // Write to shared SRAM using inline asm
        unsafe {
            core::arch::asm!(
                "lui t0, 0x20100
                 sw {0}, 0(t0)",
                in(reg) counter,
                out("t0") _,
            );
        }

        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
