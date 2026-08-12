//! Dual-core test: V3F wakes V5F, then both write to shared SRAM.
//!
//! V3F counter @ 0x200a0000 (ITCM), V5F counter @ 0x20100000 (shared SRAM).
//!
//! V5F binary must be placed at 0x08008000 in flash before flashing V3F.
//! Merge: dd if=v5f.bin of=v3f.bin bs=1 seek=32768 conv=notrunc

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use ch32_hal as hal;
use qingke::pfic;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

const V5F_ENTRY: u32 = 0x0800_2000;

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHse;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    // Wake V5F
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    loop {
        led.toggle();
        unsafe {
            core::ptr::write_volatile(0x200a0000 as *mut u32, counter);
            core::ptr::write_volatile(0x20100000 as *mut u32, counter);
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
