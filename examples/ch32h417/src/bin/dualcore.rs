//! Dual-core demo: V3F wakes V5F, both run independently.
//!
//! V3F: blinks LED0 (PF2) and writes a loop counter to ITCM (0x200a0000)
//!      and shared SRAM (0x20100000) for `wlink dump` verification.
//! V5F: see examples/ch32h417-v5f — writes 0xDEADBEEF to ITCM
//!      (0x200a0100) at boot, or toggles LED1 (PF0).
//!
//! Flashing: V3F and V5F binaries share one flash image. The V5F binary
//! is linked at 0x08002000; merge it into the V3F image before flashing:
//!
//!   dd if=v5f.bin of=v3f.bin bs=1 seek=8192 conv=notrunc
//!   wlink flash v3f.bin

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

/// Must match examples/ch32h417-v5f/memory.x FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0800_2000;

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc.sysclk = hal::rcc::SysClk::Pll400MHsi;
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    // Wake V5F (CPU-originated WAKEIP + SENDEVENT; debug-bus writes to
    // PFIC_SCTLR do not generate the wake event).
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
