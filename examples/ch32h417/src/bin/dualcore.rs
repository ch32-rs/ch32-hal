//! Dual-core demo: V3F wakes V5F, both run independently.
//!
//! V3F: blinks LED0 (PF2) and writes a loop counter to ITCM (0x200a0000)
//!      and to the cross-core mailbox in shared RAM (0x2017800C) for
//!      `wlink dump` verification.
//! V5F: see examples/ch32h417-v5f.
//!
//! # Flash / RAM partition
//!
//! Both images tile one flash chip the way the WCH CSDK does
//! (`Ld/V3F/Link_v3f.ld` + `Ld/V5F/Link_v5f.ld`): V3F owns the first 64K,
//! the V5F image starts at 0x00010000. Either image states its own range, so
//! an oversized build fails to link instead of overwriting the other image.
//! Flash this image with `--no-run` first, then the V5F image (which resets
//! and runs):
//!
//! ```text
//! wlink flash -R target/riscv32imafc-unknown-none-elf/release/dualcore
//! (cd ../ch32h417-v5f && wlink flash target/riscv32imafbc-unknown-none-elf/release/v5f_probe)
//! ```
//!
//! `wlink` maps the V5F image's 0x00010000-based sections onto flash
//! 0x08010000, so no `dd` merge is needed.
//!
//! # Cross-core mailbox (shared RAM, same layout in both crates)
//!
//! ```text
//! 0x20178000  sdi_cpuid: console token
//! 0x20178004  sdi_cpuid: V5F tick counter
//! 0x20178008  v5f_probe: 0xDEADBEEF liveness marker
//! 0x2017800C  dualcore:  V3F loop counter
//! ```

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

/// Must match `examples/ch32h417-v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

/// Cross-core mailbox slot (see the module docs).
const MAILBOX_COUNTER: *mut u32 = 0x2017_800C as *mut u32;

#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi();
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
            core::ptr::write_volatile(MAILBOX_COUNTER, counter);
        }
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
