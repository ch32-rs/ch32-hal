//! Dual-core demo: V3F wakes V5F, both run independently.
//!
//! V3F: blinks LED0 (PF2) and writes a loop counter to ITCM (0x200a0000) and to
//!      the cross-core mailbox for `cargo xtask report` verification.
//! V5F: `v5f/src/bin/dualcore.rs` marks the mailbox and idles; this core owns
//! the console, the LEDs and the clocks.
//!
//! # Building and flashing
//!
//! ```text
//! cargo xtask run --example dualcore
//! ```
//!
//! `xtask` builds both cores, writes this image first with `--no-run`, then the
//! V5F image — and that last write resets and runs the chip. The split is the
//! WCH CSDK one (`Ld/V3F/Link_v3f.ld` + `Ld/V5F/Link_v5f.ld`): V3F owns the
//! first 64K of flash, the V5F image starts at 0x00010000. Either image states
//! its own range, so an oversized build fails to link instead of overwriting
//! the other image.
//!
//! # Cross-core mailbox
//!
//! The mailbox is the `ch32h417-ipc` crate: one `#[repr(C)]` structure that both
//! harts place at the start of `SRAM_SHARED` through their own linker scripts
//! (see `ipc/shared.x`), so the fields cannot drift between the two images — or
//! from `cargo xtask report`, which links the same crate.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use core::panic::PanicInfo;
use hal::delay::Delay;
use hal::gpio::{Level, Output};
use qingke::pfic;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;


#[ch32_hal::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hsi();
    let p = hal::init(config);

    let mut led = Output::new(p.PF2, Level::Low, Default::default());
    let mut delay = Delay;
    let mut counter: u32 = 0;

    // Clear the mailbox before the handover so `xtask report` describes this run
    // rather than leftovers from another example.
    ipc::mailbox().clear();

    // Wake V5F (CPU-originated WAKEIP + SENDEVENT; debug-bus writes to
    // PFIC_SCTLR do not generate the wake event).
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    loop {
        led.toggle();
        ipc::mailbox()
            .dualcore_counter
            .store(counter, core::sync::atomic::Ordering::Relaxed);
        unsafe { core::ptr::write_volatile(0x200a0000 as *mut u32, counter) };
        counter = counter.wrapping_add(1);
        delay.delay_ms(500);
    }
}
