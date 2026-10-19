//! Minimal dual-core example — V5F half.
//!
//! Woken by `v3f/src/bin/hello.rs`, it toggles LED1 (PF0). The boot core already
//! configured the clock tree and this GPIO port, so hart 1 must not run
//! `ch32_hal::init` again (one shared RCC block; `init` re-programs the PLL).
//! It drives the pin through [`ch32h417_v5f::pac`] and needs no embassy — only
//! `qingke-rt`'s entry.
//!
//! ```text
//! cargo xtask flash --example hello --dual-core
//! ```

#![no_std]
#![no_main]

use ch32h417_v5f::pac;
use panic_halt as _;

/// Port F, pin 0 (`LED1` on the EVT board).
const PORT_F: usize = 5;
const PIN: usize = 0;

#[qingke_rt::entry]
fn main() -> ! {
    let gpiof = pac::GPIO(PORT_F);

    // The PAC names each CNF encoding after its input-mode meaning: CNF=00 is
    // "analog in / push-pull out", CNF=01 is "floating in / open drain out".
    // A plain LED wants the former.
    gpiof.cfglr().modify(|w| {
        w.set_mode(PIN, pac::gpio::vals::Mode::OUTPUT_50MHZ);
        w.set_cnf(PIN, pac::gpio::vals::Cnf::ANALOG_IN__PUSH_PULL_OUT);
    });

    loop {
        gpiof.bshr().write(|w| w.set_bs(PIN, true));
        spin();
        gpiof.bshr().write(|w| w.set_br(PIN, true));
        spin();
    }
}

/// Crude delay: `hal::delay::Delay` is calibrated by the boot core's
/// `hal::init()`, so hart 1 spins instead.
fn spin() {
    for _ in 0..5_000_000 {
        unsafe { core::arch::asm!("nop") };
    }
}
