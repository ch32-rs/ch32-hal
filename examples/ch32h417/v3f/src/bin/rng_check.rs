//! RNG blocking self-check (V3F only).
//!
//! Draws 32 words, discards the first as the driver requires, and checks that
//! the rest are not all zero and do not repeat. Mailbox (`RngCheck`): stage 1
//! = init returned, 2 = the sample passed; `failures` counts failed checks,
//! `words` the accepted samples and `checksum` their XOR.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::RngCheck;
use hal::rng::{InterruptHandler, Rng};

ipc::mailbox!(RngCheck);
use panic_halt as _;

hal::bind_interrupts!(struct Irqs {
    RNG => InterruptHandler<hal::peripherals::RNG>;
});

fn check(failures: &mut u32, condition: bool) {
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let p = hal::init(hal::Config::default());
    let mut rng = Rng::new(p.RNG, Irqs);

    let mut failures = 0u32;
    let mut checksum = 0u32;
    let mut previous = 0u32;
    let mut words = 0u32;
    let mut distinct = 0u32;

    for index in 0..32 {
        match rng.blocking_next_u32() {
            Ok(value) => {
                checksum ^= value;
                if index != 0 && value != previous {
                    distinct += 1;
                }
                previous = value;
                words += 1;
            }
            Err(_) => failures += 1,
        }
    }

    check(&mut failures, words == 32);
    check(&mut failures, checksum != 0);
    // A stuck generator would return one constant; demand real variation.
    check(&mut failures, distinct > 24);

    mailbox.payload.words.store(words, Relaxed);
    mailbox.payload.checksum.store(checksum, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);

    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
