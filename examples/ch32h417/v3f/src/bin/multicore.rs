//! `ch32_hal::multicore` — one image, two harts, `embassy-rp` style.
//!
//! ```text
//! WLINK_PROBE=1a86:8010:D7708F065C7F cargo xtask flash --example multicore
//! sleep 3 && wlink dump 0x20178000 24 -o /var/tmp/obs.bin   # read once!
//! ```
//!
//! Expected: `h1_hart = 1`, `lock_value = 200000` (100 000 `SpinLock`
//! increments on each hart, no lost updates), and `h1_seq` advancing in the
//! second read. As with every dual-core test on this board, read the block
//! **once**: a later `wlink dump` leaves hart 0 halted and `h0_seq` freezes.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use ch32_hal as hal;
use hal::multicore::{self, Stack, SpinLock};
use panic_halt as _;

/// Fixed at the start of `SRAM_SHARED` (`0x2017_8000`) by `ipc/shared.x`.
#[repr(C)]
struct Obs {
    h0_seq: AtomicU32,
    h1_seq: AtomicU32,
    lock_value: AtomicU32,
    h1_done: AtomicU32,
    h1_hart: AtomicU32,
}

#[link_section = ".shared"]
static OBS: Obs = Obs {
    h0_seq: AtomicU32::new(0),
    h1_seq: AtomicU32::new(0),
    lock_value: AtomicU32::new(0),
    h1_done: AtomicU32::new(0),
    h1_hart: AtomicU32::new(0),
};

/// State that both harts mutate: the counter is plain `u32`, so every access
/// goes through the lock.
static SHARED: SpinLock<u32> = SpinLock::new(0);

/// Increments per hart. Both must land, or the lock lost an update.
const INCREMENTS: u32 = 100_000;

static mut CORE1_STACK: Stack<8192> = Stack::new();

#[ch32_hal::entry]
fn main() -> ! {
    let p = hal::init(Default::default());

    for f in [
        &OBS.h0_seq,
        &OBS.h1_seq,
        &OBS.lock_value,
        &OBS.h1_done,
        &OBS.h1_hart,
    ] {
        f.store(0, Ordering::Relaxed);
    }

    // Move the peripherals hart 1 will own into the closure. There is no
    // second `Peripherals` instance to take; ownership is by move.
    let pf0 = p.PF0;

    let stack = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) };
    multicore::spawn_core1(stack, move || {
        OBS.h1_hart
            .store(multicore::hart_id() as u32, Ordering::Relaxed);

        let mut led = hal::gpio::Output::new(pf0, hal::gpio::Level::Low, Default::default());

        // Litmus for the cross-core lock: exactly half of the increments are
        // done on this hart.
        for _ in 0..INCREMENTS {
            SHARED.with(|v| *v += 1);
        }
        OBS.h1_done.store(1, Ordering::Release);

        loop {
            led.toggle();
            OBS.h1_seq.fetch_add(1, Ordering::Relaxed);
            hal::delay::Delay.delay_ms(500);
        }
    });

    for _ in 0..INCREMENTS {
        SHARED.with(|v| *v += 1);
    }
    while OBS.h1_done.load(Ordering::Acquire) == 0 {
        core::hint::spin_loop();
    }
    OBS.lock_value.store(SHARED.with(|v| *v), Ordering::Relaxed);

    loop {
        OBS.h0_seq.fetch_add(1, Ordering::Relaxed);
        hal::delay::Delay.delay_ms(500);
    }
}
