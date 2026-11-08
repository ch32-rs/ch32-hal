//! Flash erase/program smoke test for CH32L103.
//!
//! Uses the fast 256-byte page path (`flash_l1`). Operates on pages near
//! the middle of the 64 KiB flash so the running firmware (linked at the
//! start of flash) is left alone.
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin flash
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::flash::{Flash, WRITE_SIZE};
use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    // Give the debugger a beat to reattach if a previous flash op hung the core.
    Timer::after_millis(500).await;

    defmt::info!("FLASH smoke test (WRITE_SIZE={})", WRITE_SIZE);
    defmt::info!("clocks: {:?}", defmt::Debug2Format(&hal::rcc::clocks()));

    let mut flash = Flash::new_blocking(p.FLASH);

    // Page at 32 KiB — safely above typical example image size.
    let offset: u32 = 32 * 1024;
    let size = WRITE_SIZE as u32;

    let mut before = [0u8; 16];
    unwrap!(flash.blocking_read(offset, &mut before));
    defmt::info!("before erase: {:02x}", before);

    unwrap!(flash.blocking_erase(offset, offset + size));

    let mut erased = [0u8; 16];
    unwrap!(flash.blocking_read(offset, &mut erased));
    defmt::info!("after erase:  {:02x}", erased);
    defmt::assert!(erased.iter().all(|&b| b == 0xff));

    let mut page = [0u8; WRITE_SIZE];
    for (i, b) in page.iter_mut().enumerate() {
        *b = (i & 0xff) as u8;
    }
    unwrap!(flash.blocking_write(offset, &page));

    let mut after = [0u8; 16];
    unwrap!(flash.blocking_read(offset, &mut after));
    defmt::info!("after write:  {:02x}", after);
    defmt::assert_eq!(&after[..], &page[..16]);

    // Spot-check the last 16 bytes of the page too.
    let mut tail = [0u8; 16];
    unwrap!(flash.blocking_read(offset + size - 16, &mut tail));
    defmt::info!("page tail:    {:02x}", tail);
    defmt::assert_eq!(&tail[..], &page[WRITE_SIZE - 16..]);

    defmt::info!("flash test OK");
    loop {
        Timer::after_millis(1000).await;
    }
}
