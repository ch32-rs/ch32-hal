//! Flash erase/program smoke test for CH32X035.
//!
//! Uses the fast 256-byte page path (`flash_x0`). Operates on pages at
//! 32 KiB, above the running firmware image, so the code being executed is
//! left alone. Logs go over defmt RTT and are read by `probe-rs run`.
//!
//! ```text
//! cd examples/ch32x035 && cargo run --release --bin flash
//! ```
//!
//! After the run, the host can also dump the flash to check the result:
//! `0x08008000` holds the page left erased, `0x08008100` the programmed one.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::flash::{Flash, WRITE_SIZE};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

defmt::timestamp!("{=u64:us}", embassy_time::Instant::now().as_micros());

// `defmt::assert!` refers to this hook; `panic-halt` still owns the actual
// panic handler, so a failed check simply halts after the last log line.
#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    // Give the debugger a beat to attach and start the RTT session.
    Timer::after_millis(500).await;

    defmt::info!("FLASH smoke test (WRITE_SIZE={})", WRITE_SIZE);
    defmt::info!(
        "clocks: sysclk {} Hz, hclk {} Hz",
        hal::rcc::clocks().sysclk.0,
        hal::rcc::clocks().hclk.0
    );

    let mut flash = Flash::new_blocking(p.FLASH);

    // Page at 32 KiB — safely above the example image size.
    let offset: u32 = 32 * 1024;
    let size = WRITE_SIZE as u32;

    let mut before = [0u8; 16];
    flash.blocking_read(offset, &mut before).unwrap();
    defmt::info!("before erase: {:02x}", before);

    flash.blocking_erase(offset, offset + size).unwrap();

    let mut erased = [0u8; 16];
    flash.blocking_read(offset, &mut erased).unwrap();
    defmt::info!("after erase:  {:02x}", erased);
    defmt::assert!(erased.iter().all(|&b| b == 0xff));

    let mut page = [0u8; WRITE_SIZE];
    for (i, b) in page.iter_mut().enumerate() {
        *b = (i & 0xff) as u8;
    }
    flash.blocking_write(offset, &page).unwrap();

    let mut after = [0u8; 16];
    flash.blocking_read(offset, &mut after).unwrap();
    defmt::info!("after write:  {:02x}", after);
    defmt::assert_eq!(after, page[..16]);

    // Spot-check the last 16 bytes of the page too.
    let mut tail = [0u8; 16];
    flash.blocking_read(offset + size - 16, &mut tail).unwrap();
    defmt::info!("page tail:    {:02x}", tail);
    defmt::assert_eq!(tail, page[WRITE_SIZE - 16..]);

    // Erase the programmed page again. This is the check that the erase path
    // actually clears data (a no-op erase would leave the pattern behind),
    // and it leaves a blank page for the host-side dump.
    flash.blocking_erase(offset, offset + size).unwrap();
    let mut blank = [0u8; 16];
    flash.blocking_read(offset, &mut blank).unwrap();
    defmt::info!("erase again:  {:02x}", blank);
    defmt::assert!(blank.iter().all(|&b| b == 0xff));

    // Leave the next page programmed so the host can dump it after the run.
    let marker_offset = offset + size;
    flash.blocking_erase(marker_offset, marker_offset + size).unwrap();
    flash.blocking_write(marker_offset, &page).unwrap();

    defmt::info!("flash test OK");
    loop {
        Timer::after_millis(1000).await;
    }
}
