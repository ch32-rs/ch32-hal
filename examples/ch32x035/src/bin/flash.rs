//! Flash erase/program smoke test for CH32X035.
//!
//! Uses the fast 256-byte page path (`flash_x0`). Operates on a page at
//! 32 KiB, above the running firmware image, so the code being executed is
//! left alone. Output goes over SDI.
//!
//! ```text
//! cd examples/ch32x035 && cargo build --release --bin flash
//! wlink flash --address 0x08000000 target/riscv32imc-unknown-none-elf/release/flash
//! wlink sdi-print
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::flash::{Flash, WRITE_SIZE};
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

const HEX: &[u8; 16] = b"0123456789abcdef";

fn print_hex(label: &str, bytes: &[u8]) {
    let mut line = [0u8; 3 * 16];
    let mut n = 0;
    for &b in bytes.iter().take(16) {
        line[n] = HEX[(b >> 4) as usize];
        line[n + 1] = HEX[(b & 0x0f) as usize];
        line[n + 2] = b' ';
        n += 3;
    }
    let text = core::str::from_utf8(&line[..n]).unwrap();
    hal::try_println!("{}: {}", label, text);
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    hal::debug::SDIPrint::enable();
    hal::try_println!("flash example: starting");

    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });
    hal::try_println!(
        "flash example: sysclk {} Hz, hclk {} Hz",
        hal::rcc::clocks().sysclk.0,
        hal::rcc::clocks().hclk.0
    );

    // Give the debugger a beat to reattach if a previous flash op hung the core.
    Timer::after_millis(500).await;

    hal::try_println!("FLASH smoke test (WRITE_SIZE={})", WRITE_SIZE);

    let mut flash = Flash::new_blocking(p.FLASH);

    // Page at 32 KiB — safely above the example image size.
    let offset: u32 = 32 * 1024;
    let size = WRITE_SIZE as u32;

    let mut before = [0u8; 16];
    flash.blocking_read(offset, &mut before).unwrap();
    print_hex("before erase", &before);

    flash.blocking_erase(offset, offset + size).unwrap();

    let mut erased = [0u8; 16];
    flash.blocking_read(offset, &mut erased).unwrap();
    print_hex("after erase", &erased);
    if !erased.iter().all(|&b| b == 0xff) {
        hal::try_println!("FAIL: page is not erased");
        loop {}
    }

    let mut page = [0u8; WRITE_SIZE];
    for (i, b) in page.iter_mut().enumerate() {
        *b = (i & 0xff) as u8;
    }
    flash.blocking_write(offset, &page).unwrap();

    let mut after = [0u8; 16];
    flash.blocking_read(offset, &mut after).unwrap();
    print_hex("after write", &after);
    if after != page[..16] {
        hal::try_println!("FAIL: page head mismatch");
        loop {}
    }

    // Spot-check the last 16 bytes of the page too.
    let mut tail = [0u8; 16];
    flash.blocking_read(offset + size - 16, &mut tail).unwrap();
    print_hex("page tail", &tail);
    if tail != page[WRITE_SIZE - 16..] {
        hal::try_println!("FAIL: page tail mismatch");
        loop {}
    }

    // Erase the programmed page again. This is the check that the erase path
    // actually clears data (a no-op erase would leave the pattern behind),
    // and it leaves a blank page for the host-side dump.
    flash.blocking_erase(offset, offset + size).unwrap();
    let mut blank = [0u8; 16];
    flash.blocking_read(offset, &mut blank).unwrap();
    print_hex("erase after write", &blank);
    if !blank.iter().all(|&b| b == 0xff) {
        hal::try_println!("FAIL: erase did not clear the page");
        loop {}
    }

    // Leave the next page programmed so the host can dump it after the run.
    let marker_offset = offset + size;
    flash.blocking_erase(marker_offset, marker_offset + size).unwrap();
    flash.blocking_write(marker_offset, &page).unwrap();

    hal::try_println!("flash test OK");
    loop {
        Timer::after_millis(1000).await;
    }
}
