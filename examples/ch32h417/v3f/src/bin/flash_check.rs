//! FLASH self-check (V3F only).
//!
//! `cargo xtask flash --example flash_check && cargo xtask report --example flash_check`
//!
//! Destructive: erases and reprograms the **last 64 KB block** of the user
//! region (`FLASH_SIZE - BLOCK_ERASE_SIZE`). Both images live far below it.
//! Covers sector erase, fast 256-byte page programming, standard halfword
//! programming, block erase, the not-erased guard and the XOR NOR adapter.
//! Mailbox (`FlashCheck`): stage 1 = init returned, 2 = every case ran;
//! `failures`/`checks` count the cases, `erased_word` is `0xE339E339` when the
//! erase path works, `readback` the first word after the page write.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::FlashCheck;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use hal::flash::{Error, Flash, BLOCK_ERASE_SIZE, ERASED_HALFWORD, ERASE_SIZE, FLASH_SIZE, WRITE_SIZE};

ipc::mailbox!(FlashCheck);
use panic_halt as _;

/// Last 64 KB block: block-aligned and clear of both core images.
const TEST_OFFSET: u32 = FLASH_SIZE as u32 - BLOCK_ERASE_SIZE as u32;

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

fn erased(bytes: &[u8]) -> bool {
    bytes
        .chunks_exact(2)
        .all(|pair| u16::from_le_bytes([pair[0], pair[1]]) == ERASED_HALFWORD)
}

fn word(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let p = hal::init(hal::Config::default());
    let mut failures = 0u32;
    let mut checks = 0u32;

    let mut flash = Flash::new_blocking(p.FLASH);
    let mut delay = hal::delay::Delay;

    mailbox.payload.capacity.store(flash.capacity() as u32, Relaxed);
    mailbox.payload.geometry.store(
        ((BLOCK_ERASE_SIZE as u32 / 256) << 16) | (ERASE_SIZE as u32 / 256),
        Relaxed,
    );

    // Geometry comes from the chip's metapac memory layout: DBMODE=1 parts
    // have 8 KB sectors, 64 KB blocks and 256-byte fast pages.
    check(
        &mut failures,
        &mut checks,
        flash.capacity() == 960 * 1024 && ERASE_SIZE == 8192 && BLOCK_ERASE_SIZE == 65536 && WRITE_SIZE == 256,
    );

    // Argument validation must reject without touching the FPEC.
    let mut header = [0u8; 16];
    check(&mut failures, &mut checks, flash.blocking_read(0, &mut header).is_ok());
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(u32::MAX, &mut header) == Err(Error::Size),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_erase(0, 256) == Err(Error::Unaligned),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_erase(4096, 0) == Err(Error::Size),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_write(FLASH_SIZE as u32, &[]).is_ok(),
    );

    // Standard sector erase, then confirm the 0xE339 erase pattern.
    let mut sector = [0u8; 512];
    check(
        &mut failures,
        &mut checks,
        flash
            .blocking_erase(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE as u32)
            .is_ok(),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(TEST_OFFSET, &mut sector).is_ok() && erased(&sector),
    );
    mailbox.payload.erased_word.store(word(&sector), Relaxed);

    // Fast 256-byte page programming: 512 bytes spans two pages.
    let source: [u8; 512] = core::array::from_fn(|index| (index as u8).wrapping_mul(37));
    check(
        &mut failures,
        &mut checks,
        flash.blocking_write(TEST_OFFSET, &source).is_ok(),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(TEST_OFFSET, &mut sector).is_ok() && sector == source,
    );
    mailbox.payload.readback.store(word(&sector), Relaxed);
    // A programmed halfword is not blank, so a second write must be refused.
    check(
        &mut failures,
        &mut checks,
        flash.blocking_write(TEST_OFFSET, &source[..WRITE_SIZE]) == Err(Error::NotErased),
    );

    // Standard halfword programming in the next sector.
    let halfwords: [u8; 64] = core::array::from_fn(|index| index as u8);
    let next = TEST_OFFSET + ERASE_SIZE as u32;
    check(
        &mut failures,
        &mut checks,
        flash.blocking_erase(next, next + ERASE_SIZE as u32).is_ok(),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_write_halfwords(next, &halfwords[..3]) == Err(Error::Unaligned),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_write_halfwords(next, &halfwords).is_ok(),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(next, &mut sector[..64]).is_ok() && sector[..64] == halfwords,
    );

    // Block erase clears all eight sectors in one go.
    check(
        &mut failures,
        &mut checks,
        flash
            .blocking_erase_blocks(TEST_OFFSET, TEST_OFFSET + BLOCK_ERASE_SIZE as u32)
            .is_ok(),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(TEST_OFFSET, &mut sector).is_ok() && erased(&sector),
    );
    check(
        &mut failures,
        &mut checks,
        flash.blocking_read(next, &mut sector).is_ok() && erased(&sector),
    );

    // XOR NOR adapter normalizes the 0xE339 pattern to 0xFF.
    let mut nor = flash
        .into_nor_flash(TEST_OFFSET, TEST_OFFSET + ERASE_SIZE as u32)
        .expect("sector-aligned partition");
    check(
        &mut failures,
        &mut checks,
        nor.capacity() == ERASE_SIZE && nor.erase(0, ERASE_SIZE as u32).is_ok(),
    );
    let mut erased_read = [0u8; 4];
    check(
        &mut failures,
        &mut checks,
        nor.read(0, &mut erased_read).is_ok() && erased_read == [0xFF; 4],
    );
    let payload: [u8; WRITE_SIZE] = core::array::from_fn(|index| index as u8 ^ 0xa5);
    check(&mut failures, &mut checks, nor.write(0, &payload).is_ok());
    let mut payload_read = [0u8; WRITE_SIZE];
    check(
        &mut failures,
        &mut checks,
        nor.read(0, &mut payload_read).is_ok() && payload_read == payload,
    );
    let mut flash = nor.into_inner();
    check(&mut failures, &mut checks, flash.blocking_read(0, &mut header).is_ok());

    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);

    loop {
        delay.delay_ms(1000);
    }
}
