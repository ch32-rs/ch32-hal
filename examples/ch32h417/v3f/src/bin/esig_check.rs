//! ESIG self-check: flash size, UID and chip ID through `hal::signature`.
//!
//! `src/signature.rs` hard-codes the ESIG addresses (0x1FFFF7E0 base, UID at
//! +8, chip ID at 0x1FFFF704). The probe reads the same block, so the values
//! in the mailbox can be compared with `wlink`/`probe-rs`: the flash size must
//! be 960 KB, the UID must not be erased, and the chip ID must be nonzero.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::EsigCheck;

ipc::mailbox!(EsigCheck);
use panic_halt as _;

/// `CH32H417QEU6` has 960 KB of code flash.
const EXPECTED_FLASH_KB: u16 = 960;

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let _p = hal::init(hal::Config::default());

    let flash_kb = hal::signature::flash_size_kb();
    let uid = hal::signature::unique_id();
    let chip_id = hal::signature::chip_id();
    let chip_raw = u32::from(chip_id.dev_id()) | (u32::from(chip_id.rev_id()) << 16);
    let uid_words = [
        u32::from_le_bytes([uid[0], uid[1], uid[2], uid[3]]),
        u32::from_le_bytes([uid[4], uid[5], uid[6], uid[7]]),
        u32::from_le_bytes([uid[8], uid[9], uid[10], uid[11]]),
    ];

    let mut checks = 0u32;
    let mut failures = 0u32;
    check(&mut failures, &mut checks, flash_kb == EXPECTED_FLASH_KB);
    check(
        &mut failures,
        &mut checks,
        uid.iter().any(|byte| *byte != 0) && uid.iter().any(|byte| *byte != 0xFF),
    );
    check(&mut failures, &mut checks, chip_raw != 0 && chip_raw != 0xFFFF_FFFF);

    mailbox.payload.flash_kb.store(u32::from(flash_kb), Relaxed);
    mailbox.payload.uid0.store(uid_words[0], Relaxed);
    mailbox.payload.uid1.store(uid_words[1], Relaxed);
    mailbox.payload.uid2.store(uid_words[2], Relaxed);
    mailbox.payload.chip_id.store(chip_raw, Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
