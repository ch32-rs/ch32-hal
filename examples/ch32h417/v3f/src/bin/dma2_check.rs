//! DMA2 memory-to-memory self-check (V3F only).
//!
//! `dma_m2m` only reaches DMA1. This runs the same transfer through DMA2
//! channel 1 — a 512-byte pattern with `PINC` and `MEM2MEM`, request 0 through
//! the DMAMUX slot — and publishes the comparison, the transfer-complete flag
//! and the remaining count. It also repeats a 16-byte transfer with a second
//! channel to check that channel indexing is independent.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32_hal::dma::{Transfer, TransferOptions};
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::Dma2Check;

ipc::mailbox!(Dma2Check);
use panic_halt as _;

const LEN: usize = 512;

fn options() -> TransferOptions {
    let mut options = TransferOptions::default();
    options.peripheral_increment = true;
    options.memory_to_memory = true;
    options.complete_transfer_ir = false;
    options.half_transfer_ir = false;
    options
}

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);
    mailbox.payload.length.store(LEN as u32, Relaxed);

    let mut p = hal::init(hal::Config::default());

    let mut src = [0u8; LEN];
    let mut dst = [0u8; LEN];
    for (i, byte) in src.iter_mut().enumerate() {
        *byte = (i as u8).wrapping_mul(7).wrapping_add(3);
    }

    let transfer = unsafe { Transfer::new_read::<u8>(p.DMA2_CH1.reborrow(), 0, src.as_mut_ptr(), &mut dst, options()) };
    while transfer.get_remaining_transfers() != 0 {}
    core::mem::forget(transfer);

    let dma = hal::pac::DMA2;
    let matched = src.iter().zip(dst.iter()).filter(|(a, b)| a == b).count() as u32;
    let tcif = u32::from(dma.intfr().read().tcif(0));
    let remaining = dma.ch(0).cntr().read().ndt() as u32;

    let mut checks = 0u32;
    let mut failures = 0u32;
    check(&mut failures, &mut checks, matched == LEN as u32);
    check(&mut failures, &mut checks, tcif == 1);
    check(&mut failures, &mut checks, remaining == 0);

    // A second channel, with a different length, to show the per-channel
    // registers and flags are not shared with channel 1.
    let mut small_src = [0u8; 16];
    let mut small_dst = [0u8; 16];
    for (i, byte) in small_src.iter_mut().enumerate() {
        *byte = 0xa0 | i as u8;
    }
    let transfer = unsafe {
        Transfer::new_read::<u8>(
            p.DMA2_CH4.reborrow(),
            0,
            small_src.as_mut_ptr(),
            &mut small_dst,
            options(),
        )
    };
    while transfer.get_remaining_transfers() != 0 {}
    core::mem::forget(transfer);
    let matched_small = small_src.iter().zip(small_dst.iter()).filter(|(a, b)| a == b).count() as u32;
    let tcif4 = u32::from(dma.intfr().read().tcif(3));
    check(&mut failures, &mut checks, matched_small == 16);
    check(&mut failures, &mut checks, tcif4 == 1);

    mailbox.payload.matched.store(matched, Relaxed);
    mailbox.payload.tcif.store(tcif, Relaxed);
    mailbox.payload.remaining.store(remaining, Relaxed);
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
