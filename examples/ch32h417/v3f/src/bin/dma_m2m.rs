//! DMA memory-to-memory copy on the boot core, read back with `wlink dump`.
//!
//! Moves a 256-byte pattern through DMA1 channel 1 and records what happened
//! at a fixed address at the top of `SRAM_SHARED`, so the result can be read
//! without the mailbox or the second core:
//!
//! ```text
//! cargo build --release --bin dma_m2m
//! wlink flash target/riscv32imafc-unknown-none-elf/release/dma_m2m --chip CH32H41X
//! wlink dump 0x2017FF00 16 --chip CH32H41X
//! ```
//!
//! The dump is four little-endian words: a marker, the transfer-complete flag,
//! how many of the 256 bytes match, and the remaining count.

#![no_std]
#![no_main]

use ch32_hal::dma::{Transfer, TransferOptions};
use {ch32_hal as hal, panic_halt as _};

/// Top of `SRAM_SHARED` (`memory.x`: `0x20178000` + 32K). Nothing else in this
/// image uses the region, and the mailbox lives at its start.
const RESULT: *mut u32 = 0x2017_FF00 as *mut u32;

/// `b"DMA1"`, so a stale dump is obvious.
const MARKER: u32 = u32::from_le_bytes(*b"DMA1");

const LEN: usize = 256;

#[ch32_hal::entry]
fn main() -> ! {
    let p = hal::init(hal::Config::default());

    let mut src = [0u8; LEN];
    let mut dst = [0u8; LEN];
    for (i, byte) in src.iter_mut().enumerate() {
        *byte = i as u8;
    }

    // Memory to memory through the HAL: `new_read` treats `src` as the
    // peripheral address and `dst` as the memory buffer, and the two options
    // turn on `PINC` and `MEM2MEM`. Request 0 is the manual's MEM2MEM source;
    // the channel does not wait on it, but it still goes through the DMAMUX
    // slot, so the mux path is exercised too.
    // Completion interrupts stay off so the flag register is not cleared by
    // the handler before it is read back. `blocking_wait` cannot be used then:
    // it only returns once the handler has counted a completion.
    let mut options = TransferOptions::default();
    options.peripheral_increment = true;
    options.memory_to_memory = true;
    options.complete_transfer_ir = false;
    options.half_transfer_ir = false;
    let transfer = unsafe {
        Transfer::new_read::<u8>(p.DMA1_CH1, 0, src.as_mut_ptr(), &mut dst, options)
    };
    while transfer.get_remaining_transfers() != 0 {}
    core::mem::forget(transfer);

    unsafe {
        let dma = hal::pac::DMA1;
        let matched = src.iter().zip(dst.iter()).filter(|(a, b)| a == b).count() as u32;
        let tcif = u32::from(dma.intfr().read().tcif(0));
        let remaining = dma.ch(0).cntr().read().ndt() as u32;

        core::ptr::write_volatile(RESULT, MARKER);
        core::ptr::write_volatile(RESULT.add(1), tcif);
        core::ptr::write_volatile(RESULT.add(2), matched);
        core::ptr::write_volatile(RESULT.add(3), remaining);
    }

    loop {
        hal::delay::Delay.delay_ms(1000);
    }
}
