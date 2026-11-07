//! SPI1 blocking self-check (V3F only).
//!
//! Runs all four clock modes and both bit orders on **SCK=PA5, MOSI=PA7,
//! MISO=PA6** (AF5) with 8- and 16-bit words. Mailbox (`SpiCheck`): stage 1 =
//! init returned, 2 = every mode ran, 3 = no check failed; `failures` counts
//! failed checks, `cases` the completed transfers, and `ctlr1`/`hscr` are the
//! registers read back afterwards.
//!
//! With `--features spi-loopback` and PA7 jumpered to PA6, the last transfer
//! also checks the received bytes and raises `loopback` to 1. Without the
//! jumper the transfer still runs and only its completion is checked.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::SpiCheck;
use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};
use hal::spi::{BitOrder, Config, Spi};

ipc::mailbox!(SpiCheck);
use panic_halt as _;

/// Count a failed check, at most once.
fn check(failures: &mut u32, condition: bool) {
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let p = hal::init(hal::Config::default());

    let mut failures = 0u32;
    let mut cases = 0u32;

    let mut config = Config::default();
    let mut spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, config);

    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        for bit_order in [BitOrder::MsbFirst, BitOrder::LsbFirst] {
            config.mode = mode;
            config.bit_order = bit_order;
            check(&mut failures, spi.set_config(&config).is_ok());

            let current = spi.get_current_config();
            check(
                &mut failures,
                current.mode == mode && current.bit_order == bit_order && current.frequency <= config.frequency,
            );

            check(&mut failures, spi.blocking_write(&[0x55u8; 8]).is_ok());
            cases += 1;
            check(&mut failures, spi.blocking_write(&[0x35u16; 6]).is_ok());
            cases += 1;

            let mut transfer = [0x55u8, 0xaa, 0x00, 0xff, 0x81];
            check(&mut failures, spi.blocking_transfer_in_place(&mut transfer).is_ok());
            cases += 1;
            #[cfg(feature = "spi-loopback")]
            {
                check(&mut failures, transfer == [0x55, 0xaa, 0x00, 0xff, 0x81]);
                mailbox.payload.loopback.store(1, Relaxed);
            }
            #[cfg(not(feature = "spi-loopback"))]
            let _ = transfer;

            // A transfer must leave the peripheral idle and error-free.
            let status = hal::pac::SPI1.statr().read();
            check(&mut failures, !status.bsy() && !status.ovr());
        }
    }

    mailbox.payload.ctlr1.store(hal::pac::SPI1.ctlr1().read().0, Relaxed);
    mailbox.payload.hscr.store(hal::pac::SPI1.hscr().read().0, Relaxed);
    mailbox.payload.cases.store(cases, Relaxed);
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
