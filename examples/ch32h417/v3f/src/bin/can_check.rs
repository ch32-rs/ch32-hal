//! bxCAN blocking self-check (V3F only).
//!
//! Runs CAN1 (PA11 = RX, PA12 = TX) in silent loopback at 500 kbit/s, so no
//! transceiver is needed. It checks the shared 42-bank filter table through
//! CAN1 (banks 0, 13, 14 and 41 are inside the two `FMCFGR`/`FMCFGR1` halves;
//! bank 42 must be rejected), then exchanges 72 standard/extended data and
//! remote frames over both receive FIFOs. Finally it constructs CAN2
//! (PB12 = RX, PB13 = TX) with CAN1 as the filter master and repeats a short
//! exchange on bank 14, which is CAN2's first bank under the reset split.
//!
//! Mailbox (`CanCheck`): stage 1 = CAN1 up, 2 = CAN1 cases and frames done,
//! 3 = CAN2 done; `failures` must be 0. `frames`/`can2_frames` count verified
//! loopback frames, `tx_ok`/`rx_ok` the successful calls, and `tstatr`/`errsr`
//! are the CAN1 registers read back at the end.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::CanCheck;
use hal::can::{Can, CanFifo, CanFrame, CanMode, Config, ExtendedId, Filter, Id, StandardId};

ipc::mailbox!(CanCheck);
use panic_halt as _;

/// Count a failed check, at most once.
fn check(failures: &mut u32, condition: bool) {
    *failures += u32::from(!condition);
}

/// Filters plus 72 loopback frames, shared by CAN1 and (with shorter counts)
/// the CAN2 node. Returns the number of frames whose echo matched.
fn exercise<T: hal::can::Instance>(
    can: &mut Can<'_, T>,
    standard: StandardId,
    extended: ExtendedId,
    rounds: usize,
    checks: &mut u32,
    failures: &mut u32,
    tx_ok: &mut u32,
    rx_ok: &mut u32,
    last_id: &mut u32,
) -> u32 {
    check(
        failures,
        can.set_filter(Filter::exact(0, CanFifo::Fifo0, standard, None)).is_ok(),
    );
    check(
        failures,
        can.set_filter(Filter::exact(13, CanFifo::Fifo1, extended, None))
            .is_ok(),
    );
    *checks += 2;

    let mut frames = 0;
    for round in 0..rounds {
        let id = if round % 2 == 0 {
            Id::from(standard)
        } else {
            Id::from(extended)
        };
        let length = round % 9;
        let data = [round as u8, 1, 2, 3, 0x80, 0xff, 6, 7];
        let remote = round % 4 >= 2;
        let frame = if remote {
            CanFrame::new_remote(id, length)
        } else {
            CanFrame::new(id, &data[..length])
        }
        .unwrap();
        let fifo = if round % 2 == 0 { CanFifo::Fifo0 } else { CanFifo::Fifo1 };

        match can.blocking_transmit(&frame) {
            Ok(()) => *tx_ok += 1,
            Err(_) => *failures += 1,
        }
        match can.blocking_receive(fifo) {
            Ok(received) => {
                *rx_ok += 1;
                if received == frame {
                    frames += 1;
                } else {
                    *failures += 1;
                }
                *last_id = match received.id() {
                    Id::Standard(id) => u32::from(id.as_raw()),
                    Id::Extended(id) => id.as_raw(),
                };
            }
            Err(_) => *failures += 1,
        }
        *checks += 2;
    }
    frames
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut peripherals = hal::init(hal::Config::default());

    let standard = StandardId::new(0x123).unwrap();
    let extended = ExtendedId::new(0x123_4567).unwrap();
    let mut config = Config::default();
    config.mode = CanMode::SilentLoopback;

    let mut failures = 0u32;
    let mut checks = 0u32;
    let mut tx_ok = 0u32;
    let mut rx_ok = 0u32;
    let mut last_id = 0u32;

    // CAN1 is the filter master, so its constructor also owns the shared table.
    {
        let mut can = Can::<hal::peripherals::CAN1>::new_blocking(
            peripherals.CAN1.reborrow(),
            peripherals.PA11.reborrow(),
            peripherals.PA12.reborrow(),
            config,
        )
        .unwrap();

        // Bank 41 lives in the second half of the table (`FMCFGR1`/`FWR1`);
        // bank 42 is past the last bank and must be rejected. Bank 14 is
        // CAN2's first bank by the reset split but is still configured here.
        check(
            &mut failures,
            can.set_filter(Filter::accept_all(14, CanFifo::Fifo0)).is_ok(),
        );
        check(
            &mut failures,
            can.set_filter(Filter::accept_all(41, CanFifo::Fifo1)).is_ok(),
        );
        check(&mut failures, can.disable_filter(41).is_ok());
        check(&mut failures, can.disable_filter(14).is_ok());
        check(
            &mut failures,
            can.set_filter(Filter::accept_all(42, CanFifo::Fifo0)).is_err(),
        );
        check(&mut failures, can.disable_filter(42).is_err());
        checks += 6;

        let frames = exercise(
            &mut can,
            standard,
            extended,
            72,
            &mut checks,
            &mut failures,
            &mut tx_ok,
            &mut rx_ok,
            &mut last_id,
        );
        mailbox.payload.frames.store(frames, Relaxed);
        mailbox.payload.tstatr.store(hal::pac::CAN1.tstatr().read().0, Relaxed);
        mailbox.payload.errsr.store(hal::pac::CAN1.errsr().read().0, Relaxed);
    }

    mailbox.payload.stage.store(2, Relaxed);
    mailbox.payload.filter_banks.store(42, Relaxed);
    mailbox.payload.pclk1.store(hal::rcc::clocks().pclk1.0, Relaxed);
    mailbox.payload.tx_ok.store(tx_ok, Relaxed);
    mailbox.payload.rx_ok.store(rx_ok, Relaxed);
    mailbox.payload.last_id.store(last_id, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);

    // CAN2 borrows CAN1 for the shared filter table. CAN1 was released above.
    {
        let mut can = Can::<hal::peripherals::CAN2>::new_blocking(
            peripherals.CAN1.reborrow(),
            peripherals.CAN2.reborrow(),
            peripherals.PB12.reborrow(),
            peripherals.PB13.reborrow(),
            config,
        )
        .unwrap();
        check(
            &mut failures,
            can.set_filter(Filter::exact(14, CanFifo::Fifo0, standard, None))
                .is_ok(),
        );
        checks += 1;

        let frame = CanFrame::new(standard, &[0xca, 0x12, 0x34]).unwrap();
        let mut can2_frames = 0u32;
        for _ in 0..16 {
            check(&mut failures, can.blocking_transmit(&frame).is_ok());
            let received = can.blocking_receive(CanFifo::Fifo0);
            check(
                &mut failures,
                matches!(received, Ok(ref received) if *received == frame),
            );
            can2_frames += 1;
        }
        checks += 32;
        mailbox.payload.can2_frames.store(can2_frames, Relaxed);
    }

    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(3, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(4, Relaxed);
    }

    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
