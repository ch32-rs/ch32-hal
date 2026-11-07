//! CAN1 async self-check in silent loopback (V3F only, `embassy` feature).
//!
//! `cargo xtask flash --example can_async_check && cargo xtask report --example can_async_check`
//!
//! `can_check` only exercises the blocking path. This builds the same node in
//! async mode, so the four interrupt handlers (TX, RX0, RX1, SCE) and the
//! waker plumbing are actually run: 16 frames alternate between a standard
//! data frame routed to FIFO0 and an extended remote frame routed to FIFO1,
//! and each one is awaited back through `transmit()`/`receive()`.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::CanAsyncCheck;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::bind_interrupts;
use hal::can::{Can, CanFifo, CanFrame, CanMode, Config as CanConfig, ExtendedId, Filter, Id, StandardId};

ipc::mailbox!(CanAsyncCheck);
use panic_halt as _;

bind_interrupts!(struct Irqs {
    CAN1_TX => hal::can::TransmitInterruptHandler<hal::peripherals::CAN1>;
    CAN1_RX0 => hal::can::ReceiveInterruptHandler<hal::peripherals::CAN1, 0>;
    CAN1_RX1 => hal::can::ReceiveInterruptHandler<hal::peripherals::CAN1, 1>;
    CAN1_SCE => hal::can::ErrorInterruptHandler<hal::peripherals::CAN1>;
});

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, core::sync::atomic::Ordering::Relaxed);

    let mut p = hal::init(hal::Config::default());
    let mut config = CanConfig::default();
    config.mode = CanMode::SilentLoopback;

    let mut can = Can::<hal::peripherals::CAN1, hal::mode::Async>::new(
        p.CAN1.reborrow(),
        p.PA11.reborrow(),
        p.PA12.reborrow(),
        Irqs,
        config,
    )
    .unwrap();

    let standard = StandardId::new(0x321).unwrap();
    let extended = ExtendedId::new(0x654_321).unwrap();
    let mut checks = 0u32;
    let mut failures = 0u32;
    let mut frames = 0u32;
    let mut tx_ok = 0u32;
    let mut rx_ok = 0u32;
    let mut last_id = 0u32;

    match can.set_filter(Filter::exact(0, CanFifo::Fifo0, standard, None)) {
        Ok(()) => checks += 1,
        Err(_) => {
            checks += 1;
            failures += 1;
        }
    }
    match can.set_filter(Filter::exact(1, CanFifo::Fifo1, extended, None)) {
        Ok(()) => checks += 1,
        Err(_) => {
            checks += 1;
            failures += 1;
        }
    }

    for round in 0..16usize {
        let (id, fifo) = if round % 2 == 0 {
            (Id::from(standard), CanFifo::Fifo0)
        } else {
            (Id::from(extended), CanFifo::Fifo1)
        };
        let frame = if round % 4 == 3 {
            CanFrame::new_remote(id, round % 9).unwrap()
        } else {
            CanFrame::new(id, &[round as u8, 1, 2, 3, 4]).unwrap()
        };

        match can.transmit(&frame).await {
            Ok(()) => tx_ok += 1,
            Err(_) => failures += 1,
        }
        match can.receive(fifo).await {
            Ok(received) => {
                rx_ok += 1;
                if received == frame {
                    frames += 1;
                } else {
                    failures += 1;
                }
                last_id = match received.id() {
                    Id::Standard(id) => u32::from(id.as_raw()),
                    Id::Extended(id) => id.as_raw(),
                };
            }
            Err(_) => failures += 1,
        }
        checks += 2;
    }

    mailbox
        .payload
        .frames
        .store(frames, core::sync::atomic::Ordering::Relaxed);
    mailbox
        .payload
        .tx_ok
        .store(tx_ok, core::sync::atomic::Ordering::Relaxed);
    mailbox
        .payload
        .rx_ok
        .store(rx_ok, core::sync::atomic::Ordering::Relaxed);
    mailbox
        .payload
        .last_id
        .store(last_id, core::sync::atomic::Ordering::Relaxed);
    mailbox
        .payload
        .checks
        .store(checks, core::sync::atomic::Ordering::Relaxed);
    mailbox
        .payload
        .failures
        .store(failures, core::sync::atomic::Ordering::Relaxed);
    mailbox.payload.stage.store(2, core::sync::atomic::Ordering::Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, core::sync::atomic::Ordering::Relaxed);
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
