#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal::can::{Can, CanFifo, CanFilter, CanFrame, CanMode, StandardId};
use embassy_time::{Duration, Ticker};
use embassy_executor::Spawner;

use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());

    let can = Can::new(p.CAN1, p.PA11, p.PA12, CanFifo::Fifo1, CanMode::Normal, 500_000).expect("Valid");
    can.add_filter(CanFilter::accept_all());

    let mut ticker = Ticker::every(Duration::from_millis(200));

    let body: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];

    loop {
        let frame = CanFrame::new(StandardId::new(0x317).unwrap(), &body).unwrap();

        let _ = can.transmit(&frame);
        ticker.next().await;
    }
}
