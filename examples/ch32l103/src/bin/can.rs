#![no_std]
#![no_main]

use ch32_hal::can::{Can, CanFifo, CanFilter, CanFrame, CanMode, ExtendedId, StandardId};
use ch32_hal::{self as hal};
use embassy_executor::Spawner;
use hal::println;

use embedded_can::blocking::Can as CanBlocking;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    let mut can = Can::new_blocking(
        p.CAN1,
        p.PA11,
        p.PA12,
        CanFifo::Fifo0,
        CanMode::Normal,
        1_000_000,
        Default::default(),
    )
    .expect("Valid");
    let mut filter = CanFilter::new_id_list();

    filter
        .get(0)
        .unwrap()
        .set(StandardId::new(0x580 | 0x42).unwrap().into(), Default::default());

    can.add_filter(CanFilter::accept_all());

    let body: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];

    loop {
        // Send standard ID frame
        let frame = CanFrame::new(StandardId::new(0x317).unwrap(), &body).unwrap();
        let _ = can.transmit(&frame);

        // Send extended ID frame
        let frame = CanFrame::new(ExtendedId::new(0x12345678).unwrap(), &body).unwrap();
        let _ = can.transmit(&frame);

        let _ = can
            .receive()
            .and_then(|frame| {
                println!("Received frame: {:?}", frame);
                Ok(())
            })
            .or_else(|e| {
                println!("No frame received: {:?}", e);
                Err(())
            });
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
