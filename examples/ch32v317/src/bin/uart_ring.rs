#![no_std]
#![no_main]

//! Echo over USART1 (PA9 TX / PA10 RX: the on-board WCH-Link's serial port, with the J2
//! jumpers on), receiving through a DMA ring buffer: bytes keep arriving while the echo
//! is being sent, nothing is lost between reads.

use embassy_executor::Spawner;
use hal::usart::{self, Uart};
use static_cell::StaticCell;
use {ch32_hal as hal, panic_halt as _};

use ch32_hal::bind_interrupts;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let cfg = usart::Config::default();
    let (mut tx, rx) = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, cfg)
        .unwrap()
        .split();

    static RING: StaticCell<[u8; 256]> = StaticCell::new();
    let mut rx = rx.into_ring_buffered(RING.init([0; 256]));

    tx.write(b"ring-buffered echo\r\n").await.unwrap();
    let mut buf = [0u8; 64];
    loop {
        match rx.read(&mut buf).await {
            Ok(n) => tx.write(&buf[..n]).await.unwrap(),
            // an overrun: the ring filled before it was read. the next read restarts it
            Err(_) => tx.write(b"\r\n[overrun]\r\n").await.unwrap(),
        }
    }
}
