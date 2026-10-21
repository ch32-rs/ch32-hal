#![no_std]
#![no_main]

use ch32_hal::i2c::{I2c, SlaveConfig, SlaveAddress, SlaveCommand};
use ch32_hal::time::Hertz;
use embassy_executor::Spawner;
use {ch32_hal as hal, panic_halt as _};

hal::bind_interrupts!(struct Irqs {
    I2C1_EV => hal::i2c::EventInterruptHandler<hal::peripherals::I2C1>;
    I2C1_ER => hal::i2c::ErrorInterruptHandler<hal::peripherals::I2C1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let i2c_master = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::khz(100),
        Default::default(),
    );

    let mut slave = i2c_master.into_slave(SlaveConfig {
        address: SlaveAddress::SevenBit(0x50),
        general_call: false,
    });

    let mut rx_buf = [0u8; 64];
    let tx_data = b"Hello from CH32V203 I2C slave";

    loop {
        match slave.listen().await {
            Ok(SlaveCommand::WriteCommand) => {
                match slave.read(&mut rx_buf).await {
                    Ok(_received_bytes) => {
                        // handle received data
                    }
                    Err(_) => {}
                }
            }
            Ok(SlaveCommand::ReadCommand) => {
                let _ = slave.write(tx_data).await;
            }
            Ok(SlaveCommand::GeneralCall) => {}
            Err(_) => {}
        }
    }
}
