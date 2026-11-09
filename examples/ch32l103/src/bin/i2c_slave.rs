//! I2C slave example for CH32L103.
//!
//! Emulates a tiny register device at address 0x42 on **I2C2**: **PB10 = SCL**,
//! **PB11 = SDA**. I2C2 is the fixed mapping and is not shared with the USBPD
//! CC pins, so the slave sits on a clean bus.
//!
//! | Register | Name     | Access | Meaning                        |
//! |----------|----------|--------|--------------------------------|
//! | 0x00     | STATUS   | R      | transaction counter            |
//! | 0x01     | SCRATCH  | R/W    | general-purpose byte           |
//! | 0x02     | LED      | R/W    | bit 0 mirrors the on-board LED |
//! | 0x0F     | WHO_AM_I | R      | always reads 0x42              |
//!
//! Protocol (the usual I2C register convention):
//!
//! ```text
//! write register: [START] [0x42+W] [reg] [data...] [STOP]
//! read register:  [START] [0x42+W] [reg] [Sr] [0x42+R] [data...] [STOP]
//! ```
//!
//! The counterpart master lives in `examples/ch32v203/src/bin/i2c_master.rs`.
//! Wire **V203 PB6 to L103 PB10** (SCL) and **V203 PB7 to L103 PB11** (SDA),
//! join the grounds, and fit 4.7 kΩ pull-ups from both lines to 3.3 V. The
//! V203 has no USBPD, so neither side of this bus touches the CC pins.
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin i2c_slave
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::i2c::{Config, ErrorInterruptHandler, EventInterruptHandler, I2c, SlaveCommandKind, SlaveConfig};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use panic_halt as _;

const SLAVE_ADDR: u8 = 0x42;

bind_interrupts!(struct Irqs {
    I2C2_EV => EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => ErrorInterruptHandler<peripherals::I2C2>;
});

#[defmt::panic_handler]
fn panic() -> ! {
    loop {}
}

#[repr(u8)]
enum Reg {
    Status = 0x00,
    Scratch = 0x01,
    Led = 0x02,
    WhoAmI = 0x0F,
}

impl Reg {
    fn from_addr(addr: u8) -> Option<Self> {
        match addr {
            0x00 => Some(Self::Status),
            0x01 => Some(Self::Scratch),
            0x02 => Some(Self::Led),
            0x0F => Some(Self::WhoAmI),
            _ => None,
        }
    }
}

struct RegFile {
    status: u8,
    scratch: u8,
    led: u8,
    who_am_i: u8,
}

impl RegFile {
    fn read(&self, addr: u8) -> u8 {
        match Reg::from_addr(addr) {
            Some(Reg::Status) => self.status,
            Some(Reg::Scratch) => self.scratch,
            Some(Reg::Led) => self.led,
            Some(Reg::WhoAmI) => self.who_am_i,
            None => 0x00,
        }
    }
}

#[embassy_executor::task]
async fn bus_monitor() {
    loop {
        Timer::after_secs(2).await;
        let indr = pac::GPIOB.indr().read();
        defmt::info!("slave bus idle: SCL(PB10)={} SDA(PB11)={}", indr.idr(10), indr.idr(11));
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    // I2C2: TX = DMA1_CH4, RX = DMA1_CH5 on CH32L103.
    let mut slave = I2c::new_slave_with_dma::<0>(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Hertz::khz(100),
        Config::default(),
        SlaveConfig::new(SLAVE_ADDR),
    )
    .unwrap();

    defmt::info!("I2C slave 0x{:02X} listening on PB10/PB11", SLAVE_ADDR);
    spawner.spawn(bus_monitor().unwrap());

    let mut regs = RegFile {
        status: 0,
        scratch: 0,
        led: 0,
        who_am_i: SLAVE_ADDR,
    };
    let mut reg_ptr: u8 = 0;

    loop {
        let command = match slave.listen().await {
            Ok(command) => command,
            Err(e) => {
                defmt::warn!("listen error: {:?}", e);
                continue;
            }
        };

        // The peripheral only stretches the clock while the address is being
        // matched. Once `listen()` releases it, the master clocks the data byte
        // straight in and follows with a repeated START, so the response has to
        // start right away: no logging between `listen()` and `respond_to_*()`.
        match command.kind {
            SlaveCommandKind::Write | SlaveCommandKind::GeneralCall => {
                let mut buf = [0u8; 8];
                let len = match slave.respond_to_write(&mut buf).await {
                    Ok(len) => len,
                    Err(e) => {
                        defmt::warn!("write error: {:?}", e);
                        continue;
                    }
                };

                match len {
                    1 => reg_ptr = buf[0],
                    2 => {
                        reg_ptr = buf[0];
                        match Reg::from_addr(reg_ptr) {
                            Some(Reg::Scratch) => regs.scratch = buf[1],
                            Some(Reg::Led) => {
                                regs.led = buf[1] & 0x01;
                                led.set_level(if regs.led != 0 { Level::High } else { Level::Low });
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }

                defmt::info!("write {:?} len={} ptr=0x{:02X}", command.address, len, reg_ptr);
            }

            SlaveCommandKind::Read => {
                regs.status = regs.status.wrapping_add(1);

                let mut buf = [0u8; 8];
                for (i, byte) in buf.iter_mut().enumerate() {
                    *byte = regs.read(reg_ptr.wrapping_add(i as u8));
                }

                let status = match slave.respond_to_read(&buf).await {
                    Ok(status) => status,
                    Err(e) => {
                        defmt::warn!("read error: {:?}", e);
                        continue;
                    }
                };

                defmt::info!("read {:?} ptr=0x{:02X} status={:?}", command.address, reg_ptr, status);
            }
        }
    }
}
