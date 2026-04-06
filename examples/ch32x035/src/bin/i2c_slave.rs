//! I2C slave example — simple register device.
//!
//! Emulates a tiny I2C register device at address 0x42:
//!
//!   Reg 0x00  STATUS     (R)   — transaction counter (wraps at 255)
//!   Reg 0x01  SCRATCH    (R/W) — general-purpose scratch byte
//!   Reg 0x02  LED        (R/W) — bit 0 controls LED (1=on, 0=off)
//!   Reg 0x0F  WHO_AM_I   (R)   — always reads the slave address
//!
//! Protocol (standard I2C register convention):
//!   Write register:  [START] [0x42+W] [reg_addr] [data...] [STOP]
//!   Read  register:  [START] [0x42+W] [reg_addr] [Sr] [0x42+R] [data...] [STOP]
//!
//! Pins: PC16 = SCL, PC17 = SDA (I2C1 remap 2)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use hal::gpio::{Level, Output};
use hal::i2c::slave::{I2cSlave, SlaveCommandKind, SlaveConfig};
use hal::println;
use hal::{bind_interrupts, peripherals};

bind_interrupts!(
    struct Irqs {
        I2C1_EV => hal::i2c::EventInterruptHandler<peripherals::I2C1>;
        I2C1_ER => hal::i2c::ErrorInterruptHandler<peripherals::I2C1>;
    }
);

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

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    let mut slave_config = SlaveConfig::default();
    slave_config.addr = 0x42;

    let mut i2c = I2cSlave::new::<2>(p.I2C1, p.PC16, p.PC17, Irqs, slave_config);

    println!("I2C slave ready at 0x42");

    let mut regs = RegFile {
        status: 0,
        scratch: 0,
        led: 0,
        who_am_i: slave_config.addr,
    };
    let mut reg_ptr: u8 = 0;

    loop {
        let cmd = match i2c.listen().await {
            Ok(cmd) => cmd,
            Err(e) => {
                println!("listen error: {:?}", e);
                continue;
            }
        };

        match cmd.kind {
            SlaveCommandKind::Write => {
                let mut buf = [0u8; 8];
                match i2c.respond_to_write(&mut buf).await {
                    Ok(1) => reg_ptr = buf[0],
                    Ok(2) => {
                        reg_ptr = buf[0];
                        match Reg::from_addr(reg_ptr) {
                            Some(Reg::Scratch) => regs.scratch = buf[1],
                            Some(Reg::Led) => {
                                regs.led = buf[1] & 0x01;
                                if regs.led != 0 {
                                    led.set_high();
                                } else {
                                    led.set_low();
                                }
                            }
                            _ => {}
                        }
                    }
                    Ok(_) => {}
                    Err(e) => println!("write error: {:?}", e),
                }
            }

            SlaveCommandKind::Read => {
                regs.status = regs.status.wrapping_add(1);

                let mut buf = [0u8; 8];
                for (i, b) in buf.iter_mut().enumerate() {
                    *b = regs.read(reg_ptr.wrapping_add(i as u8));
                }

                match i2c.respond_to_read(&buf).await {
                    Ok(status) => println!("read from 0x{:02X}: {:?}", reg_ptr, status),
                    Err(e) => println!("read error: {:?}", e),
                }
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
