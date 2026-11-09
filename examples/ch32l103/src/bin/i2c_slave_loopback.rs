//! Single-board I2C slave loopback test for CH32L103.
//!
//! The slave under test runs on **I2C2**: `PB10 = SCL`, `PB11 = SDA`. That
//! pair is not shared with the USBPD CC pins, so nothing pulls it down at
//! reset. The reference master runs on I2C1's default mapping, `PB6 = SCL`,
//! `PB7 = SDA`; those two are the USBPD CC pins, so the example clears the
//! Type-C Rd first.
//!
//! Wire the two buses together:
//!
//! ```text
//! PB6 (I2C1 SCL) <-> PB10 (I2C2 SCL)
//! PB7 (I2C1 SDA) <-> PB11 (I2C2 SDA)
//! 4.7 kΩ pull-ups from SCL and SDA to 3.3 V
//! ```
//!
//! `PB12` is the LED (also I2C2's `SMBA`; SMBus alert is unused).
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin i2c_slave_loopback
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::i2c::{Config, Error, ErrorInterruptHandler, EventInterruptHandler, I2c, SlaveCommandKind, SlaveConfig};
use hal::time::Hertz;
use hal::{bind_interrupts, pac, peripherals};
use panic_halt as _;

const SLAVE_ADDR: u8 = 0x42;
const WRONG_ADDR: u8 = 0x43;

const STATUS: u8 = 0x00;
const SCRATCH: u8 = 0x01;
const LED: u8 = 0x02;
const WHO_AM_I: u8 = 0x0F;

bind_interrupts!(struct Irqs {
    I2C1_EV => EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => ErrorInterruptHandler<peripherals::I2C1>;
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

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());

    // PB6/PB7 are the USBPD CC pins. The block must be clocked or the
    // CC_PD writes are dropped and the 5.1 kΩ Rd keeps the master's bus low.
    pac::RCC.ahbpcenr().modify(|w| w.set_usbpden(true));
    pac::USBPD.port_cc1().modify(|w| w.set_cc_pd(false));
    pac::USBPD.port_cc2().modify(|w| w.set_cc_pd(false));

    let mut led = Output::new(p.PB12, Level::Low, Default::default());

    // Slave under test: I2C2 on USBPD-free pins, DMA1_CH4/CH5.
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

    // Reference master: I2C1 default mapping, DMA1_CH6/CH7.
    let mut master = I2c::new::<0>(
        p.I2C1,
        p.PB6,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::khz(100),
        Default::default(),
    );

    defmt::info!(
        "I2C loopback: slave 0x{:02X} on PB10/PB11, master on PB6/PB7",
        SLAVE_ADDR
    );

    join(
        async {
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
                    }

                    SlaveCommandKind::Read => {
                        regs.status = regs.status.wrapping_add(1);

                        let mut buf = [0u8; 8];
                        for (i, byte) in buf.iter_mut().enumerate() {
                            *byte = regs.read(reg_ptr.wrapping_add(i as u8));
                        }

                        match slave.respond_to_read(&buf).await {
                            Ok(status) => defmt::info!("read 0x{:02X}: {:?}", reg_ptr, status),
                            Err(e) => defmt::warn!("read error: {:?}", e),
                        }
                    }
                }
            }
        },
        async {
            // Give the slave task a chance to arm before the first transaction.
            Timer::after_millis(10).await;

            loop {
                let mut failed = 0u32;

                // Both buses should idle high with the pull-ups fitted.
                let indr = pac::GPIOB.indr().read();
                defmt::info!(
                    "bus idle: I2C1 SCL(PB6)={} SDA(PB7)={}, I2C2 SCL(PB10)={} SDA(PB11)={}",
                    indr.idr(6),
                    indr.idr(7),
                    indr.idr(10),
                    indr.idr(11)
                );

                let mut who_am_i = [0u8; 1];
                match master.write_read(SLAVE_ADDR, &[WHO_AM_I], &mut who_am_i).await {
                    Ok(()) => {
                        defmt::info!("WHO_AM_I = 0x{:02X}", who_am_i[0]);
                        if who_am_i[0] == SLAVE_ADDR {
                            defmt::info!("PASS who_am_i");
                        } else {
                            failed += 1;
                            defmt::warn!("FAIL who_am_i");
                        }
                    }
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL who_am_i: {:?}", e);
                    }
                }

                match master.write(SLAVE_ADDR, &[SCRATCH, 0x5A]).await {
                    Ok(()) => defmt::info!("PASS write_scratch"),
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL write_scratch: {:?}", e);
                    }
                }

                let mut scratch = [0u8; 1];
                match master.write_read(SLAVE_ADDR, &[SCRATCH], &mut scratch).await {
                    Ok(()) => {
                        defmt::info!("SCRATCH = 0x{:02X}", scratch[0]);
                        if scratch[0] == 0x5A {
                            defmt::info!("PASS read_scratch");
                        } else {
                            failed += 1;
                            defmt::warn!("FAIL read_scratch");
                        }
                    }
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL read_scratch: {:?}", e);
                    }
                }

                let mut over_read = [0u8; 16];
                match master.write_read(SLAVE_ADDR, &[STATUS], &mut over_read).await {
                    Ok(()) => {
                        defmt::info!("over-read {:02X}", over_read);
                        let padded = over_read[8..] == [0u8; 8];
                        let scratch_visible = over_read[1] == 0x5A;
                        if padded && scratch_visible {
                            defmt::info!("PASS over_read");
                        } else {
                            failed += 1;
                            defmt::warn!("FAIL over_read (pad={}, scratch={})", padded, scratch_visible);
                        }
                    }
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL over_read: {:?}", e);
                    }
                }

                match master.write(SLAVE_ADDR, &[STATUS]).await {
                    Ok(()) => defmt::info!("PASS zero_length_write"),
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL zero_length_write: {:?}", e);
                    }
                }

                match master.write(WRONG_ADDR, &[SCRATCH, 0x00]).await {
                    Err(Error::Nack) => defmt::info!("PASS address_nack"),
                    Ok(()) => {
                        failed += 1;
                        defmt::warn!("FAIL address_nack (unexpected ACK)");
                    }
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL address_nack: {:?}", e);
                    }
                }

                match master.write(SLAVE_ADDR, &[LED, 0x01]).await {
                    Ok(()) => match master.write_read(SLAVE_ADDR, &[LED], &mut scratch).await {
                        Ok(()) => {
                            defmt::info!("LED = 0x{:02X}", scratch[0]);
                            if scratch[0] == 0x01 {
                                defmt::info!("PASS led_register");
                            } else {
                                failed += 1;
                                defmt::warn!("FAIL led_register");
                            }
                        }
                        Err(e) => {
                            failed += 1;
                            defmt::warn!("FAIL led_register: {:?}", e);
                        }
                    },
                    Err(e) => {
                        failed += 1;
                        defmt::warn!("FAIL led_register: {:?}", e);
                    }
                }
                let _ = master.write(SLAVE_ADDR, &[LED, 0x00]).await;

                if failed == 0 {
                    defmt::info!("round: all checks passed");
                } else {
                    defmt::warn!("round: {} check(s) failed", failed);
                }

                Timer::after_millis(2000).await;
            }
        },
    )
    .await;

    loop {}
}
