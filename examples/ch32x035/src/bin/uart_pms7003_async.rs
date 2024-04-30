#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::gpio::{Level, Output, Pin};
use hal::usart::{self, UartRx};
use hal::{bind_interrupts, peripherals, println};

bind_interrupts!(struct Irqs {
    USART4 => usart::InterruptHandler<peripherals::USART4>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);
    hal::embassy::init();

    // Connector pinout:
    // GND, VCC, PC17, PC16
    // GND, VCC, SDA, SCL (I2C1) - not working
    // GND, VCC, TX, RX (USART4, remap=5)
    // GND, VCC, RX, TX (USART4, remap=2)

    // GPIO
    let mut led = Output::new(p.PB12, Level::High, Default::default());

    let mut uart_conf: hal::usart::Config = Default::default();
    uart_conf.baudrate = 9600;
    uart_conf.detect_previous_overrun = false;
    let mut rx = UartRx::new(p.USART4, Irqs, p.PC16, p.DMA1_CH8, uart_conf).unwrap();

    let mut buf = [0u8; 32];
    loop {
        rx.read(&mut buf).await.unwrap();

        let m = Measurement::new(&buf);

        if let Some(m) = m {
            println!("=> PM1.0: {} ug/m3\t PM2.5: {} ug/m3", m.pm1_0(), m.pm2_5());
            // println!("=> PM10: {} ug/m3", m.pm10());
        } else {
            println!("=> invalid data");
        }

        led.toggle();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {}", _info);
    loop {}
}

#[derive(Debug)]
pub struct Measurement<'a>(&'a [u8; 32]);

impl Measurement<'_> {
    pub fn new<'a>(data: &'a [u8; 32]) -> Option<Measurement<'a>> {
        if data[0] == 0x42 && data[1] == 0x4d {
            let this = Measurement(data);
            if this.checksum() {
                return Some(this);
            }
        }
        None
    }

    #[inline]
    fn checksum(&self) -> bool {
        let checksum: u16 = self.0.iter().take(30).map(|&n| n as u16).sum();
        if checksum == u16::from_be_bytes([self.0[30], self.0[31]]) {
            true
        } else {
            false
        }
    }

    pub fn pm1_0(&self) -> u16 {
        u16::from_be_bytes([self.0[4], self.0[5]])
    }

    pub fn pm2_5(&self) -> u16 {
        u16::from_be_bytes([self.0[6], self.0[7]])
    }

    pub fn pm10(&self) -> u16 {
        u16::from_be_bytes([self.0[8], self.0[9]])
    }

    pub fn pm1_0_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[10], self.0[11]])
    }

    pub fn pm2_5_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[12], self.0[13]])
    }

    pub fn pm10_atm(&self) -> u16 {
        u16::from_be_bytes([self.0[14], self.0[15]])
    }

    pub fn gt0_3um(&self) -> u16 {
        u16::from_be_bytes([self.0[16], self.0[17]])
    }

    pub fn gt0_5um(&self) -> u16 {
        u16::from_be_bytes([self.0[18], self.0[19]])
    }

    pub fn gt1_0um(&self) -> u16 {
        u16::from_be_bytes([self.0[20], self.0[21]])
    }

    pub fn gt2_5um(&self) -> u16 {
        u16::from_be_bytes([self.0[22], self.0[23]])
    }

    pub fn gt5_0um(&self) -> u16 {
        u16::from_be_bytes([self.0[24], self.0[25]])
    }

    pub fn gt10um(&self) -> u16 {
        u16::from_be_bytes([self.0[26], self.0[27]])
    }
}
