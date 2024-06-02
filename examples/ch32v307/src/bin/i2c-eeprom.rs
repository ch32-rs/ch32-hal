#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use hal::gpio::{AnyPin, Level, Output, Pin, Speed};
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::time::Hertz;
use hal::{peripherals, println};

pub struct EEPROM {
    i2c: I2c<'static, peripherals::I2C2, Blocking>,
    address: u8,
}

impl EEPROM {
    pub fn new(i2c: I2c<'static, peripherals::I2C2, Blocking>, address: u8) -> Self {
        Self { i2c, address }
    }

    pub fn write_byte(&mut self, addr: u16, byte: u8) -> Result<(), hal::i2c::Error> {
        let addr = addr.to_be_bytes();
        let mut data = [0u8; 3];
        data[0] = addr[0];
        data[1] = addr[1];
        data[2] = byte;
        println!("write {:?}", data);
        self.i2c.blocking_write(self.address, &data)
    }

    pub fn read_byte(&mut self, addr: u16) -> Result<u8, hal::i2c::Error> {
        let addr = addr.to_be_bytes();
        let mut data = [0u8; 1];
        self.i2c.blocking_write_read(self.address, &addr[..], &mut data)?;
        Ok(data[0])
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    // let p = hal::init(Default::default());
    //let p = hal::init(Default::default());
    let p = hal::init(hal::Config::default());
    hal::embassy::init();

    let i2c_sda = p.PB11;
    let i2c_scl = p.PB10;

    println!("init ok");

    let mut i2c = I2c::new_blocking(p.I2C2, i2c_scl, i2c_sda, Hertz::khz(100), Default::default());

    // 7-bit address
    const FT24C32A_ADDR: u8 = 0b1010_000;
    // 32Kbit -> 4KByte

    let mut buf = [0u8; 2];

    let mut eeprom = EEPROM::new(i2c, FT24C32A_ADDR);

    println!("init i2c ok");

    // GPIO
    spawner.spawn(blink(p.PA15.degrade())).unwrap();

    for addr in 0..16 {
        eeprom.write_byte(addr, b'x').unwrap();
    }

    Timer::after_millis(10).await; // must

    for addr in 0..16 {
        let data = eeprom.read_byte(addr).unwrap();
        println!("addr: {:02x}, data: {:02x}", addr, data);
    }

    loop {
        Timer::after_millis(2000).await;
        println!("tick in main");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}

#[embassy_executor::task]
async fn blink(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, Speed::Low);

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(1000)).await;
        led.set_low();
        Timer::after(Duration::from_millis(1000)).await;

        println!("tick in blink");
    }
}
