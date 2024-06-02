#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;

use hal::gpio::{Level, Output};
use hal::i2c::I2c;
use hal::mode::Blocking;
use hal::println;
use hal::time::Hertz;

pub mod regs {
    /// ALS/UVS operation mode control, SW reset
    pub const MAIN_CTRL: u8 = 0x00;

    /// ALS/UVS measurement rate and resolution in Active Mode
    pub const ALS_UVS_MEAS_RATE: u8 = 0x04;

    /// ALS/UVS analog Gain range
    pub const ALS_UVS_GAIN: u8 = 0x05;

    /// Part number ID and revision ID
    pub const PART_ID: u8 = 0x06;

    /// Power-On status, Interrupt status, Data status
    pub const MAIN_STATUS: u8 = 0x07;

    /// ALS ADC measurement data, LSB
    pub const ALS_DATA_0: u8 = 0x0D;

    /// ALS ADC measurement data
    pub const ALS_DATA_1: u8 = 0x0E;

    /// ALS ADC measurement data, MSB
    pub const ALS_DATA_2: u8 = 0x0F;

    /// UVS ADC measurement data, LSB
    pub const UVS_DATA_0: u8 = 0x10;

    /// UVS ADC measurement data
    pub const UVS_DATA_1: u8 = 0x11;

    /// UVS ADC measurement data, MSB
    pub const UVS_DATA_2: u8 = 0x12;

    /// Interrupt configuration
    pub const INT_CFG: u8 = 0x19;

    /// Interrupt persist setting
    pub const INT_PST: u8 = 0x1A;

    /// ALS/UVS interrupt upper threshold, LSB
    pub const ALS_UVS_THRES_UP_0: u8 = 0x21;

    /// ALS/UVS interrupt upper threshold, intervening bits
    pub const ALS_UVS_THRES_UP_1: u8 = 0x22;

    /// ALS/UVS interrupt upper threshold, MSB
    pub const ALS_UVS_THRES_UP_2: u8 = 0x23;

    /// ALS/UVS interrupt lower threshold, LSB
    pub const ALS_UVS_THRES_LOW_0: u8 = 0x24;

    /// ALS/UVS interrupt lower threshold, intervening bits
    pub const ALS_UVS_THRES_LOW_1: u8 = 0x25;

    /// ALS/UVS interrupt lower threshold, MSB
    pub const ALS_UVS_THRES_LOW_2: u8 = 0x26;
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let _ = spawner;
    let mut config = hal::Config::default();
    {
        use hal::rcc::*;

        config.rcc = Config {
            hse: Some(Hse {
                freq: Hertz::mhz(12),
                mode: HseMode::Oscillator,
            }),
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV1,
                mul: PllMul::MUL12, // 12 * 12 = max 144MHz, or else overclock
            }),
            sys: Sysclk::PLL,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV4, // 144 / 4 = 36MHz, max input clock of i2c is 36MHz
            apb2_pre: APBPrescaler::DIV1,

            ..Default::default()
        }
    }
    let p = hal::init(config);
    hal::embassy::init();

    Timer::after_millis(100).await;

    println!("embassy init ok");

    let clks = hal::rcc::clocks();
    println!("Clocks: {:?}", clks);

    let mut led = Output::new(p.PA8, Level::Low, Default::default());

    let scl = p.PB10;
    let sda = p.PB11;

    // on APB1
    let mut i2c_config = hal::i2c::Config::default();
    //  i2c_config.scl_pullup = true;
    //i2c_config.sda_pullup = true;
    let mut i2c = I2c::new_blocking(p.I2C2, scl, sda, Hertz::hz(400_000), Default::default());

    let addr = 0x53;

    let mut buf = [0u8; 1];
    i2c.blocking_write_read(addr, &[regs::PART_ID], &mut buf).unwrap();

    println!("Part ID: 0x{:02X}", buf[0]);

    loop {
        Timer::after_secs(1).await;

        led.toggle();
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
