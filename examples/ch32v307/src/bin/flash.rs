#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal::flash::{Flash, FLASH_BASE};
use ch32_hal::rcc::*;
use ch32_hal::{interrupt, println};
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // NOTE: TRM, 3.2:
    // > When carrying out FLASH-related operations, it is strongly recommended that the system main
    // > frequency is not greater than 120M.
    // Here we configure the clock at 60MHz
    let p = ch32_hal::init(ch32_hal::Config {
        rcc: ch32_hal::rcc::Config {
            hse: None,
            sys: Sysclk::PLL,
            pll_src: PllSource::HSI,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV2,
                mul: PllMul::MUL15,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default(),
            hspll_src: HsPllSource::HSI,
            hspll: None,
        },
        dma_interrupt_priority: interrupt::Priority::P0,
    });
    ch32_hal::debug::SDIPrint::enable();

    // Give a chance to catch CPU if it craps itself writing to FLASH
    Timer::after_millis(1000).await;

    let mut f = Flash::new_blocking(p.FLASH);
    const size: u32 = 256;
    let start: u32 = 1024 * 32;
    let stop = start + 256 * 3;
    for offset in (start..stop).step_by(size as usize) {
        println!("Testing offset: {:#X}, size: {:#X}", offset, size);

        println!("Reading...");
        let mut buf = [0u8; 32];
        println!("{:?}", f.blocking_read(offset, &mut buf));
        println!("Read: {:?}", buf);

        println!("Erasing...");
        println!("{:?}", f.blocking_erase(offset, offset + size));

        println!("Reading...");
        let mut buf = [0u8; 32];
        println!("{:?}", f.blocking_read(offset, &mut buf));
        println!("Read after erase: {:?}", buf);

        println!("Writing...");
        println!("{:?}", f.blocking_write(offset, &[0xabu8; size as usize]));

        println!("Reading...");
        let mut buf = [0u8; 32];
        println!("{:?}", f.blocking_read(offset, &mut buf));
        println!("Read: {:?}", buf);
    }

    println!("Blocking tests done.");
    loop {}
}
