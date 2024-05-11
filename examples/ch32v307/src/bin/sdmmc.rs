#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use ch32_hal::sdio::Sdmmc;
use ch32_hal::time::mhz;
use ch32_hal::{bind_interrupts, peripherals, sdio};
use embassy_executor::Spawner;
use hal::println;

bind_interrupts!(struct Irqs {
    SDIO => sdio::InterruptHandler<peripherals::SDIO>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSE;
    let p = hal::init(config);
    hal::embassy::init();

    println!("clk => {}", hal::rcc::clocks().hclk.0);

    println!("Hello World!");

    let mut sdmmc = Sdmmc::new_4bit(
        p.SDIO,
        Irqs,
        p.DMA2_CH4,
        p.PC12,
        p.PD2,
        p.PC8,
        p.PC9,
        p.PC10,
        p.PC11,
        Default::default(),
    );

    //  let mut tfsw = Output::new(p.PD7, Level::Low, Default::default());
    // tfsw.set_low();

    // Should print 400kHz for initialization
    println!("Configured clock: {}", sdmmc.clock().0);

    sdmmc.init_card(mhz(24)).await.unwrap();

    println!("init ok");

    let card = sdmmc.card().unwrap();

    println!("Card: {:#?}", card);

    loop {}
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
