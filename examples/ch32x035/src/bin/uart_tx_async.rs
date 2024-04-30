#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::arch::asm;

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::dma::Priority;
use hal::gpio::{AnyPin, Level, Output, Pin};
use hal::usart::UartTx;
use hal::{interrupt, println};

//bind_interrupts!(struct Irqs {
//    USART4 => usart::InterruptHandler<peripherals::USART4>;
//});

//#[no_mangle]
//unsafe extern "C" fn DMA1_CHANNEL1() {
//    println!("in irq");
//}

#[embassy_executor::task(pool_size = 3)]
async fn blink(pin: AnyPin, interval_ms: u64) {
    let mut led = Output::new(pin, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(interval_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    config.dma_interrupt_priority = interrupt::Priority::P0;
    config = Default::default();
    let p = hal::init(config);
    hal::embassy::init();

    // Connector pinout:
    // GND, VCC, PC17, PC16
    // GND, VCC, SDA, SCL (I2C1) - not working
    // GND, VCC, TX, RX (USART4, remap=5)
    // GND, VCC, RX, TX (USART4, remap=2)

    let mut uart_config = hal::usart::Config::default();
    uart_config.baudrate = 9600;
    let mut tx = UartTx::new(p.USART4, p.PC17, p.DMA1_CH1, uart_config).unwrap();

    // GPIO
    // let mut led = Output::new(p.PB12, Level::High, Default::default());
    spawner.spawn(blink(p.PB12.degrade(), 1000)).unwrap();

    let buf = b"Hello World\r\n";
    loop {
        tx.write(buf).await.unwrap();
        //tx.blocking_flush();
        // Timer::after_millis(100).await;
        // led.toggle();
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    println!("panic: {:?}", _info);
    loop {}
}
