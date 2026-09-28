#![no_std]
#![no_main]

use core::fmt::Write;

use ch32_hal as hal;
use ch32_hal::usb::EndpointDataBuffer512;
use ch32_hal::usbhs::{self, Driver};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use hal::adc::{Pga, SampleTime};
use hal::{bind_interrupts, peripherals};
use heapless::String;
use panic_halt as _;

bind_interrupts!(
    struct Irqs {
        ADC => hal::adc::InterruptHandler<peripherals::ADC1>;
        USBHS => usbhs::InterruptHandler<peripherals::USBHS>;
        USBHS_WKUP => usbhs::WakeupInterruptHandler<peripherals::USBHS>;
    }
);

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    });

    let mut adc = hal::adc::Adc::new_async(p.ADC1, Default::default(), Irqs);
    let mut ch = p.PA5;

    let mut ep_buffer: [EndpointDataBuffer512; 4] = core::array::from_fn(|_| EndpointDataBuffer512::default());
    let driver = Driver::new(p.USBHS, Irqs, p.PB7, p.PB6, &mut ep_buffer);

    let mut config = embassy_usb::Config::new(0xC0DE, 0xCAFE);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("ADC async");
    config.serial_number = Some("12345678");
    config.max_power = 100;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 512);
    let mut usb = builder.build();

    let usb_fut = usb.run();
    let adc_fut = async {
        loop {
            class.wait_connection().await;

            loop {
                let val = adc.convert(&mut ch, SampleTime::CYCLES239_5, Pga::X1).await;

                let mut line: String<32> = String::new();
                let _ = writeln!(line, "adc: {}", val);
                if class.write_packet(line.as_bytes()).await.is_err() {
                    break;
                }

                Timer::after(Duration::from_millis(100)).await;
            }
        }
    };

    join(usb_fut, adc_fut).await;
}
