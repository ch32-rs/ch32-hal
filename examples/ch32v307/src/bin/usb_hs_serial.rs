#![no_std]
#![no_main]

use panic_halt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use ch32_hal::usb::EndpointDataBuffer;
use ch32_hal::usbhs::{Driver};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use ch32_hal::usbhs::{InterruptHandler, WakeupInterruptHandler, Instance};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;

bind_interrupts!(struct Irq {
    USBHS => InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => WakeupInterruptHandler<peripherals::USBHS>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);


    // Create the driver, from the HAL.
    let mut buffer: [EndpointDataBuffer; 4] = core::array::from_fn(|_| EndpointDataBuffer::default());
    let driver = Driver::new(p.USBHS, Irq,  p.PB7, p.PB6, &mut buffer);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            let _ = echo(&mut class).await;
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd, const NR_EP: usize>(class: &mut CdcAcmClass<'d, Driver<'d, T, NR_EP>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        class.write_packet(data).await?;
    }
}
