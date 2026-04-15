//! USB CDC ACM (Virtual COM Port) example for CH32V208
//!
//! This example demonstrates USB Device functionality using the USBD peripheral.
//! Connect the USB port and it will appear as a serial port on the host.
//! Data received will be echoed back.

#![no_std]
#![no_main]

use ch32_hal::usbd::{Driver, InterruptHandler};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irq {
    USB_LP_CAN1_RX0 => InterruptHandler<peripherals::USBD>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);

    // Create the driver from the HAL
    let driver = Driver::new(p.USBD, Irq, p.PA12, p.PA11);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("WCH");
    config.product = Some("CH32V208 USB CDC");
    config.serial_number = Some("12345678");

    // Required for Windows compatibility
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config
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

    // Create classes on the builder
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder
    let mut usb = builder.build();

    // Run the USB device
    let usb_fut = usb.run();

    // Echo task
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            ch32_hal::println!("USB connected");
            let _ = echo(&mut class).await;
            ch32_hal::println!("USB disconnected");
        }
    };

    // Run everything concurrently
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

async fn echo<'d>(class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USBD>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        class.write_packet(data).await?;
    }
}

