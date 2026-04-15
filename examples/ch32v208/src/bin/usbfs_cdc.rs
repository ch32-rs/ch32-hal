//! USB CDC ACM (Virtual COM Port) example for CH32V208 using USBFS
//!
//! This example demonstrates USB Device functionality using the USBFS peripheral.
//! USBFS uses PB6 (DM) and PB7 (DP) pins.
//!
//! Connect the USB port and it will appear as a serial port on the host.
//! Data received will be echoed back.

#![no_std]
#![no_main]

use ch32_hal::gpio::{Level, Output};
use ch32_hal::usb::EndpointDataBuffer64;
use ch32_hal::usbfs::{Driver, InterruptHandler};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irq {
    OTG_FS => InterruptHandler<peripherals::USBFS>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    hal::debug::SDIPrint::enable();

    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);

    ch32_hal::println!("USBFS CDC example starting...");
    ch32_hal::println!("Using PB6 (DM) and PB7 (DP)");

    // Create LED for heartbeat on PA8
    let mut led = Output::new(p.PA8, Level::Low, Default::default());
    ch32_hal::println!("PA8 LED initialized for heartbeat");

    // Endpoint buffers - need 4 endpoints: EP0 (control), EP1 (CDC notify), EP2 (CDC data)
    let mut ep_buffer: [EndpointDataBuffer64; 4] = core::array::from_fn(|_| EndpointDataBuffer64::default());
    ch32_hal::println!("EP buffers allocated");

    // Create the driver from the HAL
    // USBFS uses PB7 (DP) and PB6 (DM)
    let driver = Driver::new(p.USBFS, p.PB7, p.PB6, &mut ep_buffer);
    ch32_hal::println!("USBFS driver created");

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("WCH");
    config.product = Some("CH32V208 USBFS CDC");
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
    ch32_hal::println!("CDC ACM class created");

    // Build the builder
    let mut usb = builder.build();
    ch32_hal::println!("USB device built, starting...");

    // Run the USB device
    let usb_fut = usb.run();

    // Echo task
    let echo_fut = async {
        ch32_hal::println!("Echo task started, waiting for connection...");
        loop {
            class.wait_connection().await;
            ch32_hal::println!("USBFS connected!");
            let _ = echo(&mut class).await;
            ch32_hal::println!("USBFS disconnected, waiting for reconnection...");
        }
    };

    // Blinky task on PA8 as heartbeat indicator
    let blinky_fut = async {
        loop {
            led.toggle();
            Timer::after(Duration::from_millis(500)).await;
        }
    };

    // Run everything concurrently
    ch32_hal::println!("Running USB, echo, and blinky tasks...");
    join3(usb_fut, echo_fut, blinky_fut).await;
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

async fn echo<'d, const NR_EP: usize, const SIZE: usize>(
    class: &mut CdcAcmClass<'d, Driver<'d, peripherals::USBFS, NR_EP, SIZE>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    ch32_hal::println!("Echo loop started");
    loop {
        let n = class.read_packet(&mut buf).await?;
        ch32_hal::println!("Received {} bytes", n);
        let data = &buf[..n];
        class.write_packet(data).await?;
        ch32_hal::println!("Echoed {} bytes", n);
    }
}

