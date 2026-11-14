//! USBSS SuperSpeed bulk benchmark with 16-packet bursts.
//!
//! ```text
//! cargo xtask run --example usbss_bench
//! ```
//!
//! Bulk OUT data is discarded. Bulk IN sends the same 64 KiB buffer forever;
//! every 1024-byte packet in it holds the bytes 0..=255 four times.
//!
//! The vendor request 1 (OUT, no data) stops the IN source with wValue 0 and
//! restarts it with wValue 1. Stop it and clear the IN endpoint's halt to
//! measure OUT with no IN transfer queued.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use ch32_hal as hal;
use ch32_hal::rcc::{UsbssPll, UsbsspllRefsel};
use ch32_hal::usbss::{Driver, InterruptHandler, LinkInterruptHandler};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use embassy_usb::control::{OutResponse, Request, RequestType};
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::{Builder, EndpointOptions, Handler, UsbDeviceSpeed, UsbVersion};
use panic_halt as _;
use static_cell::ConstStaticCell;

bind_interrupts!(struct Irqs {
    USBSS => InterruptHandler<peripherals::USBSS>;
    USBSS_LINK => LinkInterruptHandler<peripherals::USBSS>;
});

type UsbDriver = Driver<'static, peripherals::USBSS>;
type BulkIn = <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointIn;
type BulkOut = <UsbDriver as embassy_usb::driver::Driver<'static>>::EndpointOut;

const PACKET: usize = 1024;
const MAX_BURST: u8 = 15;
const BURST: usize = PACKET * (MAX_BURST as usize + 1);
const TRANSFER: usize = 4 * BURST;

#[repr(align(4))]
struct Ep0Memory([u8; 512]);

static EP0_MEMORY: ConstStaticCell<Ep0Memory> = ConstStaticCell::new(Ep0Memory([0; 512]));
static SINK: ConstStaticCell<[u8; TRANSFER]> = ConstStaticCell::new([0; TRANSFER]);
static SOURCE: ConstStaticCell<[u8; TRANSFER]> = ConstStaticCell::new([0; TRANSFER]);
static CONFIG_DESCRIPTOR: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0; 64]);
static BOS_DESCRIPTOR: ConstStaticCell<[u8; 32]> = ConstStaticCell::new([0; 32]);
static CONTROL_BUFFER: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0; 64]);

static SOURCE_ON: AtomicBool = AtomicBool::new(true);

struct SourceSwitch;

impl Handler for SourceSwitch {
    fn control_out(&mut self, request: Request, _data: &[u8]) -> Option<OutResponse> {
        if request.request_type != RequestType::Vendor || request.request != 1 {
            return None;
        }
        SOURCE_ON.store(request.value != 0, Ordering::Relaxed);
        Some(OutResponse::Accepted)
    }
}

static SOURCE_SWITCH: ConstStaticCell<SourceSwitch> = ConstStaticCell::new(SourceSwitch);

#[embassy_executor::task]
async fn sink(mut ep: BulkOut) {
    let buffer = SINK.take();
    loop {
        ep.wait_enabled().await;
        while ep.read_transfer(buffer).await.is_ok() {}
    }
}

#[embassy_executor::task]
async fn source(mut ep: BulkIn) {
    let buffer = SOURCE.take();
    for (i, byte) in buffer.iter_mut().enumerate() {
        *byte = i as u8;
    }
    loop {
        ep.wait_enabled().await;
        info!("USBSS bench configured");
        loop {
            while !SOURCE_ON.load(Ordering::Relaxed) {
                Timer::after_millis(1).await;
            }
            if ep.write_transfer(buffer, false).await.is_err() {
                break;
            }
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse();
    config.rcc.aux_plls.usbss = Some(UsbssPll {
        refsel: UsbsspllRefsel::F25MHZ,
    });
    let p = hal::init(config);

    let driver = Driver::new(p.USBSS, Irqs, &mut EP0_MEMORY.take().0).unwrap();

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcaff);
    usb_config.manufacturer = Some("ch32-hal");
    usb_config.product = Some("CH32H4 USBSS bench");
    usb_config.serial_number = Some("H4-USBSS");
    usb_config.max_power = 100;
    usb_config.bcd_usb = UsbVersion::Three;
    usb_config.max_speed = UsbDeviceSpeed::Super;
    usb_config.max_packet_size_0 = 9;
    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.take(),
        BOS_DESCRIPTOR.take(),
        &mut [],
        CONTROL_BUFFER.take(),
    );
    builder.handler(SOURCE_SWITCH.take());
    let mut function = builder.function(0xff, 0, 0);
    let mut interface = function.interface();
    let mut alt = interface.alt_setting(0xff, 0, 0, None);
    let mut options = EndpointOptions::new();
    options.max_burst = MAX_BURST;
    let out_ep = alt.endpoint_bulk_out(None, options);
    let in_ep = alt.endpoint_bulk_in(None, options);
    drop(function);
    let mut usb = builder.build();
    info!("USBSS bench ready");

    spawner.spawn(sink(out_ep).unwrap());
    spawner.spawn(source(in_ep).unwrap());
    usb.run().await
}
