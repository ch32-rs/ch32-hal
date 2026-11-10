//! USBSS SuperSpeed bulk benchmark with 16-packet bursts.
//!
//! ```text
//! cargo xtask run --example usbss_bench
//! ```
//!
//! Bulk OUT data is discarded. Bulk IN sends the same 128 KiB buffer forever;
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
/// Eight bursts per transfer; the controller DMAs straight from these buffers.
const TRANSFER: usize = 8 * BURST;

/// 512 bytes for EP0 and one burst for each bulk endpoint.
#[repr(align(4))]
struct EndpointMemory([u8; 512 + 2 * BURST]);

static mut ENDPOINT_MEMORY: EndpointMemory = EndpointMemory([0; 512 + 2 * BURST]);
#[repr(align(4))]
struct TransferBuffer([u8; TRANSFER]);

static mut SINK: TransferBuffer = TransferBuffer([0; TRANSFER]);
static mut SOURCE: TransferBuffer = TransferBuffer([0; TRANSFER]);
static mut CONFIG_DESCRIPTOR: [u8; 64] = [0; 64];
static mut BOS_DESCRIPTOR: [u8; 32] = [0; 32];
static mut MSOS_DESCRIPTOR: [u8; 32] = [0; 32];
static mut CONTROL_BUFFER: [u8; 64] = [0; 64];

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

static mut SOURCE_SWITCH: SourceSwitch = SourceSwitch;

#[embassy_executor::task]
async fn sink(mut ep: BulkOut) {
    let buffer = unsafe { &mut (*core::ptr::addr_of_mut!(SINK)).0 };
    loop {
        ep.wait_enabled().await;
        while ep.read_transfer(buffer).await.is_ok() {}
    }
}

#[embassy_executor::task]
async fn source(mut ep: BulkIn) {
    let buffer = unsafe { &mut (*core::ptr::addr_of_mut!(SOURCE)).0 };
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

    let memory = unsafe { &mut *core::ptr::addr_of_mut!(ENDPOINT_MEMORY) };
    let driver = Driver::new(p.USBSS, Irqs, &mut memory.0).unwrap();

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
        unsafe { &mut *core::ptr::addr_of_mut!(CONFIG_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(BOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(MSOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(CONTROL_BUFFER) },
    );
    builder.handler(unsafe { &mut *core::ptr::addr_of_mut!(SOURCE_SWITCH) });
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
