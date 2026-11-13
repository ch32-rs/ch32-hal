//! USBHS full-speed CDC-ACM echo on the V3F core.
//!
//! The H4 USBHS block keeps the V4x7 device layout, so this exercises the
//! `ch32_hal::usbhs` driver that the V4x7 parts already use. Nothing configures
//! PB8/PB9: once the USBHS block is clocked the muxed pins connect to the
//! internal 480 MHz PHY by themselves (RM table 9-11). The USBHS PLL runs from
//! HSI, the default `hal::init` clock tree, so no crystal is needed.
//!
//! ```text
//! cargo xtask run --example usbhs_cdc
//! ```
//!
//! The host sees a `ch32-hal CH32H4 USBHS CDC echo` serial port, and every
//! packet is echoed back.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::rcc::{UsbhsPll, UsbhspllRefsel, Usbhspllsrc};
use ch32_hal::usbhs::{Driver, InterruptHandler};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::device::{CdcAcmClass, State};
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBHS => InterruptHandler<peripherals::USBHS>;
});

type UsbDriver = Driver<'static, peripherals::USBHS>;

/// Endpoint memory: 64 bytes for EP0 plus one 64-byte packet per class
/// endpoint. The driver only requires 4-byte alignment and RAM residence.
#[repr(align(4))]
struct EndpointMemory([u8; 256]);

// The USB device and its echo task run concurrently, so everything they share
// has to outlive `main`.
static mut ENDPOINT_MEMORY: EndpointMemory = EndpointMemory([0; 256]);
static mut CONFIG_DESCRIPTOR: [u8; 128] = [0; 128];
static mut BOS_DESCRIPTOR: [u8; 32] = [0; 32];
static mut MSOS_DESCRIPTOR: [u8; 32] = [0; 32];
static mut CONTROL_BUFFER: [u8; 64] = [0; 64];
static mut CDC_STATE: State = State::new();

#[embassy_executor::task]
async fn echo(mut class: CdcAcmClass<'static, UsbDriver>) {
    let mut packet = [0; 64];
    loop {
        class.wait_connection().await;
        info!("USBHS CDC connected");
        loop {
            match class.read_packet(&mut packet).await {
                Ok(length) => {
                    if class.write_packet(&packet[..length]).await.is_err() {
                        break;
                    }
                    // A full packet must be followed by a short one, or the
                    // host keeps waiting for the rest of the transfer.
                    if length == packet.len() && class.write_packet(&[]).await.is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        info!("USBHS CDC disconnected");
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc.aux_plls.usbhs = Some(UsbhsPll {
        src: Usbhspllsrc::HSI,
        refsel: UsbhspllRefsel::F25MHZ,
    });
    let p = hal::init(config);
    info!(
        "sysclk={} usbhs_pll_ready={}",
        rcc::clocks().sysclk.0,
        hal::pac::RCC.ctlr().read().usbhs_pllrdy()
    );

    let memory = unsafe { &mut *core::ptr::addr_of_mut!(ENDPOINT_MEMORY) };
    let driver = Driver::new(p.USBHS, p.PB8, p.PB9, Irqs, &mut memory.0, Default::default()).unwrap();

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("ch32-hal");
    usb_config.product = Some("CH32H4 USBHS CDC echo");
    usb_config.serial_number = Some("H4-USBHS");
    usb_config.max_power = 100;
    let mut builder = Builder::new(
        driver,
        usb_config,
        unsafe { &mut *core::ptr::addr_of_mut!(CONFIG_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(BOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(MSOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(CONTROL_BUFFER) },
    );
    let class = CdcAcmClass::new(&mut builder, unsafe { &mut *core::ptr::addr_of_mut!(CDC_STATE) }, 64);
    let mut usb = builder.build();
    info!("USBHS CDC ready");

    spawner.spawn(echo(class).unwrap());
    usb.run().await
}
