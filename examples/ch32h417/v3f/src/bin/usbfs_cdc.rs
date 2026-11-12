//! USBFS full-speed CDC-ACM echo on the V3F core.
//!
//! H4's USBFS (`kind: usbfs`) is the OTG-v2 device block at `0x40023400`, and
//! this exercises the `ch32_hal::usbfs` driver it shares with the OTG-FS parts.
//! The port is the board's USB-FS connector (J3 USB-C, PA11/PA12). Nothing
//! configures those pins: once the block is clocked the controller drives DP/DM
//! itself (RM table 9-12).
//!
//! The FS kernel clock has to come from the USBHS PLL /10 (RM §3.4.13), so the
//! USBHS PLL is brought up from HSI even though the USBHS controller stays off.
//!
//! ```text
//! cargo xtask run --example usbfs_cdc
//! ```
//!
//! The host sees a `ch32-hal CH32H4 USBFS CDC echo` serial port, and every
//! packet is echoed back.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::rcc::{UsbhsPll, UsbhspllRefsel, Usbhspllsrc};
use ch32_hal::usb::EndpointDataBuffer64;
use ch32_hal::usbfs::{Driver, InterruptHandler};
use ch32_hal::{bind_interrupts, pac, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::device::{CdcAcmClass, Config as CdcAcmConfig, State};
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBFS => InterruptHandler<peripherals::USBFS>;
});

type UsbDriver = Driver<'static, peripherals::USBFS, 4, 64>;

/// Endpoint memory: one 64-byte packet for EP0 plus one for each endpoint the
/// CDC class allocates (data IN, data OUT, and its notification endpoint). The
/// driver only requires 4-byte alignment and RAM residence; the cast below
/// relies on `EndpointDataBuffer64` being `repr(C, align(4))`.
#[repr(align(4))]
struct EndpointMemory([u8; 4 * 64]);

static mut ENDPOINT_MEMORY: EndpointMemory = EndpointMemory([0; 4 * 64]);
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
        info!("USBFS CDC connected");
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
        info!("USBFS CDC disconnected");
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc.aux_plls.usbhs = Some(UsbhsPll {
        src: Usbhspllsrc::HSI,
        refsel: UsbhspllRefsel::F25MHZ,
    });
    config.rcc.mux.usbfssrc = pac::rcc::vals::Usbfssrc::USBHS_PLL;
    let p = hal::init(config);
    info!(
        "sysclk={} usbhs_pll_ready={}",
        rcc::clocks().sysclk.0,
        pac::RCC.ctlr().read().usbhs_pllrdy()
    );

    let memory: &mut [EndpointDataBuffer64; 4] = unsafe { &mut *core::ptr::addr_of_mut!(ENDPOINT_MEMORY.0).cast() };
    let driver = Driver::new(p.USBFS, p.PA12, p.PA11, memory);

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("ch32-hal");
    usb_config.product = Some("CH32H4 USBFS CDC echo");
    usb_config.serial_number = Some("H4-USBFS");
    usb_config.max_power = 100;
    let mut builder = Builder::new(
        driver,
        usb_config,
        unsafe { &mut *core::ptr::addr_of_mut!(CONFIG_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(BOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(MSOS_DESCRIPTOR) },
        unsafe { &mut *core::ptr::addr_of_mut!(CONTROL_BUFFER) },
    );
    let class = CdcAcmClass::new(
        &mut builder,
        unsafe { &mut *core::ptr::addr_of_mut!(CDC_STATE) },
        CdcAcmConfig::new(),
    );
    let mut usb = builder.build();
    info!("USBFS CDC ready");

    spawner.spawn(echo(class).unwrap());
    usb.run().await
}
