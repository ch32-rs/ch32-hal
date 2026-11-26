//! USBSS SuperSpeed CDC-ACM echo on the V3F core.
//!
//! ```text
//! cargo xtask run --example usbss_cdc
//! ```
//!
//! The host sees a 5 Gbit/s `ch32-hal CH32H4 USBSS CDC echo` serial port, and
//! every packet is echoed back.

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::rcc::{UsbssPll, UsbsspllRefsel};
use ch32_hal::usbss::{Driver, InterruptHandler, LinkInterruptHandler};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_usb::class::cdc_acm::device::{CdcAcmClass, Config as CdcAcmConfig, State};
use embassy_usb::{Builder, UsbDeviceSpeed, UsbVersion};
use panic_halt as _;
use static_cell::{ConstStaticCell, StaticCell};

bind_interrupts!(struct Irqs {
    USBSS => InterruptHandler<peripherals::USBSS>;
    USBSS_LINK => LinkInterruptHandler<peripherals::USBSS>;
});

type UsbDriver = Driver<'static, peripherals::USBSS>;

const PACKET: usize = 1024;

#[repr(align(4))]
struct Ep0Memory([u8; 512]);

static EP0_MEMORY: ConstStaticCell<Ep0Memory> = ConstStaticCell::new(Ep0Memory([0; 512]));
static CONFIG_DESCRIPTOR: ConstStaticCell<[u8; 128]> = ConstStaticCell::new([0; 128]);
static BOS_DESCRIPTOR: ConstStaticCell<[u8; 32]> = ConstStaticCell::new([0; 32]);
static CONTROL_BUFFER: ConstStaticCell<[u8; 64]> = ConstStaticCell::new([0; 64]);
static CDC_STATE: StaticCell<State> = StaticCell::new();

#[embassy_executor::task]
async fn echo(mut class: CdcAcmClass<'static, UsbDriver>) {
    let mut packet = [0; PACKET];
    loop {
        class.wait_connection().await;
        info!("USBSS CDC connected");
        loop {
            match class.read_packet(&mut packet).await {
                Ok(length) => {
                    if class.write_packet(&packet[..length]).await.is_err() {
                        break;
                    }
                    if length == packet.len() && class.write_packet(&[]).await.is_err() {
                        break;
                    }
                }
                Err(_) => break,
            }
        }
        info!("USBSS CDC disconnected");
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
    info!(
        "sysclk={} chip={:x}",
        rcc::clocks().sysclk.0,
        hal::signature::chip_id().dev_id()
    );

    let driver = Driver::new(p.USBSS, Irqs, &mut EP0_MEMORY.take().0).unwrap();

    let mut usb_config = embassy_usb::Config::new(0xc0de, 0xcafe);
    usb_config.manufacturer = Some("ch32-hal");
    usb_config.product = Some("CH32H4 USBSS CDC echo");
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
    let class = CdcAcmClass::new(&mut builder, CDC_STATE.init(State::new()), CdcAcmConfig::new());
    let mut usb = builder.build();
    info!("USBSS CDC ready");

    spawner.spawn(echo(class).unwrap());
    usb.run().await
}
