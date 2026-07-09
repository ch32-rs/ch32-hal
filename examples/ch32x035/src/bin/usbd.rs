#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use hal::usb::EndpointDataBuffer512;
use hal::usbfs::{Driver, Instance};
use hal::{bind_interrupts, peripherals};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBFS => hal::usbfs::InterruptHandler<peripherals::USBFS>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI,
        ..Default::default()
    });

    let mut ep_buffers: [EndpointDataBuffer512; 4] = core::array::from_fn(|_| EndpointDataBuffer512::default());
    let driver = Driver::new(p.USBFS, p.PC17, p.PC16, &mut ep_buffers);

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("ch32-rs");
    config.product = Some("CH32X035 USB serial");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    config.device_class = 0x02;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

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

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let usb_fut = usb.run();
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            let _ = echo(&mut class).await;
        }
    };

    join(usb_fut, echo_fut).await;
}

struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected,
        }
    }
}

async fn echo<'d, T: Instance + 'd, const NR_EP: usize, const SIZE: usize>(
    class: &mut CdcAcmClass<'d, Driver<'d, T, NR_EP, SIZE>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        class.write_packet(&buf[..n]).await?;
    }
}
