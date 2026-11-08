//! USBFS CDC-ACM serial for CH32L103.
//!
//! Pins:
//! - DP = PA12
//! - DM = PA11
//!
//! Echoes every received packet back to the host. Clock is HSE 8 MHz ->
//! PLL 48 MHz so USBPRE can feed a clean 48 MHz USB clock.
//!
//! ```text
//! cd examples/ch32l103 && cargo run --release --bin usbfs_serial
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use hal::usb::EndpointDataBuffer64;
use hal::usbfs::{Driver, Instance};
use hal::{bind_interrupts, peripherals, Config};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBFS => hal::usbfs::InterruptHandler<peripherals::USBFS>;
});

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let p = hal::init(Config {
        rcc: hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSE,
        ..Default::default()
    });

    defmt::info!("USBFS CDC @ 48 MHz (DP=PA12, DM=PA11)");
    defmt::info!("clocks: {:?}", defmt::Debug2Format(&hal::rcc::clocks()));

    let mut ep_buffer: [EndpointDataBuffer64; 4] =
        core::array::from_fn(|_| EndpointDataBuffer64::default());
    let driver = Driver::new(p.USBFS, Irqs, p.PA12, p.PA11, &mut ep_buffer);

    let mut config = embassy_usb::Config::new(0xC0DE, 0xCAFE);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("CH32L103 USBFS CDC");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Windows CDC-ACM compatibility.
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
            defmt::info!("CDC connected");
            let _ = echo(&mut class).await;
            defmt::info!("CDC disconnected");
        }
    };

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

async fn echo<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T, 4, 64>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        defmt::info!("rx {} bytes", n);
        class.write_packet(data).await?;
    }
}
