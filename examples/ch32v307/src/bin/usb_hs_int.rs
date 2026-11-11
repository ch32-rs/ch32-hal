//! USBHS interrupt endpoint echo (vendor class).
//!
//! One interface with an interrupt OUT endpoint (0x01) and an interrupt IN endpoint
//! (0x81), 64-byte packets, interval 1. The task reads one OUT packet and writes it back
//! to the IN endpoint. A host tool can send 64-byte, short and zero-length packets and
//! compare the echo, which exercises the interrupt transfer path (arm/ready, NAK flow
//! control, toggle handling) without a class driver.
//!
//! Build with `cargo run --release --bin usb_hs_int` and connect the board's USBHS port
//! (PB6/PB7). The libusb tool used for the checks is kept outside the repository.

#![no_std]
#![no_main]

use ch32_hal::usbhs::{Driver, InterruptHandler, WakeupInterruptHandler};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_usb::driver::{Direction, EndpointAddress, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::Builder;
use panic_halt as _;

bind_interrupts!(struct Irq {
    USBHS => InterruptHandler<peripherals::USBHS>;
    USBHS_WKUP => WakeupInterruptHandler<peripherals::USBHS>;
});

const EP_SIZE: u16 = 64;

/// One buffer for EP0, one for the interrupt OUT and one for the interrupt IN endpoint.
#[repr(align(4))]
struct EndpointMemory([u8; 192]);

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE,
        ..Default::default()
    };
    let p = hal::init(cfg);

    let mut memory = EndpointMemory([0; 192]);
    let driver = Driver::new(p.USBHS, p.PB7, p.PB6, Irq, &mut memory.0, Default::default()).unwrap();

    let mut config = embassy_usb::Config::new(0xc0de, 0xca53);
    config.manufacturer = Some("ch32-hal");
    config.product = Some("USBHS interrupt echo");
    config.serial_number = Some("int-1");
    config.max_power = 100;
    config.max_packet_size_0 = 64;
    // A single vendor function: no IAD and no composite device class.
    config.composite_with_iads = false;
    config.device_class = 0;
    config.device_sub_class = 0;
    config.device_protocol = 0;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 64];
    let mut msos_descriptor = [0; 64];
    let mut control_buf = [0; 64];

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let (ep_out, ep_in) = {
        let mut func = builder.function(0xff, 0x00, 0x00);
        let mut iface = func.interface();
        let mut alt = iface.alt_setting(0xff, 0x00, 0x00, None);
        let ep_out = alt.endpoint_interrupt_out(
            Some(EndpointAddress::from_parts(1, Direction::Out)),
            EP_SIZE,
            1,
        );
        let ep_in = alt.endpoint_interrupt_in(
            Some(EndpointAddress::from_parts(1, Direction::In)),
            EP_SIZE,
            1,
        );
        (ep_out, ep_in)
    };

    let mut usb = builder.build();
    join(usb.run(), echo(ep_out, ep_in)).await;
}

/// Read one packet from the OUT endpoint and send it back on the IN endpoint.
async fn echo<O: EndpointOut, I: EndpointIn>(mut out: O, mut in_: I) {
    let mut buf = [0u8; EP_SIZE as usize];
    loop {
        out.wait_enabled().await;
        match out.read(&mut buf).await {
            Ok(n) => {
                let _ = in_.write(&buf[..n]).await;
            }
            // disabled by a bus reset or a configuration change: wait for the host
            Err(EndpointError::Disabled) => {}
            Err(_) => {}
        }
    }
}
