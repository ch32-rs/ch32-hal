#![no_std]
#![no_main]

use panic_halt as _;
use ch32_hal::usb::EndpointDataBuffer512;
use ch32_hal::otg_fs::{self, Driver};
use ch32_hal::{self as hal, bind_interrupts, peripherals, Config};
use embassy_executor::Spawner;
use embassy_usb::control::{InResponse, OutResponse, Recipient, RequestType};
use embassy_usb::{Builder, Handler};
use usb_dfu_target::consts::{DfuAttributes, DfuRequest};
use usb_dfu_target::{DfuHandler, UsbDfuDevice};

bind_interrupts!(struct Irq {
    OTG_FS => otg_fs::InterruptHandler<peripherals::OTG_FS>;
});

const BLOCK_SIZE: usize = 64;

struct DfuDemoDevice;

impl DfuHandler for DfuDemoDevice {
    fn write_data(&mut self, offset: usize, data: &[u8]) {
        // Nothing
    }

    fn complete_download(&mut self) {
        // TODO: nothing
    }

    fn upload(&self, buffer: &mut [u8], offset: usize) -> usize {
        todo!()
    }

    fn is_write_complete(&self) -> bool {
        true
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    // setup clocks
    let cfg = Config {
        rcc: ch32_hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI,
        ..Default::default()
    };
    let p = hal::init(cfg);

    /* USB DRIVER SECION */
    let mut buffer: [EndpointDataBuffer512; 1] = core::array::from_fn(|_| EndpointDataBuffer512::default());
    let driver = Driver::new(p.OTG_FS, p.PA12, p.PA11, &mut buffer);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0x6666, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB DFU Demo");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0x00;
    config.device_sub_class = 0x00;
    config.device_protocol = 0x00;
    config.composite_with_iads = false;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    // You can also add a Microsoft OS descriptor.
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut dfu_device_handler = DfuDemoDevice;
    let mut request_handler = DfuRequestHandler {
        inner: UsbDfuDevice::new(&mut dfu_device_handler),
    };

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    let mut func = builder.function(0x00, 0x00, 0x00);
    let mut iface = func.interface();
    let mut alt = {
        use usb_dfu_target::consts::*;
        iface.alt_setting(USB_CLASS_APPN_SPEC, APPN_SPEC_SUBCLASS_DFU, DFU_PROTOCOL_DFU, None)
    };
    let mut attr = DfuAttributes::empty();
    attr.set(DfuAttributes::CAN_DOWNLOAD, true);
    alt.descriptor(
        usb_dfu_target::consts::DESC_DFU_FUNCTIONAL,
        &[
            attr.bits(),
            0xc4,
            0x09, // 2500ms timeout, doesn't affect operation as DETACH not necessary in bootloader code
            (BLOCK_SIZE & 0xff) as u8,
            ((BLOCK_SIZE & 0xff00) >> 8) as u8,
            0x10,
            0x01, // DFU 1.1
        ],
    );

    drop(func);
    builder.handler(&mut request_handler);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    usb_fut.await;
    /* END USB DRIVER */
}

struct DfuRequestHandler<'h> {
    inner: UsbDfuDevice<'h>,
}

impl<'h> Handler for DfuRequestHandler<'h> {
    fn control_out(&mut self, req: embassy_usb::control::Request, buf: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }

        match DfuRequest::try_from(req.request) {
            Ok(req) => match self.inner.handle_control_out(req, buf) {
                Ok(_) => Some(OutResponse::Accepted),
                Err(_) => Some(OutResponse::Rejected),
            },
            Err(_) => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if (req.request_type, req.recipient) != (RequestType::Class, Recipient::Interface) {
            return None;
        }
        match DfuRequest::try_from(req.request) {
            Ok(req) => match self.inner.handle_control_in(req, buf) {
                Ok(buf) => Some(InResponse::Accepted(buf)),
                Err(_) => Some(InResponse::Rejected),
            },
            Err(_) => Some(InResponse::Rejected),
        }
    }
}
