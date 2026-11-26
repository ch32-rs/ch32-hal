//! USBSS host: enumerates the device on the SS Type-A port and, for a
//! mass-storage device, reads its capacity and times 16 MiB of reads at each
//! READ size in `CHUNKS`.
//!
//! ```text
//! cargo xtask run --example usbss_host_msc
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::rcc::{UsbssPll, UsbsspllRefsel};
use ch32_hal::usbss::host::HostDriver;
use ch32_hal::usbss::{InterruptHandler, LinkInterruptHandler};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Instant;
use embassy_usb::class::msc::host::{find_msc, MscDevice, MscError};
use embassy_usb::driver::host::{DeviceEvent, UsbHostAllocator};
use embassy_usb::host::handler::EnumerationInfo;
use embassy_usb::host::{BusRoute, BusState};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBSS => InterruptHandler<peripherals::USBSS>;
    USBSS_LINK => LinkInterruptHandler<peripherals::USBSS>;
});

const CHUNKS: [usize; 2] = [4 << 10, 16 << 10];
const TOTAL: usize = 16 << 20;

#[repr(align(4))]
struct Bounce([u8; 1024]);

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse();
    config.rcc.aux_plls.usbss = Some(UsbssPll {
        refsel: UsbsspllRefsel::F25MHZ,
    });
    let p = hal::init(config);

    let mut bounce = Bounce([0; 1024]);
    let mut data = [0; 16 << 10];
    let driver = HostDriver::new(p.USBSS, Irqs, &mut bounce.0).unwrap();

    static BUS: BusState = BusState::new();
    let (mut controller, bus) = embassy_usb::host::bus(driver, &BUS);
    let mut config_descriptor = [0; 512];

    loop {
        let status = controller.controller().status();
        info!(
            "waiting for a device, link state {} term={}",
            status.state.to_bits(),
            status.term_present
        );
        let speed = controller.wait_for_connection().await;

        match bus.enumerate(BusRoute::Direct(speed), &mut config_descriptor).await {
            Ok((device, length)) => {
                let descriptor = &device.device_desc;
                info!(
                    "device {:04x}:{:04x} at address {}",
                    descriptor.vendor_id, descriptor.product_id, device.device_address
                );
                let config_descriptor = &config_descriptor[..length];
                if find_msc(config_descriptor).is_some() {
                    if let Err(e) = read_disk(&bus, &device, config_descriptor, &mut data).await {
                        warn!("mass storage: {}", e);
                    }
                } else {
                    info!("not a mass-storage device");
                }
            }
            Err(e) => warn!("enumeration failed: {}", e),
        }

        while controller.wait_for_device_event().await != DeviceEvent::Disconnected {}
        info!("device disconnected");
    }
}

async fn read_disk<'d, A: UsbHostAllocator<'d>>(
    alloc: &A,
    device: &EnumerationInfo,
    config_descriptor: &[u8],
    data: &mut [u8],
) -> Result<(), MscError> {
    let msc = MscDevice::new(alloc, device, config_descriptor).await?;
    let mut lun = msc.lun(0)?;
    let capacity = lun.capacity().await?;
    info!("{} blocks of {} bytes", capacity.block_count, capacity.block_size);

    let block_size = capacity.block_size as usize;
    for (pass, chunk) in CHUNKS.into_iter().enumerate() {
        let buf = &mut data[..chunk];
        let mut lba = (pass * TOTAL / block_size) as u64;
        let start = Instant::now();
        for _ in 0..TOTAL / chunk {
            lun.read_blocks(lba, buf).await?;
            lba += (chunk / block_size) as u64;
        }
        let micros = start.elapsed().as_micros().max(1);
        info!(
            "{} KiB READs: {} MiB in {} ms, {} kB/s",
            chunk >> 10,
            TOTAL >> 20,
            micros / 1000,
            TOTAL as u64 * 1000 / micros
        );
    }
    Ok(())
}
