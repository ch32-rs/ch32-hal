#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::println;
use ch32_hal::usbpd::{Error as UsbpdError, InterruptHandler, Sop, UsbPdPhy};
use ch32_hal::{bind_interrupts, peripherals};
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;
use usbpd::protocol_layer::message::data::request::{self, CurrentRequest, VoltageRequest};
use usbpd::protocol_layer::message::data::source_capabilities::SourceCapabilities;
use usbpd::sink::device_policy_manager::{DevicePolicyManager, Event};
use usbpd::sink::policy_engine::Sink;
use usbpd::timers::Timer as SinkTimer;
use usbpd_traits::Driver as SinkDriver;

bind_interrupts!(
    struct Irq {
        USBPD => InterruptHandler<peripherals::USBPD>;
    }
);

fn print_source_capability(pdo: &usbpd::protocol_layer::message::data::source_capabilities::PowerDataObject) {
    match pdo {
        usbpd::protocol_layer::message::data::source_capabilities::PowerDataObject::FixedSupply(fixed) => {
            println!(
                "PDO: Fixed - {} mV, {} mA",
                fixed.voltage().value,
                fixed.max_current().value
            );
        }
        usbpd::protocol_layer::message::data::source_capabilities::PowerDataObject::VariableSupply(variable) => {
            println!(
                "PDO: Variable - {}-{} mV, {} mA",
                variable.min_voltage().value,
                variable.max_voltage().value,
                variable.max_current().value
            );
        }
        usbpd::protocol_layer::message::data::source_capabilities::PowerDataObject::Augmented(augmented) => {
            match augmented {
                usbpd::protocol_layer::message::data::source_capabilities::Augmented::Spr(pps) => {
                    println!(
                        "PDO: PPS - {}-{} mV, {} mA",
                        pps.min_voltage().value,
                        pps.max_voltage().value,
                        pps.max_current().value
                    );
                }
                _ => println!("PDO: Augmented - Other"),
            }
        }
        _ => println!("PDO: Other"),
    }
}

struct EmbassySinkTimer {}

impl SinkTimer for EmbassySinkTimer {
    async fn after_millis(milliseconds: u64) {
        Timer::after_millis(milliseconds).await
    }
}

struct UsbpdSinkDriver<'d> {
    usbpd: UsbPdPhy<'d, peripherals::USBPD, hal::mode::Async>,
}

impl<'d> UsbpdSinkDriver<'d> {
    fn new(usbpd: UsbPdPhy<'d, peripherals::USBPD, hal::mode::Async>) -> Self {
        Self { usbpd }
    }
}

impl SinkDriver for UsbpdSinkDriver<'_> {
    async fn wait_for_vbus(&mut self) {
        // The sink policy engine is only running when attached. Therefore VBus is present.
    }

    async fn receive(&mut self, buffer: &mut [u8]) -> Result<usize, usbpd_traits::DriverRxError> {
        match self.usbpd.receive(buffer).await {
            Ok((Sop::Sop, size)) => Ok(size),
            Ok(_) => Err(usbpd_traits::DriverRxError::Discarded),
            Err(e) => match e {
                UsbpdError::HardReset => Err(usbpd_traits::DriverRxError::HardReset),
                UsbpdError::Rejected => Err(usbpd_traits::DriverRxError::Discarded),
                _ => Err(usbpd_traits::DriverRxError::Discarded),
            },
        }
    }

    async fn transmit(&mut self, data: &[u8]) -> Result<(), usbpd_traits::DriverTxError> {
        self.usbpd.transmit(data).await;
        Ok(())
    }

    async fn transmit_hard_reset(&mut self) -> Result<(), usbpd_traits::DriverTxError> {
        self.usbpd.transmit_hardreset().await;
        Ok(())
    }
}

struct Device {}

impl DevicePolicyManager for Device {
    async fn request(&mut self, source_capabilities: &SourceCapabilities) -> request::PowerSource {
        request::PowerSource::new_fixed(
            request::CurrentRequest::Highest,
            request::VoltageRequest::Safe5V,
            source_capabilities,
        )
        .unwrap()
    }

    async fn get_event(&mut self, source_capabilities: &SourceCapabilities) -> Event {
        println!("DevicePolicyManager: Waiting for events...");
        for (_, pdo) in source_capabilities.pdos().iter().enumerate() {
            print_source_capability(pdo);
        }
        loop {
            Timer::after_millis(1000).await;
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    hal::debug::SDIPrint::enable();

    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSE;
    let p = hal::init(config);

    let mut phy = UsbPdPhy::new_async(p.USBPD, p.PB6, p.PB7, Irq);
    phy.detect_cc().unwrap();

    let device = Device {};
    let driver = UsbpdSinkDriver::new(phy);
    let mut sink: Sink<_, EmbassySinkTimer, _> = Sink::new(driver, device);
    println!("Starting USB-PD sink policy engine...");
    let _ = sink.run().await;
    println!("Sink policy engine exited unexpectedly");
    loop {
        Timer::after_millis(1000).await;
    }
}
