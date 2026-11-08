//! USBSS link-training check (device mode).
//!
//! This does not enumerate yet: it brings the SuperSpeed PHY and the link
//! state machine up, answers the Port Capability LMPs, and logs every LTSSM
//! state change. Connect the USB-C port to a USB3 host to see training run.
//!
//! ```text
//! cargo xtask run --example usbss_link_check
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::rcc::{UsbssPll, UsbsspllRefsel};
use ch32_hal::usbss::{Driver, InterruptHandler, LinkInterruptHandler, LinkState};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBSS => InterruptHandler<peripherals::USBSS>;
    USBSS_LINK => LinkInterruptHandler<peripherals::USBSS>;
});

fn name(state: LinkState) -> &'static str {
    match state {
        LinkState::U0 => "U0",
        LinkState::U1 => "U1",
        LinkState::U2 => "U2",
        LinkState::U3 => "U3",
        LinkState::Disabled => "DISABLED",
        LinkState::RxDetect => "RXDET",
        LinkState::Inactive => "INACTIVE",
        LinkState::Polling => "POLLING",
        LinkState::Recovery => "RECOVERY",
        LinkState::HotReset => "HOTRESET",
        LinkState::Compliance => "COMPLIANCE",
        LinkState::Loopback => "LOOPBACK",
        LinkState::Unknown(_) => "UNKNOWN",
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = hal::Config::default();
    config.rcc = rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse();
    config.rcc.aux_plls.usbss = Some(UsbssPll {
        refsel: UsbsspllRefsel::F25MHZ,
    });
    let p = hal::init(config);

    info!(
        "sysclk={} usbss_pll_ready={}",
        rcc::clocks().sysclk.0,
        hal::pac::RCC.ctlr().read().usbss_pllrdy()
    );

    let mut usb = Driver::new(p.USBSS, Irqs);
    info!("USBSS initialized, link state {}", name(usb.link_state()));

    loop {
        let state = usb.wait_link_change().await;
        info!("USBSS link -> {}", name(state));
    }
}
