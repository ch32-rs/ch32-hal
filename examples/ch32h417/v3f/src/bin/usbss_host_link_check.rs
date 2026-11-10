//! USBSS link check in host mode.
//!
//! Brings the SuperSpeed root port up in downstream mode and logs the LTSSM
//! state and the Port Configuration handshake. Plug a SuperSpeed device into
//! the board's SS Type-A port: the log shows `RXDET -> POLLING -> U0` and
//! `connected=true`.
//!
//! ```text
//! cargo xtask run --example usbss_host_link_check
//! ```

#![no_std]
#![no_main]

use core::future::Future;
use core::task::Poll;

use ch32_hal as hal;
use ch32_hal::rcc::{UsbssPll, UsbsspllRefsel};
use ch32_hal::usbss::host::HostDriver;
use ch32_hal::usbss::{LinkInterruptHandler, LinkState};
use ch32_hal::{bind_interrupts, peripherals, rcc};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USBSS_LINK => LinkInterruptHandler<peripherals::USBSS>;
});

fn name(state: LinkState) -> &'static str {
    match state {
        LinkState::U0 => "U0",
        LinkState::U1 => "U1",
        LinkState::U2 => "U2",
        LinkState::U3 => "U3",
        LinkState::DISABLED => "DISABLED",
        LinkState::RXDET => "RXDET",
        LinkState::INACTIVE => "INACTIVE",
        LinkState::POLLING => "POLLING",
        LinkState::RECOVERY => "RECOVERY",
        LinkState::HOTRST => "HOTRESET",
        LinkState::COMPLIANCE => "COMPLIANCE",
        LinkState::LOOPBACK => "LOOPBACK",
        _ => "UNKNOWN",
    }
}

enum Event {
    Link(LinkState),
    Connected,
    /// Periodic status sample; termination detection and the PHY mode change
    /// without a link-state interrupt.
    Tick,
}

async fn next_event(usb: &HostDriver<'_, peripherals::USBSS>) -> Event {
    // The driver futures only register a waker and re-check a flag. The timer
    // is created once per call, so a pending tick is not restarted on every
    // poll.
    let link = usb.wait_link_change();
    let connected = usb.wait_connected();
    let timer = Timer::after(Duration::from_millis(500));
    let mut link = core::pin::pin!(link);
    let mut connected = core::pin::pin!(connected);
    let mut timer = core::pin::pin!(timer);

    core::future::poll_fn(|cx| {
        if let Poll::Ready(state) = link.as_mut().poll(cx) {
            return Poll::Ready(Event::Link(state));
        }
        if let Poll::Ready(()) = connected.as_mut().poll(cx) {
            return Poll::Ready(Event::Connected);
        }
        if let Poll::Ready(()) = timer.as_mut().poll(cx) {
            return Poll::Ready(Event::Tick);
        }
        Poll::Pending
    })
    .await
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

    let usb = HostDriver::new(p.USBSS, Irqs);
    let status = usb.status();
    info!(
        "USBSS host initialized, link state {} term={} rxdet={} ready={} pd={} connected={}",
        name(status.state),
        status.term_present,
        status.rx_detect,
        status.ready,
        status.pd_mode as u8,
        usb.connected()
    );

    loop {
        match next_event(&usb).await {
            Event::Link(state) => info!("USBSS link -> {} connected={}", name(state), usb.connected()),
            Event::Connected => info!("SuperSpeed device connected, Port Configuration complete"),
            Event::Tick => {
                let status = usb.status();
                info!(
                    "status {} term={} rxdet={} ready={} busy={} pd={}",
                    name(status.state),
                    status.term_present,
                    status.rx_detect,
                    status.ready,
                    status.busy,
                    status.pd_mode as u8
                );
                // Re-arm SS.RX_DETECT while nothing is detected; the bit is
                // self-clearing and the EVT relies on it being implicit. Once
                // a receiver shows up, start the polling handshake ourselves.
                if status.state == LinkState::RXDET {
                    if status.term_present {
                        info!("receiver termination detected, starting polling");
                        usb.start_polling();
                    } else {
                        usb.start_rx_detect();
                    }
                }
            }
        }
    }
}
