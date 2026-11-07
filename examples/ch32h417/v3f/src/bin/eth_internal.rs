//! Integrated 10M/100M Ethernet on the V3F core.
//!
//! The nanoCH32H417 board wires only the internal PHY (J11, HR911105A), so
//! this is the path that runs on hardware today. The address is static
//! (`192.168.47.10/24`) because the board is usually plugged straight into a
//! host NIC rather than a DHCP network; adjust it for your setup.
//!
//! ```text
//! cargo xtask run --example eth_internal
//! ping 192.168.47.10
//! ```

#![no_std]
#![no_main]

use ch32_hal as hal;
use ch32_hal::bind_interrupts;
use ch32_hal::eth::{Ethernet, InternalPhy, InterruptHandler, Phy, Resources};
use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::{Config as NetConfig, Ipv4Address, Ipv4Cidr, Runner, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ETH => InterruptHandler;
});

static mut ETH_RESOURCES: Resources<4, 4> = Resources::new();
static mut NET_RESOURCES: StackResources<3> = StackResources::new();

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Ethernet<'static, InternalPhy>>) -> ! {
    runner.run().await
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    info!("eth_internal: start");
    let mut rcc = hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse();
    // The integrated PHY runs from the fixed 500 MHz ETH PLL (RM §31.4.3).
    rcc.aux_plls.eth = true;
    let p = hal::init(hal::Config {
        rcc,
        ..Default::default()
    });
    info!(
        "hclk={} eth_pll_ready={}",
        hal::rcc::clocks().hclk.0,
        hal::pac::RCC.ctlr().read().eth_pllrdy()
    );

    let ethernet = match Ethernet::new_internal(
        p.ETH,
        Irqs,
        unsafe { &mut *core::ptr::addr_of_mut!(ETH_RESOURCES) },
        [0x02, 0x00, 0x00, 0x00, 0x00, 0x17],
        InternalPhy::new(),
    ) {
        Ok(ethernet) => ethernet,
        Err(error) => {
            info!("eth init failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    info!(
        "phy_id={=u32:#010x} link={:?}",
        ethernet.phy().phy_id(),
        ethernet.link()
    );

    let (stack, runner) = embassy_net::new(
        ethernet,
        NetConfig::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(Ipv4Address::new(192, 168, 47, 10), 24),
            gateway: None,
            dns_servers: Default::default(),
        }),
        unsafe { &mut *core::ptr::addr_of_mut!(NET_RESOURCES) },
        0x4170_0001,
    );
    spawner.spawn(net_task(runner).unwrap());

    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!(
            "link={} ip={:?}",
            stack.is_link_up(),
            stack.config_v4().map(|config| config.address)
        );
    }
}
