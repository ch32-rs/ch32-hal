#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};
use hal::eth::generic_smi::GenericSMI;
use hal::rcc::*;
use hal::time::Hertz;
use hal::{eth, interrupt, println};
use static_cell::StaticCell;
use {ch32_hal as hal, panic_halt as _};

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, eth::Device<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn ethernet_task(runner: eth::Runner<'static, GenericSMI>) -> ! {
    runner.run().await
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    // Configure core speed at 60MHz (16MHz HSE * 15 / 4 = 60MHz)
    let _p = ch32_hal::init(ch32_hal::Config {
        rcc: ch32_hal::rcc::Config {
            hse: Some(Hse {
                freq: Hertz(16_000_000),
                mode: HseMode::Oscillator,
            }),
            sys: Sysclk::PLL,
            pll_src: PllSource::HSE,
            pll: Some(Pll {
                prediv: PllPreDiv::DIV4,
                mul: PllMul::MUL15,
            }),
            pllx: None,
            ahb_pre: AHBPrescaler::DIV1,
            apb1_pre: APBPrescaler::DIV1,
            apb2_pre: APBPrescaler::DIV1,
            ls: LsConfig::default(),
            hspll_src: HsPllSource::HSI,
            hspll: None,
        },
        dma_interrupt_priority: interrupt::Priority::P0,
    });
    let core_freq: u32 = 60_000_000;

    ch32_hal::debug::SDIPrint::enable();
    Timer::after(Duration::from_millis(100)).await;
    println!("Hello CH32!");

    // Ethernet setup
    let mac_addr = eth::get_mac();
    println!("mac_addr: {mac_addr:?}");

    let phy = GenericSMI::new(0);
    static STATE: StaticCell<eth::State<2, 2>> = StaticCell::new();
    let state = STATE.init(eth::State::new());
    let (runner, device) = eth::new(mac_addr, core_freq, phy, state).await.unwrap();
    println!("device: done");

    // Generate random seed
    let seed = 0xdeadbeef_abadbabe;

    // Init network stack
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let config = embassy_net::Config::dhcpv4(Default::default());
    let (stack, net_runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(ethernet_task(runner).unwrap());
    spawner.spawn(net_task(net_runner).unwrap());

    println!("Waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        println!("IP address: {}", config.address);
    }

    // UDP echo server on port 1234
    let mut rx_buffer = [0; 300];
    let mut tx_buffer = [0; 300];
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut buf = [0; 300];
    loop {
        let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
        socket.bind(1234).unwrap();

        loop {
            let (n, ep) = socket.recv_from(&mut buf).await.unwrap();
            if let Ok(s) = core::str::from_utf8(&buf[..n]) {
                println!("rxd from {}: {}", ep, s);
            }
            socket.send_to(&buf[..n], ep).await.unwrap();
        }
    }
}
