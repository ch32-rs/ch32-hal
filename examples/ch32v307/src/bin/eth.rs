//! TCP echo server example for CH32V307 (built-in 10M Ethernet).
//!
//! Demonstrates initializing the EMAC with the built-in 10BASE-T PHY,
//! connecting to the network via DHCP, and running a TCP echo server
//! on port 1234.
//!
//! # Hardware
//!
//! The built-in 10M PHY uses dedicated analog TX+/TX-/RX+/RX- pins
//! through an Ethernet transformer — no GPIO pin configuration needed.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::StackResources;
use embassy_time::Timer;
use embedded_io_async::Write;
use hal::emac::{self, Ethernet, GenericPhy, PacketQueue, PhyInterface};
use hal::println;
use static_cell::StaticCell;
use {ch32_hal as hal, panic_halt as _};

#[hal::interrupt]
unsafe fn ETH() {
    emac::on_interrupt();
}

type Device = Ethernet<'static, 4, 4, GenericPhy>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSE;
    let _p = hal::init(config);

    Timer::after_millis(100).await;

    let mac_addr = [0x02, 0x00, 0x00, 0x12, 0x34, 0x56];

    static QUEUE: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let queue = QUEUE.init(PacketQueue::new());

    // Built-in 10M PHY is accessible via SMI at address 1
    let phy = GenericPhy::new(1);

    let device = Ethernet::new(queue, mac_addr, phy, PhyInterface::Internal10M, 144);

    println!(
        "MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]
    );

    let net_config = embassy_net::Config::dhcpv4(Default::default());
    let seed = 0x1234_5678_9abc_def0u64;

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(device, net_config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(net_task(runner)).unwrap();

    println!("Waiting for link...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(100).await;
    }
    println!("Link is up!");

    println!("Waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        println!("IP: {}", config.address);
    }

    let mut rx_buffer = [0u8; 4096];
    let mut tx_buffer = [0u8; 4096];
    let mut buf = [0u8; 1024];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(30)));

        println!("Listening on port 1234...");
        if let Err(e) = socket.accept(1234).await {
            println!("Accept error: {:?}", e);
            Timer::after_secs(1).await;
            continue;
        }
        println!("Client connected!");

        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    println!("Client disconnected");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            };

            if let Err(e) = socket.write_all(&buf[..n]).await {
                println!("Write error: {:?}", e);
                break;
            }
        }
    }
}
