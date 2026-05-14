//! Ethernet TCP echo server for CH32V208 (built-in 10M PHY).
//!
//! Initializes the CH32V208's built-in 10M Ethernet MAC+PHY,
//! obtains an IP via DHCP, and runs a TCP echo server on port 1234.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
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
    println!("Hello CH32 - TCP echo server!");

    // Ethernet setup
    let mac_addr = eth::get_mac();
    println!("mac_addr: {mac_addr:?}");

    let phy = GenericSMI::new(0);
    static STATE: StaticCell<eth::State<2, 2>> = StaticCell::new();
    let state = STATE.init(eth::State::new());
    let (runner, device) = eth::new(mac_addr, core_freq, phy, state).await.unwrap();
    println!("device: done");

    // Generate random seed
    let seed = 0x1234_5678_9abc_def0u64;

    // Init network stack
    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let config = embassy_net::Config::dhcpv4(Default::default());
    let (stack, net_runner) = embassy_net::new(device, config, RESOURCES.init(StackResources::new()), seed);

    spawner.spawn(ethernet_task(runner)).unwrap();
    spawner.spawn(net_task(net_runner)).unwrap();

    println!("Waiting for DHCP...");
    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        println!("IP address: {}", config.address);
    }

    // TCP echo server on port 1234
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 1024];
    let mut buf = [0u8; 512];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(30)));

        println!("Listening on TCP port 1234...");

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
