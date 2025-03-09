#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
// use panic_halt as _;
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{self, IpAddress, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use hal::delay::Delay;
use hal::eth::generic_smi::GenericSMI;
use hal::eth::{Device, EthernetStationManagement, Runner, State};
use hal::rcc::*;
use hal::time::Hertz;
use hal::{eth, interrupt, println};
use heapless::Vec;
use static_cell::StaticCell;

#[embassy_executor::task]
async fn ethernet_task(runner: Runner<'static, GenericSMI>) -> ! {
    println!("ethernet_task()");
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device<'static>>) -> ! {
    println!("net_task()");
    stack.run().await
}

#[embassy_executor::task]
async fn tick() {
    loop {
        Timer::after(Duration::from_millis(1000)).await;
        println!("tick");
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    // configure core speed at 120MHz
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
    Timer::after(Duration::from_millis(1000)).await; // let some time to the debug interface to start
    println!("Hello CH32!");
    let ret = spawner.spawn(tick()).unwrap();
    println!("tick task: {ret:?}");

    info!("plop");

    // loop {
    //     Timer::after(Duration::from_millis(1000)).await;
    //     println!("tick2");
    // }

    // Ethernet setup
    let mac_addr = eth::get_mac();
    println!("mac_addr: {mac_addr:?}");
    static STATE: StaticCell<State<2, 2>> = StaticCell::new();
    println!("STATE: done");
    let state = STATE.init_with(|| State::<2, 2>::new());
    println!("state: done");
    let phy = GenericSMI::new(0);
    println!("phy: done");
    let (device, runner) = hal::eth::new(mac_addr, state, core_freq, phy).await.unwrap();
    println!("device: done");
    // println!("runner: {runner:?}");
    let station_management: EthernetStationManagement<ch32_hal::peripherals::ETH> = EthernetStationManagement::new();
    println!("station_management: done");
    let ret = spawner.spawn(ethernet_task(runner));
    println!("eth task: {ret:?}");

    // Generate random seed
    let seed = 0xdeadbeef_abadbabe;

    // Init network stack
    static STACK: StaticCell<Stack<Device<'static>>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        device,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::<2>::new()),
        seed,
    ));

    // Launch network task
    let ret = spawner.spawn(net_task(&stack));
    println!("net task: {ret:?}");

    // loop {
    //     Timer::after(Duration::from_millis(1000)).await;
    //     // println!("tick");
    // }

    println!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    println!("IP address: {:?}", local_addr);
    // stack.set_config_v4(embassy_net::ConfigV4::Static(StaticConfigV4 {
    //     address: Ipv4Cidr::new(Ipv4Address { 0: [192, 168, 1, 10] }, 24),
    //     /// Default gateway.
    //     gateway: None,
    //     /// DNS servers.
    //     dns_servers: Vec::new(),
    // }));

    // FIXME!
    // RX packets either aren't received by the stack, or smoltcp (silently) discards them for some
    // reason. It could be worth checking what smoltcp expects as packet format, and also memory
    // content

    // }
    // Then we can use it!
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

async fn wait_for_config(stack: &'static Stack<Device<'static>>) -> embassy_net::StaticConfigV4 {
    loop {
        if let Some(config) = stack.config_v4() {
            return config.clone();
        }
        yield_now().await;
    }
}
