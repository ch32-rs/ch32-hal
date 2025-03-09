#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{self, Stack, StackResources};
use embassy_time::{Duration, Timer};
use hal::eth::generic_smi::GenericSMI;
use hal::eth::{Device, EthernetStationManagement, Runner, State};
use hal::rcc::*;
use hal::time::Hertz;
use hal::{eth, interrupt, println};
use static_cell::StaticCell;
use {ch32_hal as hal, panic_halt as _};

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

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    // configure core speed at 60MHz
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
    Timer::after(Duration::from_millis(100)).await; // let some time to the debug interface to start
    println!("Hello CH32!");

    // Ethernet setup
    let mac_addr = eth::get_mac();
    println!("mac_addr: {mac_addr:?}");
    static STATE: StaticCell<State<2, 2>> = StaticCell::new();
    let state = STATE.init_with(|| State::<2, 2>::new());
    let phy = GenericSMI::new(0);
    let (device, runner) = hal::eth::new(mac_addr, state, core_freq, phy).await.unwrap();
    let _station_management: EthernetStationManagement<ch32_hal::peripherals::ETH> = EthernetStationManagement::new();
    spawner.spawn(ethernet_task(runner)).unwrap();

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
    spawner.spawn(net_task(&stack)).unwrap();

    println!("Waiting for DHCP...");
    let cfg = wait_for_config(stack).await;
    let local_addr = cfg.address.address();
    println!("IP address: {:?}", local_addr);

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
