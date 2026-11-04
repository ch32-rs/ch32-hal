//! TCP throughput server for the CH32V4x7 integrated Ethernet PHY.
//!
//! Runs `embassy-net` over the `hal_eth` driver with static IPv4 addressing,
//! then serves a simple byte-count protocol so a host can measure uplink and
//! downlink throughput. The companion `eth_throughput_test.py` drives it.
//!
//! Protocol on TCP port 5001, one command per line:
//!
//! - `UP <bytes>\n` followed by `<bytes>` payload bytes: the payload is
//!   counted and discarded, then `OK <bytes>\n` is returned.
//! - `DOWN <bytes>\n`: `<bytes>` bytes of a repeating pattern are sent, then
//!   `OK <bytes>\n`.
//! - `QUIT\n`: closes the connection.
//!
//! Static network: board `192.168.47.1/24`, host `192.168.47.2/24`. The example
//! selects the 200 MHz PLL configuration so the core does not cap the link.

#![no_std]
#![no_main]

use core::ptr::{addr_of_mut, read_volatile, write_volatile};

use ch32_hal::eth::{Config as EthConfig, Ethernet, InterruptHandler, Resources};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, rcc};
use ch32v467_examples::{report_set, report_static};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs { ETH => InterruptHandler; });

const MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x48];
const ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 47, 1);
const PORT: u16 = 5001;
const CHUNK: usize = 1460;

const ETH_TX: usize = 8;
const ETH_RX: usize = 8;
type Device = Ethernet<'static, ETH_TX, ETH_RX>;

static mut ETH_RESOURCES: Resources<ETH_TX, ETH_RX> = Resources::new();
static mut NET_RESOURCES: StackResources<2> = StackResources::new();
static mut SOCKET_RX: [u8; 16384] = [0; 16384];
static mut SOCKET_TX: [u8; 16384] = [0; 16384];
static mut SCRATCH: [u8; CHUNK] = [0; CHUNK];

report_static!(ETH_BENCH_RESULT);
#[no_mangle]
pub static mut ETH_BENCH_BYTES: [u32; 2] = [0; 2];
#[no_mangle]
pub static mut ETH_BENCH_PHY_ID: u32 = 0;

fn result(code: u32) {
    report_set!(ETH_BENCH_RESULT, code);
}

fn record(index: usize, count: u32) {
    unsafe {
        let slot = match index {
            0 => addr_of_mut!(ETH_BENCH_BYTES[0]),
            _ => addr_of_mut!(ETH_BENCH_BYTES[1]),
        };
        write_volatile(slot, read_volatile(slot).saturating_add(count));
    }
}

fn format_ok(mut value: u32, out: &mut [u8; 16]) -> usize {
    let mut digits = [0u8; 10];
    let mut count = 0;
    if value == 0 {
        digits[0] = b'0';
        count = 1;
    } else {
        while value != 0 {
            digits[count] = b'0' + (value % 10) as u8;
            value /= 10;
            count += 1;
        }
    }
    out[..3].copy_from_slice(b"OK ");
    for index in 0..count {
        out[3 + index] = digits[count - 1 - index];
    }
    out[3 + count] = b'\n';
    4 + count
}

fn parse_count(text: &[u8]) -> Option<u32> {
    if text.is_empty() || text.len() > 10 {
        return None;
    }
    let mut value = 0u32;
    for &byte in text {
        if !byte.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add(u32::from(byte - b'0'))?;
    }
    Some(value)
}

async fn read_line(socket: &mut TcpSocket<'_>, line: &mut [u8]) -> Result<usize, ()> {
    let mut length = 0;
    loop {
        let mut byte = [0u8; 1];
        let read = socket.read(&mut byte).await.map_err(|_| ())?;
        if read == 0 {
            return Err(());
        }
        if byte[0] == b'\n' {
            return Ok(length);
        }
        if length == line.len() {
            return Err(());
        }
        line[length] = byte[0];
        length += 1;
    }
}

async fn write_all(socket: &mut TcpSocket<'_>, mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        let written = socket.write(data).await.map_err(|_| ())?;
        if written == 0 {
            return Err(());
        }
        data = &data[written..];
    }
    Ok(())
}

async fn receive_payload(socket: &mut TcpSocket<'_>, buffer: &mut [u8], mut remaining: u32) -> Result<u32, ()> {
    let mut total = 0;
    while remaining != 0 {
        let want = remaining.min(buffer.len() as u32) as usize;
        let read = socket.read(&mut buffer[..want]).await.map_err(|_| ())?;
        if read == 0 {
            return Err(());
        }
        remaining -= read as u32;
        total += read as u32;
    }
    Ok(total)
}

async fn send_pattern(socket: &mut TcpSocket<'_>, buffer: &mut [u8], mut remaining: u32) -> Result<u32, ()> {
    for (index, byte) in buffer.iter_mut().enumerate() {
        *byte = (index & 0xff) as u8;
    }
    let mut total = 0;
    while remaining != 0 {
        let want = remaining.min(buffer.len() as u32) as usize;
        write_all(socket, &buffer[..want]).await?;
        remaining -= want as u32;
        total += want as u32;
    }
    Ok(total)
}

async fn serve(socket: &mut TcpSocket<'_>, buffer: &mut [u8]) {
    let mut line = [0u8; 32];
    let mut ack = [0u8; 16];
    loop {
        let length = match read_line(socket, &mut line).await {
            Ok(0) | Err(()) => return,
            Ok(length) => length,
        };
        let command = &line[..length];
        if command == b"QUIT" {
            return;
        }
        if let Some(count) = command.strip_prefix(b"UP ").and_then(parse_count) {
            let total = match receive_payload(socket, buffer, count).await {
                Ok(total) => total,
                Err(()) => return,
            };
            record(0, total);
            result(4);
            defmt::info!("uplink {} bytes", total);
            let length = format_ok(total, &mut ack);
            if write_all(socket, &ack[..length]).await.is_err() {
                return;
            }
        } else if let Some(count) = command.strip_prefix(b"DOWN ").and_then(parse_count) {
            let total = match send_pattern(socket, buffer, count).await {
                Ok(total) => total,
                Err(()) => return,
            };
            record(1, total);
            result(5);
            defmt::info!("downlink {} bytes", total);
            let length = format_ok(total, &mut ack);
            if write_all(socket, &ack[..length]).await.is_err() {
                return;
            }
        } else if write_all(socket, b"ERR\n").await.is_err() {
            return;
        }
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    result(1);
    let mut config = ch32_hal::Config::default();
    config.rcc.hse = Some(rcc::Hse {
        freq: Hertz(25_000_000),
        mode: rcc::HseMode::Oscillator,
    });
    config.rcc.ethpll = true;
    config.rcc.sys = rcc::Sysclk::PLL;
    config.rcc.pll = Some(rcc::Pll {
        mul: rcc::PllMul::MUL20,
    });
    config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    let peripherals = ch32_hal::init(config);

    let ethernet = match Ethernet::new(
        peripherals.ETH,
        Irqs,
        unsafe { &mut *addr_of_mut!(ETH_RESOURCES) },
        EthConfig {
            mac_address: MAC,
            ..Default::default()
        },
    ) {
        Ok(driver) => driver,
        Err(error) => {
            result(10);
            defmt::panic!("ETH init failed: {:?}", error);
        }
    };
    unsafe { write_volatile(addr_of_mut!(ETH_BENCH_PHY_ID), ethernet.phy_id()) };
    defmt::info!(
        "ETH bench: PHY={=u32:x} HCLK={}",
        ethernet.phy_id(),
        rcc::clocks().hclk.0
    );

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(ADDRESS, 24),
        gateway: None,
        dns_servers: Default::default(),
    });
    let (stack, runner) = embassy_net::new(
        ethernet,
        net_config,
        unsafe { &mut *addr_of_mut!(NET_RESOURCES) },
        0x5eed_1234_5678_9abc,
    );
    spawner.spawn(net_task(runner).unwrap());

    while !stack.is_link_up() {
        Timer::after_millis(100).await;
    }
    stack.wait_config_up().await;
    defmt::info!("listening on TCP port {}", PORT);
    result(2);

    let buffer = unsafe { &mut *addr_of_mut!(SCRATCH) };
    loop {
        let rx = unsafe { &mut *addr_of_mut!(SOCKET_RX) };
        let tx = unsafe { &mut *addr_of_mut!(SOCKET_TX) };
        let mut socket = TcpSocket::new(stack, rx, tx);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if let Err(error) = socket.accept(PORT).await {
            defmt::warn!("accept failed: {:?}", error);
            continue;
        }
        result(3);
        defmt::info!("client connected");
        serve(&mut socket, buffer).await;
        defmt::info!("client disconnected");
        result(2);
    }
}
