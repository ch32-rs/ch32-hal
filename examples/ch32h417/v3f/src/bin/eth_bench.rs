//! TCP throughput server for the integrated 10M/100M PHY on the V3F core.
//!
//! This is the H4 counterpart of `examples/ch32v467`'s `hal_eth_bench`: it
//! runs `embassy-net` over the H4 driver with static IPv4 addressing, then
//! answers a small byte-count protocol so a host can measure uplink and
//! downlink throughput. `examples/ch32v467/eth_throughput_test.py` drives it.
//!
//! Protocol on TCP port 5001, one command per line:
//!
//! - `UP <bytes>\n` followed by `<bytes>` payload bytes: the payload is
//!   counted and discarded, then `OK <bytes>\n` is returned.
//! - `DOWN <bytes>\n`: `<bytes>` bytes of a repeating pattern are sent, then
//!   `OK <bytes>\n`.
//! - `QUIT\n`: closes the connection.
//!
//! Static network: board `192.168.47.10/24`, host `192.168.47.2/24`.
//!
//! Measured 2026-11-07 on `nanoCH32H417` against a macOS host: 93 Mbit/s up
//! and 92 Mbit/s down over 8 MiB rounds, i.e. 98% of what a 100BASE-TX link
//! carries in 1460-byte segments (94.9 Mbit/s). That takes all of the levers
//! this file sets, each measured on its own:
//!
//! - The boot core runs from the zero-wait shared SRAM (`v3f/link.x`). While
//!   hart 0 still executed from flash the same test gave 5.0 Mbit/s: Code FLASH
//!   is non-zero-wait and hart 0 has no instruction cache (RM §1.2.1).
//! - The V3F recipe is the 120 MHz one (`SYSCLK_480M_..._V3F_120M`), the
//!   fastest documented for this core: 54 → 60 Mbit/s up, 59 → 70 down.
//! - `opt-level = 3` in `Cargo.toml` (`opt-level = "s"` was for the flash
//!   budget): 60 → 71 Mbit/s up, 70 → 75 down.
//! - 32-deep DMA rings. smoltcp clamps the advertised TCP window to
//!   `max_burst_size` segments, and the driver reports the ring depth, so an
//!   8-deep ring pinned the window at 11.7 KB: 71 → 93 Mbit/s up. The sockets
//!   get 32 KB buffers to match.
//!
//! ```text
//! cargo xtask run --example eth_bench
//! python3 ../ch32v467/eth_throughput_test.py 192.168.47.10
//! ```

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use ch32_hal as hal;
use ch32_hal::bind_interrupts;
use ch32_hal::eth::{Ethernet, InternalPhy, InterruptHandler, Phy, Resources};
use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config as NetConfig, Ipv4Address, Ipv4Cidr, Runner, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    ETH => InterruptHandler;
});

const MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x17];
const ADDRESS: Ipv4Address = Ipv4Address::new(192, 168, 47, 10);
const PORT: u16 = 5001;
const CHUNK: usize = 1460;

// The driver advertises `min(TX, RX)` as smoltcp's `max_burst_size`, which
// caps the receive window at `max_burst_size * MSS`. Deeper rings are what
// buy the throughput.
const ETH_TX: usize = 32;
const ETH_RX: usize = 32;
type Device = Ethernet<'static, InternalPhy>;

static mut ETH_RESOURCES: Resources<ETH_TX, ETH_RX> = Resources::new();
static mut NET_RESOURCES: StackResources<1> = StackResources::new();
static mut SOCKET_RX: [u8; 32768] = [0; 32768];
static mut SOCKET_TX: [u8; 32768] = [0; 32768];
static mut SCRATCH: [u8; CHUNK] = [0; CHUNK];

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
    let mut total = 0u32;
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
            let Ok(total) = receive_payload(socket, buffer, count).await else {
                return;
            };
            info!("uplink {} bytes", total);
            let length = format_ok(total, &mut ack);
            if write_all(socket, &ack[..length]).await.is_err() {
                return;
            }
        } else if let Some(count) = command.strip_prefix(b"DOWN ").and_then(parse_count) {
            let Ok(total) = send_pattern(socket, buffer, count).await else {
                return;
            };
            info!("downlink {} bytes", total);
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
async fn net_task(mut runner: Runner<'static, Device>) -> ! {
    runner.run().await
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) {
    // 480 MHz SYSCLK, V5F 240 MHz, V3F/HCLK 120 MHz — the fastest V3F recipe,
    // and the clock the measured throughput in the module docs was taken at.
    let mut rcc = hal::rcc::Config::with_sysclk_480m_v5f_240m_v3f_120m_hse();
    // The integrated PHY runs from the fixed 500 MHz ETH PLL (RM §31.4.3).
    rcc.aux_plls.eth = true;
    let p = hal::init(hal::Config {
        rcc,
        ..Default::default()
    });

    let ethernet = match Ethernet::new_internal(
        p.ETH,
        Irqs,
        unsafe { &mut *addr_of_mut!(ETH_RESOURCES) },
        MAC,
        InternalPhy::new(),
    ) {
        Ok(driver) => driver,
        Err(error) => {
            info!("ETH init failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    info!("ETH bench: PHY={=u32:#010x}", ethernet.phy().phy_id());

    let (stack, runner) = embassy_net::new(
        ethernet,
        NetConfig::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(ADDRESS, 24),
            gateway: None,
            dns_servers: Default::default(),
        }),
        unsafe { &mut *addr_of_mut!(NET_RESOURCES) },
        0x5eed_1234_5678_9abc,
    );
    spawner.spawn(net_task(runner).unwrap());

    while !stack.is_link_up() {
        Timer::after(Duration::from_millis(100)).await;
    }
    stack.wait_config_up().await;
    info!("listening on TCP port {}", PORT);

    let buffer = unsafe { &mut *addr_of_mut!(SCRATCH) };
    loop {
        let rx = unsafe { &mut *addr_of_mut!(SOCKET_RX) };
        let tx = unsafe { &mut *addr_of_mut!(SOCKET_TX) };
        let mut socket = TcpSocket::new(stack, rx, tx);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if let Err(error) = socket.accept(PORT).await {
            warn!("accept failed: {:?}", error);
            continue;
        }
        info!("client connected");
        serve(&mut socket, buffer).await;
        info!("client disconnected");
    }
}
