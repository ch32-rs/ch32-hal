//! Raw Ethernet throughput endpoint for the CH32V4x7 integrated PHY.
//!
//! Runs below IP: a host BPF socket (`eth_raw_bench.py`) drives this firmware
//! with EtherType `0x88b5` frames addressed to the MAC below. This avoids any
//! host routing or DHCP setup, which makes it usable when the host interface
//! is not part of the system's normal IPv4 configuration.
//!
//! Control frames are 60 bytes: `dst|src|0x88b5|cmd|count_le32|padding`.
//!
//! - `D` + count: transmit `count` 1514-byte data frames, then acknowledge.
//! - `U` + count: count the next `count` data frames, then acknowledge.
//! - `P`: acknowledge immediately.
//!
//! Data frames carry `F` as their first payload byte.

#![no_std]
#![no_main]

use core::future::poll_fn;
use core::ptr::{addr_of_mut, write_volatile};
use core::task::Poll;

use ch32_hal::eth::{Config as EthConfig, Ethernet, InterruptHandler, Resources};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, rcc};
use ch32v467_examples::{report_set, report_static};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net_driver::{Driver, LinkState, RxToken, TxToken};
use embassy_time::{with_timeout, Duration};
use panic_halt as _;

bind_interrupts!(struct Irqs { ETH => InterruptHandler; });

const MAC: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x48];
const ETHERTYPE: [u8; 2] = [0x88, 0xb5];
const HEADER: usize = 14;
const FRAME_SIZE: usize = 1514;
const CONTROL_SIZE: usize = 60;
// Long enough for a normal descriptor hand-off, short enough that a link blip
// does not turn into a stuck benchmark.
const SEND_TIMEOUT_MS: u64 = 20;

const CMD_DATA: u8 = b'F';
const CMD_DOWN: u8 = b'D';
const CMD_UP: u8 = b'U';
const CMD_PING: u8 = b'P';
const CMD_ACK: u8 = b'A';

// A deep RX ring absorbs the queueing jitter of a line-rate burst; the DMA
// stops (and the driver faults) when the host cannot drain fast enough.
static mut RESOURCES: Resources<8, 32> = Resources::new();

report_static!(RAW_RESULT);
#[no_mangle]
pub static mut RAW_RX_FRAMES: u32 = 0;
#[no_mangle]
pub static mut RAW_TX_FRAMES: u32 = 0;
#[no_mangle]
pub static mut RAW_ERRORS: [u32; 3] = [0; 3];

fn result(code: u32) {
    report_set!(RAW_RESULT, code);
}

fn write_header(output: &mut [u8], destination: [u8; 6]) {
    output[..6].copy_from_slice(&destination);
    output[6..12].copy_from_slice(&MAC);
    output[12..14].copy_from_slice(&ETHERTYPE);
}

fn command_count(frame: &[u8]) -> u32 {
    u32::from_le_bytes([frame[15], frame[16], frame[17], frame[18]])
}

// `transmit` reports `None` whenever the link is down, and a PHY glitch during
// a line-rate burst can push the driver there for a while. Bound every wait so
// a transient link loss cannot park the whole benchmark in an `await`.
async fn send_control(ethernet: &mut Ethernet<'_, 8, 32>, destination: [u8; 6], command: u8, count: u32) -> bool {
    with_timeout(
        Duration::from_millis(SEND_TIMEOUT_MS),
        poll_fn(|context| {
            if let Some(tx) = ethernet.transmit(context) {
                tx.consume(CONTROL_SIZE, |output| {
                    write_header(output, destination);
                    output[14] = command;
                    output[15..19].copy_from_slice(&count.to_le_bytes());
                    output[19..].fill(0);
                });
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }),
    )
    .await
    .is_ok()
}

async fn send_data(ethernet: &mut Ethernet<'_, 8, 32>, destination: [u8; 6]) -> bool {
    with_timeout(
        Duration::from_millis(SEND_TIMEOUT_MS),
        poll_fn(|context| {
            if let Some(tx) = ethernet.transmit(context) {
                tx.consume(FRAME_SIZE, |output| {
                    write_header(output, destination);
                    output[14] = CMD_DATA;
                    output[15..].fill(0xa5);
                });
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }),
    )
    .await
    .is_ok()
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
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

    let mut ethernet = match Ethernet::new(
        peripherals.ETH,
        Irqs,
        unsafe { &mut *addr_of_mut!(RESOURCES) },
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
    defmt::info!(
        "raw bench: PHY={=u32:x} HCLK={}",
        ethernet.phy_id(),
        rcc::clocks().hclk.0
    );

    let mut rx_frames = 0u32;
    let mut tx_frames = 0u32;
    let mut up_target = 0u32;
    let mut up_count = 0u32;
    let mut up_peer = [0u8; 6];
    let mut idle_polls = 0u32;

    loop {
        let mut down_request: Option<(u32, [u8; 6])> = None;
        let mut up_request: Option<(u32, [u8; 6])> = None;
        let mut ping_peer: Option<[u8; 6]> = None;

        // Handle one received frame. During an uplink burst a frame is always
        // pending, so the timeout only bounds the idle path.
        let received = with_timeout(
            Duration::from_millis(1),
            poll_fn(|context| {
                let Some((rx, _tx)) = ethernet.receive(context) else {
                    return Poll::Pending;
                };
                rx.consume(|frame| {
                    if frame.len() < HEADER + 6 || frame[12..14] != ETHERTYPE || frame[..6] != MAC {
                        return;
                    }
                    let mut peer = [0u8; 6];
                    peer.copy_from_slice(&frame[6..12]);
                    match frame[14] {
                        CMD_DATA => {
                            rx_frames = rx_frames.saturating_add(1);
                            if up_target != 0 {
                                up_count = up_count.saturating_add(1);
                            }
                        }
                        CMD_DOWN if frame.len() >= HEADER + 5 => {
                            down_request = Some((command_count(frame), peer));
                        }
                        CMD_UP if frame.len() >= HEADER + 5 => {
                            up_request = Some((command_count(frame), peer));
                        }
                        CMD_PING => ping_peer = Some(peer),
                        _ => {}
                    }
                });
                Poll::Ready(())
            }),
        )
        .await
        .is_ok();
        if received {
            idle_polls = 0;
        } else {
            idle_polls = idle_polls.saturating_add(1);
        }

        if let Some((count, peer)) = down_request {
            let mut sent = 0;
            for _ in 0..count {
                if !send_data(&mut ethernet, peer).await {
                    break;
                }
                sent += 1;
                tx_frames = tx_frames.saturating_add(1);
            }
            send_control(&mut ethernet, peer, CMD_ACK, sent).await;
            result(5);
        }

        if let Some((count, peer)) = up_request {
            up_target = count;
            up_count = 0;
            up_peer = peer;
            result(4);
        }

        // Also answer after an idle gap: a line-rate burst can lose frames,
        // and the host needs the final count rather than a hang.
        if up_target != 0 && (up_count >= up_target || idle_polls >= 20) {
            let peer = up_peer;
            let count = up_count;
            // Keep the request pending if the link is down; the host still
            // needs this count once the PHY comes back.
            if send_control(&mut ethernet, peer, CMD_ACK, count).await {
                up_target = 0;
            }
            idle_polls = 0;
        }

        if let Some(peer) = ping_peer {
            send_control(&mut ethernet, peer, CMD_ACK, 0).await;
            result(2);
        }

        let link_up = poll_fn(|context| Poll::Ready(ethernet.link_state(context) == LinkState::Up)).await;
        if !link_up {
            result(12);
        }
        if let Some(_error) = ethernet.last_error() {
            let (tx_errors, rx_errors, dma_errors) = ethernet.error_counts();
            unsafe {
                write_volatile(addr_of_mut!(RAW_ERRORS), [tx_errors, rx_errors, dma_errors]);
            }
        }

        unsafe {
            write_volatile(addr_of_mut!(RAW_RX_FRAMES), rx_frames);
            write_volatile(addr_of_mut!(RAW_TX_FRAMES), tx_frames);
        }
    }
}
