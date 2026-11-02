#![no_std]
#![no_main]

use core::future::poll_fn;
use core::task::Poll;

use ch32_hal::eth::{Config, Ethernet, InterruptHandler, Resources};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_net_driver::{Driver, LinkState, RxToken, TxToken};
use embassy_time::{with_timeout, Duration};
use panic_halt as _;

bind_interrupts!(struct Irqs { ETH => InterruptHandler; });

const MAC: [u8; 6] = [2, 0, 0, 0, 0, 0x47];
const MAGIC: &[u8] = b"CH32V4-ECHO";
static mut RESOURCES: Resources<4, 4> = Resources::new();

#[no_mangle]
pub static mut ETH_RESULT: u32 = 0;
#[no_mangle]
pub static mut ETH_PHY_ID: u32 = 0;
#[no_mangle]
pub static mut ETH_STATS: [u32; 5] = [0; 5];

fn result(value: u32) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(ETH_RESULT), value) };
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
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    let peripherals = ch32_hal::init(config);
    let mut ethernet = match Ethernet::new(
        peripherals.ETH,
        Irqs,
        unsafe { &mut *core::ptr::addr_of_mut!(RESOURCES) },
        Config {
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
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(ETH_PHY_ID), ethernet.phy_id()) };
    defmt::info!(
        "ETH initialized: PHY={=u32:x} HCLK={}",
        ethernet.phy_id(),
        rcc::clocks().hclk.0
    );
    result(2);
    let mut received = 0u32;
    let mut echoed = 0u32;
    let mut was_up = false;
    loop {
        let up = poll_fn(|context| Poll::Ready(ethernet.link_state(context) == LinkState::Up)).await;
        if up != was_up {
            defmt::info!("ETH link: {:?}", ethernet.link());
            result(if up { 3 } else { 2 });
            was_up = up;
        }
        let _ = with_timeout(
            Duration::from_millis(100),
            poll_fn(|context| {
                let Some((rx, tx)) = ethernet.receive(context) else {
                    return Poll::Pending;
                };
                received = received.saturating_add(1);
                rx.consume(|frame| {
                    if frame.len() >= 14 + MAGIC.len()
                        && frame[12..14] == [0x88, 0xb5]
                        && frame[14..14 + MAGIC.len()] == *MAGIC
                        && frame[..6] == MAC
                    {
                        tx.consume(frame.len(), |output| {
                            output.copy_from_slice(frame);
                            output[..6].copy_from_slice(&frame[6..12]);
                            output[6..12].copy_from_slice(&MAC);
                        });
                        echoed = echoed.saturating_add(1);
                        result(4);
                    }
                });
                Poll::Ready(())
            }),
        )
        .await;
        let (tx_errors, rx_errors, dma_errors) = ethernet.error_counts();
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(ETH_STATS),
                [received, echoed, tx_errors, rx_errors, dma_errors],
            )
        };
        if let Some(error) = ethernet.last_error() {
            result(11);
            defmt::panic!("ETH error: {:?}", error);
        }
    }
}
