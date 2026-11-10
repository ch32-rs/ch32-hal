#![no_std]
#![no_main]

//! SPI slave ring: SPI2 streams in halves over circular DMA, clocked by SPI1 as the master
//! on the same chip.
//!
//! The master sends a 4-word burst every millisecond (NSS low around each) carrying a
//! running count, and checks the count the slave sends back. The slave checks every half it
//! receives and refills the same half of its TX buffer in place, for the next pass. Once a
//! second it prints the bursts sent, the halves received and any breaks in either count.
//! A half is longer than the print takes: SDI printing blocks, and a half the slave is too
//! late for is missed (its TX goes out again, a break in the master's count).
//!
//! Wiring: PA5 -> PB13 (SCK), PA7 -> PB15 (MOSI), PB14 -> PA6 (MISO), PA4 -> PB12 (NSS).

use core::sync::atomic::Ordering;

use ch32_hal::gpio::{Level, Output, Speed};
use ch32_hal::peripherals::{SPI1, SPI2};
use ch32_hal::spi::{self, SlaveConfig, Spi, SpiSlaveRing};
use ch32_hal::time::Hertz;
use ch32_hal::{self as hal, println};
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker, Timer};
use panic_halt as _;
use portable_atomic::AtomicU32;
use static_cell::ConstStaticCell;

/// Words per burst
const BURST: usize = 4;
/// Words per half: the slave wakes every 32 bursts, 32 ms
const HALF: usize = 32 * BURST;

static BURSTS: AtomicU32 = AtomicU32::new(0);
static MASTER_BREAKS: AtomicU32 = AtomicU32::new(0);
static HALVES: AtomicU32 = AtomicU32::new(0);
static MISSED: AtomicU32 = AtomicU32::new(0);
static SLAVE_BREAKS: AtomicU32 = AtomicU32::new(0);
static ERRORS: AtomicU32 = AtomicU32::new(0);

/// Checks that `w` continues a running count, resynchronising after a break.
fn follows(expect: &mut Option<u16>, w: u16) -> bool {
    let ok = expect.is_none_or(|e| e == w);
    *expect = Some(w.wrapping_add(1));
    ok
}

#[embassy_executor::task]
async fn master(mut spi: Spi<'static, SPI1, hal::mode::Async>, mut nss: Output<'static>) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut count = 0u16;
    let mut expect = None;
    loop {
        ticker.next().await;
        let tx: [u16; BURST] = core::array::from_fn(|i| count.wrapping_add(i as u16));
        count = count.wrapping_add(BURST as u16);
        let mut rx = [0u16; BURST];
        nss.set_low();
        let _ = spi.transfer(&mut rx, &tx).await;
        nss.set_high();
        for w in rx {
            if !follows(&mut expect, w) {
                MASTER_BREAKS.add(1, Ordering::Relaxed);
            }
        }
        BURSTS.add(1, Ordering::Relaxed);
    }
}

fn fill(tx: &mut [u16], count: &mut u16) {
    for w in tx {
        *w = *count;
        *count = count.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn slave(mut ring: SpiSlaveRing<'static, SPI2, u16>, mut tx_count: u16) -> ! {
    let mut expect = None;
    loop {
        let half = match ring.next_half().await {
            Ok(half) => half,
            Err(_) => {
                // an overrun or a lost lap: start again between two bursts. the master
                // shares this executor, so let it finish its burst (NSS high) between tries
                ERRORS.add(1, Ordering::Relaxed);
                while ring.restart().is_err() {
                    Timer::after_micros(20).await;
                }
                expect = None;
                continue;
            }
        };
        MISSED.add(half.missed, Ordering::Relaxed);
        let (rx, tx) = ring.half_mut(half.index);
        for &w in rx {
            if !follows(&mut expect, w) {
                SLAVE_BREAKS.add(1, Ordering::Relaxed);
            }
        }
        // goes out when the DMA comes back to this half, one pass later
        fill(tx, &mut tx_count);
        HALVES.add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    static RX: ConstStaticCell<[u16; 2 * HALF]> = ConstStaticCell::new([0; 2 * HALF]);
    static TX: ConstStaticCell<[u16; 2 * HALF]> = ConstStaticCell::new([0; 2 * HALF]);
    let mut ring = SpiSlaveRing::new(
        p.SPI2,
        p.PB13,
        p.PB15,
        p.PB14,
        p.PB12,
        p.DMA1_CH4,
        p.DMA1_CH5,
        RX.take(),
        TX.take(),
        SlaveConfig::default(),
    );
    // the first pass sends both TX halves as they are: start the count there
    let mut tx_count = 0u16;
    fill(ring.tx_half_mut(0), &mut tx_count);
    fill(ring.tx_half_mut(1), &mut tx_count);
    // NSS is high until the master runs, so this enables the slave at once
    ring.start().unwrap();
    spawner.spawn(slave(ring, tx_count).unwrap());

    let mut config = spi::Config::default();
    config.frequency = Hertz::mhz(6);
    let spi = Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH3, p.DMA1_CH2, config);
    let nss = Output::new(p.PA4, Level::High, Speed::High);
    spawner.spawn(master(spi, nss).unwrap());

    loop {
        Timer::after_secs(1).await;
        println!(
            "master: {} bursts, {} breaks | slave: {} halves, {} missed, {} breaks, {} errors",
            BURSTS.load(Ordering::Relaxed),
            MASTER_BREAKS.load(Ordering::Relaxed),
            HALVES.load(Ordering::Relaxed),
            MISSED.load(Ordering::Relaxed),
            SLAVE_BREAKS.load(Ordering::Relaxed),
            ERRORS.load(Ordering::Relaxed),
        );
    }
}
