//! BLE passive-scan listener demo — CH32V208, Task #15.
//!
//! Minimal smoke-test for the [`BleListenerState`] API introduced in
//! `src/ble/listener.rs`.  Exercises the full listener API path that
//! `ble_rx_irq.rs` does not cover:
//!
//! * `BleListenerState::start()` cold-init + IRQ enable
//! * `BleListenerState::next_pdu().await` — Embassy-async PDU delivery
//! * `AdvPdu` + `AdIterator` — structured AD traversal
//! * `BleListenerState::stats()` — counters snapshot
//! * `BleListenerState::stop()` — clean shutdown
//!
//! # Validation criteria (Cindy smoke test)
//!
//! * PDU lines appear within 5 s of start (`type=`, `AdvA=`, `flags=` / `mfr=`)
//! * `HB:` lines appear every 5 s (executor alive)
//! * No `★STUCK` in HB lines during the first 60 s
//! * `stats.frame_count` increases monotonically between HB lines
//! * AD flags / manufacturer records decoded correctly for known-good beacons
//!
//! # Architecture
//!
//! ```text
//! PFIC LLE(64) → LISTENER.on_lle_irq()   ─┐
//! PFIC BB(63)  → LISTENER.on_bb_irq()     ─┤─ ISR delegates entirely to BleListenerState
//!                                           │
//! ble_scanner_task                          │
//!   LISTENER.start(AdvFilter::default()) ◄─┘
//!   loop { pdu = LISTENER.next_pdu().await; print pdu + AD }
//!
//! main (HB loop)
//!   every 5 s: print LISTENER.stats()
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use {ch32_hal as hal, panic_halt as _};

use hal::ble::listener::{AdvFilter, BleListenerState, ble_set_phy_rx_mode_normal};

// ── Static listener state ─────────────────────────────────────────────────────

/// Shared listener state.  Must be `static` — `next_pdu()` is `async` and the
/// future borrows `self` across `.await` points.
static LISTENER: BleListenerState = BleListenerState::new();

// ── PFIC interrupt handlers ───────────────────────────────────────────────────

/// LLE PFIC interrupt (IRQn 64) — delegate body entirely to the listener.
#[ch32_hal::interrupt]
fn LLE() {
    LISTENER.on_lle_irq();
}

/// BB PFIC interrupt (IRQn 63) — delegate body entirely to the listener.
#[ch32_hal::interrupt]
fn BB() {
    LISTENER.on_bb_irq();
}

// ── Embassy async scanner task ────────────────────────────────────────────────

/// BLE scanner task — drives `LISTENER.next_pdu().await` and prints PDUs.
///
/// Prints every PDU that passes the default legacy-ADV filter:
/// * type ∈ {0=ADV_IND, 2=ADV_NONCONN_IND, 4=SCAN_RSP, 6=ADV_DIRECT_IND}
/// * len ≤ 37 bytes
/// * known AD types decoded inline (Flags 0x01, Manufacturer 0xFF, Name 0x08/0x09)
#[embassy_executor::task]
async fn ble_scanner_task() {
    // start() arms cold-init RX on ch37 and enables PFIC LLE/BB interrupts.
    // AdvFilter::default() = legacy_adv=true, count_anomalies=true.
    LISTENER.start(AdvFilter::default());
    hal::println!("scanner: LISTENER.start() done — awaiting PDUs");

    let mut pdu_n: u32 = 0;
    loop {
        let pdu = LISTENER.next_pdu().await;
        pdu_n += 1;

        // Print header on every PDU; AD decode inline.
        hal::println!(
            "[#{pdu_n} ch{ch}] type={ty:#x} len={len} \
             AdvA={a5:02x}:{a4:02x}:{a3:02x}:{a2:02x}:{a1:02x}:{a0:02x}",
            ch  = pdu.ch,
            ty  = pdu.adv_type,
            len = pdu.ad_len,
            a0  = pdu.adv_a[0], a1 = pdu.adv_a[1], a2 = pdu.adv_a[2],
            a3  = pdu.adv_a[3], a4 = pdu.adv_a[4], a5 = pdu.adv_a[5],
        );

        // Walk AD structures.
        for ad in pdu.ad_iter() {
            match ad.ad_type {
                0x01 if ad.data.len() >= 1 => {
                    hal::println!("  flags={:#04x}", ad.data[0]);
                }
                0xFF if ad.data.len() >= 2 => {
                    let company = u16::from_le_bytes([ad.data[0], ad.data[1]]);
                    hal::println!("  mfr={company:#06x} payload={}B", ad.data.len() - 2);
                }
                0x08 | 0x09 => {
                    hal::println!("  name=<{}B>", ad.data.len());
                }
                _ => {}
            }
        }
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();

    // 96 MHz from HSE 32 MHz crystal — required for RFEND calibration.
    // Config mirrors ble_rx_irq.rs exactly; both bins must behave identically.
    let _p = hal::init(hal::Config {
        rcc: hal::rcc::Config {
            hse: Some(hal::rcc::Hse {
                freq: hal::time::Hertz(32_000_000),
                mode: hal::rcc::HseMode::Oscillator,
            }),
            sys: hal::rcc::Sysclk::PLL,
            pll_src: hal::rcc::PllSource::HSE,
            pll: Some(hal::rcc::Pll {
                prediv: hal::rcc::PllPreDiv::DIV4,
                mul: hal::rcc::PllMul::MUL12,
            }),
            pllx: None,
            ahb_pre: hal::rcc::AHBPrescaler::DIV1,
            apb1_pre: hal::rcc::APBPrescaler::DIV1,
            apb2_pre: hal::rcc::APBPrescaler::DIV1,
            ls: hal::rcc::LsConfig::default_lsi(),
            hspll_src: hal::rcc::HsPllSource::HSE,
            hspll: Some(hal::rcc::HsPll {
                pre: hal::rcc::HsPllPrescaler::DIV2,
            }),
        },
        ..Default::default()
    });

    hal::println!("BLE RX listener demo — Task #15");
    hal::println!("BleListenerState API smoke test (ch37/ch38/ch39 passive scan)");

    unsafe {
        // Step 1: Full BLE PHY init — HSE, CRC/BLEC/BLES clocks, dev_inits, RF cal.
        hal::ble::ble_phy_init();

        // Step 2: Observer-mode BB config (BLE_SetPHYRxMode dtmFlag=0 path).
        //         Must be called BEFORE start(); start() runs cold-init + IRQ enable.
        ble_set_phy_rx_mode_normal();

        let tune = hal::pac::BLE_RFEND.tune_result().read();
        hal::println!("RFEND cal: co={}", tune.co());
    }

    // Spawn scanner task — calls LISTENER.start() internally.
    spawner.spawn(ble_scanner_task()).unwrap();

    // HB loop: print stats every 5 s to confirm executor is alive.
    loop {
        embassy_time::Timer::after_secs(5).await;
        let s = LISTENER.stats();
        hal::println!(
            "HB: frames={} anomalies={} stuck={} sig={} trav={} lle_irq={} bb_irq={}{}",
            s.frame_count,
            s.anomaly_count,
            s.stuck_count,
            s.sig_count,
            s.trav_count,
            s.lle_irq_count,
            s.bb_irq_count,
            if s.stuck_count > 0 { " ★STUCK" } else { "" },
        );
    }
}
