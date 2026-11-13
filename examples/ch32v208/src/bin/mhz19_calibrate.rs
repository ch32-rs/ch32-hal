//! MH-Z19B calibration tool for the CH32V208 - **WRITES TO THE SENSOR**.
//!
//! Separate from the read-only `mhz19` example on purpose: calibration stores a
//! new baseline in the sensor, and getting it wrong offsets every later reading
//! until it is redone properly.
//!
//! Uses **our own** `edrv-mhz19` driver in its async flavour, like the read-only
//! example, so this project has exactly one MH-Z19 implementation.
//!
//! Wiring is the same:
//!
//! ```text
//!   MH-Z19B           CH32V208
//!   VCC  (5V)  <----  5V      (independent, well-decoupled supply)
//!   GND        <----  GND
//!   TX         ---->  PA10    (USART1_RX)
//!   RX         <----  PA9     (USART1_TX)
//! ```
//!
//! # How this stays safe
//!
//! 1. `ARM` is `false` by default. While it is false this program **never sends
//!    a command that writes to the sensor** - it only observes and reports.
//! 2. It observes for the full soak period **before** deciding, and refuses if
//!    the link is unhealthy, the reading is unstable, or the reading is not
//!    where the chosen calibration says it should be.
//! 3. It applies at most one action per run, then stops.
//!
//! # Preconditions (from the Winsen datasheet)
//!
//! * **Zero (`0x87`)**: the sensor must have been in fresh air at ~400 ppm for
//!   at least **20 minutes**. "Zero" means 400 ppm, *not* 0 ppm. Running this in
//!   a normal room bakes a wrong baseline in.
//! * **Span (`0x88`)**: needs a **known reference gas** at the target ppm
//!   (2000 ppm is the usual choice) and the sensor soaked in it. **Do zero
//!   first.** The upstream driver's README says plainly: "I highly recommend
//!   avoiding this command unless you have the equipment to do so."
//! * **ABC (`0x79`)**: reversible, no gas needed. ABC slowly drifts the zero
//!   point to the lowest reading of the last 24 h; a manual calibration is
//!   normally done with ABC off so it is not immediately drifted away again.
//! * **Range (`0x99`)**: 2000 ppm is what upstream advises for accuracy.
//!
//! This sensor's command set is `0x86`, `0x87`, `0x88`, `0x79`, `0x99` - it has
//! no `0x85`, so the two-channel reset cross-check from the read-only example is
//! not available here. That is another reason this tool insists on a full soak
//! and a stability check instead of trusting a single reading.
//!
//! # Acknowledgements
//!
//! `edrv-mhz19` sends the calibration commands as *send-only*, which matches the
//! datasheet: several firmware revisions do not answer them. This tool still
//! wants to know which kind of sensor it is talking to, so after each write it
//! borrows the transport back with `release()` and listens briefly. A reply is
//! reported; silence is reported as normal, not as a failure.

#![no_std]
#![no_main]
// `ACTION` selects one of several variants, so the ones you are not currently
// using are only ever constructed by editing the file. That is deliberate.
#![allow(dead_code)]

#[path = "../uart_io.rs"]
mod uart_io;

use ch32_hal as hal;
use edrv_mhz19::{Error as Mhz19Error, Reading, MHZ19, PACKET_LEN, RANGE_MAX_PPM, RANGE_MIN_PPM};
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use embedded_io_async::Read as _;
use hal::usart::{self, Uart};
use hal::{bind_interrupts, peripherals, println};
use panic_halt as _;
use uart_io::{UartError, UartIo};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

// ===========================================================================
// POLICY - the only things you normally edit
// ===========================================================================

/// **THE SAFETY GATE.** No command that writes to the sensor is sent while this
/// is `false`. Set it to `true` only after reading the preconditions above and
/// after seeing a run report "Preconditions PASSED".
const ARM: bool = false;

/// What to do once the preconditions pass.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Action {
    /// `0x87` - zero point = 400 ppm. Needs fresh air, >= 20 min soak.
    Zero,
    /// `0x88` - span point in ppm. Needs reference gas, and zero done first.
    Span(u16),
    /// `0x79` - automatic baseline correction on/off. Reversible.
    Abc(bool),
    /// `0x99` - detection range in ppm, `RANGE_MIN_PPM..=RANGE_MAX_PPM`.
    /// 2000 is what upstream advises for accuracy.
    Range(u16),
}

const ACTION: Action = Action::Zero;

/// Turn ABC off before a manual zero/span calibration, so the new baseline is
/// not immediately drifted away. Mirrors the upstream library, which sends
/// "autocalibration off" automatically.
const ABC_OFF_BEFORE_CAL: bool = true;

/// Fresh air is ~400 ppm; refuse to zero outside this window.
const ZERO_WINDOW_PPM: (u16, u16) = (350, 500);

/// Refuse if the observed spread over the soak is wider than this.
const MAX_SPREAD_PPM: u16 = 30;

/// Refuse if more than this many reads failed during the soak.
const MAX_BAD_READS: u32 = 3;

/// Span calibration accepts the reading within this percentage of the target.
const SPAN_TOLERANCE_PCT: u16 = 10;

/// Datasheet preheat: the first ~3 minutes after power-on are not trustworthy.
/// These readings are printed for visibility but excluded from the soak
/// statistics, so warm-up drift cannot fail an otherwise good setup.
const PREHEAT: Duration = Duration::from_secs(3 * 60);

/// Soak time per action: the datasheet's 20 minutes for anything that stores a
/// baseline, just a short health check for the reversible settings.
fn soak_secs(action: Action) -> u64 {
    match action {
        Action::Zero | Action::Span(_) => 20 * 60,
        Action::Abc(_) | Action::Range(_) => 10,
    }
}

fn action_name(action: Action) -> &'static str {
    match action {
        Action::Zero => "zero calibration (0x87, 400 ppm)",
        Action::Span(_) => "span calibration (0x88)",
        Action::Abc(_) => "auto baseline correction (0x79)",
        Action::Range(_) => "detection range (0x99)",
    }
}

// ===========================================================================
// Sensor access
// ===========================================================================

/// The sensor speaks 9600 8N1.
const MHZ19_BAUD: u32 = 9600;

/// How long to wait for a response before giving up on the exchange.
///
/// The driver has no notion of time, so every read is wrapped in this.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(500);

/// Why a read failed - either the sensor stayed silent, or the driver rejected
/// what it sent.
#[derive(Debug)]
enum ReadError {
    /// Nothing came back within `SERIAL_TIMEOUT`.
    Timeout,
    /// The response arrived but did not validate.
    Driver(Mhz19Error<UartError>),
}

/// Read the CO2 concentration, bounded by `SERIAL_TIMEOUT`.
async fn read_co2(sensor: &mut MHZ19<UartIo>) -> Result<Reading, ReadError> {
    match with_timeout(SERIAL_TIMEOUT, sensor.read_co2()).await {
        Ok(Ok(reading)) => Ok(reading),
        Ok(Err(e)) => Err(ReadError::Driver(e)),
        Err(_) => Err(ReadError::Timeout),
    }
}

// ===========================================================================
// Reporting helpers
// ===========================================================================

/// Send one write-command and report whether the sensor acknowledged it.
///
/// `edrv-mhz19` treats these commands as send-only, so the transport is taken
/// back with `release()` and polled directly. A missing reply is reported as
/// such rather than treated as a hard failure, because not every firmware
/// revision answers the calibration commands.
///
/// Takes and returns the driver by value, because `release` consumes it.
async fn apply_and_listen(
    sensor: MHZ19<UartIo>,
    label: &str,
    sent: Result<(), Mhz19Error<UartError>>,
) -> MHZ19<UartIo> {
    if let Err(e) = sent {
        println!("  {:<26} SEND FAILED: {:?}", label, e);
        return sensor;
    }

    let mut io = sensor.release();
    let mut buf = [0u8; PACKET_LEN];

    match with_timeout(SERIAL_TIMEOUT, io.read(&mut buf)).await {
        Ok(Ok(n)) if n >= PACKET_LEN => println!("  {:<26} acked: {:02X?}", label, &buf[..]),
        Ok(Ok(_)) | Err(_) => println!("  {:<26} sent, no reply (firmware may not ack)", label),
        Ok(Err(e)) => println!("  {:<26} sent, read error: {:?}", label, e),
    }

    MHZ19::new(io)
}

fn mark(ok: bool) -> &'static str {
    if ok {
        "OK"
    } else {
        "FAIL"
    }
}

/// Park forever; calibration is deliberately one-shot per run.
async fn park() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 MH-Z19B CALIBRATION");
    println!("  USART1  PA9=TX  PA10=RX  9600 8N1");
    println!("  driver: edrv-mhz19 (async)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");
    println!("  action        : {}", action_name(ACTION));
    println!("  ARM           : {}   (false = observe only, nothing is written)", ARM);
    println!("  preheat       : {} s (not counted)", PREHEAT.as_secs());
    println!("  soak          : {} s", soak_secs(ACTION));
    println!("  total wait    : {} s", PREHEAT.as_secs() + soak_secs(ACTION));
    println!("  ABC off first : {}", ABC_OFF_BEFORE_CAL);
    println!("");

    // Reject an impossible request before spending 20 minutes on it.
    match ACTION {
        Action::Span(ppm) if ppm < 1000 => {
            println!("span target {} ppm is below the 1000 ppm minimum.", ppm);
            println!("Nothing to do; pick a higher Action::Span value.");
            park().await;
        }
        Action::Range(ppm) if !(RANGE_MIN_PPM..=RANGE_MAX_PPM).contains(&ppm) => {
            println!(
                "detection range {} ppm is outside the {}..={} ppm the sensor accepts.",
                ppm, RANGE_MIN_PPM, RANGE_MAX_PPM
            );
            println!("Nothing to do; pick a range inside that window.");
            park().await;
        }
        _ => {}
    }

    let mut cfg = usart::Config::default();
    cfg.baudrate = MHZ19_BAUD;

    let (tx, rx) = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, cfg)
        .unwrap()
        .split();
    let mut sensor = MHZ19::new(UartIo::new(rx, tx));

    // ---- phase 0: preheat -------------------------------------------------
    // The datasheet wants ~3 minutes after power-on before readings mean
    // anything, so those samples must not be allowed to pollute the soak
    // statistics (they would inflate the spread and could fail a good setup).
    if PREHEAT.as_secs() > 0 {
        println!(
            "---- preheat {} s; these readings are not counted ----",
            PREHEAT.as_secs()
        );
        let preheat_start = Instant::now();
        while preheat_start.elapsed() < PREHEAT {
            match read_co2(&mut sensor).await {
                Ok(reading) => println!(
                    "[preheat {:>3}s] CO2 {} ppm",
                    preheat_start.elapsed().as_secs(),
                    reading.co2_ppm
                ),
                Err(e) => println!(
                    "[preheat {:>3}s] read failed: {:?}",
                    preheat_start.elapsed().as_secs(),
                    e
                ),
            }
            Timer::after(Duration::from_secs(30)).await;
        }
        println!("");
    }

    // ---- phase 1: observe only -------------------------------------------
    println!("---- observing; no write command is sent in this phase ----");

    let start = Instant::now();
    let window = soak_secs(ACTION);
    let half = window / 2;
    let mut min = u16::MAX;
    let mut max = 0u16;
    let mut sum = 0u64;
    let mut samples = 0u32;
    let mut bad = 0u32;
    // First-half / second-half totals, to tell a steady drift apart from noise.
    let mut first_sum = 0u64;
    let mut first_n = 0u32;
    let mut second_sum = 0u64;
    let mut second_n = 0u32;

    while start.elapsed().as_secs() < window {
        match read_co2(&mut sensor).await {
            Ok(reading) => {
                let value = reading.co2_ppm;
                min = min.min(value);
                max = max.max(value);
                sum += value as u64;
                samples += 1;

                if start.elapsed().as_secs() < half {
                    first_sum += value as u64;
                    first_n += 1;
                } else {
                    second_sum += value as u64;
                    second_n += 1;
                }

                // Roughly one line a minute on a 20 minute soak, but keep the
                // short health-check runs chatty.
                if window <= 15 || samples % 12 == 1 {
                    println!(
                        "[t={:>5}s] CO2 {:>4} ppm   min {:>4}  max {:>4}  bad {}",
                        start.elapsed().as_secs(),
                        value,
                        min,
                        max,
                        bad
                    );
                    println!("             raw 0x86: {:02X?}", sensor.last_response());
                }
            }
            Err(e) => {
                bad += 1;
                println!("[t={:>5}s] read failed: {:?}", start.elapsed().as_secs(), e);
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }

    // ---- phase 2: verdict -------------------------------------------------
    let avg = if samples > 0 { (sum / samples as u64) as u16 } else { 0 };
    let spread = if samples > 0 { max.saturating_sub(min) } else { 0 };
    let comms_ok = samples > 0 && bad <= MAX_BAD_READS;
    let stable = spread <= MAX_SPREAD_PPM;

    // A room that is still exchanging air produces a steady ramp, which is a
    // property of the environment, not of the sensor. Report it separately so
    // "not stable" does not get misread as "sensor is noisy".
    let drift: i32 = if first_n > 0 && second_n > 0 {
        (second_sum / second_n as u64) as i32 - (first_sum / first_n as u64) as i32
    } else {
        0
    };
    let mostly_trend = drift.unsigned_abs() * 2 >= spread as u32;

    let reading_needed = matches!(ACTION, Action::Zero | Action::Span(_));
    let target_ok = match ACTION {
        Action::Zero => avg >= ZERO_WINDOW_PPM.0 && avg <= ZERO_WINDOW_PPM.1,
        Action::Span(ppm) => {
            let margin = ppm * SPAN_TOLERANCE_PCT / 100;
            avg >= ppm.saturating_sub(margin) && avg <= ppm.saturating_add(margin)
        }
        Action::Abc(_) | Action::Range(_) => true,
    };

    println!("");
    println!("---- verdict after {} sample(s) ----", samples);
    println!("  link health : {:>3} failed read(s)          {}", bad, mark(comms_ok));
    println!("  reading     : avg {}  min {}  max {} ppm", avg, min, max);
    println!(
        "  stability   : spread {} ppm (limit {})   {}",
        spread,
        MAX_SPREAD_PPM,
        mark(stable)
    );
    if samples > 0 {
        println!(
            "  drift       : 1st half {} -> 2nd half {} ppm ({:+})  {}",
            first_sum / first_n.max(1) as u64,
            second_sum / second_n.max(1) as u64,
            drift,
            if mostly_trend {
                "a trend, not noise"
            } else {
                "noise, not a trend"
            }
        );
    }
    println!("  target      : {}", mark(target_ok));
    println!("");

    let preconditions_ok = comms_ok && (!reading_needed || (stable && target_ok));

    if !preconditions_ok {
        println!("REFUSING to calibrate - preconditions not met:");
        if !comms_ok {
            println!(
                "  - link unhealthy ({} failed reads). Fix 5V supply / GND / level first.",
                bad
            );
        }
        if reading_needed && !stable {
            println!(
                "  - reading is not stable (spread {} ppm > {} ppm).",
                spread, MAX_SPREAD_PPM
            );
            if mostly_trend {
                println!(
                    "    Most of that ({:+} ppm) is a steady ramp, so the air is still",
                    drift
                );
                println!("    equilibrating rather than the sensor being noisy. Let it settle,");
                println!("    or move to a larger/draftier space where the level flattens.");
            } else {
                println!(
                    "    Drift is only {:+} ppm, so this looks like sensor noise instead:",
                    drift
                );
                println!("    check the 5V supply and keep the sensor away from air currents.");
            }
        }
        if reading_needed && !target_ok {
            match ACTION {
                Action::Zero => {
                    println!(
                        "  - {} ppm is outside the fresh-air window {}..{} ppm.",
                        avg, ZERO_WINDOW_PPM.0, ZERO_WINDOW_PPM.1
                    );
                    if avg > ZERO_WINDOW_PPM.1 {
                        println!("    An occupied room sits at 800-1500 ppm, so this most likely");
                        println!("    means the space is not fresh air rather than the sensor being");
                        println!("    broken. Zero calibration needs outdoor-grade air (~400 ppm):");
                        println!("    take the sensor outside or to a strong through-draught.");
                    } else {
                        println!("    Below the window: the sensor may already be calibrated low.");
                    }
                }
                Action::Span(ppm) => println!(
                    "  - {} ppm is not within {}% of the {} ppm reference gas.",
                    avg, SPAN_TOLERANCE_PCT, ppm
                ),
                _ => {}
            }
        }
        println!("");
        println!("Nothing was written to the sensor.");
        park().await;
    }

    if !ARM {
        println!("Preconditions PASSED, but ARM = false, so nothing was written.");
        println!(
            "This run was a dry run. To actually apply {}, edit",
            action_name(ACTION)
        );
        println!("`src/bin/mhz19_calibrate.rs`, set `const ARM: bool = true;`, and re-run.");
        println!("The soak will run again, which is what you want for zero/span.");
        park().await;
    }

    // ---- phase 3: apply ---------------------------------------------------
    println!("ARMED - applying {}", action_name(ACTION));
    println!("");

    // Take ABC out of the loop first, otherwise a fresh manual zero is drifted
    // back within a day. Skipped when ABC itself is the requested action.
    if ABC_OFF_BEFORE_CAL && !matches!(ACTION, Action::Abc(_)) {
        let sent = sensor.set_auto_calibration(false).await;
        sensor = apply_and_listen(sensor, "ABC off (0x79)", sent).await;
    }

    match ACTION {
        Action::Zero => {
            let sent = sensor.calibrate_zero().await;
            let _ = apply_and_listen(sensor, "zero point (0x87)", sent).await;
        }
        Action::Span(ppm) => {
            let sent = sensor.calibrate_span(ppm).await;
            let _ = apply_and_listen(sensor, "span point (0x88)", sent).await;
        }
        Action::Abc(enabled) => {
            let sent = sensor.set_auto_calibration(enabled).await;
            let label = if enabled { "ABC on (0x79)" } else { "ABC off (0x79)" };
            let _ = apply_and_listen(sensor, label, sent).await;
        }
        Action::Range(ppm) => {
            // Unlike the calibration commands, the range command is answered.
            match with_timeout(SERIAL_TIMEOUT, sensor.set_detection_range(ppm)).await {
                Ok(Ok(())) => println!("  {:<26} acked: {:02X?}", "range (0x99)", sensor.last_response()),
                Ok(Err(e)) => println!("  {:<26} SEND FAILED: {:?}", "range (0x99)", e),
                Err(_) => println!("  {:<26} sent, no reply", "range (0x99)"),
            }
        }
    }

    println!("");
    println!("Done. Power-cycle the sensor, then run the read-only `mhz19` example to");
    println!("confirm. Note the stored baseline now depends on the environment you ran");
    println!("this in, so re-check the reading before trusting it.");
    if ABC_OFF_BEFORE_CAL && !matches!(ACTION, Action::Abc(_)) {
        println!("ABC is now OFF; use Action::Abc(true) if you want it back on.");
    }

    park().await
}
