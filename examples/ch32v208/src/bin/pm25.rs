//! Generic PM2.5 particulate sensor reader (Plantower PMSx003 family) for the
//! CH32V208, written in embassy style.
//!
//! Uses **our own** `edrv-pmsx003` driver, in its async flavour. The blocking
//! flavour is exercised by the companion `pm25_blocking` example.
//!
//! Wiring - note the sensor only ever *transmits*, so PA9 is unused:
//!
//! ```text
//!   PMS5003/PMS7003      CH32V208
//!   VCC  (5V)  <----  5V    (the fan + laser need a real 5V, ~100 mA)
//!   GND        <----  GND
//!   TX         ---->  PA10  (USART1_RX)
//!   RX         <----  (not needed; sensor streams automatically)
//! ```
//!
//! Covers the Plantower PMSx003 family: PMS1003, PMS3003, PMS5003, PMS7003,
//! PMS9003. They all use the same protocol:
//!
//! * 9600 8N1, the sensor pushes frames continuously with no command (the unit
//!   used here sent one every ~690 ms).
//! * 32-byte frame, header `0x42 0x4D`, then `0x00 0x1C` (28 = payload length),
//!   then 13 big-endian `u16` values, then a big-endian `u16` checksum in the
//!   last two bytes.
//! * Checksum = sum of the first 30 bytes (mod 2^16).
//!
//! ```text
//!   [0..2]   0x42 0x4D header
//!   [2..4]   frame length, 28
//!   [4..6]   PM1.0  (CF=1, standard particulate)
//!   [6..8]   PM2.5  (CF=1)
//!   [8..10]  PM10   (CF=1)
//!   [10..12] PM1.0  (atmospheric environment)
//!   [12..14] PM2.5  (atmospheric environment)
//!   [14..16] PM10   (atmospheric environment)
//!   [16..28] particle counts per 0.1 L for >0.3, >0.5, >1.0, >2.5, >5.0, >10 um
//!   [28..30] version / error code
//!   [30..32] checksum
//! ```
//!
//! Unlike the MH-Z19 this is a *stream* protocol, so the reader has to
//! re-synchronise on the `0x42 0x4D` header instead of assuming it is already
//! frame-aligned. A plain "read 32 bytes and check the header" loop can latch a
//! permanent byte offset and then report every frame as invalid forever.
//! `edrv-pmsx003` owns that resynchronisation now, including the awkward
//! `0x42 0x42 0x4D` case, and simply skips frames that fail validation; that is
//! why this example no longer has a "bad frame" counter of its own.
//!
//! # The module used to develop this example
//!
//! **The exact model could not be confirmed - the label on the module was torn
//! off.** What is actually known, from the bytes it sends:
//!
//! * It speaks the PMSx003 family frame format exactly, so any driver for this
//!   family works with it. Which family member it is does not matter to the
//!   driver: PMS1003/3003/5003/7003/9003 differ in measurement range, accuracy
//!   and physical size, **not** in frame layout.
//! * `PM1.0` / `PM2.5` / `PM10` are self-consistent (`PM1.0 <= PM2.5 <= PM10`),
//!   stable across frames, and were confirmed to move when a real aerosol source
//!   was introduced, so those three fields are trustworthy.
//! * The six cumulative particle-count bins are **not real on this unit**. They
//!   break the monotonicity that cumulative bins must satisfy: the `>2.5 um`
//!   count exceeds the `>1.0 um` count, and the `>10 um` count exceeds the
//!   `>5.0 um` count, in every frame. The frame checksum stays valid, so this is
//!   the module, not the decode. Do not use those fields.
//! * The version field reads `0x8000` instead of the more usual `0x0000`, and the
//!   CF=1 and atmospheric triples are byte-identical.
//!
//! Together that points at a compatible/clone module, or a variant that simply
//! does not populate the count and atmospheric fields. The monitor below warns
//! once if it sees the violation, and on a genuine module the check just passes
//! silently.
//!
//! This demo is read-only and prints enough raw detail to confirm the values are
//! real: the packet checksum, the frame interval, the field ordering, and the
//! full raw frame on every packet (`PRINT_RAW`, on by default).
//!
//! Two debug switches at the top of the file:
//!
//! * `PRINT_RAW` - hexdump every frame. Off gives a one-line-per-second summary.
//! * `RAW_SNIFF` - drop all framing and hexdump whatever bytes arrive. This is
//!   what to reach for when nothing decodes at all, because the normal reader
//!   cannot show you a single byte if the `0x42 0x4D` header never appears.
//!   It runs before the driver is constructed, on the bare USART.
//!
//! If your sensor is an SDS011 (Nova) or another type that needs a wake-up /
//! "active mode" command before it streams, this will sit at "no frame" and you
//! need a different driver - tell me the model and I will add it.

#![no_std]
#![no_main]

#[path = "../uart_io.rs"]
mod uart_io;

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant};
use edrv_pmsx003::{Frame, Pmsx003};
use hal::usart::{self, UartRx};
use hal::{bind_interrupts, peripherals, print, println};
use panic_halt as _;
use uart_io::UartRxIo;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

/// PMSx003 family baud rate.
const BAUD: u32 = 9600;

/// Identification of the attached module, printed in the banner so the serial
/// log carries it next to the numbers.
///
/// The label was torn off, so this stays honest rather than guessing a part
/// number: the protocol identifies the family, not the model. See the module
/// documentation above for what was actually measured on this unit.
const SENSOR_ID: &str = "Plantower PMSx003-compatible (exact model unknown: label removed)";

/// The sensor streams continuously (the unit tested here sent a frame every
/// ~690 ms); if nothing arrives in this long, something is
/// wrong (wiring, power, or a sensor that needs a wake-up command).
///
/// The driver itself never times out - `read_frame` keeps hunting for a valid
/// header forever - so this timeout is what turns a silent sensor into a
/// message instead of a hang.
const FRAME_TIMEOUT: Duration = Duration::from_secs(3);

/// Print the raw 32-byte frame for **every** frame that decodes.
///
/// Leave this on while bringing a sensor up: it is only ~1 line/s, and it is the
/// only way to tell a decode problem from a sensor problem. Turn it off once the
/// readings are trusted.
const PRINT_RAW: bool = true;

/// Debug escape hatch: ignore the PMS framing completely and hexdump each burst
/// of bytes the UART delivers.
///
/// Use this when nothing decodes at all, because the normal reader cannot show
/// you anything if the `0x42 0x4D` header never appears - which is exactly the
/// case for a wrong model, a wrong baud rate, or inverted logic. Takes over the
/// main loop when `true`.
const RAW_SNIFF: bool = false;

/// Warn if this many consecutive frames are all zero.
const ZERO_RUN_WARN: u32 = 5;

/// Rough PM2.5 band, for eyeballing whether a number is reasonable indoors.
fn pm25_band(ug: u16) -> &'static str {
    match ug {
        0..=12 => "good",
        13..=35 => "moderate",
        36..=55 => "unhealthy for sensitive",
        56..=150 => "unhealthy",
        151..=250 => "very unhealthy",
        _ => "hazardous",
    }
}

/// True when every particulate reading is zero.
fn all_zero(frame: &Frame) -> bool {
    frame.pm1_0() == 0 && frame.pm2_5() == 0 && frame.pm10() == 0
}

/// Physical plausibility: the fractions must not exceed the totals.
fn ordering_ok(frame: &Frame) -> bool {
    frame.pm1_0() <= frame.pm2_5() && frame.pm2_5() <= frame.pm10()
}

/// Dump bytes as contiguous hex (`raw: 42 4D 00 1C ...`), which is easy to read
/// and to paste into a decoder or a spreadsheet.
fn print_raw(label: &str, bytes: &[u8]) {
    print!("{}", label);
    for byte in bytes {
        print!(" {:02X}", byte);
    }
    println!("");
}

/// Hexdump every burst the UART delivers, with no framing assumptions at all.
///
/// The normal reader can only report "no frame" when the `0x42 0x4D` header
/// never shows up, which is exactly the situation where you most need to see the
/// wire. This prints whatever actually arrives. It consumes the raw receiver, so
/// it is only reachable before the driver takes it over.
async fn sniff(mut rx: UartRx<'static, peripherals::USART1, hal::mode::Async>) -> ! {
    println!("RAW SNIFF mode: framing disabled, dumping each burst as it arrives.");
    println!("A healthy PMSx003 burst starts with: 42 4D 00 1C ...");
    println!("");

    let mut buf = [0u8; 128];
    let mut burst: u32 = 0;

    loop {
        // `read_until_idle` returns as soon as the line goes quiet, so each line
        // corresponds to one natural burst on the wire.
        match with_timeout(FRAME_TIMEOUT, rx.read_until_idle(&mut buf)).await {
            Err(_) => println!("[sniff] nothing received for {} s", FRAME_TIMEOUT.as_secs()),
            Ok(Err(e)) => println!("[sniff] uart error: {:?}", e),
            Ok(Ok(n)) => {
                burst += 1;
                print!("[sniff #{} len {}]", burst, n);
                for byte in &buf[..n] {
                    print!(" {:02X}", byte);
                }
                println!("");
            }
        }
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 PMSx003 PM2.5 sensor");
    println!("  USART1  PA10=RX <- sensor TX");
    println!("  {} 8N1, sensor streams continuously (~0.7 s/frame here)", BAUD);
    println!("  driver: edrv-pmsx003 (async)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("sensor: {}", SENSOR_ID);
    println!("");

    let mut cfg = usart::Config::default();
    cfg.baudrate = BAUD;
    // A streaming sensor is expected to overrun if we ever fall behind a whole
    // frame; do not turn that into a hard read error.
    cfg.detect_previous_overrun = false;

    // RX-only: the sensor never needs to be told anything. USART1 RX is
    // DMA1_CH5 on this part.
    let rx = UartRx::new(p.USART1, Irqs, p.PA10, p.DMA1_CH5, cfg).unwrap();

    // Debug escape hatch: never returns, so the normal reader below is skipped.
    if RAW_SNIFF {
        sniff(rx).await;
    }

    let mut sensor = Pmsx003::new(UartRxIo::new(rx));

    let mut frames_ok: u32 = 0;
    let mut zero_run: u32 = 0;
    let mut bins_warned = false;
    let mut last_frame: Option<Instant> = None;

    loop {
        match with_timeout(FRAME_TIMEOUT, sensor.read_frame()).await {
            Err(_) => {
                println!(
                    "no frame for {} s - check 5V, GND, sensor TX -> PA10.",
                    FRAME_TIMEOUT.as_secs()
                );
                println!("  (a sensor that needs a wake-up/active-mode command will also sit here)");
            }
            Ok(Err(e)) => {
                println!("uart error: {:?}", e);
            }
            Ok(Ok(frame)) => {
                frames_ok += 1;

                let gap = match last_frame {
                    Some(prev) => {
                        let ms = (Instant::now() - prev).as_millis();
                        last_frame = Some(Instant::now());
                        ms
                    }
                    None => {
                        last_frame = Some(Instant::now());
                        0
                    }
                };

                println!(
                    "#{:<4} PM1.0 {:>4}  PM2.5 {:>4}  PM10 {:>4} ug/m3   atm {}/{}/{}   >0.3um {:<5} {:<26} +{}ms  [{} frames]",
                    frames_ok,
                    frame.pm1_0(),
                    frame.pm2_5(),
                    frame.pm10(),
                    frame.pm1_0_atm(),
                    frame.pm2_5_atm(),
                    frame.pm10_atm(),
                    frame.bins()[0],
                    pm25_band(frame.pm2_5_atm()),
                    gap,
                    frames_ok
                );

                // Confirmation aids: the ordering check catches a scrambled
                // decode, and the raw dump lets you verify the header/checksum
                // by hand.
                if !ordering_ok(&frame) {
                    println!(
                        "  !! PM1.0 <= PM2.5 <= PM10 violated ({} {} {}) - decode or sensor suspect",
                        frame.pm1_0(),
                        frame.pm2_5(),
                        frame.pm10()
                    );
                }
                if frame.pm10() > 1000 {
                    println!("  !! PM10 {} ug/m3 is implausible for ambient air", frame.pm10());
                }

                if all_zero(&frame) {
                    zero_run += 1;
                    if zero_run == ZERO_RUN_WARN {
                        println!(
                            "  !! {} zero frames in a row - sensor may be asleep or the fan stalled",
                            zero_run
                        );
                    }
                } else {
                    zero_run = 0;
                }

                if PRINT_RAW {
                    print_raw("  raw:", frame.as_bytes());
                    let b = frame.bins();
                    println!(
                        "  bins: >0.3 {}  >0.5 {}  >1.0 {}  >2.5 {}  >5.0 {}  >10 {}   version {:#06X}",
                        b[0], b[1], b[2], b[3], b[4], b[5], frame.version()
                    );
                }

                // Warn once, not on every frame: if the bins do not accumulate,
                // they are not measurements and the mass fields are what matter.
                if !bins_warned && !frame.bins_monotonic() {
                    bins_warned = true;
                    println!("  !! cumulative particle bins are NOT monotonic (a >2.5um count");
                    println!("     cannot exceed the >1.0um count). These count fields are not");
                    println!("     real on this unit - use PM1.0/PM2.5/PM10 instead.");
                }
            }
        }
    }
}
