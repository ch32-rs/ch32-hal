//! PM2.5 particulate sensor reader (Plantower PMSx003 family), **blocking**
//! flavour, for the CH32V208.
//!
//! Sync counterpart of the `pm25` example: same sensor, same `edrv-pmsx003`
//! driver, but the blocking API and a polled USART - no embassy executor, no
//! DMA.
//!
//! Wiring - the sensor only ever *transmits*, so PA9 is unused:
//!
//! ```text
//!   PMS5003/PMS7003      CH32V208
//!   VCC  (5V)  <----  5V    (the fan + laser need a real 5V, ~100 mA)
//!   GND        <----  GND
//!   TX         ---->  PA10  (USART1_RX)
//!   RX         <----  (not needed; sensor streams automatically)
//! ```
//!
//! The frame format, the family quirks and the caveats about this particular
//! module's fake particle-count bins are all documented in the `pm25` example;
//! this file only shows the blocking call pattern.
//!
//! # The tradeoff
//!
//! The sensor streams frames on its own, so there is nothing to command and the
//! loop simply takes whatever arrives - which makes the blocking path a
//! perfectly good fit here. The catch is that `read_frame` waits forever for a
//! valid header: a sensor that is off, on the wrong baud rate or on a wrong
//! model just stops the program silently. The async example can time that out
//! and say so, which is why it is the better bring-up tool.

#![no_std]
#![no_main]

#[path = "../uart_io.rs"]
mod uart_io;

use ch32_hal as hal;
use edrv_pmsx003::{blocking::Pmsx003, Frame};
use hal::usart::{self, UartRx};
use hal::{print, println};
use panic_halt as _;
use uart_io::BlockingUartRxIo;

/// PMSx003 family baud rate.
const BAUD: u32 = 9600;

/// Print the raw 32-byte frame for every frame that decodes.
const PRINT_RAW: bool = true;

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

/// Dump bytes as contiguous hex, easy to read and to paste into a decoder.
fn print_raw(label: &str, bytes: &[u8]) {
    print!("{}", label);
    for byte in bytes {
        print!(" {:02X}", byte);
    }
    println!("");
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 PMSx003 PM2.5 sensor");
    println!("  USART1  PA10=RX <- sensor TX");
    println!("  {} 8N1, polled (no DMA, no executor)", BAUD);
    println!("  driver: edrv-pmsx003 (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    let mut cfg = usart::Config::default();
    cfg.baudrate = BAUD;
    // A streaming sensor is expected to overrun if we ever fall behind a whole
    // frame; do not turn that into a hard read error.
    cfg.detect_previous_overrun = false;

    // RX-only: the sensor never needs to be told anything.
    let rx = UartRx::new_blocking(p.USART1, p.PA10, cfg).unwrap();
    let mut sensor = Pmsx003::new(BlockingUartRxIo::new(rx));

    let mut frames_ok: u32 = 0;
    let mut zero_run: u32 = 0;
    let mut bins_warned = false;

    loop {
        // Frames that fail validation are skipped inside the driver, and there
        // is no timeout here: see the note at the top of the file.
        match sensor.read_frame() {
            Ok(frame) => {
                frames_ok += 1;

                println!(
                    "#{:<4} PM1.0 {:>4}  PM2.5 {:>4}  PM10 {:>4} ug/m3   atm {}/{}/{}   >0.3um {:<5} {:<26} [{} frames]",
                    frames_ok,
                    frame.pm1_0(),
                    frame.pm2_5(),
                    frame.pm10(),
                    frame.pm1_0_atm(),
                    frame.pm2_5_atm(),
                    frame.pm10_atm(),
                    frame.bins()[0],
                    pm25_band(frame.pm2_5_atm()),
                    frames_ok
                );

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
            Err(e) => println!("uart error: {:?}", e),
        }
    }
}
