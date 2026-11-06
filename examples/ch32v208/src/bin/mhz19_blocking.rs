//! MH-Z19B CO2 sensor demo, **blocking** flavour, for the CH32V208.
//!
//! This is the sync counterpart of the `mhz19` example: same sensor, same
//! `edrv-mhz19` driver, but the blocking API and a polled USART - no embassy
//! executor, no async, no DMA.
//!
//! Wiring is identical to `mhz19`:
//!
//! ```text
//!   MH-Z19B           CH32V208
//!   VCC  (5V)  <----  5V
//!   GND        <----  GND
//!   TX         ---->  PA10   (USART1_RX)
//!   RX         <----  PA9    (USART1_TX)
//! ```
//!
//! # The tradeoff
//!
//! Blocking is simpler to reason about - the code reads top to bottom, and there
//! is no executor to schedule - but **a read that never gets an answer blocks
//! forever**. There is no timeout to be had without a timer, so a disconnected
//! sensor just stops the program at the first `read_co2` instead of reporting
//! it. The async example is the one to use when that matters, because it can
//! wrap the read in `with_timeout`.
//!
//! That is also why this example prints a sample counter rather than an uptime:
//! uptime needs a clock, and a clock needs the same timer machinery that the
//! blocking path deliberately does without.

#![no_std]
#![no_main]

#[path = "../uart_io.rs"]
mod uart_io;

use ch32_hal as hal;
use edrv_mhz19::blocking::MHZ19;
use hal::delay::Delay;
use hal::usart::{self, Uart};
use hal::println;
use panic_halt as _;
use uart_io::BlockingUartIo;

/// The sensor speaks 9600 8N1.
const MHZ19_BAUD: u32 = 9600;

/// The sensor refreshes its measurement roughly every 5 s; polling faster just
/// returns the same value.
const POLL_PERIOD_MS: u32 = 5000;

/// `0x86` reports this while the sensor is recovering from a reset.
const RESET_SIGNATURE_PPM: u16 = 410;

/// Rough indoor-air guidance for a ppm reading (not part of the sensor spec).
fn air_quality(ppm: u16) -> &'static str {
    match ppm {
        0..=400 => "fresh outdoor air",
        401..=1000 => "good",
        1001..=2000 => "stuffy - ventilate",
        2001..=5000 => "poor - ventilate now",
        _ => "very poor",
    }
}

/// Turn a driver error into a hint about what is actually wrong on the bench.
fn report_error(e: &edrv_mhz19::Error<uart_io::UartError>) {
    use edrv_mhz19::Error;

    match e {
        Error::BadChecksum { .. } | Error::BadStartByte(_) | Error::WrongCommand { .. } => {
            println!("             framing is corrupt: usually noise or a marginal logic level.");
        }
        Error::Transport(_) => println!("             USART error: wiring or power."),
        Error::UnexpectedEof => {
            println!("             the response was cut short mid-packet: wiring or baud rate.");
        }
        Error::InvalidArgument => println!("             the driver rejected an argument."),
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 MH-Z19B CO2 sensor");
    println!("  USART1  PA9=TX -> sensor RX");
    println!("          PA10=RX <- sensor TX");
    println!("  {} 8N1, polled (no DMA, no executor)", MHZ19_BAUD);
    println!("  driver: edrv-mhz19 (blocking)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("Give the sensor ~3 minutes to warm up before trusting the value.");
    println!("");
    println!("Note: this blocking demo has no timeout, so it stops here if the");
    println!("sensor is silent. The `mhz19` example reports that case instead.");
    println!("");

    let mut cfg = usart::Config::default();
    cfg.baudrate = MHZ19_BAUD;

    // `new_blocking` takes RX before TX, and needs neither IRQs nor DMA.
    let (tx, rx) = Uart::new_blocking(p.USART1, p.PA10, p.PA9, cfg)
        .unwrap()
        .split();

    let mut sensor = MHZ19::new(BlockingUartIo::new(rx, tx));
    let mut delay = Delay;

    let mut good: u32 = 0;
    let mut bad: u32 = 0;
    let mut sample: u32 = 0;

    loop {
        sample += 1;

        match sensor.read_co2() {
            Ok(reading) => {
                good += 1;

                println!(
                    "#{:<4} CO2: {:>4} ppm ({})  temp: {} C  acc: {}  [{} ok / {} bad]",
                    sample,
                    reading.co2_ppm,
                    air_quality(reading.co2_ppm),
                    reading.temperature_c,
                    reading.status,
                    good,
                    bad
                );

                // Always show the raw packet: this is the evidence that the
                // value was decoded from what the sensor really sent.
                println!("             raw 0x86: {:02X?}", sensor.last_response());

                if reading.min_co2_ppm != 0 {
                    println!("             min CO2 field (diagnostic only): {}", reading.min_co2_ppm);
                }

                // This part has no `0x85` channel, so a reset cannot be told
                // apart from a genuinely low reading; see the `mhz19` example.
                if reading.co2_ppm == RESET_SIGNATURE_PPM {
                    println!(
                        "             note: {} ppm is also the reset signature - watch whether",
                        RESET_SIGNATURE_PPM
                    );
                    println!("             the value moves off it.");
                }
            }
            Err(e) => {
                bad += 1;
                println!("#{:<4} read failed: {:?}  [{} ok / {} bad]", sample, e, good, bad);
                report_error(&e);
            }
        }

        delay.delay_ms(POLL_PERIOD_MS);
    }
}
