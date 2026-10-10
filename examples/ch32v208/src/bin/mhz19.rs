//! MH-Z19B CO2 sensor demo + diagnostics for the CH32V208, in embassy style.
//!
//! Uses **our own** `edrv-mhz19` driver, in its async flavour. The blocking
//! flavour is exercised by the companion `mhz19_blocking` example.
//!
//! Wiring:
//!
//! ```text
//!   MH-Z19B           CH32V208
//!   -------           --------
//!   VCC  (5V)  <----  5V     (the sensor needs 5V, ~150 mA pulses)
//!   GND        <----  GND
//!   TX         ---->  PA10   (USART1_RX)
//!   RX         <----  PA9    (USART1_TX)
//! ```
//!
//! The MH-Z19B is an NDIR CO2 sensor with a 9600 8N1 UART protocol. Both
//! directions use 9-byte packets:
//!
//! ```text
//!   [0] 0xFF  start byte
//!   [1] 0x01  sensor address (command) / command echo (response)
//!   [2] command
//!   [3..8] data, then checksum in [8]
//! ```
//!
//! Checksum = `255 - sum(bytes[1..8]) + 1`.
//!
//! # Provisioning the UART
//!
//! `edrv-mhz19` is generic over `embedded_io_async::Read + Write`, so ch32-hal's
//! USART is bridged to it by the small adapter in `uart_io.rs`. The HAL is not
//! modified for this.
//!
//! # Why the reading can jump and then stick
//!
//! A checksum-valid value is a real sensor value: a misparsed packet fails the
//! start-byte/echo/checksum checks and is reported as an error, it does not turn
//! into a plausible-looking number. So a sudden jump to a stuck value is the
//! sensor's own state, not a decode bug.
//!
//! The upstream Arduino driver (`MHZ19` by strange-v) documents the most common
//! cause. Command `0x86` returns the *limited/clipped* value and command `0x85`
//! the *unlimited* value. From `MHZ19.cpp`:
//!
//! > "Limited CO2 stays at 410ppm during reset, so comparing unlimited which
//! > instead shows an abnormal value, reset duration can be found."
//!
//! So a `0x86` reading pinned at **410 ppm** while `0x85` reads something very
//! different means the sensor has **reset and is recovering** - the number is
//! garbage until it settles. Upstream ships a "filter mode" that does exactly
//! this comparison.
//!
//! **This board's sensor has no `0x85`**, so the cross-check cannot run here:
//! its command set is only `0x86`, `0x87`, `0x88`, `0x79` and `0x99`, and
//! `edrv-mhz19` mirrors the commands this family actually answers. A pinned
//! 410 ppm reading is therefore reported as *suspicious* below rather than
//! *confirmed*, and the raw packet is printed so the byte pattern can be checked
//! by hand. On an MH-Z19 that does implement `0x85`, the same signature plus an
//! abnormal unlimited reading is what confirms a reset.
//!
//! If the sensor keeps resetting, suspect power: the NDIR lamp draws ~150 mA
//! pulses, and a weak 5 V rail or long thin wires brown the sensor out. Also
//! check the logic level - MH-Z19B RX has a high input threshold at 5 V VCC, so
//! a 3.3 V TX is marginal and causes intermittent command corruption (which is
//! what the occasional failed reads usually are). Note also that the upstream
//! README warns counterfeit MH-Z19s exist with poor ppm stability, and that ABC
//! needs ~24 h and a stable baseline before readings are trustworthy.
//!
//! The demo only ever reads, so it never changes stored calibration. The
//! destructive calibration commands live in the separate `mhz19_calibrate`
//! example, which refuses to run unless explicitly armed.

#![no_std]
#![no_main]

#[path = "../uart_io.rs"]
mod uart_io;

use ch32_hal as hal;
use edrv_mhz19::MHZ19;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use hal::usart::{self, Uart};
use hal::{bind_interrupts, peripherals, println};
use panic_halt as _;
use uart_io::UartIo;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

/// The sensor speaks 9600 8N1.
const MHZ19_BAUD: u32 = 9600;

/// How long to wait for a response before giving up on the exchange.
///
/// This sits above the driver, not inside it: `edrv-mhz19` has no notion of
/// time, so its futures must be wrapped in a timeout by the caller or a silent
/// sensor blocks forever.
const SERIAL_TIMEOUT: Duration = Duration::from_millis(500);

/// The sensor refreshes its measurement roughly every 5 s; polling faster just
/// returns the same value.
const POLL_PERIOD: Duration = Duration::from_secs(5);

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

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 MH-Z19B CO2 sensor");
    println!("  USART1  PA9=TX -> sensor RX");
    println!("          PA10=RX <- sensor TX");
    println!("  {} 8N1", MHZ19_BAUD);
    println!("  driver: edrv-mhz19 (async)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("Give the sensor ~3 minutes to warm up before trusting the value.");
    println!("");

    let mut cfg = usart::Config::default();
    cfg.baudrate = MHZ19_BAUD;

    // Uart::new takes RX before TX; REMAP is inferred as 0 from PA9/PA10.
    let (tx, rx) = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, cfg)
        .unwrap()
        .split();

    let mut sensor = MHZ19::new(UartIo::new(rx, tx));
    let start = Instant::now();

    let mut good: u32 = 0;
    let mut bad: u32 = 0;

    loop {
        let uptime = start.elapsed().as_secs();

        // `read_co2` sends `0x86` and validates the response; the timeout has to
        // come from here. See `SERIAL_TIMEOUT`.
        match with_timeout(SERIAL_TIMEOUT, sensor.read_co2()).await {
            Ok(Ok(reading)) => {
                good += 1;

                println!(
                    "[t={:>5}s] CO2: {:>4} ppm ({})  temp: {} C  acc: {}  [{} ok / {} bad]",
                    uptime,
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

                // `min_co2_ppm` is `response[6..8]`; the upstream driver notes it
                // needs further calculation before it is usable, so it is shown
                // as a diagnostic only.
                if reading.min_co2_ppm != 0 {
                    println!("             min CO2 field (diagnostic only): {}", reading.min_co2_ppm);
                }

                // Without `0x85` we cannot positively distinguish "the room
                // really is at 410 ppm" from "the sensor just reset", so say so
                // rather than picking one.
                if reading.co2_ppm == RESET_SIGNATURE_PPM {
                    println!(
                        "             note: {} ppm is also the reset signature (limited CO2 sits",
                        RESET_SIGNATURE_PPM
                    );
                    println!("             there while the sensor recovers). This part has no 0x85");
                    println!("             channel, so watch whether the value moves off it.");
                }
            }
            Ok(Err(e)) => {
                bad += 1;
                println!("[t={:>5}s] read failed: {:?}  [{} ok / {} bad]", uptime, e, good, bad);
                report_error(&e);
            }
            Err(_) => {
                bad += 1;
                println!(
                    "[t={:>5}s] no response within {:?}  [{} ok / {} bad]",
                    uptime, SERIAL_TIMEOUT, good, bad
                );
                println!("             nothing came back: check 5V, GND, sensor TX -> PA10,");
                println!("             and the 3.3V/5V logic level.");
            }
        }

        Timer::after(POLL_PERIOD).await;
    }
}

/// Turn a driver error into a hint about what is actually wrong on the bench.
fn report_error(e: &edrv_mhz19::Error<uart_io::UartError>) {
    use edrv_mhz19::Error;

    match e {
        Error::BadChecksum { .. } | Error::BadStartByte(_) | Error::WrongCommand { .. } => {
            println!("             framing is corrupt: usually noise or a marginal logic level.");
        }
        Error::Transport(_) => {
            println!("             USART error: wiring or power.");
        }
        Error::UnexpectedEof => {
            println!("             the response was cut short mid-packet: wiring or baud rate.");
        }
        Error::InvalidArgument => println!("             the driver rejected an argument."),
    }
}
