//! USART1 echo-back (loopback) test for the CH32V208, written in embassy style.
//!
//! Wiring:
//!
//! ```text
//!   PA9  (USART1_TX) --jumper-- PA10 (USART1_RX)
//!   GND  -- common ground with whatever you attach later
//! ```
//!
//! With TX jumpered straight to RX, everything the chip transmits is fed back
//! into its own receiver. This demo therefore does not blindly echo what it
//! receives (that would saturate the line in an endless feedback loop). It
//! sends a known pattern, reads the echo back and compares the two:
//!
//! * `PASS` - every byte came back bit-for-bit, so TX, RX, the pin mux and the
//!   baud rate are all correct.
//! * `TIMEOUT` - nothing came back; the jumper is missing, or the pins are not
//!   actually muxed to USART1.
//! * `FAIL` - the echo came back short or corrupted (mismatch position and both
//!   byte values are printed), which usually means a bad jumper, wiring noise,
//!   or a baud-rate mismatch at the other end.
//!
//! The test pattern is a full `0x00..=0xFF` sweep, so every bit position is
//! exercised in every byte value, repeated once per round.
//!
//! If you later attach a USB-TTL adapter instead (adapter RX -> PA9, adapter TX
//! -> PA10) you want an interactive echo, not this self test - see the note at
//! the bottom of this file.

#![no_std]
#![no_main]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use hal::mode::Async;
use hal::usart::{self, Uart, UartRx, UartTx};
use hal::{bind_interrupts, peripherals, println};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

/// USART1 halves, PA9 = TX, PA10 = RX.
type Uart1Tx = UartTx<'static, peripherals::USART1, Async>;
type Uart1Rx = UartRx<'static, peripherals::USART1, Async>;

/// 9600 8N1. The other framing defaults (8 data bits, no parity, 1 stop bit)
/// are exactly N1, so only the rate needs changing.
const BAUD: u32 = 9600;

/// How long to wait for the echo before declaring the loop open.
const ECHO_TIMEOUT: Duration = Duration::from_millis(500);

/// Pause between test rounds.
const ROUND_PAUSE: Duration = Duration::from_secs(1);

/// One byte per value: 256 bytes, ~267 ms on the wire at 9600 8N1.
const PATTERN_LEN: usize = 256;

/// Send `pattern` and compare the echo that arrives on RX.
///
/// TX and RX are driven concurrently on purpose: at 9600 baud the 256-byte
/// burst takes ~267 ms, so the receiver has to be armed before the first byte
/// leaves, otherwise the DMA has nothing listening and the echo is lost.
async fn echo_round(tx: &mut Uart1Tx, rx: &mut Uart1Rx, pattern: &[u8], buf: &mut [u8], round: u32) -> bool {
    let sent = pattern.len();
    let outcome = with_timeout(
        ECHO_TIMEOUT,
        join(tx.write(pattern), rx.read_until_idle(&mut buf[..sent])),
    )
    .await;

    match outcome {
        Err(_) => {
            println!("round {}: TIMEOUT - no echo after {} bytes.", round, sent);
            println!("         check the PA9 -> PA10 jumper and that nothing else drives PA10.");
            false
        }
        Ok((Err(e), _)) => {
            println!("round {}: TX error: {:?}", round, e);
            false
        }
        Ok((_, Err(e))) => {
            println!("round {}: RX error: {:?}", round, e);
            false
        }
        Ok((Ok(()), Ok(n))) if n != sent => {
            println!("round {}: FAIL - sent {} bytes, got {} back.", round, sent, n);
            false
        }
        Ok((Ok(()), Ok(n))) => match buf[..n].iter().zip(pattern).position(|(got, want)| got != want) {
            Some(pos) => {
                println!(
                    "round {}: FAIL - mismatch at byte {}: sent 0x{:02X}, got 0x{:02X}",
                    round, pos, pattern[pos], buf[pos]
                );
                false
            }
            None => true,
        },
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("========================================");
    println!("  CH32V208 USART1 echo-back test");
    println!("  PA9=TX  PA10=RX  {} 8N1", BAUD);
    println!("  (PA9 jumpered to PA10)");
    println!("========================================");
    println!("CHIP: {}", hal::signature::chip_id().name());
    println!("");

    let mut cfg = usart::Config::default();
    cfg.baudrate = BAUD;

    // Uart::new takes RX before TX; REMAP is inferred as 0 from PA9/PA10.
    let (mut tx, mut rx) = Uart::new(p.USART1, p.PA10, p.PA9, Irqs, p.DMA1_CH4, p.DMA1_CH5, cfg)
        .unwrap()
        .split();

    let mut pattern = [0u8; PATTERN_LEN];
    for (i, byte) in pattern.iter_mut().enumerate() {
        *byte = i as u8;
    }
    let mut buf = [0u8; PATTERN_LEN];

    let mut round: u32 = 0;
    let mut passed: u32 = 0;
    let mut failed: u32 = 0;

    loop {
        round += 1;

        if echo_round(&mut tx, &mut rx, &pattern, &mut buf, round).await {
            passed += 1;
            println!(
                "round {}: PASS - {} bytes echoed intact ({} ok / {} bad)",
                round, PATTERN_LEN, passed, failed
            );
        } else {
            failed += 1;
            println!("         score: {} ok / {} bad", passed, failed);
        }

        Timer::after(ROUND_PAUSE).await;
    }
}

// ---------------------------------------------------------------------------
// Interactive echo instead of the loopback self test
// ---------------------------------------------------------------------------
//
// With a USB-TTL adapter (adapter RX -> PA9, adapter TX -> PA10) and the
// PA9/PA10 jumper REMOVED, echo typed bytes back with:
//
//     let mut byte = [0u8; 1];
//     loop {
//         rx.read(&mut byte).await.unwrap();
//         tx.write(&byte).await.unwrap();
//     }
//
// `rx.read_until_idle(&mut line)` reads a whole burst instead of one byte, which
// is nicer for line-oriented input. Do not use either form while PA9 and PA10
// are jumpered together: the echoed bytes would be received again immediately
// and the line would never go quiet.
