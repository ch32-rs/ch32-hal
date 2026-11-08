#![no_std]
#![no_main]

//! Drive the USART1 receive error paths by bit-banging PA9 as a GPIO: a
//! framing error (stop bit low), a break, and a parity error, checking that
//! `UartRx` reports the matching `Error` and recovers on the next frame.
//!
//! PA9 must be jumpered to PA10. Results go to defmt/RTT and to
//! `UART_ERRORS_RESULT` in RAM.
//!
//! The technique follows embassy-rp's `uart_dma`/`uart_buffered` tests: no
//! second UART is needed because the test writes the frame itself, one bit at
//! a time. The bit clock is spun off the SysTick counter rather than taken
//! from an embassy timer: a timer wake costs about 45 us on this core (no
//! I-cache, slow flash fetch), which at 1 kbaud stretches every bit by 4 %
//! and walks the last data samples into the stop bit. The counter runs at
//! HCLK/8, the same clock the USART's baud generator divides, so the
//! calibrated bit is exact rather than merely close.

use ch32_hal as hal;
use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output, Speed};
use ch32_hal::mode::Async;
use ch32_hal::usart::{self, DataBits, Parity, UartRx};
use ch32v467_examples::{report_check, report_set, report_static};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

report_static!(UART_ERRORS_RESULT);

/// Bytes used to calibrate the bit clock against the UART.
const CAL_BYTES: usize = 50;

/// Busy-wait for `ticks` SysTick counts. The counter runs at HCLK/8 and does
/// not wrap during a frame.
fn spin_ticks(ticks: u32) {
    let start = ch32_hal::pac::SYSTICK.cntl().read();
    while ch32_hal::pac::SYSTICK.cntl().read().wrapping_sub(start) < ticks {}
}

/// One frame to put on the line.
#[derive(Clone, Copy)]
struct Frame {
    value: u8,
    /// Parity bit to drive; `None` sends no parity bit.
    parity: Option<bool>,
    /// Drive the stop bit low, producing a framing error.
    stop_low: bool,
}

/// Write one frame; the caller must have calibrated `ticks_per_bit` against
/// the UART.
fn send_frame(pin: &mut Output<'_>, frame: Frame, ticks_per_bit: u32) {
    pin.set_low(); // start bit
    spin_ticks(ticks_per_bit);

    for bit in 0..8 {
        if frame.value & (1 << bit) != 0 {
            pin.set_high();
        } else {
            pin.set_low();
        }
        spin_ticks(ticks_per_bit);
    }

    if let Some(parity) = frame.parity {
        if parity {
            pin.set_high();
        } else {
            pin.set_low();
        }
        spin_ticks(ticks_per_bit);
    }

    if frame.stop_low {
        pin.set_low();
    } else {
        pin.set_high();
    }
    spin_ticks(ticks_per_bit);

    // Idle line before the next frame.
    pin.set_high();
    spin_ticks(ticks_per_bit);
}

fn send_break(pin: &mut Output<'_>, bits: u32, ticks_per_bit: u32) {
    pin.set_low();
    spin_ticks(ticks_per_bit * bits);
    pin.set_high();
    spin_ticks(ticks_per_bit);
}

/// Receive while `frame` goes out on the line. `want` is the DMA transfer
/// length: 1 lets a single good frame complete, anything larger leaves the
/// error path as the only way for the read to finish.
async fn read_frame<T: usart::Instance>(
    rx: &mut UartRx<'_, T, Async>,
    pin: &mut Output<'_>,
    want: usize,
    frame: Frame,
    ticks_per_bit: u32,
    buf: &mut [u8],
) -> Option<Result<(), usart::Error>> {
    with_timeout(
        Duration::from_millis(200),
        join(rx.read(&mut buf[..want]), async {
            send_frame(pin, frame, ticks_per_bit)
        }),
    )
    .await
    .ok()
    .map(|(result, ())| result)
}

/// A break has no dedicated `Error` variant; it must at least surface as a
/// framing error and not wedge the receiver.
async fn read_break<T: usart::Instance>(
    rx: &mut UartRx<'_, T, Async>,
    pin: &mut Output<'_>,
    ticks_per_bit: u32,
    buf: &mut [u8],
) -> Option<Result<(), usart::Error>> {
    with_timeout(
        Duration::from_millis(200),
        join(rx.read(&mut buf[..4]), async { send_break(pin, 12, ticks_per_bit) }),
    )
    .await
    .ok()
    .map(|(result, ())| result)
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut p = hal::init(Default::default());
    report_set!(UART_ERRORS_RESULT, 1);

    let mut config = usart::Config::default();
    config.baudrate = 1000;

    // Calibrate the bit clock against the UART's: the DMA transmits
    // back-to-back, so the elapsed time covers exactly `CAL_BYTES * 10` bit
    // times. Both clocks come from HCLK, so the counts are exact.
    let ticks_per_bit = {
        let mut tx = usart::UartTx::new::<0>(
            p.USART1.reborrow(),
            p.PA9.reborrow(),
            Irqs,
            p.DMA1_CH4.reborrow(),
            config,
        )
        .unwrap();
        let start = ch32_hal::pac::SYSTICK.cntl().read();
        tx.write(&[0u8; CAL_BYTES]).await.unwrap();
        tx.flush().await.unwrap();
        let ticks = ch32_hal::pac::SYSTICK.cntl().read().wrapping_sub(start);
        let ticks_per_bit = ticks / (CAL_BYTES as u32 * 10);
        defmt::info!("uart_errors: UART bit measured as {} ticks", ticks_per_bit);
        ticks_per_bit
    };

    // Drive the line high before the receiver is enabled: PA9 is a floating
    // input out of reset, and a receiver enabled on a floating line can lock
    // onto a bogus frame before the first real start bit arrives.
    let mut pin = Output::new(p.PA9, Level::High, Speed::Low);
    Timer::after_millis(2).await;

    let mut rx = UartRx::new::<0>(
        p.USART1.reborrow(),
        Irqs,
        p.PA10.reborrow(),
        p.DMA1_CH5.reborrow(),
        config,
    )
    .unwrap();
    Timer::after_millis(2).await;
    let mut buf = [0u8; 8];

    let clean = Frame {
        value: 0x5A,
        parity: None,
        stop_low: false,
    };
    let bad_stop = Frame {
        value: 0xA3,
        parity: None,
        stop_low: true,
    };
    let next = Frame {
        value: 0x42,
        parity: None,
        stop_low: false,
    };

    // 0. Baseline: an ordinary frame is received, so a later error is
    //    attributable to the injected defect and not to the bit-banging.
    let result = read_frame(&mut rx, &mut pin, 1, clean, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Ok(()))),
        1,
        "clean frame lost"
    );
    report_check!(UART_ERRORS_RESULT, buf[0] == 0x5A, 2, "clean frame data");
    defmt::info!("uart_errors: clean frame pass");

    // 1. Framing error: the stop bit is sampled low.
    let result = read_frame(&mut rx, &mut pin, 4, bad_stop, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Err(usart::Error::Framing))),
        3,
        "expected a framing error"
    );
    defmt::info!("uart_errors: framing error pass");

    // The receiver must come back after the error.
    let result = read_frame(&mut rx, &mut pin, 1, next, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Ok(()))),
        4,
        "no recovery after framing"
    );
    report_check!(UART_ERRORS_RESULT, buf[0] == 0x42, 5, "recovery frame data");
    defmt::info!("uart_errors: framing recovery pass");

    // 2. A break is a long low; the USART reports it as a framing error too.
    let result = read_break(&mut rx, &mut pin, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Err(usart::Error::Framing))),
        6,
        "expected a break to report framing"
    );
    let result = read_frame(&mut rx, &mut pin, 1, next, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Ok(()))),
        7,
        "no recovery after break"
    );
    defmt::info!("uart_errors: break pass");
    report_set!(UART_ERRORS_RESULT, 2);

    // 3. Parity error: 8E1 with a wrong parity bit. On this USART the parity
    //    bit is the last bit of the 8/9-bit data word (RM §17.2), so eight
    //    data bits plus parity are selected with a 9-bit word; `DataBits8`
    //    plus parity would put the parity in place of the eighth data bit.
    //    The receiver is recreated so no state from the 8N1 phases is left.
    config.data_bits = DataBits::DataBits9;
    config.parity = Parity::ParityEven;
    drop(rx);
    Timer::after_millis(2).await;
    let mut rx = UartRx::new::<0>(
        p.USART1.reborrow(),
        Irqs,
        p.PA10.reborrow(),
        p.DMA1_CH5.reborrow(),
        config,
    )
    .unwrap();
    Timer::after_millis(2).await;

    let even = Frame {
        value: 0x00,
        parity: Some(false),
        stop_low: false,
    };
    let odd = Frame {
        value: 0x00,
        parity: Some(true),
        stop_low: false,
    };
    let odd_data = Frame {
        value: 0x01,
        parity: Some(true),
        stop_low: false,
    };

    let result = read_frame(&mut rx, &mut pin, 1, even, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Ok(()))),
        10,
        "clean parity frame lost"
    );
    report_check!(UART_ERRORS_RESULT, buf[0] == 0x00, 11, "clean parity frame data");
    defmt::info!("uart_errors: clean parity frame pass");

    let result = read_frame(&mut rx, &mut pin, 4, odd, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Err(usart::Error::Parity))),
        12,
        "expected a parity error"
    );
    defmt::info!("uart_errors: parity error pass");

    let result = read_frame(&mut rx, &mut pin, 1, odd_data, ticks_per_bit, &mut buf).await;
    report_check!(
        UART_ERRORS_RESULT,
        matches!(result, Some(Ok(()))),
        13,
        "no recovery after parity"
    );
    report_check!(UART_ERRORS_RESULT, buf[0] == 0x01, 14, "recovery parity data");
    defmt::info!("uart_errors: parity recovery pass");
    report_set!(UART_ERRORS_RESULT, 3);

    report_set!(UART_ERRORS_RESULT, 0xA5A5_FFFF);
    defmt::info!("uart_errors: PASS");
    loop {
        Timer::after_secs(1).await;
    }
}
