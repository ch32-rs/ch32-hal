#![no_std]
#![no_main]

//! Self-test for `RingBufferedUartRx` on USART1: continuous DMA reception
//! across idle gaps, three 500 kbaud stress shapes (random chunks and gaps,
//! back-to-back, and short bursts at the idle-line boundary), overrun
//! recovery through `request_pause`/`set_count`, and the V4x7 clock refcount
//! on drop.
//!
//! PA9 (TX) must be jumpered to PA10 (RX): the test drives TX and reads the
//! bytes back through the ring buffer, so no host serial is needed. Results
//! go to defmt/RTT and to `UART_RING_RESULT` in RAM.

use core::ptr::addr_of_mut;

use ch32_hal as hal;
use ch32_hal::mode::Async;
use ch32_hal::usart::{self, RingBufferedUartRx, Uart};
use ch32_hal::{bind_interrupts, pac};
use ch32v467_examples::{report_check, report_fail, report_set, report_static};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

report_static!(UART_RING_RESULT);

static mut RING: [u8; 256] = [0; 256];

/// Read exactly `buf.len()` bytes, however many DMA wake-ups it takes.
async fn read_full<T: usart::Instance>(rx: &mut RingBufferedUartRx<'_, T>, buf: &mut [u8]) -> Result<(), usart::Error> {
    let mut filled = 0;
    while filled < buf.len() {
        filled += rx.read(&mut buf[filled..]).await?;
    }
    Ok(())
}

const STRESS_BAUD: u32 = 500_000;

/// Spacing between transmitted chunks.
#[derive(Clone, Copy)]
enum Gap {
    /// Back-to-back: no idle line between chunks.
    None,
    /// A fixed gap, chosen to always cross the idle-line threshold.
    Fixed(u64),
    /// A pseudo-random gap of `0..max` microseconds.
    RandomUpTo(u32),
}

/// RX-side failure of a stress phase.
enum StressError {
    Uart(usart::Error),
    /// The received byte diverged from the expected stream at this offset.
    Mismatch(usize),
}

/// Byte `offset` of the stream seeded with `seed`. Each phase uses its own
/// seed so data left over from a previous phase cannot accidentally match.
fn sequence_byte(offset: usize, seed: u32) -> u8 {
    let word = seed.wrapping_add((offset / 4) as u32);
    word.to_le_bytes()[offset % 4]
}

/// Drive `total` bytes in chunks of `chunk_min..=chunk_max` bytes separated by
/// `gap`. The chunk/gap patterns follow embassy-stm32's
/// `usart_rx_ringbuffered` and `lpuart_bbq_rx_continuous` tests.
async fn stress_tx<T: usart::Instance>(
    tx: &mut usart::UartTx<'_, T, Async>,
    seed: u32,
    total: usize,
    chunk_min: usize,
    chunk_max: usize,
    gap: Gap,
) -> Result<(), usart::Error> {
    let mut rng: u32 = seed ^ 0x2545_f491;
    let mut sent = 0usize;
    let mut chunk = [0u8; 256];

    while sent < total {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;

        let span = chunk_max - chunk_min + 1;
        let len = (chunk_min + rng as usize % span).min(total - sent);
        for (offset, byte) in chunk[..len].iter_mut().enumerate() {
            *byte = sequence_byte(sent + offset, seed);
        }
        tx.write(&chunk[..len]).await?;
        sent += len;

        match gap {
            Gap::None => {}
            Gap::Fixed(micros) => Timer::after_micros(micros).await,
            Gap::RandomUpTo(max) => Timer::after_micros(u64::from(rng >> 16) % u64::from(max)).await,
        }
    }
    tx.flush().await
}

/// Verify the stream positionally, in whatever sized pieces the ring buffer
/// hands out.
async fn stress_rx<T: usart::Instance>(
    rx: &mut RingBufferedUartRx<'_, T>,
    seed: u32,
    total: usize,
) -> Result<(), StressError> {
    let mut received = 0usize;
    let mut buf = [0u8; 64];

    while received < total {
        let len = rx.read(&mut buf).await.map_err(StressError::Uart)?;
        for (offset, &byte) in buf[..len].iter().enumerate() {
            let at = received + offset;
            let want = sequence_byte(at, seed);
            if byte != want {
                defmt::error!(
                    "uart_ring: mismatch at {}: got {=u8}, want {=u8}, chunk {}, sr {=u32:08x}",
                    at,
                    byte,
                    want,
                    len,
                    pac::USART1.statr().read().0
                );
                return Err(StressError::Mismatch(at));
            }
        }
        received += len;
    }
    Ok(())
}

/// Run one stress phase; failures are reported under `code..=code + 3`.
async fn run_stress<T: usart::Instance>(
    tx: &mut usart::UartTx<'_, T, Async>,
    rx: &mut RingBufferedUartRx<'_, T>,
    seed: u32,
    total: usize,
    chunk_min: usize,
    chunk_max: usize,
    gap: Gap,
    code: u32,
) {
    let result = with_timeout(
        Duration::from_secs(20),
        join(
            stress_tx(tx, seed, total, chunk_min, chunk_max, gap),
            stress_rx(rx, seed, total),
        ),
    )
    .await;
    report_check!(UART_RING_RESULT, result.is_ok(), code, "stress phase timed out");
    let (tx_result, rx_result) = result.unwrap();
    match rx_result {
        Ok(()) => {}
        Err(StressError::Mismatch(at)) => {
            defmt::error!("uart_ring: stress mismatch at byte {}", at);
            report_fail!(UART_RING_RESULT, code + 1, "stress stream mismatch");
        }
        Err(StressError::Uart(err)) => {
            defmt::error!("uart_ring: stress rx error {}", err);
            report_fail!(UART_RING_RESULT, code + 2, "stress rx error");
        }
    }
    report_check!(UART_RING_RESULT, tx_result.is_ok(), code + 3, "stress tx error");
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(Default::default());
    report_set!(UART_RING_RESULT, 1);

    let (mut tx, mut rx) = Uart::new::<0>(
        p.USART1,
        p.PA10,
        p.PA9,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Default::default(),
    )
    .unwrap()
    .split();

    let data = [0x81u8, 0x55, 0x00, 0xff, 0xa5, 0x5a, 0x7e, 0x42];
    let mut buf = [0u8; 64];

    // 0. One-shot path: `read_until_idle` stops its own DMA transfer when it
    //    sees the idle flag, so it must keep working even though the ISR no
    //    longer clears DMAR on IDLE.
    let result = with_timeout(
        Duration::from_millis(200),
        join(rx.read_until_idle(&mut buf[..data.len()]), tx.write(&data)),
    )
    .await;
    report_check!(UART_RING_RESULT, result.is_ok(), 1, "read_until_idle timed out");
    let (read, write) = result.unwrap();
    write.unwrap();
    report_check!(
        UART_RING_RESULT,
        read.unwrap() == data.len(),
        2,
        "read_until_idle length"
    );
    report_check!(UART_RING_RESULT, buf[..data.len()] == data, 3, "read_until_idle data");
    tx.flush().await.unwrap();
    defmt::info!("uart_ring: read_until_idle pass");

    let ring = unsafe { &mut *addr_of_mut!(RING) };
    let mut rx = rx.into_ring_buffered(ring);

    // 1. Basic path: TX -> jumper -> RX DMA -> ring buffer -> read().
    for round in 0u32..4 {
        let result = with_timeout(
            Duration::from_millis(200),
            join(read_full(&mut rx, &mut buf[..data.len()]), tx.write(&data)),
        )
        .await;
        report_check!(UART_RING_RESULT, result.is_ok(), 10 + round, "ring read timed out");
        let (read, write) = result.unwrap();
        write.unwrap();
        read.unwrap();
        report_check!(
            UART_RING_RESULT,
            buf[..data.len()] == data,
            14 + round,
            "ring read mismatch"
        );
        tx.flush().await.unwrap();
    }
    defmt::info!("uart_ring: basic pass");
    report_set!(UART_RING_RESULT, 2);

    // 2. Overrun: fill the 256-byte ring several times without reading. The
    //    next read() reports Overrun and pauses the channel; the read after
    //    that restarts from a reset buffer (request_pause + set_count).
    let flood = [0x5au8; 96];
    for _ in 0..8 {
        tx.write(&flood).await.unwrap();
    }
    tx.flush().await.unwrap();
    Timer::after_millis(2).await;

    let mut overrun = false;
    for _ in 0..4 {
        match with_timeout(Duration::from_millis(100), rx.read(&mut buf)).await {
            Ok(Err(usart::Error::Overrun)) => {
                overrun = true;
                break;
            }
            Ok(_) => {}
            Err(_) => break,
        }
    }
    report_check!(UART_RING_RESULT, overrun, 20, "expected a ring overrun");
    defmt::info!("uart_ring: overrun detected");

    let result = with_timeout(
        Duration::from_millis(200),
        join(read_full(&mut rx, &mut buf[..data.len()]), tx.write(&data)),
    )
    .await;
    report_check!(UART_RING_RESULT, result.is_ok(), 21, "timed out after overrun reset");
    let (read, write) = result.unwrap();
    write.unwrap();
    read.unwrap();
    report_check!(
        UART_RING_RESULT,
        buf[..data.len()] == data,
        22,
        "no data after overrun reset"
    );
    tx.flush().await.unwrap();
    defmt::info!("uart_ring: overrun recovery pass");
    report_set!(UART_RING_RESULT, 3);

    // 3. Stress at 500 kbaud in three traffic shapes: random chunks and gaps
    //    (embassy's `usart_rx_ringbuffered` pattern), back-to-back chunks with
    //    no idle line at all, and 61-byte bursts separated by a gap that
    //    always crosses the idle-line threshold. The ring buffer must keep up
    //    without losing a byte. The baud rate only exists in the shared BRR,
    //    so reconfiguring the TX half retunes the receiver too.
    let mut fast = usart::Config::default();
    fast.baudrate = STRESS_BAUD;
    tx.set_config(&fast).unwrap();

    run_stress(
        &mut tx,
        &mut rx,
        0x1111_1111,
        100_000,
        1,
        256,
        Gap::RandomUpTo(1_000),
        23,
    )
    .await;
    defmt::info!("uart_ring: stress random pass");

    run_stress(&mut tx, &mut rx, 0x2222_2222, 100_000, 256, 256, Gap::None, 27).await;
    defmt::info!("uart_ring: stress back-to-back pass");

    run_stress(&mut tx, &mut rx, 0x3333_3333, 61 * 1024, 61, 61, Gap::Fixed(100), 31).await;
    defmt::info!("uart_ring: stress idle-boundary pass");

    tx.set_config(&Default::default()).unwrap();
    report_set!(UART_RING_RESULT, 4);

    // 4. Clock refcount: the ring buffer owns the RX half's reference taken by
    //    `UartRx::new_inner` (`into_ring_buffered` forgets the `UartRx`).
    //    Dropping it must release that reference, not gate the clock under a
    //    live TX.
    drop(rx);
    report_check!(
        UART_RING_RESULT,
        pac::USART1.ctlr1().read().ue() && pac::USART1.ctlr1().read().te(),
        40,
        "dropping RX disabled the USART while TX was alive"
    );
    let alive = with_timeout(Duration::from_millis(100), tx.write(b"TX-ALIVE\r\n")).await;
    report_check!(
        UART_RING_RESULT,
        alive.is_ok(),
        41,
        "TX died after dropping the ring-buffered RX"
    );
    let _ = with_timeout(Duration::from_millis(100), tx.flush()).await;

    // Last user: the refcount must reach zero and disable the peripheral.
    drop(tx);
    let ctlr1 = pac::USART1.ctlr1().read();
    let ctlr3 = pac::USART1.ctlr3().read();
    report_check!(
        UART_RING_RESULT,
        !ctlr1.ue() && !ctlr3.dmar(),
        42,
        "the last user did not disable the USART"
    );

    report_set!(UART_RING_RESULT, 0xA5A5_FFFF);
    defmt::info!("uart_ring: PASS");
    loop {
        Timer::after_secs(1).await;
    }
}
