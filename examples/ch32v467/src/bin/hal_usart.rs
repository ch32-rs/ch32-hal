#![no_std]
#![no_main]

use ch32_hal::interrupt::typelevel::Binding;
use ch32_hal::mode::{Async, Blocking};
use ch32_hal::usart::{self, InterruptHandler, UartTx};
use ch32_hal::{bind_interrupts, pac, rcc, Peri};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

bind_interrupts!(struct Irqs {
    USART1 => InterruptHandler<ch32_hal::peripherals::USART1>;
    USART2 => InterruptHandler<ch32_hal::peripherals::USART2>;
    USART3 => InterruptHandler<ch32_hal::peripherals::USART3>;
    USART4 => InterruptHandler<ch32_hal::peripherals::USART4>;
    USART5 => InterruptHandler<ch32_hal::peripherals::USART5>;
    USART6 => InterruptHandler<ch32_hal::peripherals::USART6>;
    USART7 => InterruptHandler<ch32_hal::peripherals::USART7>;
    USART8 => InterruptHandler<ch32_hal::peripherals::USART8>;
    USART9 => InterruptHandler<ch32_hal::peripherals::USART9>;
    USART10 => InterruptHandler<ch32_hal::peripherals::USART10>;
});

report_static!(USART_RESULT);
#[no_mangle]
pub static mut USART_CHANNELS_PASSED: u32 = 0;
#[no_mangle]
pub static mut USART_SNAPSHOTS: [[u32; 4]; 10] = [[0; 4]; 10];

fn check(condition: bool, code: u32) {
    report_check!(USART_RESULT, condition, code, "USART_RESULT check failed");
}

#[allow(dead_code)]
fn check_rx_mappings(peripherals: ch32_hal::Peripherals) {
    macro_rules! mapping {
        ($instance:ident, $rx:ident, $tx:ident, $tx_dma:ident, $rx_dma:ident) => {
            drop(
                usart::Uart::new::<0>(
                    peripherals.$instance,
                    peripherals.$rx,
                    peripherals.$tx,
                    Irqs,
                    peripherals.$tx_dma,
                    peripherals.$rx_dma,
                    Default::default(),
                )
                .unwrap(),
            );
        };
    }
    mapping!(USART1, PA10, PA9, DMA1_CH4, DMA1_CH5);
    mapping!(USART2, PA3, PA2, DMA1_CH7, DMA1_CH6);
    mapping!(USART3, PB11, PB10, DMA1_CH2, DMA1_CH3);
    mapping!(USART4, PC11, PC10, DMA2_CH5, DMA2_CH3);
    mapping!(USART5, PD2, PC12, DMA2_CH4, DMA2_CH2);
    mapping!(USART6, PC1, PC0, DMA2_CH6, DMA2_CH7);
    mapping!(USART7, PC3, PC2, DMA2_CH8, DMA2_CH9);
    mapping!(USART8, PC5, PC4, DMA2_CH10, DMA2_CH11);
    mapping!(USART9, PB2, PB1, DMA1_CH8, DMA1_CH9);
    mapping!(USART10, PE1, PE0, DMA1_CH10, DMA1_CH11);
}

async fn exercise<T: usart::Instance>(
    mut peripheral: Peri<'_, T>,
    mut pin: Peri<'_, impl usart::TxPin<T>>,
    mut dma: Peri<'_, impl usart::TxDma<T>>,
    irq: impl Binding<T::Interrupt, InterruptHandler<T>> + Copy,
    registers: pac::usart::Usart,
    clock: u32,
    index: usize,
) {
    let data = [0x55u8; 64];
    for baudrate in [9600, 115200, (clock / 16).min(1_000_000)] {
        let mut config = usart::Config::default();
        config.baudrate = baudrate;
        let mut uart = UartTx::<T, Blocking>::new_blocking::<0>(peripheral.reborrow(), pin.reborrow(), config).unwrap();
        let divisor = (u64::from(clock) + u64::from(baudrate) / 2) / u64::from(baudrate);
        check(u64::from(registers.brr().read().0) == divisor, 10);
        let started = Instant::now();
        uart.blocking_write(&data).unwrap();
        uart.blocking_flush().unwrap();
        let elapsed = started.elapsed().as_micros();
        let expected = data.len() as u64 * 10 * divisor * 1_000_000 / u64::from(clock);
        check(elapsed >= expected * 95 / 100 && elapsed <= expected * 2 + 1000, 11);
    }
    let mut config = usart::Config::default();
    config.baudrate = 9600;
    let mut uart =
        UartTx::<T, Async>::new::<0>(peripheral.reborrow(), pin.reborrow(), irq, dma.reborrow(), config).unwrap();
    uart.write(&[]).await.unwrap();
    for _round in 0..3 {
        check(
            matches!(
                with_timeout(Duration::from_secs(1), uart.write(&data)).await,
                Ok(Ok(()))
            ),
            12,
        );
        check(
            matches!(with_timeout(Duration::from_millis(10), uart.flush()).await, Ok(Ok(()))),
            13,
        );
        check(registers.statr().read().tc(), 14);
        check(!registers.ctlr3().read().dmat() && !registers.ctlr1().read().tcie(), 15);
    }
    check(
        with_timeout(Duration::from_millis(1), uart.write(&data)).await.is_err(),
        16,
    );
    check(!registers.ctlr3().read().dmat(), 17);
    check(
        matches!(with_timeout(Duration::from_millis(10), uart.flush()).await, Ok(Ok(()))),
        18,
    );
    uart.write(&[0x55]).await.unwrap();
    check(with_timeout(Duration::from_micros(10), uart.flush()).await.is_err(), 19);
    check(!registers.ctlr1().read().tcie(), 20);
    check(
        matches!(with_timeout(Duration::from_millis(10), uart.flush()).await, Ok(Ok(()))),
        21,
    );
    uart.write(&[0xa5, 0x5a]).await.unwrap();
    check(
        matches!(with_timeout(Duration::from_millis(10), uart.flush()).await, Ok(Ok(()))),
        22,
    );
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(USART_SNAPSHOTS[index]),
            [
                clock,
                registers.brr().read().0,
                registers.ctlr1().read().0,
                registers.ctlr3().read().0,
            ],
        );
        core::ptr::write_volatile(core::ptr::addr_of_mut!(USART_CHANNELS_PASSED), index as u32 + 1);
    }
    defmt::info!("USART{} TX PASS", index + 1);
}

#[cfg(feature = "usart-loopback")]
async fn loopback(
    peripheral: Peri<'_, ch32_hal::peripherals::USART1>,
    rx: Peri<'_, ch32_hal::peripherals::PA10>,
    tx: Peri<'_, ch32_hal::peripherals::PA9>,
    tx_dma: Peri<'_, ch32_hal::peripherals::DMA1_CH4>,
    rx_dma: Peri<'_, ch32_hal::peripherals::DMA1_CH5>,
) {
    let uart = usart::Uart::new::<0>(peripheral, rx, tx, Irqs, tx_dma, rx_dma, Default::default()).unwrap();
    let (mut tx, mut rx) = uart.split();
    let data = [0x81u8, 0x55, 0x00, 0xff, 0xa5, 0x5a, 0x7e, 0x42];
    let mut received = [0u8; 8];
    for _round in 0..16 {
        let result = with_timeout(
            Duration::from_millis(100),
            embassy_futures::join::join(rx.read(&mut received), tx.write(&data)),
        )
        .await;
        check(matches!(result, Ok((Ok(()), Ok(())))), 30);
        check(received == data, 31);
        tx.flush().await.unwrap();
    }
    let mut idle_buffer = [0u8; 32];
    let result = with_timeout(
        Duration::from_millis(100),
        embassy_futures::join::join(rx.read_until_idle(&mut idle_buffer), tx.write(&data)),
    )
    .await;
    check(matches!(result, Ok((Ok(8), Ok(())))), 32);
    check(idle_buffer[..8] == data, 33);
    tx.flush().await.unwrap();
    check(
        with_timeout(Duration::from_millis(2), rx.read(&mut received))
            .await
            .is_err(),
        34,
    );
    let result = with_timeout(
        Duration::from_millis(100),
        embassy_futures::join::join(rx.read(&mut received), tx.write(&data)),
    )
    .await;
    check(matches!(result, Ok((Ok(()), Ok(())))), 35);
    check(received == data, 36);
    drop(rx);
    check(pac::USART1.ctlr1().read().ue() && pac::USART1.ctlr1().read().te(), 37);
    tx.write(&data).await.unwrap();
    tx.flush().await.unwrap();
    defmt::info!("USART1 loopback PASS");
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(USART_RESULT, 1);
    qingke::riscv::asm::delay(2_000_000);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    let mut peripherals = ch32_hal::init(config);
    macro_rules! test_tx {
        ($instance:ident, $pin:ident, $dma:ident, $clock:ident, $index:expr) => {
            exercise(
                peripherals.$instance.reborrow(),
                peripherals.$pin.reborrow(),
                peripherals.$dma.reborrow(),
                Irqs,
                pac::$instance,
                rcc::clocks().$clock.0,
                $index,
            )
            .await;
        };
    }
    test_tx!(USART1, PA9, DMA1_CH4, pclk2, 0);
    test_tx!(USART2, PA2, DMA1_CH7, pclk1, 1);
    test_tx!(USART3, PB10, DMA1_CH2, pclk1, 2);
    test_tx!(USART4, PC10, DMA2_CH5, pclk1, 3);
    test_tx!(USART5, PC12, DMA2_CH4, pclk1, 4);
    test_tx!(USART6, PC0, DMA2_CH6, pclk1, 5);
    test_tx!(USART7, PC2, DMA2_CH8, pclk1, 6);
    test_tx!(USART8, PC4, DMA2_CH10, pclk1, 7);
    test_tx!(USART9, PB1, DMA1_CH8, pclk1, 8);
    test_tx!(USART10, PE0, DMA1_CH10, pclk1, 9);
    #[cfg(feature = "usart-loopback")]
    loopback(
        peripherals.USART1,
        peripherals.PA10,
        peripherals.PA9,
        peripherals.DMA1_CH4,
        peripherals.DMA1_CH5,
    )
    .await;
    report_set!(USART_RESULT, 2);
    defmt::info!(
        "USART_RESULT=PASS tx_instances=10 loopback={}",
        cfg!(feature = "usart-loopback")
    );
    loop {
        Timer::after_secs(1).await;
    }
}
