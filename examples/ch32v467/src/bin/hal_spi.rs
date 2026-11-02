#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::interrupt::typelevel::Binding;
use ch32_hal::mode::{Async, Blocking};
use ch32_hal::spi::{self, BitOrder, InterruptHandler, Spi};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, pac, rcc, Peri};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer};
use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    SPI1 => InterruptHandler<ch32_hal::peripherals::SPI1>;
    SPI2 => InterruptHandler<ch32_hal::peripherals::SPI2>;
    SPI3 => InterruptHandler<ch32_hal::peripherals::SPI3>;
});

#[no_mangle]
pub static mut SPI_RESULT: u32 = 0;
#[no_mangle]
pub static mut SPI_CASES: u32 = 0;
#[no_mangle]
pub static mut SPI_LOOPBACK_RESULT: u32 = 0;
#[no_mangle]
pub static mut SPI_SNAPSHOTS: [[u32; 3]; 3] = [[0; 3]; 3];

fn check(condition: bool, code: u32) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_RESULT), code) };
        defmt::panic!("SPI_RESULT=FAIL code={}", code);
    }
}

async fn exercise<T: spi::Instance>(
    mut peripheral: Peri<'_, T>,
    mut sck: Peri<'_, impl spi::SckPin<T>>,
    mut mosi: Peri<'_, impl spi::MosiPin<T>>,
    mut dma: Peri<'_, impl spi::TxDma<T>>,
    irq: impl Binding<T::Interrupt, InterruptHandler<T>> + Copy,
    registers: pac::spi::Spi,
    index: usize,
) {
    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        for bit_order in [BitOrder::MsbFirst, BitOrder::LsbFirst] {
            let mut config = spi::Config::default();
            config.mode = mode;
            config.bit_order = bit_order;
            let mut bus = Spi::<T, Blocking>::new_blocking_txonly::<0>(
                peripheral.reborrow(),
                sck.reborrow(),
                mosi.reborrow(),
                config,
            );
            bus.blocking_write(&[0x55u8, 0xaa, 0, 0xff]).unwrap();
            bus.blocking_write(&[0x1234u16, 0xabcd, 0, 0xffff]).unwrap();
            check(!registers.statr().read().bsy() && !registers.statr().read().ovr(), 10);
            drop(bus);
            let mut bus = Spi::<T, Async>::new_txonly::<0>(
                peripheral.reborrow(),
                sck.reborrow(),
                mosi.reborrow(),
                irq,
                dma.reborrow(),
                config,
            );
            for frequency in [1_000_000, 3_000_000] {
                config.frequency = Hertz(frequency);
                bus.set_config(&config).unwrap();
                check(bus.get_current_config().frequency.0 <= frequency, 11);
                let result = with_timeout(Duration::from_millis(100), bus.write(&[0x55u8; 64])).await;
                check(matches!(result, Ok(Ok(()))), 12);
                let result = with_timeout(Duration::from_millis(100), bus.write(&[0xabcd_u16; 64])).await;
                check(matches!(result, Ok(Ok(()))), 13);
                check(!registers.statr().read().bsy() && !registers.statr().read().ovr(), 14);
            }
            config.frequency = Hertz(1_000_000);
            bus.set_config(&config).unwrap();
            {
                let data = [0x55u8; 2048];
                let mut transfer = pin!(bus.write(&data));
                poll_fn(|context| {
                    check(transfer.as_mut().poll(context).is_pending(), 15);
                    Poll::Ready(())
                })
                .await;
            }
            check(
                !registers.ctlr2().read().txdmaen() && !registers.ctlr1().read().spe(),
                16,
            );
            let result = with_timeout(Duration::from_millis(100), bus.write(&[0xa5u8; 16])).await;
            check(matches!(result, Ok(Ok(()))), 17);
            unsafe {
                core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_CASES), SPI_CASES + 1);
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(SPI_SNAPSHOTS[index]),
                    [
                        registers.ctlr1().read().0,
                        registers.statr().read().0,
                        bus.get_current_config().frequency.0,
                    ],
                );
            }
        }
    }
}

#[cfg(feature = "spi-loopback")]
async fn loopback(mut peripherals: ch32_hal::Peripherals) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_LOOPBACK_RESULT), 1) };
    for mode in [MODE_0, MODE_1, MODE_2, MODE_3] {
        for bit_order in [BitOrder::MsbFirst, BitOrder::LsbFirst] {
            let mut config = spi::Config::default();
            config.mode = mode;
            config.bit_order = bit_order;
            let mut bus = Spi::new::<0>(
                peripherals.SPI1.reborrow(),
                peripherals.PA5.reborrow(),
                peripherals.PA7.reborrow(),
                peripherals.PA6.reborrow(),
                Irqs,
                peripherals.DMA1_CH3.reborrow(),
                peripherals.DMA1_CH2.reborrow(),
                config,
            );
            let mut bytes = [0x55u8, 0xaa, 0, 0xff, 0x81];
            bus.blocking_transfer_in_place(&mut bytes).unwrap();
            check(bytes == [0x55, 0xaa, 0, 0xff, 0x81], 20);
            let mut words = [0x1234u16, 0xabcd, 0, 0xffff];
            check(
                matches!(
                    with_timeout(Duration::from_millis(100), bus.transfer_in_place(&mut words)).await,
                    Ok(Ok(()))
                ),
                21,
            );
            check(words == [0x1234, 0xabcd, 0, 0xffff], 22);
            let mut receive = [0xffu8; 7];
            check(
                matches!(
                    with_timeout(Duration::from_millis(100), bus.transfer(&mut receive, &[1, 2, 3])).await,
                    Ok(Ok(()))
                ),
                23,
            );
            check(receive == [1, 2, 3, 0, 0, 0, 0], 24);
            check(
                matches!(
                    with_timeout(
                        Duration::from_millis(100),
                        bus.transfer(&mut receive[..2], &[4, 5, 6, 7])
                    )
                    .await,
                    Ok(Ok(()))
                ),
                25,
            );
            check(receive[..2] == [4, 5], 26);
            words.fill(0xffff);
            check(
                matches!(
                    with_timeout(Duration::from_millis(100), bus.read(&mut words)).await,
                    Ok(Ok(()))
                ),
                27,
            );
            check(words == [0; 4], 28);
        }
    }
    let mut bytes = [0u8; 3];
    let mut bus = Spi::new_rxonly::<0>(
        peripherals.SPI1.reborrow(),
        peripherals.PA5.reborrow(),
        peripherals.PA6.reborrow(),
        Irqs,
        peripherals.DMA1_CH2.reborrow(),
        Default::default(),
    );
    check(
        matches!(
            with_timeout(Duration::from_millis(100), bus.read(&mut bytes)).await,
            Ok(Ok(()))
        ),
        29,
    );
    drop(bus);
    let mut bus = Spi::new_rxonly_with_tx_dma::<0>(
        peripherals.SPI1.reborrow(),
        peripherals.PA5.reborrow(),
        peripherals.PA6.reborrow(),
        Irqs,
        peripherals.DMA1_CH3.reborrow(),
        peripherals.DMA1_CH2.reborrow(),
        Default::default(),
    );
    check(
        matches!(
            with_timeout(Duration::from_millis(100), bus.read(&mut bytes)).await,
            Ok(Ok(()))
        ),
        30,
    );
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_LOOPBACK_RESULT), 2) };
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_RESULT), 1) };
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
    exercise(
        peripherals.SPI1.reborrow(),
        peripherals.PA5.reborrow(),
        peripherals.PA7.reborrow(),
        peripherals.DMA1_CH3.reborrow(),
        Irqs,
        pac::SPI1,
        0,
    )
    .await;
    exercise(
        peripherals.SPI2.reborrow(),
        peripherals.PB13.reborrow(),
        peripherals.PB15.reborrow(),
        peripherals.DMA1_CH5.reborrow(),
        Irqs,
        pac::SPI2,
        1,
    )
    .await;
    exercise(
        peripherals.SPI3.reborrow(),
        peripherals.PB3.reborrow(),
        peripherals.PB5.reborrow(),
        peripherals.DMA2_CH2.reborrow(),
        Irqs,
        pac::SPI3,
        2,
    )
    .await;
    #[cfg(feature = "spi-loopback")]
    loopback(peripherals).await;
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SPI_RESULT), 2) };
    defmt::info!("SPI_RESULT=PASS");
    loop {
        Timer::after_secs(1).await;
    }
}
