#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::sdio::{Config, DataBlock, Error, InterruptHandler, Sdmmc, Signalling};
use ch32_hal::time::Hertz;
use ch32_hal::{bind_interrupts, pac, peripherals, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::Timer;
use panic_halt as _;

bind_interrupts!(struct Irqs {
    SDIO => InterruptHandler<peripherals::SDIO>;
});

#[no_mangle]
pub static mut SDIO_RESULT: u32 = 0;
#[no_mangle]
pub static mut SDIO_CARD_INFO: [u32; 5] = [0; 5];
#[no_mangle]
pub static mut SDIO_FIRST_BYTES: [u8; 64] = [0; 64];
#[no_mangle]
pub static mut SDIO_CANCELLED: u32 = 0;

fn check(condition: bool, code: u32) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_RESULT), code) };
        defmt::panic!("SDIO_RESULT=FAIL code={}", code);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_RESULT), 1) };
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV4;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    let peripherals = ch32_hal::init(config);
    let mut config = Config::default();
    config.allow_clock_bypass = cfg!(feature = "sdio-bypass");
    #[cfg(not(feature = "sdio-remap"))]
    let (d0, d1) = (peripherals.PC8, peripherals.PC9);
    #[cfg(feature = "sdio-remap")]
    let (d0, d1) = (peripherals.PB14, peripherals.PB15);
    #[cfg(feature = "sdio-4bit")]
    let mut sd = Sdmmc::new_4bit(
        peripherals.SDIO,
        Irqs,
        peripherals.DMA2_CH4,
        peripherals.PC12,
        peripherals.PD2,
        d0,
        d1,
        peripherals.PC10,
        peripherals.PC11,
        config,
    )
    .unwrap();
    #[cfg(not(feature = "sdio-4bit"))]
    let mut sd = {
        let _ = d1;
        Sdmmc::new_1bit(
            peripherals.SDIO,
            Irqs,
            peripherals.DMA2_CH4,
            peripherals.PC12,
            peripherals.PD2,
            d0,
            config,
        )
        .unwrap()
    };
    let frequency = Hertz(if cfg!(feature = "sdio-high-speed") {
        50_000_000
    } else {
        25_000_000
    });
    defmt::info!(
        "SDIO init: hclk={} requested={} wide={}",
        rcc::clocks().hclk.0,
        frequency.0,
        cfg!(feature = "sdio-4bit")
    );
    match sd.init_card(frequency).await {
        Ok(()) => {}
        Err(Error::Timeout) => {
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_RESULT), 2) };
            defmt::warn!("SDIO: no card response; check power, card and wiring. No read test performed.");
            drop(sd);
            loop {
                Timer::after_secs(1).await;
            }
        }
        Err(error) => {
            defmt::error!("SDIO init error: {:?}", error);
            check(false, 10);
        }
    }
    let card = sd.card().unwrap();
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(SDIO_CARD_INFO),
            [
                card.block_count as u32,
                (card.block_count >> 32) as u32,
                sd.clock().0,
                u32::from(card.rca),
                (sd.signalling() == Signalling::SDR25) as u32,
            ],
        );
    }
    defmt::info!("SDIO card: blocks={} clock={}", card.block_count, sd.clock().0);
    let status = sd.read_sd_status().await;
    check(status.is_ok(), 11);
    let mut first = DataBlock([0; 512]);
    check(sd.read_block(0, &mut first).await.is_ok(), 12);
    unsafe {
        let mut snapshot = [0; 64];
        snapshot.copy_from_slice(&first[..64]);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_FIRST_BYTES), snapshot);
    }
    let mut repeated = DataBlock([0; 512]);
    check(sd.blocking_read_block(0, &mut repeated).is_ok(), 13);
    check(first == repeated, 14);
    let mut blocks = [DataBlock([0; 512]), DataBlock([0; 512])];
    check(sd.read_blocks(0, &mut blocks).await.is_ok(), 15);
    check(blocks[0] == first, 16);
    check(sd.read_block(1, &mut repeated).await.is_ok(), 17);
    check(blocks[1] == repeated, 18);
    let cancelled = {
        let mut future = pin!(sd.read_block(0, &mut repeated));
        poll_fn(|context| {
            Poll::Ready(match future.as_mut().poll(context) {
                Poll::Pending => true,
                Poll::Ready(result) => {
                    check(result.is_ok(), 19);
                    false
                }
            })
        })
        .await
    };
    if cancelled {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_CANCELLED), 1) };
        check(sd.card().is_err(), 20);
        check(!pac::DMA2.ch(3).cfgr().read().en(), 21);
        check(sd.init_card(frequency).await.is_ok(), 22);
        check(sd.read_block(0, &mut repeated).await.is_ok(), 23);
        check(first == repeated, 24);
    }
    drop(sd);
    check(!pac::RCC.hbpcenr().read().sdioen(), 25);
    check(!pac::DMA2.ch(3).cfgr().read().en(), 26);
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(SDIO_RESULT), 3) };
    defmt::info!("SDIO_RESULT=PASS read-only cancelled={}", cancelled);
    loop {
        Timer::after_secs(1).await;
    }
}
