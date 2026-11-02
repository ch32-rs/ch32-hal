#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::can::{
    BitTiming, Can, CanError, CanFifo, CanFrame, CanMode, Config, ErrorInterruptHandler, ExtendedId, Filter,
    ReceiveInterruptHandler, StandardId, TransmitInterruptHandler,
};
use ch32_hal::{bind_interrupts, pac, peripherals, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;

bind_interrupts!(struct Irqs {
    CAN1_TX => TransmitInterruptHandler<peripherals::CAN1>;
    CAN1_RX0 => ReceiveInterruptHandler<peripherals::CAN1, 0>;
    CAN1_RX1 => ReceiveInterruptHandler<peripherals::CAN1, 1>;
    CAN1_SCE => ErrorInterruptHandler<peripherals::CAN1>;
});

#[no_mangle]
pub static mut CAN_RESULT: u32 = 0;
#[no_mangle]
pub static mut CAN_SNAPSHOTS: [u32; 6] = [0; 6];
#[no_mangle]
pub static mut CAN_FRAMES: [u32; 4] = [0; 4];

fn check(condition: bool, code: u32) {
    if !condition {
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(CAN_RESULT), code) };
        defmt::panic!("CAN_RESULT=FAIL code={}", code);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(CAN_RESULT), 1) };
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
    }
    let mut peripherals = ch32_hal::init(config);
    let clock = rcc::clocks().pclk1.0;
    let standard = StandardId::new(0x123).unwrap();
    let extended = ExtendedId::new(0x123_4567).unwrap();
    let mut can_config = Config::default();
    can_config.mode = CanMode::SilentLoopback;
    for bitrate in [125_000, 500_000, 1_000_000] {
        can_config.bitrate = bitrate;
        let mut can = Can::new_blocking(
            peripherals.CAN1.reborrow(),
            peripherals.PA11.reborrow(),
            peripherals.PA12.reborrow(),
            can_config,
        )
        .unwrap();
        can.set_filter(Filter::accept_all(0, CanFifo::Fifo0)).unwrap();
        let frame = CanFrame::new(standard, &[0x12, 0x34, 0x56, 0x78]).unwrap();
        check(can.blocking_transmit(&frame).is_ok(), 10);
        check(can.blocking_receive(CanFifo::Fifo0) == Ok(frame), 11);
        let timing = BitTiming::from_bitrate(clock, bitrate).unwrap();
        check(
            u64::from(clock)
                == u64::from(bitrate)
                    * u64::from(timing.prescaler)
                    * (1 + u64::from(timing.seg1) + u64::from(timing.seg2)),
            12,
        );
    }
    can_config.bitrate = 500_000;
    {
        let mut can = Can::new(
            peripherals.CAN1.reborrow(),
            peripherals.PA11.reborrow(),
            peripherals.PA12.reborrow(),
            Irqs,
            can_config,
        )
        .unwrap();
        can.set_filter(Filter::exact(0, CanFifo::Fifo0, standard, None))
            .unwrap();
        can.set_filter(Filter::exact(13, CanFifo::Fifo1, extended, None))
            .unwrap();
        check(can.set_filter(Filter::accept_all(14, CanFifo::Fifo0)).is_err(), 13);
        let mut counts = [0u32; 4];
        for round in 0..72usize {
            let id = if round % 2 == 0 {
                ch32_hal::can::Id::from(standard)
            } else {
                ch32_hal::can::Id::from(extended)
            };
            let length = round % 9;
            let data = [round as u8, 1, 2, 3, 0x80, 0xff, 6, 7];
            let remote = round % 4 >= 2;
            let frame = if remote {
                CanFrame::new_remote(id, length)
            } else {
                CanFrame::new(id, &data[..length])
            }
            .unwrap();
            let fifo = if round % 2 == 0 { CanFifo::Fifo0 } else { CanFifo::Fifo1 };
            check(
                matches!(
                    with_timeout(Duration::from_millis(100), can.transmit(&frame)).await,
                    Ok(Ok(()))
                ),
                14,
            );
            let received = with_timeout(Duration::from_millis(100), can.receive(fifo)).await;
            check(matches!(received, Ok(Ok(ref received)) if *received == frame), 15);
            counts[round % 4] += 1;
        }
        for fifo in [CanFifo::Fifo0, CanFifo::Fifo1] {
            {
                let mut future = pin!(can.receive(fifo));
                poll_fn(|context| {
                    check(future.as_mut().poll(context).is_pending(), 16);
                    Poll::Ready(())
                })
                .await;
            }
            check(pac::CAN1.intenr().read().0 == 0, 17);
            let id = if fifo == CanFifo::Fifo0 {
                ch32_hal::can::Id::from(standard)
            } else {
                ch32_hal::can::Id::from(extended)
            };
            let frame = CanFrame::new(id, &[0xa5]).unwrap();
            check(can.blocking_transmit(&frame).is_ok(), 18);
            check(can.blocking_receive(fifo) == Ok(frame), 19);
        }
        let rejected = CanFrame::new(StandardId::new(0x456).unwrap(), &[1]).unwrap();
        check(can.blocking_transmit(&rejected).is_ok(), 20);
        Timer::after_millis(1).await;
        check(
            pac::CAN1.rfifo(0).read().fmp() == 0 && pac::CAN1.rfifo(1).read().fmp() == 0,
            21,
        );
        let frame = CanFrame::new(standard, &[0xaa]).unwrap();
        for _round in 0..4 {
            check(can.blocking_transmit(&frame).is_ok(), 22);
        }
        check(pac::CAN1.rfifo(0).read().fovr(), 23);
        check(
            matches!(
                can.try_receive(CanFifo::Fifo0),
                Err(nb::Error::Other(CanError::Overrun))
            ),
            24,
        );
        for _round in 0..3 {
            check(can.blocking_receive(CanFifo::Fifo0) == Ok(frame.clone()), 25);
        }
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(CAN_FRAMES), counts);
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(CAN_SNAPSHOTS),
                [
                    clock,
                    pac::CAN1.btimr().read().0,
                    pac::CAN1.fwr().read().0,
                    pac::CAN1.fafifor().read().0,
                    pac::CAN1.intenr().read().0,
                    pac::CAN1.errsr().read().0,
                ],
            );
        }
    }
    check(!pac::RCC.pb1pcenr().read().can1en(), 26);
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(CAN_RESULT), 2) };
    defmt::info!("CAN_RESULT=PASS clock={} frames=72", clock);
    loop {
        Timer::after_secs(1).await;
    }
}
