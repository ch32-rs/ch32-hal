#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::dac::{Dac, DualValue, TriggerSel, Value, ValueArray, Waveform};
use ch32_hal::dma::NoDma;
use ch32_hal::time::Hertz;
use ch32_hal::timer::low_level::Timer as HardwareTimer;
use ch32_hal::{pac, rcc, Peri};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

report_static!(DAC_RESULT);
#[no_mangle]
pub static mut DAC_CASES: u32 = 0;
#[no_mangle]
pub static mut DAC_SNAPSHOTS: [[u32; 3]; 4] = [[0; 3]; 4];
#[no_mangle]
pub static mut DAC_WAVE_RANGE: [u32; 2] = [0; 2];

fn check(condition: bool, code: u32) {
    report_check!(DAC_RESULT, condition, code, "DAC_RESULT check failed");
}

fn snapshot(index: usize) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(DAC_SNAPSHOTS[index]),
            [
                pac::DAC1.cr().read().0,
                u32::from(pac::DAC1.dor(0).read().dor()),
                u32::from(pac::DAC1.dor(1).read().dor()),
            ],
        );
    }
}

fn no_dma() -> Peri<'static, NoDma> {
    unsafe { Peri::new_unchecked(NoDma) }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(DAC_RESULT, 1);
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
    {
        let mut dac = Dac::new(
            peripherals.DAC1.reborrow(),
            no_dma(),
            no_dma(),
            peripherals.PA4.reborrow(),
            peripherals.PA5.reborrow(),
        );
        for (value, first, second) in [
            (DualValue::Bit8(0x12, 0xab), 0x120, 0xab0),
            (DualValue::Bit12Right(0x123, 0xabc), 0x123, 0xabc),
            (DualValue::Bit12Left(0x456, 0x789), 0x456, 0x789),
        ] {
            dac.set(value);
            Timer::after_micros(100).await;
            check(dac.ch1().read() == first && dac.ch2().read() == second, 10);
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(DAC_CASES), DAC_CASES + 1) };
        }
        snapshot(0);
        dac.ch1().set_trigger(TriggerSel::SOFTWARE);
        dac.ch2().set_trigger(TriggerSel::SOFTWARE);
        dac.ch1().set_triggering(true);
        dac.ch2().set_triggering(true);
        dac.ch1().enable();
        dac.ch2().enable();
        dac.set(DualValue::Bit12Right(0x321, 0x654));
        dac.trigger();
        Timer::after_micros(100).await;
        check(dac.ch1().read() == 0x321 && dac.ch2().read() == 0x654, 11);
        snapshot(1);
        let (first, mut second) = dac.split();
        drop(first);
        check(pac::RCC.pb1pcenr().read().dacen() && pac::DAC1.cr().read().en(1), 12);
        second.set_triggering(false);
        second.set(Value::Bit12Right(0x987));
        Timer::after_micros(100).await;
        check(second.read() == 0x987, 13);
        drop(second);
        check(!pac::RCC.pb1pcenr().read().dacen(), 14);
    }
    let mut dac = Dac::new(
        peripherals.DAC1.reborrow(),
        peripherals.DMA2_CH3.reborrow(),
        peripherals.DMA2_CH4.reborrow(),
        peripherals.PA4.reborrow(),
        peripherals.PA5.reborrow(),
    );
    dac.ch1().set_trigger(TriggerSel::TIM6_TRGO);
    dac.ch2().set_trigger(TriggerSel::TIM6_TRGO);
    dac.ch1().set_triggering(true);
    dac.ch2().set_triggering(true);
    let timer = HardwareTimer::new(peripherals.TIM6.reborrow());
    timer.set_frequency(Hertz(1000));
    pac::TIM6
        .ctlr2()
        .modify(|reg| reg.set_mms(pac::timer::vals::Mms::UPDATE));
    timer.start();
    for format in 0..3 {
        let first = [0x123u16, 0x456, 0x789, 0xabc];
        let second = [0x12u8, 0x34, 0x56, 0x78];
        let left = [0x1230u16, 0x4560, 0x7890, 0xabc0];
        let data = match format {
            0 => ValueArray::Bit8(&second),
            1 => ValueArray::Bit12Right(&first),
            _ => ValueArray::Bit12Left(&left),
        };
        let result = with_timeout(Duration::from_millis(100), dac.ch1().try_write(data, false)).await;
        check(matches!(result, Ok(Ok(()))), 15);
        Timer::after_millis(2).await;
        let expected = if format == 0 { 0x780 } else { 0xabc };
        check(dac.ch1().read() == expected, 16);
    }
    let (mut first, mut second) = dac.split();
    let result = with_timeout(
        Duration::from_millis(100),
        join(
            first.try_write(ValueArray::Bit12Right(&[0x111, 0x222]), false),
            second.try_write(ValueArray::Bit12Right(&[0x333, 0x444]), false),
        ),
    )
    .await;
    check(matches!(result, Ok((Ok(()), Ok(())))), 17);
    Timer::after_millis(2).await;
    check(first.read() == 0x222 && second.read() == 0x444, 18);
    snapshot(2);
    {
        let mut transfer = pin!(first.try_write(ValueArray::Bit12Right(&[1, 2, 3, 4]), true));
        poll_fn(|context| {
            check(transfer.as_mut().poll(context).is_pending(), 19);
            Poll::Ready(())
        })
        .await;
    }
    check(!pac::DAC1.cr().read().dmaen(0) && !pac::DAC1.cr().read().en(0), 20);
    check(pac::DAC1.cr().read().en(1), 21);
    let result = with_timeout(
        Duration::from_millis(100),
        first.try_write(ValueArray::Bit12Right(&[0x555, 0x666]), false),
    )
    .await;
    check(matches!(result, Ok(Ok(()))), 22);
    Timer::after_millis(2).await;
    check(first.read() == 0x666, 23);
    first.set_waveform(Waveform::TRIANGLE, 3).unwrap();
    first.set(Value::Bit12Right(0));
    Timer::after_millis(2).await;
    let mut minimum = u16::MAX;
    let mut maximum = 0;
    for _sample in 0..20 {
        Timer::after_millis(1).await;
        let value = first.read();
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }
    check(minimum < maximum && maximum <= 15, 24);
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(DAC_WAVE_RANGE),
            [u32::from(minimum), u32::from(maximum)],
        );
    }
    first.set_waveform(Waveform::DISABLED, 0).unwrap();
    snapshot(3);
    timer.stop();
    report_set!(DAC_RESULT, 2);
    defmt::info!("DAC_RESULT=PASS digital register and DMA checks; measure PA4/PA5 for analog accuracy");
    loop {
        Timer::after_secs(1).await;
    }
}
