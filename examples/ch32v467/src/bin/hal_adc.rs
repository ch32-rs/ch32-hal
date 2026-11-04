#![no_std]
#![no_main]

use core::future::{poll_fn, Future};
use core::pin::pin;
use core::task::Poll;

use ch32_hal::adc::{Adc, Channel, Config, InterruptHandler, SampleTime, ADC_MAX};
use ch32_hal::{bind_interrupts, pac, rcc};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

bind_interrupts!(struct Irqs {
    ADC1_2 => InterruptHandler;
});

report_static!(ADC_RESULT);
#[no_mangle]
pub static mut ADC_CLOCK_HZ: u32 = 0;
#[no_mangle]
pub static mut ADC_SAMPLES: [u16; 68] = [0; 68];
#[no_mangle]
pub static mut ADC_CANCEL_PENDING: u32 = 0;

fn check(condition: bool, code: u32) {
    report_check!(ADC_RESULT, condition, code, "ADC_RESULT check failed");
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(ADC_RESULT, 1);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV2;
    }
    if cfg!(feature = "clock-dividers") {
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV4;
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV4;
    }
    if cfg!(feature = "adc-usbhs-pll") {
        config.rcc.hspll = Some(rcc::HsPllSource::HSI);
        config.rcc.adc_src = rcc::AdcClockSource::UsbHsPll { divisor: 8 };
    }
    let mut peripherals = ch32_hal::init(config);
    let frequency = rcc::adc_frequency().unwrap().0;
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(ADC_CLOCK_HZ), frequency) };
    check(frequency <= 14_000_000, 10);
    let mut first = Adc::new(peripherals.ADC1.reborrow(), Config::default()).unwrap();
    let mut second = Adc::new(peripherals.ADC2.reborrow(), Config::default()).unwrap();
    let mut vref = first.enable_vref();
    let mut temperature = first.enable_temperature();
    let reference = first.blocking_read(&mut vref, SampleTime::S239_5).unwrap();
    let thermal = first.blocking_read(&mut temperature, SampleTime::S239_5).unwrap();
    check(
        (1000..=2200).contains(&reference) && (700..=2700).contains(&thermal),
        11,
    );
    unsafe {
        core::ptr::addr_of_mut!(ADC_SAMPLES)
            .cast::<u16>()
            .write_volatile(reference);
        core::ptr::addr_of_mut!(ADC_SAMPLES)
            .cast::<u16>()
            .add(1)
            .write_volatile(thermal);
    }
    let result = with_timeout(
        Duration::from_millis(500),
        join(
            first.read(&mut vref, SampleTime::S239_5, Irqs),
            second.read(&mut peripherals.PA0, SampleTime::S239_5, Irqs),
        ),
    )
    .await;
    check(matches!(result, Ok((Ok(_), Ok(_)))), 12);
    let (reference_async, external) = result.unwrap();
    let reference_async = reference_async.unwrap();
    let external = external.unwrap();
    check(
        reference.abs_diff(reference_async) <= 80 && u32::from(external) <= ADC_MAX,
        13,
    );
    unsafe {
        core::ptr::addr_of_mut!(ADC_SAMPLES)
            .cast::<u16>()
            .add(2)
            .write_volatile(reference_async);
        core::ptr::addr_of_mut!(ADC_SAMPLES)
            .cast::<u16>()
            .add(3)
            .write_volatile(external);
    }
    let mut scan = [0u16; 64];
    {
        let ranks = [
            Channel::new(&mut vref, SampleTime::S239_5),
            Channel::new(&mut temperature, SampleTime::S239_5),
        ];
        let result = with_timeout(
            Duration::from_millis(500),
            first.read_dma(peripherals.DMA1_CH12.reborrow(), &ranks, &mut scan),
        )
        .await;
        check(matches!(result, Ok(Ok(()))), 14);
    }
    for frame in scan.chunks_exact(2) {
        check(
            reference.abs_diff(frame[0]) <= 80 && thermal.abs_diff(frame[1]) <= 120,
            15,
        );
    }
    unsafe {
        for (index, value) in scan.iter().enumerate() {
            core::ptr::addr_of_mut!(ADC_SAMPLES)
                .cast::<u16>()
                .add(index + 4)
                .write_volatile(*value);
        }
    }
    let mut external_samples = [0u16; 32];
    {
        let ranks = [Channel::new(&mut peripherals.PA0, SampleTime::S239_5)];
        let result = with_timeout(
            Duration::from_millis(500),
            second.read_dma(peripherals.DMA1_CH13.reborrow(), &ranks, &mut external_samples),
        )
        .await;
        check(matches!(result, Ok(Ok(()))), 16);
    }
    check(external_samples.iter().all(|sample| u32::from(*sample) <= ADC_MAX), 17);
    {
        let ranks = [Channel::new(&mut vref, SampleTime::S239_5)];
        let mut future = pin!(first.read_dma(peripherals.DMA1_CH12.reborrow(), &ranks, &mut scan));
        let pending = poll_fn(|context| Poll::Ready(future.as_mut().poll(context).is_pending())).await;
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(ADC_CANCEL_PENDING), pending as u32) };
    }
    check(!pac::ADC1.ctlr2().read().adon() && !pac::ADC1.ctlr2().read().dma(), 18);
    check(!pac::DMA1.ech(4).cfgr().read().en(), 19);
    check(first.blocking_read(&mut vref, SampleTime::S239_5).is_ok(), 20);
    drop(second);
    check(
        pac::RCC.pb2pcenr().read().adc1en() && !pac::RCC.pb2pcenr().read().adc2en(),
        21,
    );
    check(first.blocking_read(&mut vref, SampleTime::S239_5).is_ok(), 22);

    #[cfg(feature = "adc-loopback")]
    {
        use ch32_hal::dac::{DacCh1, Value};
        use ch32_hal::dma::NoDma;
        let mut output = DacCh1::new(
            peripherals.DAC1,
            unsafe { ch32_hal::Peri::new_unchecked(NoDma) },
            peripherals.PA4,
        );
        for target in [512u16, 2048, 3584] {
            output.set(Value::Bit12Right(target));
            Timer::after_millis(2).await;
            let sample = first.blocking_read(&mut peripherals.PA0, SampleTime::S239_5).unwrap();
            check(sample.abs_diff(target) <= 100, 23);
        }
    }

    report_set!(ADC_RESULT, 2);
    defmt::info!(
        "ADC_RESULT=PASS clock_hz={} vref_raw={} temperature_raw={}",
        frequency,
        reference,
        thermal
    );
    loop {
        Timer::after_secs(1).await;
    }
}
