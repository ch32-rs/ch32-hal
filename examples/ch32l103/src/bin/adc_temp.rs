#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use hal::adc::SampleTime;
use hal::gpio::{Level, Output};
use hal::println;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_96MHZ_HSE;
    let p = hal::init(config);
    hal::embassy::init();

    let mut adc = hal::adc::Adc::new(p.ADC1, Default::default());

    let mut temp = hal::adc::Temperature;
    let mut vref = hal::adc::VrefInt;
    let mut vdda_half = hal::adc::VddaDiv2;

    // GPIO
    let mut led = Output::new(p.PB0, Level::Low, Default::default());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        println!("Starting conversion!");
        let raw = adc.convert(&mut temp, SampleTime::CYCLES239_5);
        let val = convert_temp(raw);

        println!("raw: {}, val: {}C", raw, val);

        let raw = adc.convert(&mut vref, SampleTime::CYCLES55_5);
        let vref = raw as f32 * 3.3 / 4096.0;
        println!("vref: raw={}, {:.3}V", raw, vref);

        let raw = adc.convert(&mut vdda_half, SampleTime::CYCLES55_5);
        let vdda_half = raw as f32 * 3.3 / 4096.0;
        println!("Vdda/2: raw={}, {:.3}V", raw, vdda_half);
        println!("Vdda = {:.3}V", vdda_half * 2.0);
    }
}

fn convert_temp(raw: u16) -> f32 {
    // V_25 and Avg_Slope from datasheet
    let v25 = 1.45;
    let avg_slope = 4.2e-3;

    let v = raw as f32 * 3.3 / 4096.0;
    let temp = (v - v25) / avg_slope + 25.0;

    temp
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = println!("\n\n\n{}", info);

    loop {}
}
