#![no_std]
#![no_main]

use core::cell::RefCell;

use ch32_hal as hal;
use critical_section::Mutex;
use hal::gpio::{Level, Output};
use hal::pac::systick::vals::Stclk;
use panic_halt as _;
use qingke::interrupt::Priority;
use qingke::riscv;
use qingke_rt::{interrupt, CoreInterrupt};

const BLINK_PERIOD_MS: u32 = 500;

static LED: Mutex<RefCell<Option<Output<'static>>>> = Mutex::new(RefCell::new(None));

#[interrupt(core)]
fn SysTick() {
    let systick = &hal::pac::SYSTICK;
    systick.sr().write(|w| w.set_cntif(false));

    critical_section::with(|cs| {
        if let Some(led) = LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
}

fn init_systick() {
    let systick = &hal::pac::SYSTICK;
    let hclk = u64::from(hal::rcc::clocks().hclk.0);
    let ticks = hclk * u64::from(BLINK_PERIOD_MS) / 8 / 1_000;
    let compare = u32::try_from(ticks - 1).unwrap();

    systick.ctlr().write(|_| {});
    systick.cmp().write_value(compare);
    systick.cnt().write_value(0);
    systick.sr().write(|w| w.set_cntif(false));
    systick.ctlr().write(|w| {
        w.set_stclk(Stclk::HCLK_DIV8);
        w.set_stre(true);
        w.set_stie(true);
        w.set_ste(true);
    });
}

#[qingke_rt::entry]
fn main() -> ! {
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = hal::init(config);

    let led = Output::new(p.PD6, Level::Low, Default::default());
    critical_section::with(|cs| {
        *LED.borrow(cs).borrow_mut() = Some(led);
    });

    init_systick();

    unsafe {
        qingke::pfic::set_priority(CoreInterrupt::SysTick as u8, Priority::P15 as u8);
        qingke::pfic::enable_interrupt(CoreInterrupt::SysTick as u8);
    }

    loop {
        riscv::asm::wfi();
    }
}
