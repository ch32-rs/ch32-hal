#![no_std]
#![no_main]

use ch32_hal::gpio::{Level, Output, Speed};
use defmt_rtt as _;
use panic_halt as _;
use qingke_rt::entry;

#[no_mangle]
pub static mut GPIO_OUT_RESULT: u32 = 0;

#[entry]
fn main() -> ! {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_OUT_RESULT), 1) };
    let peripherals = ch32_hal::init(Default::default());
    let mut led8 = Output::new(peripherals.PE8, Level::Low, Speed::Low);
    let mut led9 = Output::new(peripherals.PE9, Level::High, Speed::Low);
    defmt::info!("HAL GPIO: alternating PE8/PE9");
    let mut toggles = 0u32;
    loop {
        led8.toggle();
        led9.toggle();
        #[cfg(not(feature = "embassy"))]
        ch32_hal::delay::Delay.delay_ms(100);
        #[cfg(feature = "embassy")]
        qingke::riscv::asm::delay(ch32_hal::rcc::clocks().hclk.0 / 10);
        let expected = if toggles & 1 == 0 { 1 << 8 } else { 1 << 9 };
        let output = ch32_hal::pac::GPIOE.outdr().read().0;
        let input = ch32_hal::pac::GPIOE.indr().read().0;
        if output & 0x300 != expected || input & 0x300 != expected {
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_OUT_RESULT), 3) };
            defmt::panic!("GPIO output mismatch: outdr={=u32:08x} indr={=u32:08x}", output, input);
        }
        toggles = toggles.wrapping_add(1);
        if toggles == 20 {
            unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(GPIO_OUT_RESULT), 2) };
            defmt::info!("GPIO_OUT_RESULT=PASS: both PE8/PE9 levels read back correctly");
        }
        if toggles % 10 == 0 {
            defmt::info!(
                "GPIO toggles={} outdr={=u32:08x} indr={=u32:08x}",
                toggles,
                output,
                input
            );
        }
    }
}
