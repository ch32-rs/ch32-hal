#![no_std]

pub use ch32_metapac as pac;

pub const HCLK_HZ: u32 = 20_000_000;

// Debugger-visible markers also work when no SDI console or LED is attached.
#[no_mangle]
pub static mut BOOT_STAGE: u32 = 0;
#[no_mangle]
pub static mut HEARTBEAT: u32 = 0;

/// Select the internal 20 MHz oscillator, without HSE, PLL, or overclocking.
/// Owns RCC and SysTick; call once, before using peripherals or interrupts.
pub fn init() {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(BOOT_STAGE), 1) };
    pac::RCC.ctlr().modify(|w| w.set_hsion(true));
    while !pac::RCC.ctlr().read().hsirdy() {}
    pac::RCC.cfgr0().modify(|w| w.set_sw(0));
    while pac::RCC.cfgr0().read().sws() != 0 {}
    pac::RCC.cfgr0().modify(|w| w.set_hpre(0));

    pac::SYSTICK.ctlr().write(|w| {
        w.set_stclk(pac::systick::vals::Stclk::HCLK);
        w.set_mode(pac::systick::vals::Mode::UPCOUNT);
    });
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(BOOT_STAGE), 2) };
}

/// Poll SysTick without a SysTick interrupt. This example owns the whole counter.
pub fn delay_ms(ms: u32) {
    for _ in 0..ms {
        pac::SYSTICK.ctlr().modify(|w| w.set_ste(false));
        pac::SYSTICK.cmph().write_value(0);
        pac::SYSTICK.cmpl().write_value(HCLK_HZ / 1_000 - 1);
        pac::SYSTICK.cnth().write_value(0);
        pac::SYSTICK.cntl().write_value(0);
        pac::SYSTICK.sr().write(|w| w.set_cntif(false));
        pac::SYSTICK.ctlr().modify(|w| w.set_ste(true));
        while !pac::SYSTICK.sr().read().cntif() {}
    }
    pac::SYSTICK.ctlr().modify(|w| w.set_ste(false));
}

pub fn heartbeat(value: u32) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(HEARTBEAT), value) };
}

/// Board LED on PB12. Keep SDI/JTAG pins intact.
pub fn init_led() {
    pac::RCC.pb2pcenr().modify(|w| w.set_iopben(true));
    let _ = pac::RCC.pb2pcenr().read();
    set_led_level(false);
    pac::GPIOB.cfghr().modify(|w| {
        // CFGHR indexes pins 8..15 as 0..7, so PB12 is index 4.
        w.set_mode(4, pac::gpio::vals::Mode::OUTPUT_2MHZ);
        w.set_cnf(4, pac::gpio::vals::Cnf::ANALOG_IN__PUSH_PULL_OUT);
    });
}

/// Set the electrical pin level; LED polarity does not affect blinking.
pub fn set_led_level(high: bool) {
    pac::GPIOB.bshr().write(|w| {
        if high {
            w.set_bs(12, true);
        } else {
            w.set_br(12, true);
        }
    });
}
