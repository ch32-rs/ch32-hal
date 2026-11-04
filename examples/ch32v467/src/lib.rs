#![no_std]

pub mod report;

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
        pac::SYSTICK.cmpl().write_value(HCLK_HZ / 1_000 - 1);
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

/// Board LEDs on PE8 and PE9. Keep SDI/JTAG pins intact.
pub fn init_led() {
    pac::RCC.pb2pcenr().modify(|w| w.set_iopeen(true));
    let _ = pac::RCC.pb2pcenr().read();
    set_led_level(false);
    pac::GPIOE.cfghr().modify(|w| {
        // CFGHR indexes pins 8..15 as 0..7, so PE8/PE9 are indices 0/1.
        for pin in 0..2 {
            w.set_mode(pin, pac::gpio::vals::Mode::OUTPUT_2MHZ);
            w.set_cnf(pin, pac::gpio::vals::Cnf::ANALOG_IN__PUSH_PULL_OUT);
        }
    });
}

/// Set PE8 to `high` and PE9 to the opposite level for alternating LEDs.
pub fn set_led_level(high: bool) {
    pac::GPIOE.bshr().write(|w| {
        for pin in 8..10 {
            if high == (pin == 8) {
                w.set_bs(pin, true);
            } else {
                w.set_br(pin, true);
            }
        }
    });
}
