#![no_std]
#![no_main]

use ch32v467_examples as board;
use defmt_rtt as _;
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    defmt::info!("CH32V467 PAC boot: entering clock init");
    board::init();
    board::init_led();
    defmt::info!(
        "clock ready: HCLK={=u32} Hz, RCC_CFGR0={=u32:#010x}; LED=PB12",
        board::HCLK_HZ,
        board::pac::RCC.cfgr0().read().0,
    );

    let mut ticks = 0u32;
    loop {
        ticks = ticks.wrapping_add(1);
        let high = ticks & 1 != 0;
        board::set_led_level(high);
        board::heartbeat(ticks);
        defmt::info!("heartbeat={=u32}, PB12 high={=bool}", ticks, high);
        board::delay_ms(500);
    }
}
