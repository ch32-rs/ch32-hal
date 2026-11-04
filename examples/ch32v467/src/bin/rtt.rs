#![no_std]
#![no_main]

use ch32v467_examples as board;
use defmt_rtt as _;
use panic_halt as _;

#[qingke_rt::entry]
fn main() -> ! {
    defmt::info!("CH32V467 PAC boot: entering clock init");
    board::init();

    let mut ticks = 0u32;
    loop {
        ticks = ticks.wrapping_add(1);
        board::heartbeat(ticks);
        defmt::info!(
            "CH32V467 PAC boot: tick={=u32} HCLK={=u32} RCC_CFGR0={=u32:#010x}",
            ticks,
            board::HCLK_HZ,
            board::pac::RCC.cfgr0().read().0,
        );
        board::delay_ms(1000);
    }
}
