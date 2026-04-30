//! BLE ADV_NONCONN_IND advertising example for CH32V208.
//!
//! Broadcasts a non-connectable undirected advertising packet on all three
//! BLE advertising channels (ch37=2402 / ch38=2426 / ch39=2480 MHz) in a loop.
//! Verifiable with nRF Connect (phone), Wireshark + BLE sniffer, or SDR.
//!
//! # How to verify
//!
//! 1. Flash: `cargo run --bin ble_adv --release`
//! 2. Open nRF Connect on a phone → Scanner → look for "ch32-hal"
//! 3. Optional: HackRF/RTL-SDR at 2402/2426/2480 MHz, observe GFSK bursts
//! 4. Optional: Wireshark + nRF Sniffer → confirm ADV_NONCONN_IND PDU + correct AdvA

#![no_std]
#![no_main]

use {ch32_hal as hal, panic_halt as _};

use hal::ble::adv::{ADV_DATA_MAX, ad_complete_name, ad_flags, adv_event, adv_event_verbose, diag_read};

// 6-byte device address LE order. Random static requires bits[47:46] = 11
// (= last byte bits[7:6] = 11, BLE Core Spec Vol 6 Part B §1.3.2.1).
const ADDR: [u8; 6] = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xC2]; // 0xC2 = 0b11000010 → bits[7:6]=11 ✓

#[qingke_rt::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let _p = hal::init(Default::default());

    hal::println!("BLE ADV — CH32V208");
    hal::println!("addr: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        ADDR[5], ADDR[4], ADDR[3], ADDR[2], ADDR[1], ADDR[0]);

    // Build AdvData: Flags + Complete Local Name "ch32-hal"
    let mut adv_data = [0u8; ADV_DATA_MAX];
    let mut pos = 0;
    pos += ad_flags(&mut adv_data[pos..], 0x06); // LE General Discoverable | BR/EDR Not Supported
    pos += ad_complete_name(&mut adv_data[pos..], b"ch32-hal");
    let adv_data = &adv_data[..pos];

    unsafe {
        hal::ble::ble_phy_init();
        hal::println!("PHY init done, advertising...");
        hal::println!("[DIAG] state_machine + IRQ diagnostic mode");

        let mut total = 0u32;
        let mut ok_total = 0u32;

        loop {
            let ok;
            if total < 5 {
                // Verbose diagnostic for first 5 events.
                let (n, stats) = adv_event_verbose(&ADDR, true, adv_data);
                ok = n;
                for (i, (init, fin, irq_final, state_pre, state_post, irq_post)) in stats.iter().enumerate() {
                    let ch = [37u8, 38, 39][i];
                    // state_pre/post: 108=Sleep; non-108 means HW accepted GO.
                    // irq_post bits[15:0]: non-zero means TX-completion IRQ fired right after GO.
                    hal::println!("[DIAG#{total}:ch{ch}] bb64={init}->{fin} irq_final=0x{irq_final:08x} | state={state_pre}->{state_post} irq_post=0x{irq_post:08x}");
                }
                let (lle_2c, bb04, ctrl) = diag_read();
                hal::println!("[DIAG#{total}] lle_2c=0x{lle_2c:08x} bb04=0x{bb04:02x} ctrl=0x{ctrl:08x} ch_2c={} wh={}",
                    (lle_2c >> 25) & 0x3F, (ctrl >> 6) & 1);
            } else {
                ok = adv_event(&ADDR, true, adv_data);
            }
            ok_total += ok as u32;
            total += 1;

            if total <= 5 || total % 500 == 0 {
                hal::println!("adv #{}: {}/3 channels ok (cumulative {}/{})",
                    total, ok, ok_total, total * 3);
            }

            // ~247 ms gap — distinctive timing signature for SDR ON/OFF differential.
            // (At 144 MHz, ~4 cycles/iter: 8_900_000 * 4 / 144_000_000 ≈ 247 ms)
            qingke::riscv::asm::delay(20_000 * 445);
        }
    }
}
