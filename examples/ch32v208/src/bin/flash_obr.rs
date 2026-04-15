#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

//! Read FLASH_OBR register to check ROM/RAM split configuration
//!
//! For CH32V20x_D8W (CH32V208):
//! - 00x: CODE-128KB + RAM-64KB
//! - 01x: CODE-144KB + RAM-48KB
//! - 1xx: CODE-160KB + RAM-32KB

use hal::println;
use {ch32_hal as hal, panic_halt as _};

const FLASH_BASE: u32 = 0x4002_2000;
const FLASH_OBR_OFFSET: u32 = 0x1C;

#[qingke_rt::entry]
fn main() -> ! {
    let _p = hal::init(Default::default());
    hal::debug::SDIPrint::enable();

    println!("=== CH32V208 FLASH_OBR Register ===");
    println!("CHIP: {}", hal::signature::chip_id().name());

    // Read FLASH_OBR register
    let flash_obr = unsafe { core::ptr::read_volatile((FLASH_BASE + FLASH_OBR_OFFSET) as *const u32) };

    println!("FLASH_OBR: 0x{:08X}", flash_obr);

    // Extract RAM_CODE_MOD[2:0] from bits [9:7]
    let ram_code_mod = (flash_obr >> 7) & 0x7;
    println!("RAM_CODE_MOD[2:0]: 0b{:03b} ({})", ram_code_mod, ram_code_mod);

    // Decode configuration for CH32V20x_D8W
    let (code_kb, ram_kb) = match ram_code_mod {
        0b000 | 0b001 => (128, 64),  // 00x
        0b010 | 0b011 => (144, 48),  // 01x
        _ => (160, 32),               // 1xx
    };

    println!("Configuration: CODE-{}KB + RAM-{}KB", code_kb, ram_kb);
    println!("RAM range: 0x20000000 - 0x{:08X}", 0x20000000u32 + (ram_kb * 1024));

    // Other bits
    let rdprt = (flash_obr >> 1) & 0x1;
    let oberr = flash_obr & 0x1;
    let iwdg_sw = (flash_obr >> 2) & 0x1;
    let stop_rst = (flash_obr >> 3) & 0x1;
    let standy_rst = (flash_obr >> 4) & 0x1;

    println!("\nOther flags:");
    println!("  RDPRT (Read Protection): {}", rdprt);
    println!("  OBERR (Option Byte Error): {}", oberr);
    println!("  IWDG_SW: {}", iwdg_sw);
    println!("  STOP_RST: {}", stop_rst);
    println!("  STANDY_RST: {}", standy_rst);

    println!("\n=== Done ===");

    loop {
        qingke::riscv::asm::wfi();
    }
}

