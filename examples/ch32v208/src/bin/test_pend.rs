//! Test software interrupt pending/wake mechanism
//!
//! This test verifies that PFIC software interrupt can wake up WFI

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::println;

#[embassy_executor::task]
async fn blinky(pin: hal::Peri<'static, hal::peripherals::PA8>) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    
    // Configure RCC for 144MHz using HSI + PLL
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    
    let p = hal::init(config);

    println!("\n=== Test Software Interrupt Wake ===");
    
    // Check PFIC SCTLR configuration
    let sctlr = hal::pac::PFIC.sctlr().read();
    println!("SCTLR: 0x{:08x}", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());
    
    // Check SysTick configuration
    let stk = hal::pac::SYSTICK;
    let ctlr = stk.ctlr().read();
    println!("SYSTICK CTLR: 0x{:08x}", ctlr.0);
    println!("  STE (enable): {}", ctlr.ste());
    println!("  STIE (int enable): {}", ctlr.stie());
    println!("SysTick IRQ enabled in PFIC: {}", qingke::pfic::is_enabled(12));
    
    // Don't spawn blinky to simplify debugging
    let _ = p.PA8;
    
    let stk = hal::pac::SYSTICK;
    
    // Direct Timer test - no yield_now first
    println!("\nTesting Timer::after_millis(100)...");
    println!("Before: now={}, STIE={}", 
        embassy_time::Instant::now().as_ticks(),
        stk.ctlr().read().stie());
    
    Timer::after_millis(100).await;
    println!("Timer completed!");
    
    // Test loop with frame counter
    let mut frame = 0u32;
    loop {
        if frame % 10 == 0 {
            println!("Frame {}", frame);
        }
        Timer::after_millis(100).await;
        frame += 1;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}

