//! Direct test of software interrupt waking up WFI
//!
//! This test does NOT use embassy executor, just raw WFI + software interrupt

#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::println;
use core::sync::atomic::{AtomicU32, Ordering};

static WAKE_COUNT: AtomicU32 = AtomicU32::new(0);

#[ch32_hal::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    let _p = hal::init(Default::default());

    println!("\n=== Direct WFI Wake Test ===");
    
    // Configure PFIC for WFE mode
    let pfic = hal::pac::PFIC;
    
    // Read initial SCTLR
    let sctlr = pfic.sctlr().read();
    println!("Initial SCTLR: 0x{:08x}", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());
    
    // Enable SEVONPEND and WFITOWFE
    pfic.sctlr().modify(|w| {
        w.set_sevonpend(true);
        w.set_wfitowfe(true);
    });
    
    let sctlr = pfic.sctlr().read();
    println!("After config SCTLR: 0x{:08x}", sctlr.0);
    
    const SOFTWARE_IRQ: u8 = 14;
    
    // Test 1: Clear pending, then pend, then check if pending
    println!("\n--- Test 1: pend/unpend ---");
    unsafe { qingke::pfic::unpend_interrupt(SOFTWARE_IRQ); }
    println!("After unpend: is_pending(14) = {}", qingke::pfic::is_pending(SOFTWARE_IRQ));
    
    unsafe { qingke::pfic::pend_interrupt(SOFTWARE_IRQ); }
    println!("After pend: is_pending(14) = {}", qingke::pfic::is_pending(SOFTWARE_IRQ));
    
    unsafe { qingke::pfic::unpend_interrupt(SOFTWARE_IRQ); }
    println!("After unpend: is_pending(14) = {}", qingke::pfic::is_pending(SOFTWARE_IRQ));
    
    // Test 2: WFI wake test with timer interrupt
    println!("\n--- Test 2: WFI with SysTick ---");
    println!("Configuring SysTick to interrupt in 1 second...");
    
    // Skip SysTick test, go straight to software interrupt test
    println!("(Skipping SysTick test)");
    
    // Test 3: WFI with software interrupt
    println!("\n--- Test 3: WFI with Software Interrupt ---");
    
    // Clear software interrupt
    unsafe { qingke::pfic::unpend_interrupt(SOFTWARE_IRQ); }
    println!("Software IRQ pending: {}", qingke::pfic::is_pending(SOFTWARE_IRQ));
    
    // Don't enable software interrupt in PFIC - SEVONPEND should wake us
    // But pend it immediately to test if SEVONPEND works
    println!("Pending software IRQ...");
    unsafe { qingke::pfic::pend_interrupt(SOFTWARE_IRQ); }
    println!("Software IRQ pending: {}", qingke::pfic::is_pending(SOFTWARE_IRQ));
    
    println!("Entering WFI... (should wake immediately if SEVONPEND works)");
    unsafe { core::arch::asm!("wfi"); }
    println!("WFI returned!");
    
    // Test 4: Skip for now
    println!("\n--- Test 4: Skipped ---");
    
    println!("\n=== Test Complete ===");
    
    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\nPANIC: {}", info);
    loop {}
}

