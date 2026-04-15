//! Async DMA test using executor
//!
//! This tests the executor wake mechanism with DMA interrupts.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use hal::dma::{Transfer, TransferOptions};
use hal::println;
use embassy_executor::Spawner;

const BUF_SIZE: usize = 64;

#[repr(C, align(4))]
struct AlignedBuf([u8; BUF_SIZE]);

static mut SRC_BUF: AlignedBuf = AlignedBuf([0u8; BUF_SIZE]);
static mut DST_BUF: AlignedBuf = AlignedBuf([0u8; BUF_SIZE]);

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    
    // Use 144MHz for proper timing
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let p = hal::init(config);

    println!("\n=== Async DMA Test ===");
    
    // Check PFIC configuration
    let sctlr = hal::pac::PFIC.sctlr().read();
    println!("SCTLR: 0x{:08x}", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());

    // Initialize buffers
    let src = unsafe { &mut SRC_BUF.0 };
    let dst = unsafe { &mut DST_BUF.0 };
    
    for i in 0..BUF_SIZE {
        src[i] = i as u8;
        dst[i] = 0xFF;
    }
    
    println!("Source: {:02x} {:02x} {:02x} {:02x}...", src[0], src[1], src[2], src[3]);
    println!("Dest before: {:02x} {:02x} {:02x} {:02x}...", dst[0], dst[1], dst[2], dst[3]);

    // Check DMA interrupt is enabled
    let dma1_ch1_irq = 27u8;
    println!("DMA1_CH1 IRQ({}) enabled: {}", dma1_ch1_irq, qingke::pfic::is_enabled(dma1_ch1_irq));

    println!("\n--- Starting async DMA transfer ---");
    
    // Manual DMA configuration for mem-to-mem
    let dma1 = hal::pac::DMA1;
    let ch = dma1.ch(0);
    
    // Disable channel first
    ch.cr().write(|w| w.set_en(false));
    
    // Configure for mem-to-mem
    ch.par().write_value(src.as_ptr() as u32);
    ch.mar().write_value(dst.as_mut_ptr() as u32);
    ch.ndtr().write(|w| w.set_ndt(BUF_SIZE as u16));
    
    ch.cr().write(|w| {
        w.set_mem2mem(true);  // Key: enable mem-to-mem
        w.set_minc(true);
        w.set_pinc(true);
        w.set_tcie(true);
        w.set_en(false);  // Don't enable yet
    });
    
    println!("DMA configured for mem2mem");
    println!("  CR: 0x{:08x}", ch.cr().read().0);
    println!("  MEM2MEM: {}", ch.cr().read().mem2mem());
    println!("  TCIE: {}", ch.cr().read().tcie());
    
    // Now use Transfer to wrap this channel for async await
    // We need to enable the channel AFTER creating the Transfer
    let transfer = unsafe {
        // Create transfer without starting (channel already configured)
        Transfer::new_write(
            p.DMA1_CH1,
            (),
            src,
            dst.as_mut_ptr(),
            TransferOptions::default(),
        )
    };
    
    // Check what Transfer did to our config
    println!("After Transfer::new_write:");
    println!("  CR: 0x{:08x}", ch.cr().read().0);
    println!("  MEM2MEM: {}", ch.cr().read().mem2mem());
    println!("  EN: {}", ch.cr().read().en());
    
    println!("Awaiting...");
    
    // This should block until DMA completes
    transfer.await;
    
    println!("Transfer.await returned!");

    // Check result
    println!("\nDest after: {:02x} {:02x} {:02x} {:02x}...", dst[0], dst[1], dst[2], dst[3]);
    
    let mut match_count = 0;
    for i in 0..BUF_SIZE {
        if src[i] == dst[i] {
            match_count += 1;
        }
    }
    println!("Match: {}/{}", match_count, BUF_SIZE);
    
    if match_count == BUF_SIZE {
        println!("\n=== Async DMA SUCCESS ===");
    } else {
        println!("\n=== Async DMA FAILED ===");
    }

    println!("\nTest complete. Looping.");
    loop {
        // Use yield to test executor continues working
        embassy_futures::yield_now().await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\nPANIC: {}", info);
    loop {}
}

