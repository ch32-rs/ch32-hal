//! Async SPI DMA test
//!
//! This tests the executor wake mechanism with SPI DMA.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use hal::println;
use hal::spi::{Config, Spi};
use hal::time::Hertz;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

const BUF_SIZE: usize = 64;

#[repr(C, align(4))]
struct AlignedBuf([u8; BUF_SIZE]);

static mut TX_BUF: AlignedBuf = AlignedBuf([0u8; BUF_SIZE]);

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(_spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    
    // Use 144MHz for proper timing
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let p = hal::init(config);

    println!("\n=== Async SPI DMA Test ===");
    
    // Check PFIC configuration
    let sctlr = hal::pac::PFIC.sctlr().read();
    println!("SCTLR: 0x{:08x}", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());

    // Initialize buffer
    let buf = unsafe { &mut TX_BUF.0 };
    for i in 0..BUF_SIZE {
        buf[i] = i as u8;
    }

    // Configure SPI with DMA
    let mut spi_config = Config::default();
    spi_config.frequency = Hertz::mhz(1);  // 1MHz for easier debugging
    
    let mut spi = Spi::new_txonly(
        p.SPI1,
        p.PA5,      // SCK
        p.PA7,      // MOSI
        p.DMA1_CH3, // TX DMA
        spi_config,
    );
    
    println!("SPI configured at 1MHz with DMA1_CH3");
    
    // Check DMA interrupt is enabled
    let dma1_ch3_irq = 29u8;
    println!("DMA1_CH3 IRQ({}) enabled: {}", dma1_ch3_irq, qingke::pfic::is_enabled(dma1_ch3_irq));

    println!("\n--- Starting async SPI transfer ---");
    println!("Buffer: {:02x} {:02x} {:02x} {:02x}...", buf[0], buf[1], buf[2], buf[3]);
    
    // Check DMA state before
    let dma1 = hal::pac::DMA1;
    println!("Before spi.write:");
    println!("  CH3 CR: 0x{:08x}", dma1.ch(2).cr().read().0);
    
    // Start SPI write but poll manually
    println!("Creating spi.write future...");
    
    // Manually poll DMA for a bit to see if it completes
    for i in 0..10 {
        let cr = dma1.ch(2).cr().read();
        let ndtr = dma1.ch(2).ndtr().read().ndt();
        let isr = dma1.isr().read();
        println!("poll {}: EN={} NDTR={} TCIF3={} GIF3={}", 
            i, cr.en(), ndtr, isr.tcif(2), isr.gif(2));
        
        if isr.tcif(2) {
            println!("DMA complete!");
            break;
        }
        
        // Delay
        for _ in 0..100000 { core::hint::spin_loop(); }
    }
    
    // Now do the actual write
    println!("\nCalling spi.write().await...");
    let result = spi.write(buf).await;
    
    println!("After spi.write:");
    println!("  CH3 NDTR: {}", dma1.ch(2).ndtr().read().ndt());
    println!("  ISR: 0x{:08x}", dma1.isr().read().0);
    
    match result {
        Ok(()) => println!("spi.write() completed successfully!"),
        Err(_) => println!("spi.write() failed!"),
    }

    println!("\n=== Async SPI Test Complete ===");

    println!("\nLooping...");
    loop {
        embassy_futures::yield_now().await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\nPANIC: {}", info);
    loop {}
}

