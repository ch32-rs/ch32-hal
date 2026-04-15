//! Simple Memory-to-Memory DMA test
//!
//! This tests the basic DMA mechanism using manual register configuration.

#![no_std]
#![no_main]

use ch32_hal as hal;
use hal::println;

const BUF_SIZE: usize = 64;

#[repr(C, align(4))]
struct AlignedBuf([u8; BUF_SIZE]);

static mut SRC_BUF: AlignedBuf = AlignedBuf([0u8; BUF_SIZE]);
static mut DST_BUF: AlignedBuf = AlignedBuf([0u8; BUF_SIZE]);

#[hal::entry]
fn main() -> ! {
    hal::debug::SDIPrint::enable();
    
    // Use 144MHz for proper timing
    let mut config = hal::Config::default();
    config.rcc = hal::rcc::Config::SYSCLK_FREQ_144MHZ_HSI;
    let _p = hal::init(config);

    println!("\n=== DMA Mem-to-Mem Test ===");
    
    // Check PFIC configuration
    let sctlr = hal::pac::PFIC.sctlr().read();
    println!("SCTLR: 0x{:08x}", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());

    // Initialize source buffer
    let src = unsafe { &mut SRC_BUF.0 };
    let dst = unsafe { &mut DST_BUF.0 };
    
    for i in 0..BUF_SIZE {
        src[i] = i as u8;
        dst[i] = 0xFF;
    }
    
    println!("Source: {:02x} {:02x} {:02x} {:02x}...", src[0], src[1], src[2], src[3]);
    println!("Dest before: {:02x} {:02x} {:02x} {:02x}...", dst[0], dst[1], dst[2], dst[3]);

    // Enable DMA1 clock
    hal::pac::RCC.ahbpcenr().modify(|w| w.set_dma1en(true));

    // Configure DMA1_CH1 for memory-to-memory transfer
    let dma1 = hal::pac::DMA1;
    let ch = dma1.ch(0); // Channel 1 = index 0
    
    // Disable channel first
    ch.cr().write(|w| w.set_en(false));
    
    // Configure addresses
    ch.par().write_value(src.as_ptr() as u32);
    ch.mar().write_value(dst.as_mut_ptr() as u32);
    ch.ndtr().write(|w| w.set_ndt(BUF_SIZE as u16));
    
    // Configure channel:
    // - MEM2MEM = 1 (memory to memory)
    // - PL = 0 (low priority)
    // - MSIZE = 0 (8-bit)
    // - PSIZE = 0 (8-bit)
    // - MINC = 1 (memory increment)
    // - PINC = 1 (peripheral/source increment)
    // - CIRC = 0 (no circular)
    // - DIR = 0 (read from peripheral/source)
    // - TEIE = 0
    // - HTIE = 0
    // - TCIE = 1 (transfer complete interrupt enable)
    ch.cr().write(|w| {
        w.set_mem2mem(true);
        w.set_minc(true);
        w.set_pinc(true);
        w.set_tcie(true);
        w.set_en(false); // Don't enable yet
    });
    
    println!("\nDMA1_CH1 configured:");
    println!("  PAR: 0x{:08x}", ch.par().read());
    println!("  MAR: 0x{:08x}", ch.mar().read());
    println!("  NDTR: {}", ch.ndtr().read().ndt());
    println!("  CR: 0x{:08x}", ch.cr().read().0);

    // Check DMA interrupt is enabled in PFIC
    let dma1_ch1_irq = 27u8;
    println!("DMA1_CH1 IRQ({}) enabled: {}", dma1_ch1_irq, qingke::pfic::is_enabled(dma1_ch1_irq));

    println!("\n--- Starting DMA transfer ---");
    
    // Enable the channel to start transfer
    ch.cr().modify(|w| w.set_en(true));
    
    // Poll for completion
    for i in 0..1000 {
        let ndtr = ch.ndtr().read().ndt();
        let isr = dma1.isr().read();
        
        if i < 5 || ndtr == 0 || i % 100 == 0 {
            println!("poll {}: NDTR={} TCIF1={} GIF1={}", i, ndtr, isr.tcif(0), isr.gif(0));
        }
        
        if isr.tcif(0) {
            println!("DMA complete at poll {} (TCIF set)", i);
            // Clear interrupt flag
            dma1.ifcr().write(|w| w.set_tcif(0, true));
            break;
        }
        
        if ndtr == 0 && i > 0 {
            println!("DMA complete at poll {} (NDTR=0)", i);
            break;
        }
    }
    
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
        println!("\n=== DMA Transfer SUCCESS ===");
    } else {
        println!("\n=== DMA Transfer FAILED ===");
    }

    println!("\nDone. Halting.");
    loop {
        unsafe { core::arch::asm!("wfi"); }
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\nPANIC: {}", info);
    loop {}
}
