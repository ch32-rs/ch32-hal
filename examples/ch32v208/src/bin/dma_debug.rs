//! SPI DMA debug test
//!
//! This tests if SPI DMA transfers work correctly.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::mode::Async;
use hal::println;
use hal::spi::{Config, Spi};
use hal::time::Hertz;

#[embassy_executor::task]
async fn blinky(pin: hal::Peri<'static, hal::peripherals::PA8>) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

// Test buffer - aligned for DMA
#[repr(C, align(4))]
struct AlignedBuffer([u8; 64]);
static mut TEST_BUFFER: AlignedBuffer = AlignedBuffer([0xAAu8; 64]);

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let p = hal::init(Default::default());

    println!("SPI DMA Debug Test");

    // Start blinky to show system is alive
    spawner.spawn(blinky(p.PA8)).unwrap();
    println!("Blinky started");

    // Initialize SPI with DMA
    let mut spi_config = Config::default();
    spi_config.frequency = Hertz::mhz(6);

    // SPI1_TX uses DMA1_CH3
    let mut spi = Spi::new_txonly::<0>(
        p.SPI1,
        p.PA5, // SCK
        p.PA7, // MOSI
        p.DMA1_CH3, // TX DMA channel
        spi_config,
    );

    println!("SPI initialized with DMA1_CH3");

    let buffer = unsafe { &mut TEST_BUFFER.0 };
    
    // Fill buffer with test pattern
    for i in 0..64 {
        buffer[i] = i as u8;
    }

    // Check registers before transfer
    let dma1 = hal::pac::DMA1;
    let spi1 = hal::pac::SPI1;
    
    // Check PFIC SCTLR for SEVONPEND
    let sctlr = hal::pac::PFIC.sctlr().read();
    println!("\n=== PFIC SCTLR: 0x{:08x} ===", sctlr.0);
    println!("  SEVONPEND: {}", sctlr.sevonpend());
    println!("  WFITOWFE: {}", sctlr.wfitowfe());
    
    println!("\n=== Before SPI DMA transfer ===");
    println!("DMA1 ISR: 0x{:08x}", dma1.isr().read().0);
    println!("DMA1_CH3 CR: 0x{:08x}", dma1.ch(2).cr().read().0);
    println!("DMA1_CH3 NDTR: {}", dma1.ch(2).ndtr().read().ndt());
    println!("SPI1 CTLR1: 0x{:08x}", spi1.ctlr1().read().0);
    println!("SPI1 CTLR2: 0x{:08x}", spi1.ctlr2().read().0);
    println!("SPI1 STATR: 0x{:08x}", spi1.statr().read().0);
    
    // Check PFIC - DMA1_CH3 is IRQ 29
    let isr0 = unsafe { core::ptr::read_volatile(0xE000E000 as *const u32) };
    println!("PFIC ISR0: 0x{:08x}", isr0);
    println!("DMA1_CH3 IRQ (29) enabled: {}", (isr0 >> 29) & 1 == 1);
    
    println!("\n=== Debugging waker mechanism ===");
    
    // Manually poll to understand the flow
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, RawWaker, RawWakerVTable, Waker};
    
    // Create a waker that prints when called
    static mut WAKE_CALLED: bool = false;
    
    fn dummy_clone(ptr: *const ()) -> RawWaker {
        RawWaker::new(ptr, &VTABLE)
    }
    fn dummy_wake(_ptr: *const ()) {
        unsafe { WAKE_CALLED = true; }
        println!("  >>> WAKE() called! <<<");
    }
    fn dummy_wake_by_ref(_ptr: *const ()) {
        unsafe { WAKE_CALLED = true; }
        println!("  >>> WAKE_BY_REF() called! <<<");
    }
    fn dummy_drop(_ptr: *const ()) {}
    
    static VTABLE: RawWakerVTable = RawWakerVTable::new(dummy_clone, dummy_wake, dummy_wake_by_ref, dummy_drop);
    
    let raw_waker = RawWaker::new(core::ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut cx = Context::from_waker(&waker);
    
    let mut write_fut = pin!(spi.write(buffer));
    
    unsafe { WAKE_CALLED = false; }
    
    // Print channel info to verify
    println!("=== DMA Channel Info ===");
    println!("  SPI is using DMA1_CH3 for TX");
    
    println!("Before poll 1");
    
    // Check DMA channel state index
    // DMA1_CH3 should map to channel index 2 in STATE array
    // Let's verify by checking what the SPI driver uses
    
    // Check complete_count before poll
    // We need to access the channel to call debug_complete_count
    // But the SPI owns the DMA channel, so we can't access it directly here
    // Let's just trace through the poll
    
    let poll1 = write_fut.as_mut().poll(&mut cx);
    println!("Poll 1: pending={}", poll1.is_pending());
    
    // Read DMA state manually
    let cr = dma1.ch(2).cr().read();
    let ndtr = dma1.ch(2).ndtr().read().ndt();
    let isr = dma1.isr().read();
    
    println!("  DMA1_CH3 CR: 0x{:08x}", cr.0);
    println!("    EN={} TCIE={} HTIE={} CIRC={}", cr.en(), cr.tcie(), cr.htie(), cr.circ());
    println!("  DMA1_CH3 NDTR: {}", ndtr);
    println!("  DMA1 ISR: 0x{:08x}", isr.0);
    println!("    GIF3={} TCIF3={} HTIF3={} TEIF3={}", 
             isr.gif(2), isr.tcif(2), isr.htif(2), isr.teif(2));
    
    // Check ISR bit positions manually
    // CH1: bits 0-3, CH2: bits 4-7, CH3: bits 8-11, CH4: bits 12-15
    // Within each group: GIF=0, TCIF=1, HTIF=2, TEIF=3
    let raw_isr = isr.0;
    println!("    Raw: GIF3(b8)={} TCIF3(b9)={} HTIF3(b10)={} TEIF3(b11)={}",
             (raw_isr >> 8) & 1,
             (raw_isr >> 9) & 1,
             (raw_isr >> 10) & 1,
             (raw_isr >> 11) & 1);
    
    // Check what tcif() returns for different indices
    // This tells us if the function expects 0-based or 1-based
    println!("    tcif(0)={} tcif(1)={} tcif(2)={} tcif(3)={}",
             isr.tcif(0), isr.tcif(1), isr.tcif(2), isr.tcif(3));
    
    // According to CH32V_V4C.yaml, SPI1_TX uses channel 3 (1-based)
    // So in on_irq, info.num = 3, and tcif(3) is used
    // But tcif(3) checks CH4's TCIF (bit 13), not CH3's (bit 9)!
    println!("    If info.num=3 (1-based), but funcs expect 0-based, we have off-by-one!");
    
    // Verify: check DMA channel registers at different indices
    // CH3 hardware should be at index 2 (0-based)
    println!("\n=== Verifying channel register access ===");
    println!("dma1.ch(0).cr() = 0x{:08x}", dma1.ch(0).cr().read().0);
    println!("dma1.ch(1).cr() = 0x{:08x}", dma1.ch(1).cr().read().0);
    println!("dma1.ch(2).cr() = 0x{:08x}", dma1.ch(2).cr().read().0); // This should be CH3
    println!("dma1.ch(3).cr() = 0x{:08x}", dma1.ch(3).cr().read().0); // This should be CH4
    println!("Expected: ch(2) has the active config (0x309b), others should be 0");
    println!("  WAKE_CALLED: {}", unsafe { WAKE_CALLED });
    
    // Check: is this the waker being called during register() or from IRQ?
    // The timing tells us: if WAKE is called before the CR/NDTR reads, it's from IRQ
    
    // Wait for DMA to complete (spin wait)
    println!("Waiting for DMA...");
    for i in 0..100 {
        let ndtr = dma1.ch(2).ndtr().read().ndt();
        let isr = dma1.isr().read().0;
        if ndtr == 0 || (isr & 0x200) != 0 {
            println!("  DMA done at iteration {}: NDTR={} ISR=0x{:08x}", i, ndtr, isr);
            break;
        }
        for _ in 0..1000 { core::hint::spin_loop(); }
    }
    
    println!("After DMA complete:");
    println!("  WAKE_CALLED: {}", unsafe { WAKE_CALLED });
    println!("  DMA1 ISR: 0x{:08x}", dma1.isr().read().0);
    
    // Poll again
    unsafe { WAKE_CALLED = false; }
    let poll2 = write_fut.as_mut().poll(&mut cx);
    println!("Poll 2: pending={}", poll2.is_pending());
    println!("  WAKE_CALLED after poll2: {}", unsafe { WAKE_CALLED });
    
    println!("\nManual poll test done!");
    
    // Show final registers
    println!("\nFinal state:");
    println!("DMA1 ISR: 0x{:08x}", dma1.isr().read().0);
    println!("DMA1_CH3 CR: 0x{:08x}", dma1.ch(2).cr().read().0);

    println!("\nTest complete!");
    
    loop {
        Timer::after_secs(10).await;
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}

