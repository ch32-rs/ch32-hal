//! WS2812 LED control using PWM + DMA

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(impl_trait_in_assoc_type)]

use ch32_hal as hal;
use embassy_executor::Spawner;
use embassy_time::Timer;
use hal::gpio::{Level, Output};
use hal::println;
use hal::time::Hertz;
use hal::timer::low_level::CountingMode;
use hal::timer::simple_pwm::{PwmPin, SimplePwm};
use hal::timer::Channel;

// Number of WS2812 LEDs
const NUM_LEDS: usize = 1; // Start with 1 LED for debugging
const RESET_LEN: usize = 50;
const BUFFER_LEN: usize = NUM_LEDS * 24 + RESET_LEN;

static mut DMA_BUFFER: [u16; BUFFER_LEN] = [0u16; BUFFER_LEN];

// Blinky task to show system is alive
#[embassy_executor::task]
async fn blinky(pin: hal::Peri<'static, hal::peripherals::PA8>) {
    let mut led = Output::new(pin, Level::Low, Default::default());
    loop {
        led.toggle();
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::main(entry = "ch32_hal::entry")]
async fn main(spawner: Spawner) -> ! {
    hal::debug::SDIPrint::enable();
    let mut p = hal::init(Default::default());

    println!("WS2812 PWM + DMA Debug ({} LED)", NUM_LEDS);

    // Start blinky task on PA8
    spawner.spawn(blinky(p.PA8)).unwrap();
    println!("Blinky task started on PA8");

    let ch1_pin = PwmPin::new_ch1::<1>(p.PA15);

    let mut pwm = SimplePwm::new(
        p.TIM2,
        Some(ch1_pin),
        None,
        None,
        None,
        Hertz::khz(800),
        CountingMode::EdgeAlignedUp,
    );

    let max_duty = pwm.get_max_duty() as u16;
    println!("max_duty: {} (ARR+1)", max_duty);

    // WS2812 bit encoding:
    // Bit 0: ~30% duty (T0H ~0.4us)
    // Bit 1: ~70% duty (T1H ~0.8us)
    let n0 = (3 * max_duty / 10) as u16; // 3 for max_duty=10
    let n1 = (7 * max_duty / 10) as u16; // 7 for max_duty=10
    println!("n0={}, n1={}", n0, n1);

    let pwm_channel = Channel::Ch1;
    pwm.set_duty(pwm_channel, 0);
    pwm.enable(pwm_channel);

    let buffer = unsafe { &mut DMA_BUFFER };

    // WS2812 data format: GRB, MSB first
    // For RED color: G=0, R=255, B=0
    // Bits 0-7: Green (all 0 = n0)
    // Bits 8-15: Red (all 1 = n1)
    // Bits 16-23: Blue (all 0 = n0)

    let dma1 = hal::pac::DMA1;
    let tim2 = hal::pac::TIM2;

    println!("\n=== Timer config ===");
    println!("TIM2 ATRLR (ARR): {}", tim2.atrlr().read());
    println!("TIM2 PSC: {}", tim2.psc().read());

    // Test 1: Send RED (G=0, R=255, B=0)
    println!("\n=== Test 1: RED (G=0, R=255, B=0) ===");
    build_color(buffer, n0, n1, 0, 255, 0);
    print_buffer(buffer);
    send_buffer(&mut pwm, pwm_channel, buffer, &dma1, &tim2, p.DMA1_CH2.reborrow()).await;
    println!("LED should be RED. Waiting 3s...");
    Timer::after_millis(3000).await;

    // Test 2: Send GREEN (G=255, R=0, B=0)
    println!("\n=== Test 2: GREEN (G=255, R=0, B=0) ===");
    build_color(buffer, n0, n1, 255, 0, 0);
    print_buffer(buffer);
    send_buffer(&mut pwm, pwm_channel, buffer, &dma1, &tim2, p.DMA1_CH2.reborrow()).await;
    println!("LED should be GREEN. Waiting 3s...");
    Timer::after_millis(3000).await;

    // Test 3: Send BLUE (G=0, R=0, B=255)
    println!("\n=== Test 3: BLUE (G=0, R=0, B=255) ===");
    build_color(buffer, n0, n1, 0, 0, 255);
    print_buffer(buffer);
    send_buffer(&mut pwm, pwm_channel, buffer, &dma1, &tim2, p.DMA1_CH2.reborrow()).await;
    println!("LED should be BLUE. Waiting 3s...");
    Timer::after_millis(3000).await;

    // Test 4: Send CYAN (G=255, R=0, B=255)
    println!("\n=== Test 4: CYAN (G=255, R=0, B=255) ===");
    build_color(buffer, n0, n1, 255, 0, 255);
    send_buffer(&mut pwm, pwm_channel, buffer, &dma1, &tim2, p.DMA1_CH2.reborrow()).await;
    println!("LED should be CYAN. Waiting 3s...");
    Timer::after_millis(3000).await;

    println!("\n=== Cycling R-G-B ===");
    loop {
        build_color(buffer, n0, n1, 0, 64, 0); // Red
        pwm.waveform_up(p.DMA1_CH2.reborrow(), pwm_channel, buffer).await;
        println!("RED");
        Timer::after_millis(1000).await;

        build_color(buffer, n0, n1, 64, 0, 0); // Green
        pwm.waveform_up(p.DMA1_CH2.reborrow(), pwm_channel, buffer).await;
        println!("GREEN");
        Timer::after_millis(1000).await;

        build_color(buffer, n0, n1, 0, 0, 64); // Blue
        pwm.waveform_up(p.DMA1_CH2.reborrow(), pwm_channel, buffer).await;
        println!("BLUE");
        Timer::after_millis(1000).await;
    }
}

fn build_color(buffer: &mut [u16], n0: u16, n1: u16, g: u8, r: u8, b: u8) {
    // WS2812: GRB order, MSB first
    // Green (bits 0-7)
    for i in 0..8 {
        buffer[i] = if (g >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
    // Red (bits 8-15)
    for i in 0..8 {
        buffer[8 + i] = if (r >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
    // Blue (bits 16-23)
    for i in 0..8 {
        buffer[16 + i] = if (b >> (7 - i)) & 1 == 1 { n1 } else { n0 };
    }
    // Reset signal
    for i in 24..BUFFER_LEN {
        buffer[i] = 0;
    }
}

fn print_buffer(buffer: &[u16]) {
    println!("Buffer [0..23]:");
    println!("  G: {} {} {} {} {} {} {} {}", 
        buffer[0], buffer[1], buffer[2], buffer[3],
        buffer[4], buffer[5], buffer[6], buffer[7]);
    println!("  R: {} {} {} {} {} {} {} {}", 
        buffer[8], buffer[9], buffer[10], buffer[11],
        buffer[12], buffer[13], buffer[14], buffer[15]);
    println!("  B: {} {} {} {} {} {} {} {}", 
        buffer[16], buffer[17], buffer[18], buffer[19],
        buffer[20], buffer[21], buffer[22], buffer[23]);
}

async fn send_buffer<'a, T: hal::timer::GeneralInstance16bit>(
    pwm: &mut SimplePwm<'_, T>,
    channel: Channel,
    buffer: &[u16],
    dma1: &hal::pac::dma::Dma,
    tim2: &hal::pac::timer::Gptm,
    dma_ch: hal::Peri<'a, impl hal::timer::UpDma<T>>,
) {
    use hal::dma::{Transfer, TransferOptions};

    // Stop timer
    tim2.ctlr1().modify(|w| w.set_cen(false));

    // Enable UDE
    tim2.dmaintenr().modify(|w| w.set_ude(true));

    // Clear DMA flags
    dma1.ifcr().write(|w| {
        w.set_gif(1, true);
        w.set_tcif(1, true);
        w.set_htif(1, true);
        w.set_teif(1, true);
    });

    let ccr_addr = tim2.chcvr(0).as_ptr() as *mut u16;

    // Create transfer
    let mut transfer = unsafe {
        Transfer::new_write(dma_ch, (), buffer, ccr_addr, TransferOptions::default())
    };

    println!("DMA: NDTR={}", dma1.ch(1).ndtr().read().ndt());

    // Start timer
    tim2.ctlr1().modify(|w| w.set_cen(true));

    // Poll for completion
    let mut count = 0u32;
    loop {
        let ndtr = dma1.ch(1).ndtr().read().ndt();
        let running = transfer.is_running();

        if !running {
            println!("Done! NDTR={}", ndtr);
            break;
        }

        count += 1;
        if count > 100000 {
            println!("Timeout! NDTR={}", ndtr);
            break;
        }
    }

    // Cleanup
    tim2.dmaintenr().modify(|w| w.set_ude(false));
    pwm.set_duty(channel, 0);
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let _ = hal::println!("\n\n\n{}", info);
    loop {}
}
