//! USART2–8 and SPI2–4 self-check (V3F only).
//!
//! The generic USART/SPI drivers were only ever run on the first instance
//! (USART1 in `hello`, SPI1 in `spi_check`). This constructs every other
//! instance with its own pins, pushes a byte through the transmitter and
//! requires the completion flag, so a wrong RCC gate or pin AF shows up.
//! Reception is not covered: there is nothing on the other end of the pins.
//! `usart_mask`/`spi_mask` have one bit per passing instance (USART2 is bit 0,
//! USART8 bit 6; SPI2 bit 0, SPI4 bit 2).
//!
//! I2C2–4 are construction-only. Their pins have no pull-ups on this board and
//! a blocking write has no timeout unless `embassy` is enabled, so a transfer
//! attempt could spin forever; the check stops at `CTLR1.PE` instead.

#![no_std]
#![no_main]

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::BusCheck;

ipc::mailbox!(BusCheck);
use panic_halt as _;

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    let mut p = hal::init(hal::Config::default());
    let mut checks = 0u32;
    let mut failures = 0u32;
    let mut usart_mask = 0u32;
    let mut spi_mask = 0u32;
    let mut i2c_mask = 0u32;

    macro_rules! usart {
        ($peri:ident, $rx:ident, $tx:ident, $bit:expr) => {{
            let mut cfg = hal::usart::Config::default();
            cfg.baudrate = 115_200;
            match hal::usart::Uart::new_blocking(p.$peri.reborrow(), p.$rx.reborrow(), p.$tx.reborrow(), cfg) {
                Ok(mut uart) => {
                    let written = uart.blocking_write(&[0x55, 0xa5]).is_ok();
                    let flushed = uart.blocking_flush().is_ok();
                    check(&mut failures, &mut checks, written);
                    check(&mut failures, &mut checks, flushed);
                    if written && flushed {
                        usart_mask |= 1 << $bit;
                    }
                }
                Err(_) => {
                    check(&mut failures, &mut checks, false);
                    check(&mut failures, &mut checks, false);
                }
            }
        }};
    }

    macro_rules! spi {
        ($peri:ident, $sck:ident, $mosi:ident, $miso:ident, $bit:expr) => {{
            let mut bus = hal::spi::Spi::new_blocking(
                p.$peri.reborrow(),
                p.$sck.reborrow(),
                p.$mosi.reborrow(),
                p.$miso.reborrow(),
                hal::spi::Config::default(),
            );
            let written = bus.blocking_write(&[0x55u8; 4]).is_ok();
            check(&mut failures, &mut checks, written);
            if written {
                spi_mask |= 1 << $bit;
            }
        }};
    }

    usart!(USART2, PA3, PA2, 0);
    usart!(USART3, PB11, PB10, 1);
    usart!(USART4, PC7, PC6, 2);
    usart!(USART5, PF5, PE3, 3);
    usart!(USART6, PA1, PA0, 4);
    usart!(USART7, PB5, PB6, 5);
    usart!(USART8, PA8, PA15, 6);

    spi!(SPI2, PB13, PB15, PB14, 0);
    spi!(SPI3, PB3, PB2, PB4, 1);
    spi!(SPI4, PE2, PE6, PE5, 2);

    macro_rules! i2c {
        ($peri:ident, $scl:ident, $sda:ident, $pac:ident, $bit:expr) => {{
            let _bus = hal::i2c::I2c::new_blocking(
                p.$peri.reborrow(),
                p.$scl.reborrow(),
                p.$sda.reborrow(),
                hal::time::Hertz(100_000),
                hal::i2c::Config::default(),
            );
            let enabled = hal::pac::$pac.ctlr1().read().pe();
            check(&mut failures, &mut checks, enabled);
            if enabled {
                i2c_mask |= 1 << $bit;
            }
        }};
    }

    i2c!(I2C2, PC0, PC1, I2C2, 0);
    i2c!(I2C3, PA14, PC9, I2C3, 1);
    i2c!(I2C4, PD12, PD13, I2C4, 2);

    mailbox.payload.usart_mask.store(usart_mask, Relaxed);
    mailbox.payload.spi_mask.store(spi_mask, Relaxed);
    mailbox.payload.i2c_mask.store(i2c_mask, Relaxed);
    mailbox.payload.pclk1.store(hal::rcc::clocks().pclk1.0, Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.stage.store(2, Relaxed);
    if failures == 0 {
        mailbox.payload.stage.store(3, Relaxed);
    }

    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
