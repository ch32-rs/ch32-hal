//! SDIO self-check (V3F only).
//!
//! Brings up the SDIO host on the EVT wiring — CK=PB11, CMD=PB10 and
//! D0..D3=PE8..PE11, all AF8 — with DMA1 channel 1, then initializes a card at
//! 2.5 MHz. The data path needs an inserted microSD card; without one the check
//! still validates controller bring-up and reports the no-response outcome
//! instead of failing. Mailbox (`SdioCheck`): stage 1 = controller
//! constructed, 2 = card init attempted (and the read checks done when a card
//! answered), 3 = no check failed; `card` is 0 for no card and 1 for an
//! initialized one, `error` is 1 on a timeout and 2 on any other init error.

#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::SdioCheck;
use hal::sdio::{Config, DataBlock, Error, InterruptHandler, Sdmmc};
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals};

ipc::mailbox!(SdioCheck);
use panic_halt as _;

bind_interrupts!(struct Irqs {
    SDIO => InterruptHandler<peripherals::SDIO>;
});

fn error_code(error: Error) -> u32 {
    match error {
        Error::Timeout | Error::NoCard => 1,
        _ => 2,
    }
}

fn check(failures: &mut u32, checks: &mut u32, condition: bool) {
    *checks += 1;
    *failures += u32::from(!condition);
    log!("check {}: {}", *checks, if condition { "ok" } else { "FAIL" });
}

#[ch32_hal::entry]
fn main() -> ! {
    let mailbox = mailbox();
    mailbox.clear();
    mailbox.payload.stage.store(1, Relaxed);

    // Use the EVT clock tree with a conservative SDIO rate verified on this board.
    let config = hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    };
    let p = hal::init(config);
    let mut failures = 0u32;
    let mut checks = 0u32;

    let mut sd = match Sdmmc::new_4bit(
        p.SDIO,
        Irqs,
        p.DMA1_CH1,
        p.PB11,
        p.PB10,
        p.PE8,
        p.PE9,
        p.PE10,
        p.PE11,
        Config::default(),
    ) {
        Ok(sd) => sd,
        Err(error) => {
            // A rejected configuration is a real failure even without a card.
            mailbox.payload.error.store(error_code(error), Relaxed);
            mailbox.payload.failures.store(1, Relaxed);
            mailbox.payload.stage.store(2, Relaxed);
            idle();
        }
    };
    // The constructor must have taken the SDIO clock and written the per-pin
    // AF numbers: PB10/PB11 sit in AFR[3] fields 2 and 3, PE8..PE11 in AFR[9]
    // fields 0..3.
    let afio = hal::pac::AFIO;
    check(&mut failures, &mut checks, hal::pac::RCC.hb2pcenr().read().sdioen());
    check(
        &mut failures,
        &mut checks,
        afio.gpio_afr(3).read().afr(2) == 8
            && afio.gpio_afr(3).read().afr(3) == 8
            && (0..4).all(|field| afio.gpio_afr(9).read().afr(field) == 8),
    );

    mailbox.payload.stage.store(2, Relaxed);
    log!("init at 2.5 MHz");
    let init = sd.blocking_init_card(Hertz(2_500_000));
    match init {
        Ok(()) => {
            log!("init ok, clock {}", sd.clock().0);
            mailbox.payload.card.store(1, Relaxed);
            let card = *sd.card().expect("card was just initialized");
            mailbox.payload.blocks.store(card.block_count as u32, Relaxed);
            mailbox.payload.clock.store(sd.clock().0, Relaxed);
            check(&mut failures, &mut checks, sd.clock().0 <= 2_500_000);

            let mut first = DataBlock([0; 512]);
            check(
                &mut failures,
                &mut checks,
                sd.blocking_read_block(0, &mut first).is_ok(),
            );
            mailbox
                .payload
                .first_word
                .store(u32::from_le_bytes([first[0], first[1], first[2], first[3]]), Relaxed);

            let mut repeated = DataBlock([0; 512]);
            check(
                &mut failures,
                &mut checks,
                sd.blocking_read_block(0, &mut repeated).is_ok(),
            );
            check(&mut failures, &mut checks, first == repeated);

            let mut blocks = [DataBlock([0; 512]), DataBlock([0; 512])];
            check(
                &mut failures,
                &mut checks,
                sd.blocking_read_blocks(0, &mut blocks).is_ok(),
            );
            check(&mut failures, &mut checks, blocks[0] == first);

            let mut second = DataBlock([0; 512]);
            check(
                &mut failures,
                &mut checks,
                sd.blocking_read_block(1, &mut second).is_ok(),
            );
            check(&mut failures, &mut checks, blocks[1] == second);

            check(&mut failures, &mut checks, sd.blocking_read_sd_status().is_ok());
        }
        Err(error) => {
            log!("init failed: err {}", error_code(error));
            // No card is a valid outcome: the controller still came up and the
            // command path timed out as designed. Anything else is a defect.
            mailbox.payload.error.store(error_code(error), Relaxed);
            failures += u32::from(error_code(error) != 1);
        }
    }
    log!("done: {}/{} failed", failures, checks);

    mailbox.payload.status.store(hal::pac::SDIO.sta().read().0, Relaxed);
    mailbox.payload.failures.store(failures, Relaxed);
    mailbox.payload.checks.store(checks, Relaxed);
    mailbox.payload.stage.store(if failures == 0 { 3 } else { 2 }, Relaxed);
    idle();
}

fn idle() -> ! {
    let mut delay = hal::delay::Delay;
    loop {
        delay.delay_ms(1000);
    }
}
