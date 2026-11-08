#![no_std]
#![no_main]

//! On-board check for the CRC, IWDG and WWDG drivers.
//!
//! The CRC vectors are checked in-run. A watchdog can only be observed by
//! letting it reset the chip, so the test walks a small state machine driven
//! by the RCC reset flags: the first boot checks the CRC and feeds the IWDG
//! for a while before arming it, the next boot records the IWDG reset and
//! arms the WWDG, and the boot after that records the WWDG reset and parks
//! with `STAGE_PASS`.
//!
//! Progress lives in `.uninit`, so it survives the watchdog resets. Run the
//! test headless and read the markers afterwards:
//!
//! ```text
//! probe-rs download out/wdg_crc --chip CH32V208WBU6 --probe <probe>
//! probe-rs reset --chip CH32V208WBU6 --probe <probe>
//! probe-rs read b32 <CRC_VALUE address> 1
//! probe-rs read b32 <STAGE address> 1
//! ```
//!
//! `STAGE_PASS` means the CRC matched, the IWDG was kept alive by `pet()`
//! and then reset the chip when left alone, and the WWDG was petted inside
//! its window and then reset the chip.

use core::sync::atomic::{AtomicU32, Ordering};

use ch32_hal as hal;
use ch32_hal::crc::Crc;
use ch32_hal::wdg::{IndependentWatchdog, WindowWatchdog};
use ch32_hal::{pac, peripherals, Peri};
use panic_halt as _;
use qingke::riscv;

/// CRC of the big-endian byte stream `"12345678"`, fed as two words.
const CRC_12345678: u32 = 0x49e3_c2fb;
/// CRC of the word `0x4142_4344` (`"ABCD"`).
const CRC_ABCD: u32 = 0xabcf_9a63;

const STAGE_IWDG_FEEDING: u32 = 0x10;
const STAGE_IWDG_ARMED: u32 = 0x11;
const STAGE_IWDG_RESET: u32 = 0x12;
const STAGE_WWDG_FEEDING: u32 = 0x13;
const STAGE_WWDG_ARMED: u32 = 0x14;
const STAGE_PASS: u32 = 0xa5a5_ffff;

const STAGE_FAIL_IWDG_FEED: u32 = 0xdead_0001;
const STAGE_FAIL_IWDG_NO_RESET: u32 = 0xdead_0002;
const STAGE_FAIL_WWDG_FEED: u32 = 0xdead_0003;
const STAGE_FAIL_WWDG_NO_RESET: u32 = 0xdead_0004;

/// Last CRC value the hardware produced, for the read-back when an assertion
/// fails. `#[used]` keeps it in the image although only a debugger reads it.
#[used]
#[link_section = ".uninit.crc_value"]
static CRC_VALUE: AtomicU32 = AtomicU32::new(0);

/// Test progress. This survives a watchdog reset, so a value left over from a
/// previous run tells a stale reset flag from one this run armed.
#[used]
#[link_section = ".uninit.stage"]
static STAGE: AtomicU32 = AtomicU32::new(0);

fn park() -> ! {
    loop {
        riscv::asm::delay(1_000_000);
    }
}

#[qingke_rt::entry]
fn main() -> ! {
    let p = hal::init(Default::default());
    let rcc = pac::RCC;

    let rst = rcc.rstsckr().read();
    let stage = STAGE.load(Ordering::Relaxed);

    if rst.iwdgrstf() {
        // Clear the flags so the next boot can tell the two watchdogs apart.
        rcc.rstsckr().modify(|w| w.set_rmvf(true));
        if stage == STAGE_IWDG_ARMED {
            STAGE.store(STAGE_IWDG_RESET, Ordering::Relaxed);
            run_wwdg(p.WWDG);
        } else if stage == STAGE_IWDG_FEEDING {
            // The watchdog fired while `pet()` was supposed to keep it alive.
            STAGE.store(STAGE_FAIL_IWDG_FEED, Ordering::Relaxed);
            park();
        }
        // Otherwise this is a stale flag from an earlier run: start over.
    } else if rst.wwdgrstf() {
        rcc.rstsckr().modify(|w| w.set_rmvf(true));
        if stage == STAGE_WWDG_ARMED {
            STAGE.store(STAGE_PASS, Ordering::Relaxed);
        } else if stage == STAGE_WWDG_FEEDING {
            STAGE.store(STAGE_FAIL_WWDG_FEED, Ordering::Relaxed);
        }
        park();
    }

    // First boot, or a stale reset flag: clear everything and run the test.
    rcc.rstsckr().modify(|w| w.set_rmvf(true));

    let mut crc = Crc::new(p.CRC);
    let value = crc.feed_words(&[0x3132_3334, 0x3536_3738]);
    CRC_VALUE.store(value, Ordering::Relaxed);
    assert_eq!(value, CRC_12345678);

    crc.reset();
    assert_eq!(crc.feed_word(0x4142_4344), CRC_ABCD);

    // The independent data register must not take part in the calculation.
    crc.set_independent_data(0xa5);
    assert_eq!(crc.independent_data(), 0xa5);
    assert_eq!(crc.read(), CRC_ABCD);

    run_iwdg(p.IWDG);
}

fn run_iwdg(iwdg: Peri<'static, peripherals::IWDG>) -> ! {
    // LSI / 8 with reload 0xfff is 1 s at the nominal 32.768 kHz LSI.
    let mut wdg = IndependentWatchdog::new(iwdg, 1_000_000);
    wdg.unleash();

    STAGE.store(STAGE_IWDG_FEEDING, Ordering::Relaxed);
    for _ in 0..10 {
        wdg.pet();
        // About 12.5 ms at 8 MHz; ten of these are well inside one period.
        riscv::asm::delay(100_000);
    }

    // Stop feeding: the chip must reset within the configured timeout (1 s
    // nominal; the spin below is about 2 s at 8 MHz).
    STAGE.store(STAGE_IWDG_ARMED, Ordering::Relaxed);
    riscv::asm::delay(16_000_000);
    STAGE.store(STAGE_FAIL_IWDG_NO_RESET, Ordering::Relaxed);
    park();
}

fn run_wwdg(wwdg: Peri<'static, peripherals::WWDG>) -> ! {
    // PCLK1 / 4096 with 59 ticks is 30 ms at 8 MHz; the first 15 ms are
    // closed, so a refresh before then would reset the chip.
    let mut wdg = WindowWatchdog::new(wwdg, 30_000, 15_000);

    STAGE.store(STAGE_WWDG_FEEDING, Ordering::Relaxed);
    // Wait for the counter to enter the open window, then refresh there.
    while wdg.counter() >= wdg.window() {}
    wdg.pet();

    // Stop feeding: the counter must run down to 0x3f and reset the chip.
    STAGE.store(STAGE_WWDG_ARMED, Ordering::Relaxed);
    riscv::asm::delay(4_000_000);
    STAGE.store(STAGE_FAIL_WWDG_NO_RESET, Ordering::Relaxed);
    park();
}
