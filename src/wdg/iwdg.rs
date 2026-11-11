//! Independent watchdog (IWDG).
//!
//! A 12-bit down-counter clocked from LSI (RM §7). LSI keeps running in stop
//! and standby, so the counter reaches zero and resets the chip even when the
//! core is stalled. Once [`IndependentWatchdog::unleash`] has been called the
//! watchdog cannot be stopped short of a reset.
//!
//! The API follows embassy-stm32's `wdg` module: the constructor takes a µs
//! timeout, [`unleash`](IndependentWatchdog::unleash) starts the watchdog and
//! [`pet`](IndependentWatchdog::pet) reloads it.

use crate::{pac, peripherals, Peri, PeripheralType};

/// The counter is 12 bits wide, so the reload value tops out at 0x0fff.
const MAX_RELOAD: u16 = 0x0fff;

/// IWDG counter clock before the prescaler.
///
/// The CH32X0 family has no LSI: its IWDG runs on HSI / 1024 = 46.875 kHz (CH32X035 datasheet
/// V2.2, §1.3 clock tree and the IWDG paragraph). Elsewhere it is the LSI.
#[cfg(ch32x0)]
const CLOCK_HZ: u32 = 48_000_000 / 1024;
#[cfg(not(ch32x0))]
const CLOCK_HZ: u32 = crate::rcc::LSI_FREQ.0;

/// Independent watchdog (IWDG) driver.
///
/// [`new`](Self::new) configures the timeout but leaves the watchdog stopped,
/// so construction alone cannot reset the chip. Start it with
/// [`unleash`](Self::unleash) once the system can meet the deadline.
pub struct IndependentWatchdog<'d, T: Instance> {
    _inner: Peri<'d, T>,
}

impl<'d, T: Instance> IndependentWatchdog<'d, T> {
    /// Configure the watchdog for a `timeout_us` microsecond period.
    ///
    /// The period is `(reload + 1) * prescaler / clock`; the clock is the LSI, or HSI / 1024 on
    /// CH32X0. Both are RC oscillators with a tolerance, so leave margin and pet well before
    /// the nominal deadline. Panics if the timeout does not fit the 12-bit
    /// counter with the largest prescaler (32 s at 32.768 kHz, 26 s at
    /// 40 kHz).
    pub fn new(inner: Peri<'d, T>, timeout_us: u32) -> Self {
        let (prescaler, reload) = prescaler_and_reload(timeout_us);
        let regs = T::regs();

        // Unlock PSCR/RLDR and configure them while the watchdog is stopped.
        // Starting the watchdog forces LSI on (RM §3.3.5.4); the vendor
        // sequence also configures first, so no clock has to be running yet.
        regs.ctlr().write(|w| w.set_key(0x5555));
        regs.pscr().write(|w| w.set_pr(prescaler));
        regs.rldr().write(|w| w.set_rl(reload));

        Self { _inner: inner }
    }

    /// Start the watchdog. This is irreversible until the next reset.
    pub fn unleash(&mut self) {
        // The counter holds its reset value until the reload key is written,
        // so load RLDR before enabling.
        T::regs().ctlr().write(|w| w.set_key(0xaaaa));
        T::regs().ctlr().write(|w| w.set_key(0xcccc));
    }

    /// Reload the counter from RLDR. Call this well inside every period.
    pub fn pet(&mut self) {
        T::regs().ctlr().write(|w| w.set_key(0xaaaa));
    }
}

/// Pick the smallest prescaler that can express `timeout_us` in 12 bits.
fn prescaler_and_reload(timeout_us: u32) -> (u8, u16) {
    let clock = u64::from(CLOCK_HZ);

    for prescaler_power in 2u8..=8u8 {
        let prescaler = 1u32 << prescaler_power;
        let ticks = (u64::from(timeout_us) * clock).div_ceil(u64::from(prescaler) * 1_000_000);
        if ticks <= u64::from(MAX_RELOAD) + 1 {
            let ticks = ticks.max(1) as u16;
            return (prescaler_power - 2, ticks - 1);
        }
    }

    panic!("independent watchdog timeout of {} us is out of range", timeout_us);
}

trait SealedInstance {
    fn regs() -> pac::iwdg::Iwdg;
}

/// IWDG instance trait.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + PeripheralType {}

foreach_peripheral!(
    (iwdg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> pac::iwdg::Iwdg {
                pac::$inst
            }
        }

        impl Instance for peripherals::$inst {}
    };
);
