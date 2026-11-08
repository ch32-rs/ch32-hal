//! Window watchdog (WWDG).
//!
//! A 7-bit down-counter clocked from PCLK1 / 4096 divided by 1, 2, 4 or 8
//! (RM §8). The counter resets the chip when it falls from 0x40 to 0x3f, and
//! also when it is refreshed before it has dropped below the window value.
//!
//! The API follows embassy-stm32's `wdg` module: the constructor takes the
//! timeout and the closed-window duration in µs and starts the watchdog, and
//! [`pet`](WindowWatchdog::pet) refreshes it.

use crate::{pac, peripherals, Peri};

/// The counter starts at 0x40 + the number of ticks in the period.
const RESET_OFFSET: u8 = 0x3f;

/// Window watchdog (WWDG) driver.
///
/// The watchdog is running as soon as [`new`](Self::new) returns; there is no
/// separate start step and no way to stop it short of a reset.
pub struct WindowWatchdog<'d, T: WwdgInstance> {
    _peri: Peri<'d, T>,
    /// Counter value written on every [`pet`](Self::pet).
    counter: u8,
    /// Refresh is only allowed once the live counter is below this value.
    window: u8,
}

impl<'d, T: WwdgInstance> WindowWatchdog<'d, T> {
    /// Start the watchdog with a `timeout_us` period and a `window_us` closed
    /// window.
    ///
    /// A refresh during the closed window resets the chip; pass `0` to allow
    /// a refresh at any time. Panics if `window_us >= timeout_us` or if the
    /// timeout does not fit the 7-bit counter at any prescaler (the maximum
    /// is PCLK1 / 4096 / 8 * 64 ticks).
    pub fn new(peri: Peri<'d, T>, timeout_us: u32, window_us: u32) -> Self {
        assert!(window_us < timeout_us, "WWDG window must be shorter than the timeout");

        T::enable_and_reset();
        let pclk1 = crate::rcc::clocks().pclk1.0;
        let (prescaler, ticks) = prescaler_and_ticks(pclk1, timeout_us);
        let counter = RESET_OFFSET + ticks;
        let closed_ticks = (u64::from(window_us) * u64::from(pclk1) / (u64::from(prescaler) * 4096 * 1_000_000)) as u8;
        let window = counter - closed_ticks;

        let regs = T::regs();
        regs.cfgr().write(|w| {
            w.set_w(window);
            w.set_wdgtb(prescaler.trailing_zeros() as u8);
        });
        // EWIF is raised even with the early-wakeup interrupt disabled.
        regs.statr().write(|w| w.set_ewif(false));
        regs.ctlr().write(|w| {
            w.set_t(counter);
            w.set_wdga(true);
        });

        Self {
            _peri: peri,
            counter,
            window,
        }
    }

    /// Reload the counter.
    ///
    /// Refreshing while the live counter is at or above [`window`](Self::window)
    /// resets the chip, so time the call for the open part of the period.
    pub fn pet(&mut self) {
        // Write the complete control value: a read-modify-write could write
        // back a stale, already-decremented counter value.
        T::regs().ctlr().write(|w| {
            w.set_t(self.counter);
            w.set_wdga(true);
        });
    }

    /// Read the live seven-bit down-counter.
    pub fn counter(&self) -> u8 {
        T::regs().ctlr().read().t()
    }

    /// Window value the live counter must fall below before a refresh.
    pub fn window(&self) -> u8 {
        self.window
    }

    /// Enable the early-wakeup interrupt at counter value 0x40.
    ///
    /// The caller must install and enable the WWDG interrupt handler; this
    /// bit is only cleared by a peripheral reset.
    pub fn enable_early_wakeup_interrupt(&mut self) {
        T::regs().cfgr().modify(|w| w.set_ewi(true));
    }

    /// Whether the counter has reached 0x40 since EWIF was last cleared.
    /// The flag is raised even with the early-wakeup interrupt disabled.
    pub fn early_wakeup_pending(&self) -> bool {
        T::regs().statr().read().ewif()
    }

    /// Clear the early-wakeup flag (write zero to clear).
    pub fn clear_early_wakeup(&mut self) {
        T::regs().statr().write(|w| w.set_ewif(false));
    }
}

/// Pick the smallest prescaler that keeps the period within 64 ticks.
fn prescaler_and_ticks(pclk1: u32, timeout_us: u32) -> (u32, u8) {
    for prescaler_power in 0u8..4u8 {
        let prescaler = 1u32 << prescaler_power;
        let ticks = (u64::from(timeout_us) * u64::from(pclk1)).div_ceil(u64::from(prescaler) * 4096 * 1_000_000);
        if ticks <= 64 {
            return (prescaler, ticks.max(1) as u8);
        }
    }

    panic!("window watchdog timeout of {} us is out of range", timeout_us);
}

trait SealedInstance {
    fn regs() -> pac::wwdg::Wwdg;
}

/// WWDG instance trait.
#[allow(private_bounds)]
pub trait WwdgInstance:
    SealedInstance + crate::peripheral::RccPeripheral + embassy_hal_internal::PeripheralType + 'static
{
}

foreach_peripheral!(
    (wwdg, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> pac::wwdg::Wwdg {
                pac::$inst
            }
        }

        impl WwdgInstance for peripherals::$inst {}
    };
);
