//! Window watchdog (WWDG)

use crate::{pac, peripherals, Peri};

/// WWDG clock divider applied after the fixed PCLK1 / 4096 divider.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum WindowPrescaler {
    /// PCLK1 / 4096.
    Div1 = 0,
    /// PCLK1 / 8192.
    Div2 = 1,
    /// PCLK1 / 16384.
    Div4 = 2,
    /// PCLK1 / 32768.
    Div8 = 3,
}

/// WWDG configuration, expressed in raw seven-bit counter values.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WindowConfig {
    /// Counter loaded when the watchdog is started and on each successful feed.
    /// Must be in `0x40..=0x7f`.
    pub start_counter: u8,
    /// Feed is allowed only when the live counter is *strictly less* than this
    /// value and at least `0x40`. Must be in `0x41..=0x7f`.
    pub window: u8,
    /// Counter clock divider.
    pub prescaler: WindowPrescaler,
    /// Enable the one-shot early-wakeup interrupt at counter value `0x40`.
    /// The caller must install and enable a WWDG interrupt handler separately.
    /// This hardware bit cannot be cleared except by a peripheral reset.
    pub early_wakeup: bool,
}

/// Why a requested watchdog refresh was not performed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FeedError {
    /// Counter has not fallen below the configured window. Retrying later is safe.
    TooEarly,
    /// Counter is below `0x40`; watchdog reset is already due.
    TooLate,
}

/// Window watchdog (WWDG) driver.
///
/// Dropping the handle does not stop the watchdog; it continues to run until reset.
pub struct WindowWatchdog<'d, T: WwdgInstance> {
    _peri: Peri<'d, T>,
    window: u8,
    start_counter: u8,
}

impl<'d, T: WwdgInstance> WindowWatchdog<'d, T> {
    /// Configure and start the window watchdog.
    ///
    /// Panics unless `start_counter` is `0x40..=0x7f` and `window` is
    /// `0x41..=0x7f`. A window of `0x40` has no legal feed instant.
    pub fn new(peri: Peri<'d, T>, config: WindowConfig) -> Self {
        assert!(
            (0x40..=0x7f).contains(&config.start_counter),
            "WWDG counter must be 0x40..=0x7f"
        );
        assert!(
            (0x41..=0x7f).contains(&config.window),
            "WWDG window must be 0x41..=0x7f"
        );

        T::enable_and_reset();
        let regs = T::regs();
        regs.cfgr().write(|w| {
            w.set_w(config.window);
            w.set_wdgtb(config.prescaler as u8);
            w.set_ewi(config.early_wakeup);
        });
        // EWIF is set even when EWI is disabled. It is write-zero-to-clear.
        regs.statr().write(|w| w.set_ewif(false));
        regs.ctlr().write(|w| {
            w.set_t(config.start_counter);
            w.set_wdga(true);
        });

        Self {
            _peri: peri,
            window: config.window,
            start_counter: config.start_counter,
        }
    }

    /// Read the live seven-bit downcounter.
    pub fn counter(&self) -> u8 {
        T::regs().ctlr().read().t()
    }

    /// Refresh only when `0x40 <= counter < window`; otherwise returns an error.
    ///
    /// The counter keeps ticking between the check and the write. Feed with enough
    /// margin before `0x40` to avoid a timeout reset.
    pub fn feed(&mut self) -> Result<(), FeedError> {
        let counter = self.counter();
        if counter < 0x40 {
            return Err(FeedError::TooLate);
        }
        if counter >= self.window {
            return Err(FeedError::TooEarly);
        }

        // Write the complete control value, not modify(): a read/modify/write
        // could write back a stale, already-decremented counter value.
        T::regs().ctlr().write(|w| {
            w.set_t(self.start_counter);
            w.set_wdga(true);
        });
        Ok(())
    }

    /// Whether the counter has reached `0x40` since EWIF was last cleared.
    /// The flag is raised even with the early-wakeup interrupt disabled.
    pub fn early_wakeup_pending(&self) -> bool {
        T::regs().statr().read().ewif()
    }

    /// Clear the early-wakeup flag (write zero to clear).
    pub fn clear_early_wakeup(&mut self) {
        T::regs().statr().write(|w| w.set_ewif(false));
    }
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
