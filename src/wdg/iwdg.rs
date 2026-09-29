//! Independent watchdog (IWDG)

use crate::{pac, peripherals, Peri, PeripheralType};

/// Division of the LSI clock before it clocks the watchdog counter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum IwdgPrescaler {
    /// LSI / 4.
    Div4 = 0,
    /// LSI / 8.
    Div8 = 1,
    /// LSI / 16.
    Div16 = 2,
    /// LSI / 32.
    Div32 = 3,
    /// LSI / 64.
    Div64 = 4,
    /// LSI / 128.
    Div128 = 5,
    /// LSI / 256.
    Div256 = 6,
}

impl IwdgPrescaler {
    /// Numeric prescaler (4 through 256).
    pub const fn divisor(self) -> u16 {
        4 << (self as u8)
    }
}

/// IWDG configuration error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IwdgError {
    /// The reload value is larger than the 12-bit maximum (0x0fff).
    ReloadOutOfRange,
}

/// Independent watchdog (IWDG) driver.
///
/// Dropping the handle does not stop the watchdog; it continues to run until reset.
pub struct IndependentWatchdog<'d, T: Instance> {
    _inner: Peri<'d, T>,
}

impl<'d, T: Instance> IndependentWatchdog<'d, T> {
    /// Configure and start the watchdog with a 12-bit reload value (`0..=0x0fff`).
    ///
    /// The timeout is `(reload + 1) * prescaler / LSI_frequency`. LSI frequency
    /// varies, and short timeouts may expire during register synchronization.
    /// Invalid reload values are rejected before the watchdog starts.
    pub fn new(inner: Peri<'d, T>, prescaler: IwdgPrescaler, reload: u16) -> Result<Self, IwdgError> {
        if reload > 0x0fff {
            return Err(IwdgError::ReloadOutOfRange);
        }

        let regs = T::regs();
        // Starting first forces LSI on so that the update flags can clear. The
        // counter initially uses the reset defaults (LSI / 4, reload 0x0fff).
        regs.ctlr().write(|w| w.set_key(0xcccc));
        regs.ctlr().write(|w| w.set_key(0xaaaa));
        regs.ctlr().write(|w| w.set_key(0x5555));

        // Option-byte startup or previous configuration may still be syncing.
        while regs.statr().read().pvu() || regs.statr().read().rvu() {}
        regs.pscr().write(|w| w.set_pr(prescaler as u8));
        regs.rldr().write(|w| w.set_rl(reload));

        // Extend the initial counting period before waiting for the new settings;
        // reload again after synchronization to begin the configured period.
        regs.ctlr().write(|w| w.set_key(0xaaaa));
        while regs.statr().read().pvu() || regs.statr().read().rvu() {}
        regs.ctlr().write(|w| w.set_key(0xaaaa));

        Ok(Self { _inner: inner })
    }

    /// Reload the watchdog counter.
    ///
    /// Feed it before the shortest timeout allowed by the LSI clock tolerance.
    pub fn feed(&mut self) {
        T::regs().ctlr().write(|w| w.set_key(0xaaaa));
    }
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
