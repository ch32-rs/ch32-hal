//! CRC-32 calculation unit.
//!
//! Polynomial `0x04C11DB7`, initial value `0xFFFF_FFFF`, MSB-first words.
//! No reflection or final XOR.

use crate::peripheral::RccPeripheral;
use crate::{pac, peripherals, Peri, PeripheralType};

/// CRC-32 driver.
pub struct Crc<'d, T: Instance> {
    _inner: Peri<'d, T>,
}

impl<'d, T: Instance> Crc<'d, T> {
    /// Enable the peripheral clock and start a new calculation at `0xFFFF_FFFF`.
    pub fn new(inner: Peri<'d, T>) -> Self {
        T::enable_and_reset();
        let mut crc = Self { _inner: inner };
        crc.reset();
        crc
    }

    /// Reset the CRC accumulator to `0xFFFF_FFFF` without changing the independent data register.
    pub fn reset(&mut self) {
        T::regs().ctlr().write(|w| w.set_reset(true));
        // RESET is self-clearing; wait for the accumulator before feeding a new word.
        while T::regs().datar().read().dr() != 0xffff_ffff {}
    }

    /// Feed a 32-bit word, most-significant bit first.
    pub fn update_word(&mut self, word: u32) {
        T::regs().datar().write(|w| w.set_dr(word));
    }

    /// Feed the words in slice order without resetting the accumulator.
    pub fn update_words(&mut self, words: &[u32]) {
        for &word in words {
            self.update_word(word);
        }
    }

    /// Read the raw CRC remainder without resetting the accumulator.
    pub fn result(&self) -> u32 {
        T::regs().datar().read().dr()
    }

    /// Store a byte in the independent data register.
    ///
    /// This byte is not included in the CRC calculation and survives [`Self::reset`].
    pub fn set_independent_data(&mut self, value: u8) {
        T::regs().idatar().write(|w| w.set_idr(value));
    }

    /// Read the independent data register; it does not affect the CRC calculation.
    pub fn independent_data(&self) -> u8 {
        T::regs().idatar().read().idr()
    }
}

trait SealedInstance {
    fn regs() -> pac::crc::Crc;
}

/// CRC peripheral instance.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + PeripheralType + 'static {}

foreach_peripheral!(
    (crc, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> pac::crc::Crc {
                pac::$inst
            }
        }

        impl Instance for peripherals::$inst {}
    };
);
