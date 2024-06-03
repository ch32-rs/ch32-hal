//! Patches for some peripherals
//!
//! This should be removed in the future.
//!
//! - Patches for remap implementation
//! - Patches for peripherals without remap(fake remap)

#[cfg(peri_i2c2)]
mod i2c2 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::I2C2 {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::I2C2 {}
}

/// CH32V2, CH32V3
#[cfg(all(any(ch32v2, ch32v3), peri_usart1))]
mod usart1 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::USART1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr1().modify(|w| w.set_usart1_rm(remap & 0b1 != 0));
            afio.pcfr2().modify(|w| w.set_usart1_rm2(remap & 0b10 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::USART1 {}
}

#[cfg(all(any(ch32v0), peri_usart1))]
mod usart1 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::USART1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr1().modify(|w| w.set_usart1_rm(remap & 0b1 != 0));
            afio.pcfr1().modify(|w| w.set_usart1_rm1(remap & 0b10 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::USART1 {}
}

#[cfg(all(peri_i2c1, ch32v0))]
mod i2c1 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::I2C1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr1().modify(|w| w.set_i2c1_rm(remap & 0b1 != 0));
            afio.pcfr1().modify(|w| w.set_i2c1_rm1(remap & 0b10 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::I2C1 {}
}

#[cfg(peri_spi2)]
mod spi2 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::SPI2 {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::SPI2 {}
}

#[cfg(all(peri_tim5))]
mod tim5 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::TIM5 {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::TIM5 {}
}

#[cfg(all(peri_tim6))]
mod tim6 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::TIM6 {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::TIM6 {}
}

#[cfg(all(peri_tim7))]
mod tim7 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::TIM7 {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::TIM7 {}
}

#[cfg(all(peri_sdio))]
mod sdio {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::SDIO {
        fn set_remap(_remap: u8) {}
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::SDIO {}
}

#[cfg(ch32l1)]
mod ch32l1 {
    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::USART1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_usart1_rm_h((remap & 0b110) >> 1));
            afio.pcfr1().modify(|w| w.set_usart1_rm(remap & 0b1 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::USART1 {}

    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::USART2 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_usart2_rm_h(remap & 0b10 != 0));
            afio.pcfr1().modify(|w| w.set_usart2_rm(remap & 0b1 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::USART2 {}

    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::SPI1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_spi1_rm_h(remap & 0b10 != 0));
            afio.pcfr1().modify(|w| w.set_spi1_rm(remap & 0b1 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::SPI1 {}

    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::TIM1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_tim1_rm_h(remap & 0b100 != 0));
            afio.pcfr1().modify(|w| w.set_tim1_rm(remap & 0b11));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::TIM1 {}

    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::TIM2 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_tim2_rm_h(remap & 0b100 != 0));
            afio.pcfr1().modify(|w| w.set_tim2_rm(remap & 0b11));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::TIM2 {}

    impl crate::peripheral::SealedRemapPeripheral for crate::peripherals::I2C1 {
        fn set_remap(remap: u8) {
            let afio = &crate::pac::AFIO;
            afio.pcfr2().modify(|w| w.set_i2c1_rm_h(remap & 0b10 != 0));
            afio.pcfr1().modify(|w| w.set_i2c1_rm(remap & 0b1 != 0));
        }
    }
    impl crate::peripheral::RemapPeripheral for crate::peripherals::I2C1 {}
}
