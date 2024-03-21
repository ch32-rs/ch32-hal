//! Patches for some peripherals

impl crate::peripheral::sealed::RemapPeripheral for crate::peripherals::I2C2 {
    fn set_remap(_remap: u8) {}
}
impl crate::peripheral::RemapPeripheral for crate::peripherals::I2C2 {}

impl crate::peripheral::sealed::RemapPeripheral for crate::peripherals::USART1 {
    fn set_remap(remap: u8) {
        let afio = &crate::pac::AFIO;
        afio.pcfr1().modify(|w| w.set_usart1_rm(remap & 0b1 != 0));
        afio.pcfr2().modify(|w| w.set_usart1_rm2(remap & 0b10 != 0));
    }
}
impl crate::peripheral::RemapPeripheral for crate::peripherals::USART1 {}

