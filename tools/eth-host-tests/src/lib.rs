#![allow(dead_code, unused_imports, unexpected_cfgs, static_mut_refs)]

pub use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};
#[path = "../../sdio-host-tests/src/common.rs"]
pub mod common;
#[path = "../../../src/internal/drop.rs"]
pub mod drop_impl;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/emac_v4x7.rs"]
pub mod emac_registers;
#[path = "../../../src/eth/v4x7.rs"]
pub mod eth;
#[path = "../../../src/peripheral.rs"]
mod peripheral;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/rcc_v4x7.rs"]
pub mod rcc_registers;
pub mod internal {
    pub use crate::drop_impl as drop;
}
pub mod time {
    pub struct Hertz(pub u32);
}
pub mod pac {
    pub use crate::{common, emac_registers as emac, rcc_registers as rcc};
    pub static mut ETH_WORDS: [u32; 1056] = [0; 1056];
    pub static mut RCC_WORDS: [u32; 64] = [0; 64];
    pub const ETH: emac::Eth = unsafe { emac::Eth::from_ptr(core::ptr::addr_of_mut!(ETH_WORDS).cast()) };
    pub const RCC: rcc::Rcc = unsafe { rcc::Rcc::from_ptr(core::ptr::addr_of_mut!(RCC_WORDS).cast()) };
}
pub mod interrupt {
    pub mod typelevel {
        pub trait Interrupt {
            unsafe fn enable();
            fn disable();
            fn unpend() {}
        }
        pub trait Handler<IRQ: Interrupt> {
            unsafe fn on_interrupt();
        }
        pub unsafe trait Binding<IRQ: Interrupt, HandlerType: Handler<IRQ>> {}
        pub struct ETH;
        impl Interrupt for ETH {
            unsafe fn enable() {
                crate::model::IRQ_ENABLED.store(true, core::sync::atomic::Ordering::Relaxed);
            }
            fn disable() {
                crate::model::IRQ_ENABLED.store(false, core::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}
pub mod peripherals {
    pub struct ETH;
    crate::impl_peripheral!(ETH);
    impl crate::peripheral::SealedRccPeripheral for ETH {
        fn frequency() -> crate::time::Hertz {
            crate::time::Hertz(crate::model::CLOCK_HZ.load(core::sync::atomic::Ordering::Relaxed))
        }
        fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
            crate::model::CLOCK.store(true, core::sync::atomic::Ordering::Relaxed);
            unsafe {
                crate::pac::ETH_WORDS = [0; 1056];
                crate::pac::RCC_WORDS[5] |= 1 << 14;
            }
        }
        fn disable_with_cs(_: critical_section::CriticalSection) {
            crate::model::CLOCK.store(false, core::sync::atomic::Ordering::Relaxed);
            unsafe {
                crate::pac::RCC_WORDS[5] &= !(1 << 14);
            }
        }
    }
}
pub struct Irqs;
unsafe impl interrupt::typelevel::Binding<interrupt::typelevel::ETH, eth::InterruptHandler> for Irqs {}
#[cfg(test)]
mod lifecycle;
mod model;
