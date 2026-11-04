#![allow(dead_code, unused_imports, unexpected_cfgs, static_mut_refs)]

pub use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};

#[macro_use]
#[path = "../../../src/macros.rs"]
mod macros;
#[path = "../../../src/peripheral.rs"]
mod peripheral;
pub use peripheral::RccPeripheral;
#[path = "../../host-tests-common/common_reg.rs"]
pub mod common;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/rcc_v4x7.rs"]
pub mod rcc_registers;
#[path = "../../../src/usbhs/v4x7.rs"]
pub mod usbhs;
#[cfg(test)]
mod usbhs_lifecycle_tests;
mod usbhs_model;

pub fn record_read(address: usize) {
    usbhs_model::after_read(address);
}

pub fn record_write(address: usize, value: u32, size: usize) -> bool {
    usbhs_model::write(address, value, size)
}
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/usbhs_v4x7.rs"]
pub mod usbhs_registers;

pub mod time {
    pub struct Hertz(pub u32);
}

pub mod pac {
    pub use crate::{common, rcc_registers as rcc, usbhs_registers as usbhs};
    pub static mut RCC_WORDS: [u32; 32] = [0; 32];
    pub static mut USB1_WORDS: [u32; 48] = [0; 48];
    pub static mut USB2_WORDS: [u32; 48] = [0; 48];
    pub const RCC: rcc::Rcc = unsafe { rcc::Rcc::from_ptr(core::ptr::addr_of_mut!(RCC_WORDS).cast()) };
    pub const USBHS1: usbhs::Usb = unsafe { usbhs::Usb::from_ptr(core::ptr::addr_of_mut!(USB1_WORDS).cast()) };
    pub const USBHS2: usbhs::Usb = unsafe { usbhs::Usb::from_ptr(core::ptr::addr_of_mut!(USB2_WORDS).cast()) };
}

pub mod interrupt {
    pub mod typelevel {
        use core::sync::atomic::{AtomicBool, Ordering};

        static ENABLED: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

        pub trait Interrupt {
            const INDEX: usize;
            unsafe fn enable() {
                ENABLED[Self::INDEX].store(true, Ordering::Relaxed);
            }
            fn disable() {
                ENABLED[Self::INDEX].store(false, Ordering::Relaxed);
            }
            fn unpend() {}
        }
        pub trait Handler<IRQ: Interrupt> {
            unsafe fn on_interrupt();
        }
        pub unsafe trait Binding<IRQ: Interrupt, HandlerType: Handler<IRQ>> {}
        pub struct USBHS1;
        pub struct USBHS2;
        impl Interrupt for USBHS1 {
            const INDEX: usize = 0;
        }
        impl Interrupt for USBHS2 {
            const INDEX: usize = 1;
        }
    }
}

pub mod gpio {
    pub struct AnyPin;
    crate::impl_peripheral!(AnyPin);
    pub enum Pull {
        None,
    }
    pub trait SealedPin {
        fn set_as_input(&self, _: Pull) {}
        fn set_as_disconnected(&self) {}
    }
    pub trait Pin: SealedPin + crate::PeripheralType + Into<AnyPin> {}
    impl SealedPin for AnyPin {}
    impl Pin for AnyPin {}
}

macro_rules! usb_peripheral {
    ($name:ident, $words:ident, $index:expr) => {
        pub struct $name;
        crate::impl_peripheral!($name);
        impl crate::peripheral::SealedRccPeripheral for $name {
            fn frequency() -> crate::time::Hertz {
                crate::time::Hertz(20_000_000)
            }
            fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
                crate::usbhs_model::CLOCKS[$index].store(true, core::sync::atomic::Ordering::Relaxed);
                unsafe { crate::pac::$words = [0; 48] };
            }
            fn disable_with_cs(_: critical_section::CriticalSection) {
                crate::usbhs_model::CLOCKS[$index].store(false, core::sync::atomic::Ordering::Relaxed);
            }
        }
        impl crate::peripheral::RccPeripheral for $name {}
        impl crate::usbhs::DpPin<$name> for crate::gpio::AnyPin {}
        impl crate::usbhs::DmPin<$name> for crate::gpio::AnyPin {}
    };
}

usb_peripheral!(USBHS1, USB1_WORDS, 0);
usb_peripheral!(USBHS2, USB2_WORDS, 1);

pub mod peripherals {
    pub use crate::{USBHS1, USBHS2};
}

pub mod _generated {
    pub mod peripheral_interrupts {
        #[allow(non_snake_case)]
        pub mod USBHS1 {
            pub type GLOBAL = crate::interrupt::typelevel::USBHS1;
        }
        #[allow(non_snake_case)]
        pub mod USBHS2 {
            pub type GLOBAL = crate::interrupt::typelevel::USBHS2;
        }
    }
}
