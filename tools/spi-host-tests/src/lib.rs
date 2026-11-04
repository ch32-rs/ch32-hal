#![allow(
    dead_code,
    unused_imports,
    unused_variables,
    unused_macros,
    unused_attributes,
    nonstandard_style,
    unexpected_cfgs,
    static_mut_refs
)]

extern crate self as qingke;
pub use embassy_hal_internal::{impl_peripheral, Peri};

#[macro_use]
#[path = "../../../src/macros.rs"]
mod macros;
macro_rules! foreach_interrupt {
    ($($tokens:tt)*) => {};
}
macro_rules! foreach_peripheral {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! instance { $(($pattern) => $code;)* }
        instance!((spi, SPI1));
        instance!((spi, SPI2));
        instance!((spi, SPI3));
    };
}

#[path = "../../host-tests-common/common_word.rs"]
pub mod common;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/dma_v4x7.rs"]
pub mod dma_registers;
#[path = "../../../src/internal/drop.rs"]
pub mod drop_impl;
#[path = "../../../src/peripheral.rs"]
pub mod peripheral;
#[path = "../../../src/time.rs"]
pub mod time;
pub mod internal {
    pub use crate::drop_impl as drop;
}

pub static CLOCK_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub static IRQ_ENABLED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn record_read(address: usize) {
    spi_model::read(address);
}

pub fn record_write(address: usize, value: u32) -> bool {
    if spi_model::write(address, value) {
        return true;
    }
    for (base, extended) in [
        (core::ptr::addr_of_mut!(pac::DMA1_WORDS) as usize, 0xf4),
        (core::ptr::addr_of_mut!(pac::DMA2_WORDS) as usize, 0xd4),
    ] {
        if address == base + 4 || address == base + extended {
            unsafe {
                let status = (address - 4) as *mut u32;
                let mut mask = value;
                for channel in 0..7 {
                    if mask & (1 << (channel * 4)) != 0 {
                        mask |= 0xf << (channel * 4);
                    }
                }
                status.write_volatile(status.read_volatile() & !mask);
            }
        }
    }
    false
}

pub mod pac {
    pub use crate::{dma_registers as dma, spi_registers as spi};
    pub static mut SPI1_WORDS: [u32; 16] = [0; 16];
    pub const SPI1: spi::Spi = unsafe { spi::Spi::from_ptr(core::ptr::addr_of_mut!(SPI1_WORDS).cast()) };
    pub static mut SPI2_WORDS: [u32; 16] = [0; 16];
    pub const SPI2: spi::Spi = unsafe { spi::Spi::from_ptr(core::ptr::addr_of_mut!(SPI2_WORDS).cast()) };
    pub static mut SPI3_WORDS: [u32; 16] = [0; 16];
    pub const SPI3: spi::Spi = unsafe { spi::Spi::from_ptr(core::ptr::addr_of_mut!(SPI3_WORDS).cast()) };
    pub static mut DMA1_WORDS: [u32; 64] = [0; 64];
    pub const DMA1: dma::Dma = unsafe { dma::Dma::from_ptr(core::ptr::addr_of_mut!(DMA1_WORDS).cast()) };
    pub static mut DMA2_WORDS: [u32; 64] = [0; 64];
    #[derive(Copy, Clone)]
    pub struct Interrupt;
    pub trait InterruptNumber {
        fn number(self) -> u16;
    }
    impl InterruptNumber for Interrupt {
        fn number(self) -> u16 {
            27
        }
    }
}

pub mod pfic {
    pub unsafe fn enable_interrupt(_: u8) {}
    pub unsafe fn unpend_interrupt(_: u8) {}
}

pub mod interrupt {
    pub struct Priority;
    pub mod typelevel {
        pub trait Interrupt {
            unsafe fn enable() {
                crate::IRQ_ENABLED.store(true, core::sync::atomic::Ordering::Relaxed);
            }
            fn disable() {
                crate::IRQ_ENABLED.store(false, core::sync::atomic::Ordering::Relaxed);
            }
            fn unpend() {}
            fn is_enabled() -> bool {
                crate::IRQ_ENABLED.load(core::sync::atomic::Ordering::Relaxed)
            }
        }
        pub trait Handler<I: Interrupt> {
            unsafe fn on_interrupt();
        }
        pub unsafe trait Binding<I: Interrupt, H: Handler<I>> {}
        pub struct SPI1_GLOBAL;
        pub struct SPI2_GLOBAL;
        pub struct SPI3_GLOBAL;
        impl Interrupt for SPI1_GLOBAL {}
        impl Interrupt for SPI2_GLOBAL {}
        impl Interrupt for SPI3_GLOBAL {}
    }
}

pub mod peripherals {
    macro_rules! spi_peripheral {
        ($name:ident) => {
            pub struct $name;
            crate::impl_peripheral!($name);
            impl crate::peripheral::SealedRccPeripheral for $name {
                fn frequency() -> crate::time::Hertz {
                    crate::time::Hertz(200_000_000)
                }
                fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
                    crate::spi_model::reset();
                }
                fn disable_with_cs(_: critical_section::CriticalSection) {}
            }
            impl crate::peripheral::RccPeripheral for $name {}
            impl crate::peripheral::SealedRemapPeripheral for $name {
                fn set_remap(_: u8) {}
            }
            impl crate::peripheral::RemapPeripheral for $name {}
        };
    }
    spi_peripheral!(SPI1);
    spi_peripheral!(SPI2);
    spi_peripheral!(SPI3);
}

pub mod gpio {
    pub struct AnyPin;
    crate::impl_peripheral!(AnyPin);
    pub enum Pull {
        None,
    }
    pub enum Speed {
        High,
    }
    pub enum OutputType {
        PushPull,
        OpenDrain,
    }
    #[derive(Clone, Copy)]
    pub struct AfType;
    impl AfType {
        pub fn input(_: Pull) -> Self {
            Self
        }
        pub fn output(_: OutputType, _: Speed) -> Self {
            Self
        }
    }
    pub trait SealedPin {
        fn set_low(&self) {}
        fn set_as_af(&self, _: AfType) {}
        fn set_as_disconnected(&self) {}
    }
    pub trait Pin: SealedPin + embassy_hal_internal::PeripheralType + Into<AnyPin> {}
    impl SealedPin for AnyPin {}
    impl Pin for AnyPin {}
}

pub mod mode {
    pub trait Mode {}
    pub struct Blocking;
    pub struct Async;
    impl Mode for Blocking {}
    impl Mode for Async {}
}

pub mod _generated {
    pub mod peripheral_interrupts {
        pub mod SPI1 {
            pub type GLOBAL = crate::interrupt::typelevel::SPI1_GLOBAL;
        }
        pub mod SPI2 {
            pub type GLOBAL = crate::interrupt::typelevel::SPI2_GLOBAL;
        }
        pub mod SPI3 {
            pub type GLOBAL = crate::interrupt::typelevel::SPI3_GLOBAL;
        }
    }
    use crate::dma::{ChannelInfo, DmaInfo};
    use crate::pac;
    const fn channels() -> [ChannelInfo; 24] {
        let mut channels = [const {
            ChannelInfo {
                dma: DmaInfo::Dma(unsafe { pac::dma::Dma::from_ptr(core::ptr::addr_of_mut!(pac::DMA1_WORDS).cast()) }),
                num: 0,
                irq: Some(pac::Interrupt),
            }
        }; 24];
        let mut index = 0;
        while index < 24 {
            channels[index].num = if index < 13 { index } else { index - 13 };
            if index >= 13 {
                channels[index].dma =
                    DmaInfo::Dma2(unsafe { pac::dma::Dma2::from_ptr(core::ptr::addr_of_mut!(pac::DMA2_WORDS).cast()) });
            }
            index += 1;
        }
        channels
    }
    pub(crate) const DMA_CHANNELS: &[ChannelInfo] = &channels();
    pub unsafe fn init_dma() {}
}

#[path = "../../../src/dma/mod.rs"]
pub mod dma;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/spi_v4x7.rs"]
pub mod spi_registers;
pub mod spi {
    include!(concat!(env!("OUT_DIR"), "/spi.rs"));
}

mod spi_model;
