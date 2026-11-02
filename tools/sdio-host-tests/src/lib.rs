#![allow(dead_code, unused_imports, unexpected_cfgs, static_mut_refs)]

extern crate self as qingke;
pub use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};

#[path = "../../../src/macros.rs"]
mod macros;
macro_rules! foreach_interrupt {
    ($($tokens:tt)*) => {};
}
pub mod common;
#[path = "../../../src/peripheral.rs"]
mod peripheral;
pub use peripheral::{RccPeripheral, RemapPeripheral};
#[path = "../../../src/dma/mod.rs"]
pub mod dma;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/dma_v4x7.rs"]
pub mod dma_registers;
#[path = "../../../src/internal/drop.rs"]
pub mod drop_impl;
#[path = "../../../src/sdio/v4x7.rs"]
pub mod sdio;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/sdio_v4x7.rs"]
pub mod sdio_registers;
#[path = "../../../src/time.rs"]
pub mod time;
pub mod internal {
    pub use crate::drop_impl as drop;
}
#[cfg(test)]
mod lifecycle;
mod model;

pub mod pac {
    pub use crate::{common, dma_registers as dma, sdio_registers as sdio};
    pub static mut SDIO_WORDS: [u32; 36] = [0; 36];
    pub static mut DMA1_WORDS: [u32; 64] = [0; 64];
    pub static mut DMA2_WORDS: [u32; 64] = [0; 64];
    pub const SDIO: sdio::Sdio = unsafe { sdio::Sdio::from_ptr(core::ptr::addr_of_mut!(SDIO_WORDS).cast()) };
    pub const DMA2: dma::Dma2 = unsafe { dma::Dma2::from_ptr(core::ptr::addr_of_mut!(DMA2_WORDS).cast()) };
    #[derive(Clone, Copy)]
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
    pub unsafe fn set_priority(_: u8, _: u8) {}
}

pub mod interrupt {
    pub enum Priority {
        P0,
    }
    pub mod typelevel {
        pub trait Interrupt {
            unsafe fn enable() {}
            fn disable() {}
            fn unpend() {}
        }
        pub trait Handler<IRQ: Interrupt> {
            unsafe fn on_interrupt();
        }
        pub unsafe trait Binding<IRQ: Interrupt, HandlerType: Handler<IRQ>> {}
        pub struct SDIO;
        impl Interrupt for SDIO {}
    }
}

pub mod gpio {
    pub struct AnyPin;
    crate::impl_peripheral!(AnyPin);
    pub enum AFType {
        OutputPushPull,
    }
    pub enum Speed {
        High,
    }
    pub trait SealedPin {
        fn set_as_af_output(&self, _: AFType, _: Speed) {}
        fn set_as_disconnected(&self) {}
    }
    pub trait Pin: SealedPin + crate::PeripheralType + Into<AnyPin> {}
    impl SealedPin for AnyPin {}
    impl Pin for AnyPin {}
}

pub mod peripherals {
    pub struct SDIO;
    crate::impl_peripheral!(SDIO);
    impl crate::peripheral::SealedRccPeripheral for SDIO {
        fn frequency() -> crate::time::Hertz {
            crate::time::Hertz(crate::model::CLOCK_HZ.load(core::sync::atomic::Ordering::Relaxed))
        }
        fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
            crate::model::CLOCK.store(true, core::sync::atomic::Ordering::Relaxed);
        }
        fn disable_with_cs(_: critical_section::CriticalSection) {
            crate::model::CLOCK.store(false, core::sync::atomic::Ordering::Relaxed);
        }
    }
    impl crate::peripheral::RccPeripheral for SDIO {}
    impl crate::peripheral::SealedRemapPeripheral for SDIO {
        fn set_remap(remap: u8) {
            crate::model::REMAP.store(remap, core::sync::atomic::Ordering::Relaxed);
        }
    }
    impl crate::peripheral::RemapPeripheral for SDIO {}
}

impl crate::sdio::CkPin<peripherals::SDIO> for gpio::AnyPin {}
impl crate::sdio::CmdPin<peripherals::SDIO> for gpio::AnyPin {}
impl<const REMAP: u8> crate::sdio::D0Pin<peripherals::SDIO, REMAP> for gpio::AnyPin {}
impl<const REMAP: u8> crate::sdio::D1Pin<peripherals::SDIO, REMAP> for gpio::AnyPin {}
impl<const REMAP: u8> crate::sdio::D2Pin<peripherals::SDIO, REMAP> for gpio::AnyPin {}
impl<const REMAP: u8> crate::sdio::D3Pin<peripherals::SDIO, REMAP> for gpio::AnyPin {}
impl crate::sdio::SdioDma<peripherals::SDIO> for dma::AnyChannel {
    fn request(&self) {}
}

pub mod _generated {
    pub mod peripheral_interrupts {
        #[allow(non_snake_case)]
        pub mod SDIO {
            pub type GLOBAL = crate::interrupt::typelevel::SDIO;
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
