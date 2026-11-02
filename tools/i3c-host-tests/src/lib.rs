#![allow(
    dead_code,
    unused_imports,
    unused_attributes,
    unused_variables,
    unexpected_cfgs,
    static_mut_refs,
    non_camel_case_types
)]

extern crate self as qingke;

pub use embassy_hal_internal::{impl_peripheral, Peri, PeripheralType};

macro_rules! foreach_interrupt {
    ($($tokens:tt)*) => {};
}

macro_rules! foreach_peripheral {
    ($($tokens:tt)*) => {
        impl SealedInstance for peripherals::I3C {
            fn regs() -> crate::pac::i3c::I3c {
                crate::pac::I3C
            }
            fn state() -> &'static State {
                static STATE: State = State {
                    waker: AtomicWaker::new(),
                    active: AtomicBool::new(false),
                };
                &STATE
            }
        }
        impl Instance for peripherals::I3C {
            type EventInterrupt = crate::interrupt::typelevel::Event;
            type ErrorInterrupt = crate::interrupt::typelevel::Error;
        }
    };
}

#[macro_use]
#[path = "../../../src/macros.rs"]
mod macros;
#[path = "../../sdio-host-tests/src/common.rs"]
pub mod common;
#[path = "../../../src/dma/mod.rs"]
pub mod dma;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/dma_v4x7.rs"]
pub mod dma_registers;
#[path = "../../../src/internal/drop.rs"]
pub mod drop_impl;
#[path = "../../../src/i3c.rs"]
pub mod i3c;
#[path = "../../../../ch32-data/build/ch32-metapac/src/peripherals/i3c_v4x7.rs"]
pub mod i3c_registers;
#[path = "../../../src/peripheral.rs"]
pub mod peripheral;
pub mod internal {
    pub use crate::drop_impl as drop;
}
pub mod time {
    pub struct Hertz(pub u32);
}
pub mod mode {
    pub trait Mode {}
    pub struct Blocking;
    pub struct Async;
    impl Mode for Blocking {}
    impl Mode for Async {}
}
pub mod pac {
    pub use crate::{common, dma_registers as dma, i3c_registers as i3c};
    pub static mut WORDS: [u32; 64] = [0; 64];
    pub static mut DMA1_WORDS: [u32; 64] = [0; 64];
    pub static mut DMA2_WORDS: [u32; 64] = [0; 64];
    pub const DMA1: dma::Dma = unsafe { dma::Dma::from_ptr(core::ptr::addr_of_mut!(DMA1_WORDS).cast()) };
    pub const I3C: i3c::I3c = unsafe { i3c::I3c::from_ptr(core::ptr::addr_of_mut!(WORDS).cast()) };
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
}
pub mod _generated {
    use crate::dma::{ChannelInfo, DmaInfo};
    use crate::pac;
    const fn channels() -> [ChannelInfo; 24] {
        let mut channels = [const {
            ChannelInfo {
                dma: DmaInfo::Dma(pac::DMA1),
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
pub mod gpio {
    pub enum OutputType {
        PushPull,
    }
    pub enum Speed {
        High,
    }
    #[derive(Clone, Copy)]
    pub struct AfType;
    impl AfType {
        pub fn output(_: OutputType, _: Speed) -> Self {
            Self
        }
    }
    pub trait SealedPin {
        fn set_as_af(&self, _: AfType) {}
        fn set_as_disconnected(&self) {}
    }
    pub trait Pin: SealedPin + crate::PeripheralType + Into<AnyPin> + 'static {}
    pub struct AnyPin;
    crate::impl_peripheral!(AnyPin);
    impl SealedPin for AnyPin {}
    impl Pin for AnyPin {}
}
pub mod peripherals {
    pub struct DMA1_CH4;
    pub struct DMA1_CH5;
    crate::impl_peripheral!(DMA1_CH4);
    crate::impl_peripheral!(DMA1_CH5);
    dma_channel_impl!(DMA1_CH4, 3);
    dma_channel_impl!(DMA1_CH5, 4);
    pub struct I3C;
    crate::impl_peripheral!(I3C);
    impl crate::peripheral::SealedRccPeripheral for I3C {
        fn frequency() -> crate::time::Hertz {
            crate::time::Hertz(crate::model::CLOCK_HZ.load(core::sync::atomic::Ordering::Relaxed))
        }
        fn enable_and_reset_with_cs(_: critical_section::CriticalSection) {
            crate::model::reset_registers();
        }
        fn disable_with_cs(_: critical_section::CriticalSection) {
            crate::model::CLOCK.store(false, core::sync::atomic::Ordering::Relaxed);
        }
    }
    impl crate::peripheral::RccPeripheral for I3C {}
    impl crate::peripheral::SealedRemapPeripheral for I3C {
        fn set_remap(remap: u8) {
            assert!(remap <= 1);
        }
    }
    impl crate::peripheral::RemapPeripheral for I3C {}
}
pub mod interrupt {
    pub struct Priority;
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
        pub struct Event;
        pub struct Error;
        impl Interrupt for Event {}
        impl Interrupt for Error {}
    }
}
impl i3c::SclPin<peripherals::I3C> for gpio::AnyPin {}
impl i3c::SdaPin<peripherals::I3C> for gpio::AnyPin {}
dma_trait_impl!(crate::i3c::TxDma, I3C, {channel: DMA1_CH4}, ());
dma_trait_impl!(crate::i3c::RxDma, I3C, {channel: DMA1_CH5}, ());
pub struct Irqs;
unsafe impl interrupt::typelevel::Binding<interrupt::typelevel::Event, i3c::EventInterruptHandler<peripherals::I3C>>
    for Irqs
{
}
unsafe impl interrupt::typelevel::Binding<interrupt::typelevel::Error, i3c::ErrorInterruptHandler<peripherals::I3C>>
    for Irqs
{
}

#[cfg(test)]
mod dma_tests;
mod model;
#[cfg(test)]
mod tests;
