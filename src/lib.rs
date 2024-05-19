#![no_std]
#![allow(static_mut_refs, unexpected_cfgs)]
#![feature(naked_functions)]

pub use ch32_metapac as pac;

// This must go FIRST so that all the other modules see its macros.
include!(concat!(env!("OUT_DIR"), "/_macros.rs"));

pub(crate) mod internal;

mod macros;
pub mod time;
/// Operating modes for peripherals.
pub mod mode {
    trait SealedMode {}

    /// Operating mode for a peripheral.
    #[allow(private_bounds)]
    pub trait Mode: SealedMode {}

    macro_rules! impl_mode {
        ($name:ident) => {
            impl SealedMode for $name {}
            impl Mode for $name {}
        };
    }

    /// Blocking mode.
    pub struct Blocking;
    /// Async mode.
    pub struct Async;

    impl_mode!(Blocking);
    impl_mode!(Async);
}

pub mod rcc;

pub mod debug;
pub mod prelude;

mod peripheral;
pub use peripheral::*;
// #[cfg(not(ch32v0))]
mod interrupt_ext;

pub use crate::_generated::{peripherals, Peripherals};

#[cfg(any(systick_rv2, systick_rv3))]
pub mod delay;
pub mod dma;

#[cfg(adc)]
pub mod adc;
#[cfg(peri_dac1)]
pub mod dac;
pub mod exti;
pub mod gpio;
#[cfg(i2c)]
pub mod i2c;
#[cfg(rng)]
pub mod rng;
#[cfg(sdio_v3)]
pub mod sdio;
pub mod signature;
#[cfg(spi)]
pub mod spi;
#[cfg(any(timer_x0, timer_v3))]
pub mod timer;
pub mod usart;

#[cfg(usb)]
pub mod usb;
#[cfg(usbd)]
pub mod usbd;

#[cfg(feature = "embassy")]
pub mod embassy;

// This must go last, so that it sees all the impl_foo! macros defined earlier.
pub(crate) mod _generated {
    #![allow(dead_code)]
    #![allow(unused_imports)]
    #![allow(non_snake_case)]
    #![allow(missing_docs)]

    include!(concat!(env!("OUT_DIR"), "/_generated.rs"));
}

mod patches;
pub use crate::_generated::interrupt;

pub struct Config {
    pub rcc: rcc::Config,
    pub dma_interrupt_priority: interrupt::Priority,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rcc: Default::default(),
            dma_interrupt_priority: interrupt::Priority::P0,
        }
    }
}

pub fn init(config: Config) -> Peripherals {
    unsafe {
        rcc::init(config.rcc);

        #[cfg(any(systick_rv2, systick_rv3))]
        delay::Delay::init();
    }

    ::critical_section::with(|cs| unsafe {
        gpio::init(cs);
        dma::init(cs, config.dma_interrupt_priority);
        exti::init(cs);
    });

    Peripherals::take()
}

#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}
