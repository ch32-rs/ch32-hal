#![no_std]
#![allow(static_mut_refs, unexpected_cfgs)]
#![feature(naked_functions)]

pub use ch32_metapac as pac;
pub(crate) use embassy_hal_internal::{impl_peripheral, peripherals_definition, peripherals_struct};
pub use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
#[cfg(feature = "rt")]
pub use qingke_rt::{entry, interrupt};

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
pub use peripheral::{RccPeripheral, RemapPeripheral};

// #[cfg(not(ch32v0))]
mod interrupt_ext;

pub use crate::_generated::{peripherals, Peripherals};

#[cfg(not(time_driver_systick))]
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

#[cfg(otg)]
pub mod otg_fs;
#[cfg(usbd)]
pub mod usbd;

#[cfg(usbpd)]
pub mod usbpd;

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

/// Initialize the HAL with the provided configuration.
///
/// This returns the peripheral singletons that can be used for creating drivers.
///
/// This should only be called once at startup, otherwise it panics.
pub fn init(config: Config) -> Peripherals {
    // Do this first, so that it panics if user is calling `init` a second time
    // before doing anything important.
    let p = Peripherals::take();

    unsafe {
        rcc::init(config.rcc);
        delay::init();

        #[cfg(feature = "embassy")]
        embassy::init();
    }

    ::critical_section::with(|cs| unsafe {
        gpio::init(cs);
        dma::init(cs, config.dma_interrupt_priority);
        exti::init(cs);
    });

    p
}

#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[$crate::interrupt]
            unsafe fn $irq() {
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
