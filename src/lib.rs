#![no_std]
#![allow(static_mut_refs)]

pub use ch32_metapac as pac;

// This must go FIRST so that all the other modules see its macros.
include!(concat!(env!("OUT_DIR"), "/_macros.rs"));

pub mod time;
mod traits;

pub mod rcc;

pub mod debug;
//pub mod delay;
pub mod prelude;

mod peripheral;
pub use peripheral::*;
// #[cfg(not(ch32v0))]
mod interrupt_ext;

pub use crate::_generated::{peripherals, Peripherals};

pub mod dma;
#[cfg(systick_rv2)]
pub mod delay;

pub mod adc;
pub mod exti;
pub mod gpio;
#[cfg(i2c)]
pub mod i2c;
pub mod signature;
pub mod spi;
//pub mod timer;
#[cfg(rng)]
pub mod rng;
pub mod usart;

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

#[derive(Default)]
pub struct Config {
    pub rcc: rcc::Config,
}

pub fn init(config: Config) -> Peripherals {
    unsafe {
        rcc::init(config.rcc);

        #[cfg(systick_rv2)]
        delay::Delay::init();
    }

    ::critical_section::with(|cs| unsafe {
        gpio::init(cs);
    });

    ::critical_section::with(|cs| unsafe {
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
