#![no_std]
#![allow(static_mut_refs)]

pub use ch32_metapac as pac;

// This must go FIRST so that all the other modules see its macros.
include!(concat!(env!("OUT_DIR"), "/_macros.rs"));

mod traits;

pub mod rcc;

pub mod debug;
//pub mod delay;

mod peripheral;
pub use peripheral::*;
//pub use _peripherals::Peripherals;
pub mod interrupt;
// pub mod _peripherals;
//pub mod prelude;

pub use crate::_generated::{peripherals, Peripherals};

//pub mod dma;

//pub mod adc;
pub mod exti;
pub mod gpio;
//pub mod i2c;
//pub mod pioc;
pub mod signature;
//pub mod spi;
//pub mod timer;
//pub mod usart;

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

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {}

pub fn init(config: Config) -> Peripherals {
    // rcc::init();

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
            #[link_section = ".highcode"]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::Handler<$crate::interrupt::$irq>>::on_interrupt();
                )*
            }

            $(
                unsafe impl $crate::interrupt::Binding<$crate::interrupt::$irq, $handler> for $name {}
            )*
        )*
    };
}
