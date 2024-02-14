#![no_std]

pub use ch32v3::ch32v30x as pac;

// pub mod rt;

pub mod rcc;

pub mod debug;
pub mod delay;

mod peripheral;
pub use peripheral::*;
pub use peripherals::Peripherals;
pub mod interrupt;
pub mod peripherals;
pub mod prelude;

pub mod dma;

pub mod adc;
pub mod exti;
pub mod gpio;
pub mod i2c;
pub mod pioc;
pub mod signature;
pub mod spi;
pub mod timer;
pub mod usart;

mod traits;

#[cfg(feature = "embassy")]
pub mod embassy;

#[derive(Copy, Clone, Eq, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {}

pub fn init(config: Config) -> Peripherals {
    rcc::init();

    unsafe {
        gpio::init();
    }

    ::critical_section::with(|cs| unsafe {
        exti::init(cs);
    });

    peripherals::Peripherals::take()
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
