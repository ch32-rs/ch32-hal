#![no_std]
#![allow(static_mut_refs, unexpected_cfgs)]

use core::future::Future;

pub use ch32_metapac as pac;
pub(crate) use embassy_hal_internal::{impl_peripheral, peripherals_definition, peripherals_struct};
pub use embassy_hal_internal::{Peri, PeripheralType};
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
    /// NB mode.
    pub struct NonBlocking;

    impl_mode!(Blocking);
    impl_mode!(Async);
    impl_mode!(NonBlocking);
}

// Without an Embassy time driver, hardware waits have no deadline and may
// block indefinitely if the peripheral or bus does not make progress.
#[derive(Copy, Clone)]
struct Timeout {
    #[cfg(feature = "embassy")]
    deadline: embassy_time::Instant,
}

#[allow(dead_code)]
impl Timeout {
    #[inline]
    fn check(self) -> Option<()> {
        #[cfg(feature = "embassy")]
        if embassy_time::Instant::now() > self.deadline {
            return None;
        }

        Some(())
    }

    #[inline]
    fn with<R>(self, fut: impl Future<Output = Option<R>>) -> impl Future<Output = Option<R>> {
        #[cfg(feature = "embassy")]
        {
            use futures::FutureExt;

            embassy_futures::select::select(embassy_time::Timer::at(self.deadline), fut).map(|r| match r {
                embassy_futures::select::Either::First(_) => None,
                embassy_futures::select::Either::Second(r) => r,
            })
        }

        #[cfg(not(feature = "embassy"))]
        fut
    }
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

// ADC / DAC drivers are written against the V3-family register layout
// (`pac::adc::vals::SampleTime`, `pac::dac::vals::TrigSel`, ...). On H4
// these `vals` submodules don't exist because chiptool only emits them
// for fields that carry enum types — H4's HSADC + ADC2 + DAC use a
// different register convention. Gate the V3 driver out until a dedicated
// H4 driver is ported.
#[cfg(all(adc, not(ch32h4)))]
pub mod adc;
#[cfg(all(dac, not(ch32h4)))]
pub mod dac;
pub mod exti;
pub mod gpio;
// spi/can/flash drivers reach into V3-specific register fields that
// don't exist on `*_h4`-versioned peripherals. I2C is the exception:
// H4 chip metadata flags I2C as `version: v3` (same register layout
// as the V2/V3 families), so the existing V3 driver applies directly.
#[cfg(i2c)]
pub mod i2c;
#[cfg(all(rng, not(ch32h4)))]
pub mod rng;
#[cfg(sdio_v3)]
pub mod sdio;
pub mod signature;
#[cfg(all(spi, not(ch32h4)))]
pub mod spi;
#[cfg(any(timer_x0, timer_v3, timer_h4))]
pub mod timer;
pub mod usart;

/// Common structures for USB drivers
pub mod usb;

#[cfg(otg)]
pub mod otg_fs;

#[cfg(usbd)]
pub mod usbd;

#[cfg(usbhs_v3)]
pub mod usbhs;

// H4's USBPD register layout differs from V3 — gate out for now.
#[cfg(all(usbpd, not(ch32h4)))]
pub mod usbpd;

#[cfg(any(eth, emac))]
pub mod eth;
#[cfg(all(can, not(ch32h4)))]
pub mod can;

#[cfg(all(flash, not(ch32h4)))]
pub mod flash;

mod patches;

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
        gpio::init(cs, true);
        dma::init(cs, config.dma_interrupt_priority, true);
        #[cfg(feature = "exti")]
        exti::init(cs);
    });

    p
}

/// What [`init_secondary`] may touch on a core that does not own the chip.
///
/// The defaults only do what is safe while the boot core is running: read the
/// clock tree back, and turn on the GPIO port and DMA clocks (both are
/// write-1-to-enable bits the boot core already set). The AFIO reset and the
/// DMA interrupt enables are off because each changes state the boot core
/// relies on.
#[cfg(rcc_h4)]
pub struct SecondaryConfig {
    /// Crystal frequency, needed only when the boot core switched SYSCLK to
    /// HSE or to a PLL driven by it. `None` for HSI.
    pub hse: Option<time::Hertz>,
    /// Pulse the AFIO reset. That clears `GPIO_AFR` back to AF0, dropping
    /// every alternate function the boot core configured, so leave it off
    /// unless this core owns the whole pin mux.
    pub reset_afio: bool,
    /// Enable every DMA channel interrupt. On CH32H4 an interrupt above 31 is
    /// delivered to exactly one core, and enabling it assigns it to the core
    /// that did the enable, so this takes the DMA interrupts away from the
    /// boot core. Leave it off and let each transfer enable its own channel.
    pub enable_dma_irqs: bool,
    /// Priority stored for the DMA interrupts. Only matters for the channels
    /// this core actually enables.
    pub dma_interrupt_priority: interrupt::Priority,
}

#[cfg(rcc_h4)]
impl Default for SecondaryConfig {
    fn default() -> Self {
        Self {
            hse: None,
            reset_afio: false,
            enable_dma_irqs: false,
            dma_interrupt_priority: interrupt::Priority::P0,
        }
    }
}

/// Initialize the HAL on a core that does not own the clock tree.
///
/// For the second core of a dual-core chip: the boot core has already run
/// [`init`] and programmed the PLL, and doing that again here would switch
/// SYSCLK out from under it. This reads the RCC registers back into [`rcc::clocks`]
/// instead of writing them.
///
/// # Safety
///
/// The boot core must have finished [`init`] before this runs. The two cores
/// must not both use one peripheral. Each image is linked on its own, so each
/// has its own `Peripherals` singleton and this cannot tell that the other
/// core already took one.
#[cfg(rcc_h4)]
pub unsafe fn init_secondary(config: SecondaryConfig) -> Peripherals {
    let p = Peripherals::take();

    unsafe {
        rcc::refresh(config.hse);
        delay::init();

        #[cfg(feature = "embassy")]
        embassy::init();
    }

    ::critical_section::with(|cs| unsafe {
        gpio::init(cs, config.reset_afio);
        dma::init(cs, config.dma_interrupt_priority, config.enable_dma_irqs);
        #[cfg(feature = "exti")]
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
