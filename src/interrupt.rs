pub use qingke::interrupt::Priority;

use crate::pac::interrupt::Interrupt as InterruptEnum;

mod sealed {
    pub trait Interrupt {}
}

/// Type-level interrupt.
///
/// This trait is implemented for all typelevel interrupt types in this module.
pub trait Interrupt: sealed::Interrupt {
    /// Interrupt enum variant.
    ///
    /// This allows going from typelevel interrupts (one type per interrupt) to
    /// non-typelevel interrupts (a single `Interrupt` enum type, with one variant per interrupt).
    const IRQ: InterruptEnum;
}

macro_rules! impl_irqs {
    ($($irqs:ident),* $(,)?) => {
        $(
            #[allow(non_camel_case_types)]
            #[doc=stringify!($irqs)]
            #[doc=" typelevel interrupt."]
            pub enum $irqs {}
            impl sealed::Interrupt for $irqs{}
            impl Interrupt for $irqs {
                const IRQ: InterruptEnum = InterruptEnum::$irqs;
            }
        )*
    }
}

impl_irqs!(USART1, USART2, USART3);

/// Interrupt handler trait.
///
/// Drivers that need to handle interrupts implement this trait.
/// The user must ensure `on_interrupt()` is called every time the interrupt fires.
/// Drivers must use use [`Binding`] to assert at compile time that the user has done so.
pub trait Handler<I: Interrupt> {
    /// Interrupt handler function.
    ///
    /// Must be called every time the `I` interrupt fires, synchronously from
    /// the interrupt handler context.
    ///
    /// # Safety
    ///
    /// This function must ONLY be called from the interrupt handler for `I`.
    unsafe fn on_interrupt();
}

/// Compile-time assertion that an interrupt has been bound to a handler.
///
/// For the vast majority of cases, you should use the `bind_interrupts!`
/// macro instead of writing `unsafe impl`s of this trait.
///
/// # Safety
///
/// By implementing this trait, you are asserting that you have arranged for `H::on_interrupt()`
/// to be called every time the `I` interrupt fires.
///
/// This allows drivers to check bindings at compile-time.
pub unsafe trait Binding<I: Interrupt, H: Handler<I>> {}
