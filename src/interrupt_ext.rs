use core::sync::atomic::{compiler_fence, Ordering};

use critical_section::CriticalSection;
pub use qingke::interrupt::Priority;
use qingke::pfic;

use crate::pac::InterruptNumber;

/// Generate a standard `mod interrupt` for a HAL.
#[macro_export]
macro_rules! interrupt_mod {
    ($($irqs:ident),* $(,)?) => {

        /// Interrupt definitions.
        pub mod interrupt {
            pub use crate::interrupt_ext::{InterruptExt, Priority};
            pub use crate::pac::Interrupt::*;
            pub use crate::pac::Interrupt;

            /// Type-level interrupt infrastructure.
            ///
            /// This module contains one *type* per interrupt. This is used for checking at compile time that
            /// the interrupts are correctly bound to HAL drivers.
            ///
            /// As an end user, you shouldn't need to use this module directly. Use the [`crate::bind_interrupts!`] macro
            /// to bind interrupts, and the [`crate::interrupt`] module to manually register interrupt handlers and manipulate
            /// interrupts directly (pending/unpending, enabling/disabling, setting the priority, etc...)
            pub mod typelevel {
                use super::InterruptExt;

                trait SealedInterrupt {}

                /// Type-level interrupt.
                ///
                /// This trait is implemented for all typelevel interrupt types in this module.
                #[allow(private_bounds)]
                pub trait Interrupt: SealedInterrupt {

                    /// Interrupt enum variant.
                    ///
                    /// This allows going from typelevel interrupts (one type per interrupt) to
                    /// non-typelevel interrupts (a single `Interrupt` enum type, with one variant per interrupt).
                    const IRQ: super::Interrupt;

                    /// Enable the interrupt.
                    #[inline]
                    unsafe fn enable() {
                        Self::IRQ.enable()
                    }

                    /// Disable the interrupt.
                    #[inline]
                    fn disable() {
                        Self::IRQ.disable()
                    }

                    /// Check if interrupt is enabled.
                    #[inline]
                    fn is_enabled() -> bool {
                        Self::IRQ.is_enabled()
                    }

                    /// Check if interrupt is pending.
                    #[inline]
                    fn is_pending() -> bool {
                        Self::IRQ.is_pending()
                    }

                    /// Set interrupt pending.
                    #[inline]
                    fn pend() {
                        Self::IRQ.pend()
                    }

                    /// Unset interrupt pending.
                    #[inline]
                    fn unpend() {
                        Self::IRQ.unpend()
                    }

                    /// Get the priority of the interrupt.
                    #[inline]
                    fn get_priority() -> crate::interrupt::Priority {
                        Self::IRQ.get_priority()
                    }

                    /// Set the interrupt priority.
                    #[inline]
                    fn set_priority(prio: crate::interrupt::Priority) {
                        Self::IRQ.set_priority(prio)
                    }

                    /// Set the interrupt priority with an already-acquired critical section
                    #[inline]
                    fn set_priority_with_cs(cs: critical_section::CriticalSection, prio: crate::interrupt::Priority) {
                        Self::IRQ.set_priority_with_cs(cs, prio)
                    }
                }

                $(
                    #[allow(non_camel_case_types)]
                    #[doc=stringify!($irqs)]
                    #[doc=" typelevel interrupt."]
                    pub enum $irqs {}
                    impl SealedInterrupt for $irqs{}
                    impl Interrupt for $irqs {
                        const IRQ: super::Interrupt = super::Interrupt::$irqs;
                    }
                )*

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
            }
        }
    };
}

/// Represents an interrupt type that can be configured by embassy to handle
/// interrupts.
pub unsafe trait InterruptExt: InterruptNumber + Copy {
    /// Enable the interrupt.
    #[inline]
    unsafe fn enable(self) {
        compiler_fence(Ordering::SeqCst);
        pfic::enable_interrupt(self.number() as u8)
    }

    /// Disable the interrupt.
    #[inline]
    fn disable(self) {
        unsafe { pfic::disable_interrupt(self.number() as u8) }; // TODO: fix unsafe
        compiler_fence(Ordering::SeqCst);
    }

    /// Check if interrupt is being handled.
    #[inline]
    #[cfg(not(armv6m))]
    fn is_active(self) -> bool {
        pfic::is_active(self.number() as u8)
    }

    /// Check if interrupt is enabled.
    #[inline]
    fn is_enabled(self) -> bool {
        pfic::is_enabled(self.number() as u8)
    }

    /// Check if interrupt is pending.
    #[inline]
    fn is_pending(self) -> bool {
        pfic::is_pending(self.number() as u8)
    }

    /// Set interrupt pending.
    #[inline]
    fn pend(self) {
        unsafe { pfic::pend_interrupt(self.number() as u8) }
    }

    /// Unset interrupt pending.
    #[inline]
    fn unpend(self) {
        unsafe { pfic::unpend_interrupt(self.number() as u8) }
    }

    /// Get the priority of the interrupt.
    #[inline]
    fn get_priority(self) -> Priority {
        Priority::from(pfic::get_priority(self.number() as u8))
    }

    /// Set the interrupt priority.
    #[inline]
    fn set_priority(self, prio: Priority) {
        unsafe { pfic::set_priority(self.number() as u8, prio.into()) }
    }

    /// Set the interrupt priority with an already-acquired critical section
    ///
    /// Equivalent to `set_priority`, except you pass a `CriticalSection` to prove
    /// you've already acquired a critical section. This prevents acquiring another
    /// one, which saves code size.
    #[inline]
    fn set_priority_with_cs(self, _cs: CriticalSection, prio: Priority) {
        unsafe { pfic::set_priority(self.number() as u8, prio.into()) }
    }
}

unsafe impl<T: InterruptNumber + Copy> InterruptExt for T {}
