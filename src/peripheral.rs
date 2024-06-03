use core::marker::PhantomData;
use core::ops::Deref;

use critical_section::CriticalSection;

/// An exclusive reference to a peripheral.
///
/// This is functionally the same as a `&'a mut T`. There's a few advantages in having
/// a dedicated struct instead:
///
/// - Memory efficiency: Peripheral singletons are typically either zero-sized (for concrete
///   peripherals like `PA9` or `SPI4`) or very small (for example `AnyPin`, which is 1 byte).
///   However `&mut T` is always 4 bytes for 32-bit targets, even if T is zero-sized.
///   PeripheralRef stores a copy of `T` instead, so it's the same size.
/// - Code size efficiency. If the user uses the same driver with both `SPI4` and `&mut SPI4`,
///   the driver code would be monomorphized two times. With PeripheralRef, the driver is generic
///   over a lifetime only. `SPI4` becomes `PeripheralRef<'static, SPI4>`, and `&mut SPI4` becomes
///   `PeripheralRef<'a, SPI4>`. Lifetimes don't cause monomorphization.
pub struct PeripheralRef<'a, T> {
    inner: T,
    _lifetime: PhantomData<&'a mut T>,
}

impl<'a, T> PeripheralRef<'a, T> {
    /// Create a new reference to a peripheral.
    #[inline]
    pub fn new(inner: T) -> Self {
        Self {
            inner,
            _lifetime: PhantomData,
        }
    }

    /// Unsafely clone (duplicate) a peripheral singleton.
    ///
    /// # Safety
    ///
    /// This returns an owned clone of the peripheral. You must manually ensure
    /// only one copy of the peripheral is in use at a time. For example, don't
    /// create two SPI drivers on `SPI1`, because they will "fight" each other.
    ///
    /// You should strongly prefer using `reborrow()` instead. It returns a
    /// `PeripheralRef` that borrows `self`, which allows the borrow checker
    /// to enforce this at compile time.
    pub unsafe fn clone_unchecked(&self) -> PeripheralRef<'a, T>
    where
        T: Peripheral<P = T>,
    {
        PeripheralRef::new(self.inner.clone_unchecked())
    }

    /// Reborrow into a "child" PeripheralRef.
    ///
    /// `self` will stay borrowed until the child PeripheralRef is dropped.
    pub fn reborrow(&mut self) -> PeripheralRef<'_, T>
    where
        T: Peripheral<P = T>,
    {
        // safety: we're returning the clone inside a new PeripheralRef that borrows
        // self, so user code can't use both at the same time.
        PeripheralRef::new(unsafe { self.inner.clone_unchecked() })
    }

    /// Map the inner peripheral using `Into`.
    ///
    /// This converts from `PeripheralRef<'a, T>` to `PeripheralRef<'a, U>`, using an
    /// `Into` impl to convert from `T` to `U`.
    ///
    /// For example, this can be useful to degrade GPIO pins: converting from PeripheralRef<'a, PB11>` to `PeripheralRef<'a, AnyPin>`.
    #[inline]
    pub fn map_into<U>(self) -> PeripheralRef<'a, U>
    where
        T: Into<U>,
    {
        PeripheralRef {
            inner: self.inner.into(),
            _lifetime: PhantomData,
        }
    }
}

impl<'a, T> Deref for PeripheralRef<'a, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

/// Trait for any type that can be used as a peripheral of type `P`.
///
/// This is used in driver constructors, to allow passing either owned peripherals (e.g. `TWISPI0`),
/// or borrowed peripherals (e.g. `&mut TWISPI0`).
///
/// For example, if you have a driver with a constructor like this:
///
/// ```ignore
/// impl<'d, T: Instance> Twim<'d, T> {
///     pub fn new(
///         twim: impl Peripheral<P = T> + 'd,
///         irq: impl Peripheral<P = T::Interrupt> + 'd,
///         sda: impl Peripheral<P = impl GpioPin> + 'd,
///         scl: impl Peripheral<P = impl GpioPin> + 'd,
///         config: Config,
///     ) -> Self { .. }
/// }
/// ```
///
/// You may call it with owned peripherals, which yields an instance that can live forever (`'static`):
///
/// ```ignore
/// let mut twi: Twim<'static, ...> = Twim::new(p.TWISPI0, irq, p.P0_03, p.P0_04, config);
/// ```
///
/// Or you may call it with borrowed peripherals, which yields an instance that can only live for as long
/// as the borrows last:
///
/// ```ignore
/// let mut twi: Twim<'_, ...> = Twim::new(&mut p.TWISPI0, &mut irq, &mut p.P0_03, &mut p.P0_04, config);
/// ```
///
/// # Implementation details, for HAL authors
///
/// When writing a HAL, the intended way to use this trait is to take `impl Peripheral<P = ..>` in
/// the HAL's public API (such as driver constructors), calling `.into_ref()` to obtain a `PeripheralRef`,
/// and storing that in the driver struct.
///
/// `.into_ref()` on an owned `T` yields a `PeripheralRef<'static, T>`.
/// `.into_ref()` on an `&'a mut T` yields a `PeripheralRef<'a, T>`.
pub trait Peripheral: Sized {
    /// Peripheral singleton type
    type P;

    /// Unsafely clone (duplicate) a peripheral singleton.
    ///
    /// # Safety
    ///
    /// This returns an owned clone of the peripheral. You must manually ensure
    /// only one copy of the peripheral is in use at a time. For example, don't
    /// create two SPI drivers on `SPI1`, because they will "fight" each other.
    ///
    /// You should strongly prefer using `into_ref()` instead. It returns a
    /// `PeripheralRef`, which allows the borrow checker to enforce this at compile time.
    unsafe fn clone_unchecked(&self) -> Self::P;

    /// Convert a value into a `PeripheralRef`.
    ///
    /// When called on an owned `T`, yields a `PeripheralRef<'static, T>`.
    /// When called on an `&'a mut T`, yields a `PeripheralRef<'a, T>`.
    #[inline]
    fn into_ref<'a>(self) -> PeripheralRef<'a, Self::P>
    where
        Self: 'a,
    {
        PeripheralRef::new(unsafe { self.clone_unchecked() })
    }
}

impl<'b, T: Deref> Peripheral for T
where
    T::Target: Peripheral,
{
    type P = <T::Target as Peripheral>::P;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        self.deref().clone_unchecked()
    }
}

// # macros

/// Types for the peripheral singletons.
#[macro_export]
macro_rules! peripherals_definition {
    ($($(#[$cfg:meta])? $name:ident),*$(,)?) => {
        /// Types for the peripheral singletons.
        pub mod peripherals {
            $(
                $(#[$cfg])?
                #[allow(non_camel_case_types)]
                #[doc = concat!(stringify!($name), " peripheral")]
                pub struct $name { _private: () }

                $(#[$cfg])?
                impl $name {
                    /// Unsafely create an instance of this peripheral out of thin air.
                    ///
                    /// # Safety
                    ///
                    /// You must ensure that you're only using one instance of this type at a time.
                    #[inline]
                    pub unsafe fn steal() -> Self {
                        Self{ _private: ()}
                    }
                }

                $(#[$cfg])?
                $crate::impl_peripheral!($name);
            )*
        }
    };
}

/// Define the peripherals struct.
#[macro_export]
macro_rules! peripherals_struct {
    ($($(#[$cfg:meta])? $name:ident),*$(,)?) => {
        /// Struct containing all the peripheral singletons.
        ///
        /// To obtain the peripherals, you must initialize the HAL, by calling [`crate::init`].
        #[allow(non_snake_case)]
        pub struct Peripherals {
            $(
                #[doc = concat!(stringify!($name), " peripheral")]
                $(#[$cfg])?
                pub $name: peripherals::$name,
            )*
        }

        impl Peripherals {
            ///Returns all the peripherals *once*
            #[inline]
            pub(crate) fn take() -> Self {
                critical_section::with(Self::take_with_cs)
            }

            ///Returns all the peripherals *once*
            #[inline]
            pub(crate) fn take_with_cs(_cs: critical_section::CriticalSection) -> Self {
                #[no_mangle]
                static mut _EMBASSY_DEVICE_PERIPHERALS: bool = false;

                // safety: OK because we're inside a CS.
                unsafe {
                    if _EMBASSY_DEVICE_PERIPHERALS {
                        panic!("init called more than once!")
                    }
                    _EMBASSY_DEVICE_PERIPHERALS = true;
                    Self::steal()
                }
            }
        }

        impl Peripherals {
            /// Unsafely create an instance of this peripheral out of thin air.
            ///
            /// # Safety
            ///
            /// You must ensure that you're only using one instance of this type at a time.
            #[inline]
            pub unsafe fn steal() -> Self {
                Self {
                    $(
                        $(#[$cfg])?
                        $name: peripherals::$name::steal(),
                    )*
                }
            }
        }
    };
}

/// Defining peripheral type.
#[macro_export]
macro_rules! peripherals {
    ($($(#[$cfg:meta])? $name:ident),*$(,)?) => {
        $crate::peripherals_definition!(
            $(
                $(#[$cfg])?
                $name,
            )*
        );
        $crate::peripherals_struct!(
            $(
                $(#[$cfg])?
                $name,
            )*
        );
    };
}

/// Convenience converting into reference.
#[macro_export]
macro_rules! into_ref {
    ($($name:ident),*) => {
        $(
            #[allow(unused_mut)]
            let mut $name = $name.into_ref();
        )*
    }
}

/// Implement the peripheral trait.
#[macro_export]
macro_rules! impl_peripheral {
    ($type:ident) => {
        impl $crate::Peripheral for $type {
            type P = $type;

            #[inline]
            unsafe fn clone_unchecked(&self) -> Self::P {
                #[allow(clippy::needless_update)]
                $type { ..*self }
            }
        }
    };
}

pub(crate) trait SealedRccPeripheral {
    fn frequency() -> crate::time::Hertz;
    fn enable_and_reset_with_cs(cs: CriticalSection);
    fn disable_with_cs(cs: CriticalSection);

    fn enable_and_reset() {
        critical_section::with(|cs| Self::enable_and_reset_with_cs(cs))
    }
    fn disable() {
        critical_section::with(|cs| Self::disable_with_cs(cs))
    }
}

pub(crate) trait SealedRemapPeripheral {
    fn set_remap(remap: u8);
}

#[allow(private_bounds)]
pub trait RccPeripheral: SealedRccPeripheral + 'static {}
#[allow(private_bounds)]
pub trait RemapPeripheral: SealedRemapPeripheral + 'static {}
