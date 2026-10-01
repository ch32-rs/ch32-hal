use critical_section::CriticalSection;

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

/// Supertrait used by drivers that write `AFIO.PCFR*`.
///
/// Remap chips require [`RemapPeripheral`]. CH32H4 selects the function with
/// a per-pin AF number, so the bound is empty there.
#[cfg(not(afio_h4))]
pub trait RemapBound: RemapPeripheral {}
#[cfg(not(afio_h4))]
impl<T: RemapPeripheral> RemapBound for T {}

#[cfg(afio_h4)]
pub trait RemapBound {}
#[cfg(afio_h4)]
impl<T: ?Sized> RemapBound for T {}
