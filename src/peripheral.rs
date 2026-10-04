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

#[allow(private_bounds)]
pub trait RccPeripheral: SealedRccPeripheral + 'static {}
