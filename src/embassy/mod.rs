///! The time driver for Embassy framework.
///
/// This module provides the time driver for the Embassy framework.

#[cfg(all(qingke_v4, not(time_driver_timer)))]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

#[cfg(time_driver_timer)]
#[path = "time_driver_tim.rs"]
pub mod time_driver_impl;

/// Initialize the Embassy time driver.
///
/// System global clocks must be initialized before calling this function.
///
/// # Safety
///
/// This function should be called only once.
///
/// # Implementation Notes
///
/// The WCH QingKe RISC-V core deviates from standard RISC-V specification:
/// - `WFI` instruction will not wake up from disabled interrupts
/// - Either `WFITOWFE` or `SEVONPEND` must be enabled for proper wake-up behavior
pub unsafe fn init() {
    #[cfg(feature = "rt-wfi")]
    crate::pac::PFIC.sctlr().modify(|w| w.set_sevonpend(true));

    #[cfg(any(qingke_v4, time_driver_timer))]
    critical_section::with(|cs| time_driver_impl::init(cs));
}
