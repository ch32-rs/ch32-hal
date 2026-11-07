///! The time driver for Embassy framework.
///
/// This module provides the time driver for the Embassy framework.

// CH32H4's SysTick is two 32-bit counters, so it has its own driver.
// `time_driver_systick.rs` is the 64-bit counter used by the other
// `qingke_v4` families.
#[cfg(all(qingke_v4, not(time_driver_timer), not(ch32h4)))]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

#[cfg(all(systick_v4x7, not(time_driver_timer)))]
#[path = "time_driver_systick_32bit.rs"]
pub mod time_driver_impl;

// Hart 0 only. Hart 1's counter interrupts through `SysTick1` (vector 13),
// which `qingke-rt` does not expose, so a V5F image still selects
// `time-driver-timN`.
#[cfg(all(systick_v3f_v5f, not(time_driver_timer), not(qingke_v5f)))]
#[path = "time_driver_systick_h4.rs"]
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

    #[cfg(any(
        time_driver_timer,
        systick_v4x7,
        all(qingke_v4, not(ch32h4)),
        all(systick_v3f_v5f, not(qingke_v5f)),
    ))]
    critical_section::with(|cs| time_driver_impl::init(cs));
}
