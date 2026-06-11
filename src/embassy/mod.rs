///! The time driver for Embassy framework.
///
/// This module provides the time driver for the Embassy framework.

// `time_driver_systick.rs` reaches for V3-style `SYSTICK.{ctlr,cmpl,
// cmph,cnt,sr}()` — those names don't exist on CH32H4's dual-core
// `systick_v3f_v5f` peripheral (which uses `CTLR_0`/`CMP_0`/`CNT_0`/
// `ISR`). Until a dedicated H4 systick time driver lands, H4 users
// must opt into a TIM-based time driver via `time-driver-timN`.
#[cfg(all(qingke_v4, not(time_driver_timer), not(ch32h4)))]
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

    #[cfg(any(all(qingke_v4, not(ch32h4)), time_driver_timer))]
    critical_section::with(|cs| time_driver_impl::init(cs));
}
