///! The time driver for Embassy framework.
///
/// This module provides the time driver for the Embassy framework.

#[cfg(all(qingke_v4, not(time_driver_timer)))]
#[path = "time_driver_systick.rs"]
pub mod time_driver_impl;

#[cfg(time_driver_timer)]
#[path = "time_driver_tim.rs"]
pub mod time_driver_impl;

// This should be called after global clocks inited
pub fn init() {
    #[cfg(all(qingke_v4, not(time_driver_timer)))]
    time_driver_impl::init();

    #[cfg(time_driver_timer)]
    critical_section::with(|cs| time_driver_impl::init(cs));
}
