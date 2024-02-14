pub mod time_driver_systick;

// This should be called after global clocks inited
pub fn init() {
    time_driver_systick::init();
}
