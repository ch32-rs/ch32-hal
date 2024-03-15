
#[cfg(qingke_v4)]
pub mod time_driver_systick;

// This should be called after global clocks inited
pub fn init() {
    #[cfg(qingke_v4)]
    time_driver_systick::init();

    #[cfg(qingke_v2)]
    panic!("qingke_v2 is not supported");
}
