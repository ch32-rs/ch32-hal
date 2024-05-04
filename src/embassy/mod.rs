#[cfg(qingke_v4)]
pub mod time_driver_systick;

//#[cfg(qingke_v3)]
// pub mod time_driver_systick_qingke_v3;

// This should be called after global clocks inited
pub fn init() {
    #[cfg(qingke_v4)]
    time_driver_systick::init();

    //#[cfg(qingke_v3)]
    //time_driver_systick_qingke_v3::init();

    #[cfg(qingke_v2)]
    panic!("qingke_v2(rv32ec) is not supported");
}
