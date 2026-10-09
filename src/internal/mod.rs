// TODO: replace with embassy-hal-internal

pub mod drop;

#[cfg(any(otg, usbd, usbhs_v3))]
pub(crate) fn delay_us(us: u32) {
    #[cfg(feature = "embassy")]
    embassy_time::block_for(embassy_time::Duration::from_micros(us as u64));

    #[cfg(not(feature = "embassy"))]
    crate::delay::Delay.delay_us(us);
}
