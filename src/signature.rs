pub fn flash_size_kb() -> u16 {
    const ESIG_FLACAP: *mut u16 = 0x1FFFF7E0 as *mut u16;

    unsafe { core::ptr::read_volatile(ESIG_FLACAP) }
}

pub fn unique_id() -> [u8; 12] {
    const ESIG_UID: *mut [u8; 12] = 0x1FFFF7E8 as *mut [u8; 12];

    unsafe { core::ptr::read_volatile(ESIG_UID) }
}
