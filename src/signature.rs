//! E-signature values for CH32 series MCUs

/// Returns the flash size in KByte
pub fn flash_size_kb() -> u16 {
    const ESIG_FLACAP: *mut u16 = 0x1FFFF7E0 as *mut u16;

    unsafe { core::ptr::read_volatile(ESIG_FLACAP) }
}

/// Returns the unique ID
pub fn unique_id() -> [u8; 12] {
    const ESIG_UID: *mut [u8; 12] = 0x1FFFF7E8 as *mut [u8; 12];

    unsafe { core::ptr::read_volatile(ESIG_UID) }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ChipID(u32);

impl ChipID {
    /// Returns the device revision identifier.
    pub fn rev_id(&self) -> u16 {
        (self.0 >> 16) as u16
    }

    /// Returns the device identifier.
    pub fn dev_id(&self) -> u16 {
        (self.0 & 0xFFFF) as u16
    }

    pub fn name(&self) -> &'static str {
        chip_id_to_name(self.0)
    }
}

impl core::fmt::Display for ChipID {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.debug_struct("ChipID")
            .field("rev_id", &(self.0 >> 16))
            .field("dev_id", &(self.0 & 0xFFFF))
            .field("name", &chip_id_to_name(self.0))
            .finish()
    }
}

pub fn chip_id() -> ChipID {
    const CHIP_ID: *mut u32 = 0x1FFFF704 as *mut u32;

    ChipID(unsafe { core::ptr::read_volatile(CHIP_ID) })
}

fn chip_id_to_name(id: u32) -> &'static str {
    // CH32V303CBT6-0x303305x4
    // CH32V303RBT6-0x303205x4
    // CH32V303RCT6-0x303105x4
    // CH32V303VCT6-0x303005x4
    // CH32V305FBP6-0x305205x8
    // CH32V305RBT6-0x305005x8
    // CH32V305GBU6-0x305B05x8
    // CH32V307WCU6-0x307305x8
    // CH32V307FBP6-0x307205x8
    // CH32V307RCT6-0x307105x8
    // CH32V307VCT6-0x307005x8
    match id >> 16 {
        0x3033 => "CH32V303CBT6",
        0x3032 => "CH32V303RBT6",
        0x3031 => "CH32V303RCT6",
        0x3030 => "CH32V303VCT6",
        0x3052 => "CH32V305FBP6",
        0x3050 => "CH32V305RBT6",
        0x305B => "CH32V305GBU6",
        0x3073 => "CH32V307WCU6",
        0x3072 => "CH32V307FBP6",
        0x3071 => "CH32V307RCT6",
        0x3070 => "CH32V307VCT6",
        _ => "Unknown Chip",
    }
}
