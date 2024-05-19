//! E-signature values for CH32 series MCUs

/// Returns the flash size in KByte
pub fn flash_size_kb() -> u16 {
    const ESIG_FLACAP: *const u16 = 0x1FFFF7E0 as *const u16;

    unsafe { core::ptr::read_volatile(ESIG_FLACAP) }
}

/// Returns the unique ID
pub fn unique_id() -> [u8; 12] {
    const ESIG_UID: *const [u8; 12] = 0x1FFFF7E8 as *const [u8; 12];

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
    #[cfg(not(any(ch32v0, ch32v1, ch641)))]
    const CHIP_ID: *const u32 = 0x1FFFF704 as *const u32;
    #[cfg(any(ch32v0, ch641))]
    const CHIP_ID: *const u32 = 0x1FFFF7C4 as *const u32;
    #[cfg(ch32v1)]
    const CHIP_ID: *const u32 = 0x1FFFF884 as *const u32;

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

    // CH32V203C8U6-0x203005x0
    // CH32V203C8T6-0x203105x0
    // CH32V203K8T6-0x203205x0
    // CH32V203C6T6-0x203305x0
    // CH32V203K6T6-0x203505x0
    // CH32V203G6U6-0x203605x0
    // CH32V203G8R6-0x203B05x0
    // CH32V203F8U6-0x203E05x0
    // CH32V203F6P6-0x203705x0-0x203905x0
    // CH32V203F8P6-0x203A05x0
    // CH32V203RBT6-0x203405xC
    // CH32V208WBU6-0x208005xC
    // CH32V208RBT6-0x208105xC
    // CH32V208CBU6-0x208205xC
    // CH32V208GBU6-0x208305xC

    // CH32V103C8T6-0x25004102
    // CH32V103R8T6-0x2500410F

    // CH32X035R8T6-0x035006x1
    // CH32X035C8T6-0x035106x1
    // CH32X035F8U6-0x035E06x1
    // CH32X035G8U6-0x035606x1
    // CH32X035G8R6-0x035B06x1
    // CH32X035F7P6-0x035706x1
    // CH32X033F8P6-0x035A06x1

    // CH32V003F4P6-0x003005x0
    // CH32V003F4U6-0x003105x0
    // CH32V003A4M6-0x003205x0
    // CH32V003J4M6-0x003305x0

    // CH32L103C8U6-0x103007x0
    // CH32L103C8T6-0x103107x0
    // CH32L103F8P6-0x103A07x0
    // CH32L103G8R6-0x103B07x0
    // CH32L103K8U6-0x103207x0
    // CH32L103F8U6-0x103D07x0
    // CH32L103F7P6-0x103707x0

    // CH643W-0x64300601
    // CH643Q-0x64310601
    // CH643L-0x64330601
    // CH643U-0x64340601

    // CH641F-0x641005x0
    // CH641D-0x641105x0
    // CH641U-0x641505x0
    // CH641P-0x641605x0
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

        0x2030 => "CH32V203C8U6",
        0x2031 => "CH32V203C8T6",
        0x2032 => "CH32V203K8T6",
        0x2033 => "CH32V203C6T6",
        0x2035 => "CH32V203K6T6",
        0x2036 => "CH32V203G6U6",
        0x203B => "CH32V203G8R6",
        0x203E => "CH32V203F8U6",
        0x2037 => "CH32V203F6P6",
        0x2039 => "CH32V203F6P6",
        0x203A => "CH32V203F8P6",
        0x2034 => "CH32V203RBT6",
        0x2080 => "CH32V208WBU6",
        0x2081 => "CH32V208RBT6",
        0x2082 => "CH32V208CBU6",
        0x2083 => "CH32V208GBU6",

        0x2500 => "CH32V103x8T6",

        0x0350 => "CH32X035R8T6",
        0x0351 => "CH32X035C8T6",
        0x035E => "CH32X035F8U6",
        0x0356 => "CH32X035G8U6",
        0x035B => "CH32X035G8R6",
        0x0357 => "CH32X035F7P6",
        0x035A => "CH32X033F8P6",

        0x0030 => "CH32V003F4P6",
        0x0031 => "CH32V003F4U6",
        0x0032 => "CH32V003A4M6",
        0x0033 => "CH32V003J4M6",

        0x1030 => "CH32L103C8U6",
        0x1031 => "CH32L103C8T6",
        0x103A => "CH32L103F8P6",
        0x103B => "CH32L103G8R6",
        0x1032 => "CH32L103K8U6",
        0x103D => "CH32L103F8U6",
        0x1037 => "CH32L103F7P6",

        0x6430 => "CH643W",
        0x6431 => "CH643Q",
        0x6433 => "CH643L",
        0x6434 => "CH643U",

        0x6410 => "CH641F",
        0x6411 => "CH641D",
        0x6415 => "CH641U",
        0x6416 => "CH641P",

        _ => "Unknown",
    }
}
