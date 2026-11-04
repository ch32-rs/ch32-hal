use embedded_can::{ExtendedId, Frame, Id, StandardId};

/// A Classical CAN data or remote frame with an 11-bit or 29-bit identifier.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    pub(super) id: Id,
    pub(super) data: [u8; 8],
    pub(super) dlc: u8,
    pub(super) remote: bool,
}

impl CanFrame {
    pub fn new(id: impl Into<Id>, bytes: &[u8]) -> Option<Self> {
        if bytes.len() > 8 {
            return None;
        }
        let mut data = [0; 8];
        data[..bytes.len()].copy_from_slice(bytes);
        Some(Self {
            id: id.into(),
            data,
            dlc: bytes.len() as u8,
            remote: false,
        })
    }

    pub fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        if dlc > 8 {
            return None;
        }
        Some(Self {
            id: id.into(),
            data: [0; 8],
            dlc: dlc as u8,
            remote: true,
        })
    }

    pub fn id(&self) -> Id {
        self.id
    }

    pub fn dlc(&self) -> usize {
        self.dlc as usize
    }

    pub fn data(&self) -> &[u8] {
        if self.remote {
            &[]
        } else {
            &self.data[..self.dlc as usize]
        }
    }

    pub fn is_remote_frame(&self) -> bool {
        self.remote
    }

    pub(super) fn identifier(&self) -> u32 {
        encode_id(self.id) | ((self.remote as u32) << 1)
    }

    pub(super) fn decode(identifier: u32, dlc: u8, low: u32, high: u32) -> Self {
        let id = if identifier & 4 != 0 {
            Id::Extended(ExtendedId::new(identifier >> 3).unwrap())
        } else {
            Id::Standard(StandardId::new((identifier >> 21) as u16).unwrap())
        };
        let remote = identifier & 2 != 0;
        let dlc = dlc.min(8);
        let mut data = (u64::from(low) | (u64::from(high) << 32)).to_le_bytes();
        data[if remote { 0 } else { dlc as usize }..].fill(0);
        Self { id, data, dlc, remote }
    }
}

impl Frame for CanFrame {
    fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        Self::new(id, data)
    }

    fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        Self::new_remote(id, dlc)
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
    }

    fn is_remote_frame(&self) -> bool {
        self.remote
    }

    fn id(&self) -> Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc()
    }

    fn data(&self) -> &[u8] {
        self.data()
    }
}

pub(super) fn encode_id(id: Id) -> u32 {
    match id {
        Id::Standard(id) => u32::from(id.as_raw()) << 21,
        Id::Extended(id) => (id.as_raw() << 3) | 4,
    }
}

/// One of the 14 filter banks. Raw fields follow RM §25.7.16–25.7.20.
#[derive(Clone, Copy, Debug)]
pub struct Filter {
    pub bank: u8,
    pub fifo: super::CanFifo,
    pub scale_32bit: bool,
    pub list: bool,
    pub registers: [u32; 2],
}

impl Filter {
    pub const fn accept_all(bank: u8, fifo: super::CanFifo) -> Self {
        Self {
            bank,
            fifo,
            scale_32bit: true,
            list: false,
            registers: [0; 2],
        }
    }

    /// Match an identifier and its format, with optional data/remote matching.
    pub fn exact(bank: u8, fifo: super::CanFifo, id: impl Into<Id>, remote: Option<bool>) -> Self {
        let id = id.into();
        let mask = match id {
            Id::Standard(_) => (0x7ff << 21) | 4,
            Id::Extended(_) => (0x1fff_ffff << 3) | 4,
        } | if remote.is_some() { 2 } else { 0 };
        Self {
            bank,
            fifo,
            scale_32bit: true,
            list: false,
            registers: [encode_id(id) | ((remote.unwrap_or(false) as u32) << 1), mask],
        }
    }
}

/// Standard Classical CAN timing; extended long-TS1 operation is not enabled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BitTiming {
    pub prescaler: u16,
    pub seg1: u8,
    pub seg2: u8,
    pub sync_jump_width: u8,
}

impl BitTiming {
    /// Find an exact bitrate, preferring a sample point near 87.5% (75% above
    /// 500 kbit/s), then the largest number of time quanta.
    pub fn from_bitrate(clock: u32, bitrate: u32) -> Option<Self> {
        if clock == 0 || !(1_000..=1_000_000).contains(&bitrate) {
            return None;
        }
        let target = if bitrate > 500_000 { 750 } else { 875 };
        let mut best = None;
        let mut best_score = u32::MAX;
        for total in (3..=25u32).rev() {
            let denominator = u64::from(bitrate) * u64::from(total);
            if u64::from(clock) % denominator != 0 {
                continue;
            }
            let prescaler = u64::from(clock) / denominator;
            if !(1..=1024).contains(&prescaler) {
                continue;
            }
            for seg2 in 1..=8u8 {
                if total <= u32::from(seg2) + 1 {
                    continue;
                }
                let seg1 = total - 1 - u32::from(seg2);
                if seg1 > 16 {
                    continue;
                }
                let score = ((1 + seg1) * 1000 / total).abs_diff(target);
                if score < best_score {
                    best_score = score;
                    best = Some(Self {
                        prescaler: prescaler as u16,
                        seg1: seg1 as u8,
                        seg2,
                        sync_jump_width: 1,
                    });
                }
            }
        }
        best
    }

    pub(super) fn register(self) -> Option<u32> {
        if !(1..=1024).contains(&self.prescaler)
            || !(1..=16).contains(&self.seg1)
            || !(1..=8).contains(&self.seg2)
            || !(1..=4).contains(&self.sync_jump_width)
            || self.sync_jump_width > self.seg2
        {
            return None;
        }
        Some(
            u32::from(self.prescaler - 1)
                | (u32::from(self.seg1 - 1) << 16)
                | (u32::from(self.seg2 - 1) << 20)
                | (u32::from(self.sync_jump_width - 1) << 24),
        )
    }
}
