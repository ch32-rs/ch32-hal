use core::ops::{Deref, DerefMut};

use sdio_host::{CardCapacity, SDStatus, CID, CSD, OCR, SCR};

use crate::time::Hertz;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Timeout,
    SoftwareTimeout,
    Crc,
    StBitErr,
    Overrun,
    Underrun,
    Dma,
    IncompleteTransfer,
    ResponseMismatch,
    CardStatus(u32),
    CardLocked,
    UnsupportedCardVersion,
    UnsupportedCardType,
    UnsupportedBusWidth,
    SignalingSwitchFailed,
    NoCard,
    BadClock,
    InvalidConfig,
    InvalidBufferLength,
    AddressOutOfRange,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub data_transfer_timeout: u32,
    pub software_timeout_ms: u32,
    pub poll_limit: u32,
    pub card_ready_attempts: u16,
    /// Bypass requires hardware flow control on V4x7. Disabled by default.
    pub allow_clock_bypass: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            data_transfer_timeout: 5_000_000,
            software_timeout_ms: 1_000,
            poll_limit: 1_000_000,
            card_ready_attempts: 1_000,
            allow_clock_bypass: false,
        }
    }
}

#[repr(align(4))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DataBlock(pub [u8; 512]);

impl Deref for DataBlock {
    type Target = [u8; 512];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for DataBlock {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum Signalling {
    #[default]
    SDR12,
    SDR25,
}

#[derive(Clone, Copy, Debug)]
pub struct Card {
    pub card_type: CardCapacity,
    pub ocr: OCR,
    pub rca: u16,
    pub cid: CID,
    pub csd: CSD,
    pub scr: SCR,
    pub status: SDStatus,
    /// Number of 512-byte logical blocks, including the 2 TiB SDXC boundary.
    pub block_count: u64,
}

impl Card {
    pub fn size(&self) -> u64 {
        self.block_count * 512
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) struct Clock {
    pub divisor: u8,
    pub bypass: bool,
    pub frequency: Hertz,
}

pub(super) fn clock_setting(source: Hertz, requested: u32, allow_bypass: bool) -> Result<Clock, Error> {
    if source.0 == 0 || requested == 0 || requested > 80_000_000 {
        return Err(Error::BadClock);
    }
    if allow_bypass && source.0 <= requested {
        return Ok(Clock {
            divisor: 0,
            bypass: true,
            frequency: source,
        });
    }
    let divisor = u64::from(source.0).div_ceil(u64::from(requested)).max(2);
    if divisor > 257 {
        return Err(Error::BadClock);
    }
    Ok(Clock {
        divisor: (divisor - 2) as u8,
        bypass: false,
        frequency: Hertz(source.0 / divisor as u32),
    })
}

pub(super) fn data_error(status: u32) -> Result<(), Error> {
    if status & 8 != 0 {
        Err(Error::Timeout)
    } else if status & 2 != 0 {
        Err(Error::Crc)
    } else if status & 0x20 != 0 {
        Err(Error::Overrun)
    } else if status & 0x10 != 0 {
        Err(Error::Underrun)
    } else if status & 0x200 != 0 {
        Err(Error::StBitErr)
    } else {
        Ok(())
    }
}

pub(super) fn block_count(csd: u128) -> Result<u64, Error> {
    match csd >> 126 {
        0 => {
            let block_length = ((csd >> 80) & 15) as u32;
            if !(9..=11).contains(&block_length) {
                return Err(Error::UnsupportedCardType);
            }
            let size = ((csd >> 62) & 0xfff) as u64 + 1;
            let multiplier = ((csd >> 47) & 7) as u32 + 2;
            Ok(size << (multiplier + block_length - 9))
        }
        1 => Ok((((csd >> 48) & 0x3f_ffff) as u64 + 1) * 1024),
        _ => Err(Error::UnsupportedCardType),
    }
}

pub(super) fn transfer_rate(encoded: u8) -> Result<u32, Error> {
    let units = [100_000u32, 1_000_000, 10_000_000, 100_000_000];
    let multipliers = [0u32, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80];
    if encoded & 0x80 != 0 || encoded & 7 > 3 || encoded & 0x78 == 0 {
        return Err(Error::UnsupportedCardType);
    }
    Ok(units[(encoded & 7) as usize] / 10 * multipliers[((encoded >> 3) & 15) as usize])
}

pub(super) fn block_address(card: &Card, index: u32, count: usize) -> Result<u32, Error> {
    if count == 0 || count > 511 {
        return Err(Error::InvalidBufferLength);
    }
    if u64::from(index) + count as u64 > card.block_count {
        return Err(Error::AddressOutOfRange);
    }
    match card.card_type {
        CardCapacity::SDSC => index.checked_mul(512).ok_or(Error::AddressOutOfRange),
        CardCapacity::SDHC => Ok(index),
        _ => Err(Error::UnsupportedCardType),
    }
}

pub(super) fn response_value(words: [u32; 4]) -> u128 {
    words
        .into_iter()
        .fold(0, |value, word| (value << 32) | u128::from(word))
}

pub(super) fn status_bytes(words: &[u32; 16]) -> [u8; 64] {
    core::array::from_fn(|index| words[index / 4].to_le_bytes()[index % 4])
}

pub(super) fn sd_status(words: &[u32; 16]) -> SDStatus {
    core::array::from_fn(|index| u32::from_be(words[15 - index])).into()
}
