#[allow(dead_code)]
use core::marker::PhantomData;
use core::sync::atomic::{fence, Ordering};

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embedded_storage::nor_flash::ReadNorFlash;

use super::{family, Async, Blocking, Error, FlashSector, FLASH_SIZE, READ_SIZE, WRITE_SIZE};
use crate::pac::FLASH_BASE;
use crate::peripherals::FLASH;
use crate::Peripheral;

/// Internal flash memory driver.
pub struct Flash<'d, MODE = Async> {
    pub(crate) inner: PeripheralRef<'d, FLASH>,
    pub(crate) _mode: PhantomData<MODE>,
}

impl<'d> Flash<'d, Blocking> {
    /// Create a new flash driver, usable in blocking mode.
    pub fn new_blocking(p: impl Peripheral<P = FLASH> + 'd) -> Self {
        into_ref!(p);

        Self {
            inner: p,
            _mode: PhantomData,
        }
    }
}

impl<'d, MODE> Flash<'d, MODE> {
    /// Blocking read.
    ///
    /// NOTE: `offset` is an offset from the flash start, NOT an absolute address.
    /// For example, to read address `0x0800_1234` you have to use offset `0x1234`.
    pub fn blocking_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        blocking_read(FLASH_BASE as u32, FLASH_SIZE as u32, offset, bytes)
    }

    /// Blocking write.
    ///
    /// NOTE: `offset` is an offset from the flash start, NOT an absolute address.
    /// For example, to write address `0x0800_1234` you have to use offset `0x1234`.
    pub fn blocking_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        unsafe {
            blocking_write(
                FLASH_BASE as u32,
                FLASH_SIZE as u32,
                offset,
                bytes,
                write_chunk_unlocked,
            )
        }
    }

    /// Blocking erase.
    ///
    /// NOTE: `from` and `to` are offsets from the flash start, NOT an absolute address.
    /// For example, to erase address `0x0801_0000` you have to use offset `0x1_0000`.
    pub fn blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        unsafe { blocking_erase(FLASH_BASE as u32, from, to, erase_sector_unlocked) }
    }
}

pub(super) fn blocking_read(base: u32, size: u32, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
    if offset + bytes.len() as u32 > size {
        return Err(Error::Size);
    }

    let start_address = base + offset;

    #[cfg(flash_f4)]
    family::assert_not_corrupted_read(start_address + bytes.len() as u32);

    let flash_data = unsafe { core::slice::from_raw_parts(start_address as *const u8, bytes.len()) };
    bytes.copy_from_slice(flash_data);
    Ok(())
}

pub(super) unsafe fn blocking_write(
    base: u32,
    size: u32,
    offset: u32,
    bytes: &[u8],
    write_chunk: unsafe fn(u32, &[u8]) -> Result<(), Error>,
) -> Result<(), Error> {
    if offset + bytes.len() as u32 > size {
        return Err(Error::Size);
    }
    if offset % WRITE_SIZE as u32 != 0 || bytes.len() % WRITE_SIZE != 0 {
        return Err(Error::Unaligned);
    }

    let mut address = base + offset;

    for chunk in bytes.chunks(WRITE_SIZE) {
        write_chunk(address, chunk)?;
        address += WRITE_SIZE as u32;
    }
    Ok(())
}

pub(super) unsafe fn write_chunk_unlocked(address: u32, chunk: &[u8]) -> Result<(), Error> {
    family::clear_all_err();
    fence(Ordering::SeqCst);
    family::unlock();
    fence(Ordering::SeqCst);
    family::enable_blocking_write();
    fence(Ordering::SeqCst);

    let _on_drop = OnDrop::new(|| {
        family::disable_blocking_write();
        fence(Ordering::SeqCst);
        family::lock();
    });

    family::blocking_write(address, unwrap!(chunk.try_into()))
}

pub(super) unsafe fn write_chunk_with_critical_section(address: u32, chunk: &[u8]) -> Result<(), Error> {
    critical_section::with(|_| write_chunk_unlocked(address, chunk))
}

pub(super) unsafe fn blocking_erase(
    base: u32,
    from: u32,
    to: u32,
    erase_sector: unsafe fn(&FlashSector) -> Result<(), Error>,
) -> Result<(), Error> {
    let start_address = base + from;
    let end_address = base + to;

    ensure_sector_aligned(start_address, end_address)?;
    family::unlock();
    let _on_drop = OnDrop::new(|| {
        family::lock();
    });

    trace!("Erasing from 0x{:x} to 0x{:x}", start_address, end_address);

    let mut address = start_address;
    while address < end_address {
        let sector = get_sector(address);
        trace!("Erasing sector: {:?}", sector);
        erase_sector(&sector)?;
        address += sector.size;
    }
    Ok(())
}

pub(super) unsafe fn erase_sector_unlocked(sector: &FlashSector) -> Result<(), Error> {
    family::clear_all_err();
    fence(Ordering::SeqCst);
    family::unlock();
    fence(Ordering::SeqCst);

    let _on_drop = OnDrop::new(|| family::lock());

    family::blocking_erase_sector(sector)
}

pub(super) unsafe fn erase_sector_with_critical_section(sector: &FlashSector) -> Result<(), Error> {
    critical_section::with(|_| erase_sector_unlocked(sector))
}

pub(super) fn get_sector(address: u32) -> FlashSector {
    let mut bank_offset = 0;
    let index_in_region = (address - FLASH_BASE as u32) / WRITE_SIZE as u32;
    if (address as usize) < (FLASH_BASE + FLASH_SIZE) {
        return FlashSector {
            start: FLASH_BASE as u32 + index_in_region * WRITE_SIZE as u32,
            size: WRITE_SIZE as u32,
        };
    }

    panic!("Flash sector not found");
}

pub(super) fn ensure_sector_aligned(start_address: u32, end_address: u32) -> Result<(), Error> {
    let mut address = start_address;
    while address < end_address {
        let sector = get_sector(address);
        if sector.start != address {
            return Err(Error::Unaligned);
        }
        address += sector.size;
    }
    if address != end_address {
        return Err(Error::Unaligned);
    }
    Ok(())
}

impl<MODE> embedded_storage::nor_flash::ErrorType for Flash<'_, MODE> {
    type Error = Error;
}

impl<MODE> embedded_storage::nor_flash::ReadNorFlash for Flash<'_, MODE> {
    const READ_SIZE: usize = READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        self.blocking_read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl<MODE> embedded_storage::nor_flash::NorFlash for Flash<'_, MODE> {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = WRITE_SIZE;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.blocking_write(offset, bytes)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.blocking_erase(from, to)
    }
}
