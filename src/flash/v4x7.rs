//! CH32V407/CH32V467 raw flash and an explicit XOR-encoded NOR data partition.

use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{fence, Ordering};

use super::{Async, Blocking, Error, FLASH_SIZE, READ_SIZE, WRITE_SIZE};
use crate::internal::drop::OnDrop;
use crate::{pac, peripherals, Peri};

pub const FLASH_BASE: usize = 0x0800_0000;
pub const ERASE_SIZE: usize = 4096;
pub const BLOCK_ERASE_SIZE: usize = 32768;
pub const HALFWORD_WRITE_SIZE: usize = 2;
/// Raw erased flash reads as 0xE339, rather than 0xFFFF (RM §32.5.4/32.5.7).
pub const ERASED_HALFWORD: u16 = 0xe339;
const NOR_XOR: [u8; 2] = (ERASED_HALFWORD ^ u16::MAX).to_le_bytes();
const BUSY_MASK: u32 = 3;
const WRBSY_MASK: u32 = 1 << 1;
const CLEAR_FLAGS: u32 = (1 << 4) | (1 << 5);
const OPERATION_MASK: u32 = 1 | (1 << 1) | (1 << 6) | (1 << 16) | (1 << 18) | (1 << 21);

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Register polls allowed per busy phase, not a wall-clock timeout.
    /// The default permits the datasheet's 200 ms block erase at HCLK <= 200 MHz.
    pub poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { poll_limit: 50_000_000 }
    }
}

/// Raw flash. Erase/program functions and their callees must execute from the
/// zero-wait flash region. Enable memory-split-usr to constrain normal code to
/// that region with the PAC linker layout. Interrupts are
/// masked for each physical erase/program operation. Option bytes are untouched.
pub struct Flash<'d, MODE = Async> {
    _inner: Peri<'d, peripherals::FLASH>,
    _mode: PhantomData<MODE>,
    config: Config,
    pending_cleanup: bool,
    restore_enhanced: bool,
}

impl<'d> Flash<'d, Blocking> {
    pub fn new_blocking(inner: Peri<'d, peripherals::FLASH>) -> Self {
        Self::new_blocking_with_config(inner, Config::default())
    }

    pub fn new_blocking_with_config(inner: Peri<'d, peripherals::FLASH>, config: Config) -> Self {
        Self {
            _inner: inner,
            _mode: PhantomData,
            config,
            pending_cleanup: false,
            restore_enhanced: false,
        }
    }
}

impl<'d, MODE> Flash<'d, MODE> {
    pub const fn capacity(&self) -> usize {
        FLASH_SIZE
    }

    /// Read raw memory. All addresses passed to this driver are flash offsets.
    pub fn blocking_read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        validate_slice(FLASH_SIZE, offset, bytes.len(), READ_SIZE)?;
        if bytes.is_empty() {
            return Ok(());
        }
        self.recover()?;
        if busy() {
            return Err(Error::Busy);
        }
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = unsafe { read_volatile((FLASH_BASE + offset as usize + index) as *const u8) };
        }
        Ok(())
    }

    /// Fast 256-byte page programming. Erase the destination before writing it.
    /// An entire request is checked for range, alignment, protection and blank
    /// data before the first page is started. Source bytes need no alignment.
    pub fn blocking_write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.prepare_write(offset, bytes, WRITE_SIZE)?;
        for (page, chunk) in bytes.chunks_exact(WRITE_SIZE).enumerate() {
            let address = FLASH_BASE + offset as usize + page * WRITE_SIZE;
            self.operation(|limit| unsafe { program_page(address, chunk, limit) })?;
        }
        Ok(())
    }

    /// Standard programming, using 16-bit stores. Erase before programming a
    /// halfword again; no multiple-write NOR guarantee is provided.
    pub fn blocking_write_halfwords(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        self.prepare_write(offset, bytes, HALFWORD_WRITE_SIZE)?;
        for (index, chunk) in bytes.chunks_exact(2).enumerate() {
            let address = FLASH_BASE + offset as usize + index * 2;
            let value = u16::from_le_bytes([chunk[0], chunk[1]]);
            self.operation(|limit| unsafe { program_halfword(address, value, limit) })?;
        }
        Ok(())
    }

    /// Standard 4 KiB sector erase, with an exclusive end offset.
    pub fn blocking_erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.erase(from, to, ERASE_SIZE)
    }

    /// Fast 32 KiB block erase, with an exclusive end offset.
    pub fn blocking_erase_blocks(&mut self, from: u32, to: u32) -> Result<(), Error> {
        self.erase(from, to, BLOCK_ERASE_SIZE)
    }

    fn erase(&mut self, from: u32, to: u32, size: usize) -> Result<(), Error> {
        validate_erase(FLASH_SIZE, from, to, size)?;
        if from == to {
            return Ok(());
        }
        self.recover()?;
        validate_protection(
            from,
            (to - from) as usize,
            pac::FLASH.wpr().read().wrp(),
            pac::FLASH.obr().read().rdprt(),
        )?;
        for offset in (from as usize..to as usize).step_by(size) {
            self.operation(|limit| unsafe { erase_unit(FLASH_BASE + offset, size, limit) })?;
        }
        Ok(())
    }

    fn prepare_write(&mut self, offset: u32, bytes: &[u8], alignment: usize) -> Result<(), Error> {
        validate_slice(FLASH_SIZE, offset, bytes.len(), alignment)?;
        if bytes.is_empty() {
            return Ok(());
        }
        self.recover()?;
        if busy() {
            return Err(Error::Busy);
        }
        validate_protection(
            offset,
            bytes.len(),
            pac::FLASH.wpr().read().wrp(),
            pac::FLASH.obr().read().rdprt(),
        )?;
        for index in (0..bytes.len()).step_by(2) {
            let value = unsafe { read_volatile((FLASH_BASE + offset as usize + index) as *const u16) };
            if value != ERASED_HALFWORD {
                return Err(Error::NotErased);
            }
        }
        Ok(())
    }

    /// Complete cleanup after a timed-out operation once hardware is idle.
    /// On timeout while BSY/WR_BSY is set, registers are left untouched: the RM
    /// prohibits writing control registers while hardware is busy. Keep this
    /// driver alive and retry recover before issuing further operations.
    pub fn recover(&mut self) -> Result<(), Error> {
        if self.pending_cleanup {
            critical_section::with(|_| {
                let result = wait_idle(self.config.poll_limit);
                if busy() {
                    return result;
                }
                let finish = self.finish();
                result.and(finish)
            })?;
        }
        Ok(())
    }

    fn operation(&mut self, operation: impl FnOnce(u32) -> Result<(), Error>) -> Result<(), Error> {
        self.recover()?;
        #[cfg(target_arch = "riscv32")]
        {
            let zero_wait_size = if pac::FLASH.obr().read().ram_code_mod().to_bits() == 0 {
                512 * 1024
            } else {
                576 * 1024
            };
            for address in [
                program_page as *const () as usize,
                program_halfword as *const () as usize,
                erase_unit as *const () as usize,
            ] {
                validate_execution_address(address, zero_wait_size)?;
            }
        }
        critical_section::with(|_| {
            let regs = pac::FLASH;
            if busy() {
                return Err(Error::Busy);
            }
            if regs.ctlr().read().0 & (OPERATION_MASK | (1 << 2) | (1 << 4) | (1 << 5) | (1 << 9)) != 0 {
                return Err(Error::Seq);
            }
            let limit = self.config.poll_limit;
            self.restore_enhanced = regs.ctlr().read().ehmod() || regs.statr().read().enhance_mod_sta();
            self.pending_cleanup = true;
            let cleanup = OnDrop::new(|| {
                let _ = self.finish();
            });
            let result = (|| {
                unlock()?;
                regs.statr().write_value(pac::flash::regs::Statr(CLEAR_FLAGS));
                if regs.ctlr().read().ehmod() || regs.statr().read().enhance_mod_sta() {
                    regs.ctlr().modify(|reg| reg.set_ehmod(false));
                    regs.ctlr().modify(|reg| reg.set_rsenact(true));
                    wait_enhanced(false, limit)?;
                }
                operation(limit)
            })();
            cleanup.defuse();
            let finish = self.finish();
            result.and(finish)
        })
    }

    fn finish(&mut self) -> Result<(), Error> {
        if busy() {
            return Err(Error::Busy);
        }
        let regs = pac::FLASH;
        regs.ctlr().modify(|reg| reg.0 &= !OPERATION_MASK);
        regs.statr().write_value(pac::flash::regs::Statr(CLEAR_FLAGS));
        let restored = if self.restore_enhanced {
            regs.ctlr().modify(|reg| reg.set_ehmod(true));
            wait_enhanced(true, self.config.poll_limit)
        } else {
            Ok(())
        };
        regs.ctlr().modify(|reg| {
            reg.set_flock(true);
            reg.set_lock(true);
        });
        fence(Ordering::SeqCst);
        self.pending_cleanup = false;
        restored
    }

    /// Convert a 4 KiB-aligned data partition to conventional erased-0xFF NOR.
    /// This adapter XOR-encodes stored bytes. Its read/write representation is
    /// intentionally different from raw Flash and existing firmware images.
    pub fn into_nor_flash(self, from: u32, to: u32) -> Result<NorFlash<'d, MODE>, Error> {
        validate_erase(FLASH_SIZE, from, to, ERASE_SIZE)?;
        Ok(NorFlash {
            flash: self,
            offset: from,
            capacity: (to - from) as usize,
        })
    }
}

impl<MODE> Drop for Flash<'_, MODE> {
    fn drop(&mut self) {
        if self.pending_cleanup {
            critical_section::with(|_| {
                let _ = self.finish();
            });
        }
    }
}

/// A data-only NOR partition with erased bytes normalized to 0xFF.
pub struct NorFlash<'d, MODE = Blocking> {
    flash: Flash<'d, MODE>,
    offset: u32,
    capacity: usize,
}

impl<'d, MODE> NorFlash<'d, MODE> {
    pub fn into_inner(self) -> Flash<'d, MODE> {
        self.flash
    }

    pub fn recover(&mut self) -> Result<(), Error> {
        self.flash.recover()
    }
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
impl<MODE> embedded_storage::nor_flash::ErrorType for NorFlash<'_, MODE> {
    type Error = Error;
}
impl<MODE> embedded_storage::nor_flash::ReadNorFlash for NorFlash<'_, MODE> {
    const READ_SIZE: usize = READ_SIZE;
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Error> {
        validate_slice(self.capacity, offset, bytes.len(), READ_SIZE)?;
        self.flash.blocking_read(self.offset + offset, bytes)?;
        xor_bytes(offset, bytes);
        Ok(())
    }
    fn capacity(&self) -> usize {
        self.capacity
    }
}
impl<MODE> embedded_storage::nor_flash::NorFlash for NorFlash<'_, MODE> {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = ERASE_SIZE;
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Error> {
        validate_slice(self.capacity, offset, bytes.len(), WRITE_SIZE)?;
        let mut encoded = [0u8; WRITE_SIZE];
        for (index, chunk) in bytes.chunks_exact(WRITE_SIZE).enumerate() {
            encoded.copy_from_slice(chunk);
            xor_bytes(0, &mut encoded);
            self.flash
                .blocking_write(self.offset + offset + (index * WRITE_SIZE) as u32, &encoded)?;
        }
        Ok(())
    }
    fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        validate_erase(self.capacity, from, to, ERASE_SIZE)?;
        self.flash.blocking_erase(self.offset + from, self.offset + to)
    }
}

fn xor_bytes(offset: u32, bytes: &mut [u8]) {
    for (index, byte) in bytes.iter_mut().enumerate() {
        *byte ^= NOR_XOR[(offset as usize + index) & 1];
    }
}

fn validate_slice(capacity: usize, offset: u32, length: usize, alignment: usize) -> Result<(), Error> {
    let offset = offset as usize;
    if length > capacity || offset > capacity - length {
        return Err(Error::Size);
    }
    if offset % alignment != 0 || length % alignment != 0 {
        return Err(Error::Unaligned);
    }
    Ok(())
}

fn validate_erase(capacity: usize, from: u32, to: u32, alignment: usize) -> Result<(), Error> {
    if from > to {
        return Err(Error::Size);
    }
    validate_slice(capacity, from, (to - from) as usize, alignment)
}

fn validate_protection(offset: u32, length: usize, writable: u32, read_protected: bool) -> Result<(), Error> {
    if length == 0 {
        return Ok(());
    }
    let first = offset as usize / ERASE_SIZE;
    let last = (offset as usize + length - 1) / ERASE_SIZE;
    if read_protected && first == 0 {
        return Err(Error::Protected);
    }
    for sector in first..=last {
        if writable & (1 << sector.min(31)) == 0 {
            return Err(Error::Protected);
        }
    }
    Ok(())
}

fn validate_execution_address(address: usize, size: usize) -> Result<(), Error> {
    if address < size || (FLASH_BASE..FLASH_BASE + size).contains(&address) {
        Ok(())
    } else {
        Err(Error::InvalidExecutionRegion)
    }
}

#[inline(always)]
fn busy() -> bool {
    pac::FLASH.statr().read().0 & BUSY_MASK != 0
}

#[inline(always)]
fn wait_idle(limit: u32) -> Result<(), Error> {
    for _poll in 0..limit {
        let status = pac::FLASH.statr().read();
        if status.0 & BUSY_MASK == 0 {
            return if status.wrprterr() {
                Err(Error::Protected)
            } else {
                Ok(())
            };
        }
    }
    Err(Error::Timeout)
}

/// Fast page programming keeps BSY set for the whole 256-byte load; each
/// 32-bit store only waits for WRBSY to clear (RM §32.5.6 step 6). Waiting for
/// BSY here would spin until PGSTRT, which is never reached.
#[inline(always)]
fn wait_write_buffer(limit: u32) -> Result<(), Error> {
    for _poll in 0..limit {
        let status = pac::FLASH.statr().read();
        if status.0 & WRBSY_MASK == 0 {
            return if status.wrprterr() {
                Err(Error::Protected)
            } else {
                Ok(())
            };
        }
    }
    Err(Error::Timeout)
}

#[inline(always)]
fn wait_enhanced(enabled: bool, limit: u32) -> Result<(), Error> {
    for _poll in 0..limit {
        if pac::FLASH.statr().read().enhance_mod_sta() == enabled {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

#[inline(always)]
fn unlock() -> Result<(), Error> {
    let regs = pac::FLASH;
    if regs.ctlr().read().lock() {
        regs.keyr().write(|reg| reg.set_keyr(0x4567_0123));
        fence(Ordering::SeqCst);
        regs.keyr().write(|reg| reg.set_keyr(0xcdef_89ab));
    }
    if regs.ctlr().read().lock() {
        return Err(Error::Protected);
    }
    if regs.ctlr().read().flock() {
        regs.modekeyr().write(|reg| reg.set_modekeyr(0x4567_0123));
        fence(Ordering::SeqCst);
        regs.modekeyr().write(|reg| reg.set_modekeyr(0xcdef_89ab));
    }
    fence(Ordering::SeqCst);
    if regs.ctlr().read().lock() || regs.ctlr().read().flock() {
        Err(Error::Protected)
    } else {
        Ok(())
    }
}

#[inline(never)]
#[link_section = ".text.ch32_hal_flash"]
unsafe fn program_page(address: usize, bytes: &[u8], limit: u32) -> Result<(), Error> {
    let regs = pac::FLASH;
    wait_idle(limit)?;
    regs.ctlr().modify(|reg| reg.set_ftpg(true));
    for (index, chunk) in bytes.chunks_exact(4).enumerate() {
        let value = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        write_volatile((address + index * 4) as *mut u32, value);
        fence(Ordering::SeqCst);
        wait_write_buffer(limit)?;
    }
    regs.ctlr().modify(|reg| reg.set_pgstrt(true));
    wait_idle(limit)?;
    sync_instructions();
    for (index, value) in bytes.iter().enumerate() {
        if read_volatile((address + index) as *const u8) != *value {
            return Err(Error::Verify);
        }
    }
    Ok(())
}

#[inline(never)]
#[link_section = ".text.ch32_hal_flash"]
unsafe fn program_halfword(address: usize, value: u16, limit: u32) -> Result<(), Error> {
    let regs = pac::FLASH;
    wait_idle(limit)?;
    regs.ctlr().modify(|reg| reg.set_pg(true));
    write_volatile(address as *mut u16, value);
    fence(Ordering::SeqCst);
    wait_idle(limit)?;
    sync_instructions();
    if read_volatile(address as *const u16) == value {
        Ok(())
    } else {
        Err(Error::Verify)
    }
}

#[inline(never)]
#[link_section = ".text.ch32_hal_flash"]
unsafe fn erase_unit(address: usize, size: usize, limit: u32) -> Result<(), Error> {
    let regs = pac::FLASH;
    wait_idle(limit)?;
    regs.ctlr().modify(|reg| {
        reg.set_per(size == ERASE_SIZE);
        reg.set_ber32(size == BLOCK_ERASE_SIZE);
    });
    regs.addr().write(|reg| reg.set_far(address as u32));
    fence(Ordering::SeqCst);
    regs.ctlr().modify(|reg| reg.set_strt(true));
    wait_idle(limit)?;
    sync_instructions();
    for index in (0..size).step_by(2) {
        if read_volatile((address + index) as *const u16) != ERASED_HALFWORD {
            return Err(Error::Verify);
        }
    }
    Ok(())
}

#[inline(always)]
fn sync_instructions() {
    fence(Ordering::SeqCst);
    #[cfg(target_arch = "riscv32")]
    qingke::riscv::asm::fence_i();
}
