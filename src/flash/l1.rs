//! CH32L103 (`flash_l1`) fast page program / erase.
//!
//! Geometry from `ch32-data` `memory/CH32L1.yaml`: 256-byte pages, 4-byte
//! load size. Programming follows CH32L103RM §24.4.5 (FTPG/BUFLOAD/STRT);
//! erase uses the matching 256-byte fast erase (§24.4.6, FTER).
//!
//! Register names in the metapac are `PTPG`/`PTER` (same bits as the RM's
//! FTPG/FTER). The status register has no `WR_BSY` — only `BSY` — despite the
//! RM prose mentioning WR_BSY (copy-paste from V3); we poll `BSY`.

use core::ptr::write_volatile;
use core::sync::atomic::{fence, Ordering};

use super::{FlashSector, WRITE_SIZE};
use crate::flash::Error;
use crate::pac;

const KEY1: u32 = 0x4567_0123;
const KEY2: u32 = 0xCDEF_89AB;

pub(crate) unsafe fn lock() {
    pac::FLASH.ctlr().modify(|w| {
        w.set_lock(true);
        w.set_flock(true);
    });
}

pub(crate) unsafe fn unlock() {
    if pac::FLASH.ctlr().read().lock() {
        pac::FLASH.keyr().write(|w| w.set_keyr(KEY1));
        fence(Ordering::SeqCst);
        pac::FLASH.keyr().write(|w| w.set_keyr(KEY2));
        fence(Ordering::SeqCst);
    }
    if pac::FLASH.ctlr().read().flock() {
        pac::FLASH.modekeyr().write(|w| w.set_modekeyr(KEY1));
        fence(Ordering::SeqCst);
        pac::FLASH.modekeyr().write(|w| w.set_modekeyr(KEY2));
        fence(Ordering::SeqCst);
    }
}

pub(crate) unsafe fn enable_blocking_write() {
    assert_eq!(WRITE_SIZE, 256);
    assert_eq!(0, WRITE_SIZE % 4);

    pac::FLASH.ctlr().modify(|w| {
        w.set_ptpg(true);
    });
}

pub(crate) unsafe fn disable_blocking_write() {
    pac::FLASH.ctlr().modify(|w| w.set_ptpg(false));
}

pub(crate) unsafe fn blocking_write(start_address: u32, buf: &[u8; WRITE_SIZE]) -> Result<(), Error> {
    blocking_wait_ready()?;

    // Clear the internal 256-byte program buffer.
    pac::FLASH.ctlr().modify(|w| w.set_bufrst(true));
    fence(Ordering::SeqCst);
    blocking_wait_ready()?;

    // Load 64 consecutive words into the buffer.
    let mut address = start_address;
    for chunk in buf.chunks_exact(4) {
        let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        write_volatile(address as *mut u32, word);
        fence(Ordering::SeqCst);
        pac::FLASH.ctlr().modify(|w| w.set_bufload(true));
        fence(Ordering::SeqCst);
        blocking_wait_ready()?;
        address += 4;
    }

    pac::FLASH.addr().write(|w| w.set_far(start_address));
    fence(Ordering::SeqCst);
    pac::FLASH.ctlr().modify(|w| w.set_strt(true));
    fence(Ordering::SeqCst);

    let ret = blocking_wait_ready();
    clear_all_err();
    ret
}

pub(crate) unsafe fn blocking_erase_sector(sector: &FlashSector) -> Result<(), Error> {
    blocking_wait_ready()?;

    pac::FLASH.ctlr().modify(|w| w.set_pter(true));
    pac::FLASH.addr().write(|w| w.set_far(sector.start));
    fence(Ordering::SeqCst);
    pac::FLASH.ctlr().modify(|w| w.set_strt(true));
    fence(Ordering::SeqCst);

    let ret = blocking_wait_ready();
    pac::FLASH.ctlr().modify(|w| w.set_pter(false));
    clear_all_err();
    ret
}

pub(crate) unsafe fn clear_all_err() {
    pac::FLASH.statr().modify(|w| {
        w.set_wrprterr(true);
        w.set_eop(true);
    });
}

unsafe fn blocking_wait_ready() -> Result<(), Error> {
    loop {
        let sr = pac::FLASH.statr().read();
        if !sr.bsy() {
            return get_result();
        }
    }
}

fn get_result() -> Result<(), Error> {
    if pac::FLASH.statr().read().wrprterr() {
        Err(Error::Protected)
    } else {
        Ok(())
    }
}
