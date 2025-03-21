use core::ptr::write_volatile;
use core::sync::atomic::{fence, AtomicBool, Ordering};

use super::{FlashSector, WRITE_SIZE};
use crate::flash::Error;
use crate::pac;

pub(crate) unsafe fn lock() {
    pac::FLASH.ctlr().modify(|w| {
        w.set_lock(true);
        w.set_flock(true);
    });
}

pub(crate) unsafe fn unlock() {
    if pac::FLASH.ctlr().read().lock() {
        pac::FLASH.keyr().write(|w| w.set_keyr(0x4567_0123));
        fence(Ordering::SeqCst);
        pac::FLASH.keyr().write(|w| w.set_keyr(0xCDEF_89AB));
        fence(Ordering::SeqCst);
    }
    if pac::FLASH.ctlr().read().flock() {
        pac::FLASH.modekeyr().write(|w| w.set_modekeyr(0x4567_0123));
        fence(Ordering::SeqCst);
        pac::FLASH.modekeyr().write(|w| w.set_modekeyr(0xCDEF_89AB));
        fence(Ordering::SeqCst);
    }
    // let _ctlr = pac::FLASH.ctlr().read();
}

pub(crate) unsafe fn enable_blocking_write() {
    assert_eq!(0, WRITE_SIZE % 4);

    pac::FLASH.ctlr().modify(|w| {
        w.set_page_pg(true);
    });
}

pub(crate) unsafe fn disable_blocking_write() {
    pac::FLASH.ctlr().modify(|w| w.set_pg(false));
}

pub(crate) unsafe fn blocking_write(start_address: u32, buf: &[u8; WRITE_SIZE]) -> Result<(), Error> {
    blocking_wait_ready()?;
    blocking_wait_ready_write()?;
    write_start(start_address, buf);
    blocking_wait_ready()
}

unsafe fn write_start(start_address: u32, buf: &[u8; WRITE_SIZE]) {
    let mut address = start_address;
    for val in buf.chunks(4) {
        write_volatile(address as *mut u32, u32::from_le_bytes(unwrap!(val.try_into())));
        address += val.len() as u32;
        fence(Ordering::SeqCst);
        let _ = blocking_wait_ready_write();
        fence(Ordering::SeqCst);
    }
    pac::FLASH.ctlr().modify(|w| w.set_pgstart(true));
}

pub(crate) unsafe fn blocking_erase_sector(sector: &FlashSector) -> Result<(), Error> {
    blocking_wait_ready()?;
    pac::FLASH.ctlr().modify(|w| w.set_page_er(true));
    pac::FLASH.addr().write(|w| w.set_far(sector.start));
    fence(Ordering::SeqCst);
    pac::FLASH.ctlr().modify(|w| {
        // w.set_page_er(true);
        w.set_strt(true);
    });

    let ret: Result<(), Error> = blocking_wait_ready();
    pac::FLASH.ctlr().modify(|w| w.set_page_er(false));
    clear_all_err();
    ret
}

pub(crate) unsafe fn clear_all_err() {
    pac::FLASH.statr().modify(|w| w.set_wrprterr(true));
}

unsafe fn blocking_wait_ready() -> Result<(), Error> {
    loop {
        let sr = pac::FLASH.statr().read();

        if !sr.bsy() {
            return get_result();
        }
    }
}

unsafe fn blocking_wait_ready_write() -> Result<(), Error> {
    loop {
        let sr = pac::FLASH.statr().read();

        if !sr.wr_bsy() {
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
