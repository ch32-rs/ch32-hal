//! Minimal model of the CH32V4x7 flash status flags.
//!
//! It reproduces the state sequence measured on a CH32V467 during fast page
//! programming: `BSY` is set from `FTPG` until `PGSTRT` completes, while
//! `WRBSY` is only set while a 32-bit word is entering the page buffer.
//! The old driver waited for `BSY|WRBSY` after every store and timed out
//! before it could ever reach `PGSTRT`.

use std::sync::Mutex;

const KEYR: usize = 0x04;
const STATR: usize = 0x0c;
const CTLR: usize = 0x10;
const MODEKEYR: usize = 0x24;

const EOP: u32 = 1 << 5;
const LOCK: u32 = 1 << 7;
const FLOCK: u32 = 1 << 15;
const FTPG: u32 = 1 << 16;
const PGSTRT: u32 = 1 << 21;
const KEY2: u32 = 0xcdef_89ab;

pub struct Model {
    pub bsy: bool,
    pub wrbsy: bool,
    pub eop: bool,
    pub wrprterr: bool,
    /// Number of `STATR` reads that observed `BSY=1`.
    pub busy_reads: usize,
}

pub static MODEL: Mutex<Model> = Mutex::new(Model {
    bsy: false,
    wrbsy: false,
    eop: false,
    wrprterr: false,
    busy_reads: 0,
});

fn base() -> usize {
    core::ptr::addr_of_mut!(crate::pac::FLASH_WORDS) as usize
}

unsafe fn word(offset: usize) -> u32 {
    ((base() + offset) as *const u32).read_volatile()
}

unsafe fn set_word(offset: usize, value: u32) {
    ((base() + offset) as *mut u32).write_volatile(value);
}

pub fn reset() {
    unsafe { crate::pac::FLASH_WORDS = [0; 16] };
    *MODEL.lock().unwrap() = Model {
        bsy: false,
        wrbsy: false,
        eop: false,
        wrprterr: false,
        busy_reads: 0,
    };
}

pub fn set_busy(bsy: bool) {
    MODEL.lock().unwrap().bsy = bsy;
}

/// Hold the "page buffer is accepting a word" flag so a wait on it times out.
pub fn stall_wrbsy(stall: bool) {
    MODEL.lock().unwrap().wrbsy = stall;
}

pub fn busy_reads() -> usize {
    MODEL.lock().unwrap().busy_reads
}

pub fn read(address: usize) {
    if address != base() + STATR {
        return;
    }
    let mut model = MODEL.lock().unwrap();
    if model.bsy {
        model.busy_reads += 1;
    }
    let value = (model.bsy as u32)
        | ((model.wrbsy as u32) << 1)
        | ((model.wrprterr as u32) << 4)
        | ((model.eop as u32) << 5);
    unsafe { set_word(STATR, value) };
}

/// Returns `true` if the model consumed the write.
pub fn write(address: usize, value: u32) -> bool {
    let offset = address.wrapping_sub(base());
    let mut model = MODEL.lock().unwrap();
    match offset {
        KEYR if value == KEY2 => unsafe { set_word(CTLR, word(CTLR) & !LOCK) },
        MODEKEYR if value == KEY2 => unsafe { set_word(CTLR, word(CTLR) & !FLOCK) },
        STATR => {
            // Write-1-to-clear.
            if value & EOP != 0 {
                model.eop = false;
            }
            if value & (1 << 4) != 0 {
                model.wrprterr = false;
            }
        }
        CTLR => {
            if value & PGSTRT != 0 {
                model.bsy = false;
                model.eop = true;
            } else if value & FTPG != 0 {
                // Fast page programming starts; BSY stays high for the whole
                // 256-byte load, until PGSTRT is set.
                model.bsy = true;
            }
        }
        _ => {}
    }
    false
}
