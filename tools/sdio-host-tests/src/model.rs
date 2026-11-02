use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, AtomicUsize, Ordering};
use std::sync::Mutex;

use crate::interrupt::typelevel::Handler;
use crate::pac;

pub static CLOCK: AtomicBool = AtomicBool::new(false);
pub static CLOCK_HZ: AtomicU32 = AtomicU32::new(20_000_000);
pub static REMAP: AtomicU8 = AtomicU8::new(0);
pub static AUTO_COMPLETE: AtomicBool = AtomicBool::new(true);
pub static NO_CARD: AtomicBool = AtomicBool::new(false);
pub static LEGACY: AtomicBool = AtomicBool::new(false);
pub static HIGH_SPEED: AtomicBool = AtomicBool::new(true);
pub static SWITCH_BUSY: AtomicBool = AtomicBool::new(false);
pub static SWITCH_SELECTED: AtomicU8 = AtomicU8::new(1);
pub static CRC_R3: AtomicBool = AtomicBool::new(false);
pub static FAIL_COMMAND: AtomicU8 = AtomicU8::new(255);
pub static BUSY: AtomicU32 = AtomicU32::new(0);
pub static LATE_ERROR: AtomicU32 = AtomicU32::new(0);
pub static OCR_BUSY: AtomicU32 = AtomicU32::new(0);
pub static RESPONSE_MISMATCH: AtomicBool = AtomicBool::new(false);
pub static CARD_ERROR: AtomicU32 = AtomicU32::new(0);
static DMA_ADDRESS: AtomicUsize = AtomicUsize::new(0);
static APP: AtomicBool = AtomicBool::new(false);
static DATA_COMMAND: AtomicU8 = AtomicU8::new(0);
static DATA_ARGUMENT: AtomicU32 = AtomicU32::new(0);
pub static COMMANDS: Mutex<Vec<(u8, u32)>> = Mutex::new(Vec::new());
pub static WRITES: Mutex<Vec<Vec<u8>>> = Mutex::new(Vec::new());

pub fn reset() {
    assert!(!CLOCK.load(Ordering::Relaxed));
    unsafe {
        pac::SDIO_WORDS = [0; 36];
        pac::DMA2_WORDS = [0; 64];
    }
    CLOCK_HZ.store(20_000_000, Ordering::Relaxed);
    REMAP.store(0, Ordering::Relaxed);
    AUTO_COMPLETE.store(true, Ordering::Relaxed);
    NO_CARD.store(false, Ordering::Relaxed);
    LEGACY.store(false, Ordering::Relaxed);
    HIGH_SPEED.store(true, Ordering::Relaxed);
    SWITCH_BUSY.store(false, Ordering::Relaxed);
    SWITCH_SELECTED.store(1, Ordering::Relaxed);
    CRC_R3.store(false, Ordering::Relaxed);
    FAIL_COMMAND.store(255, Ordering::Relaxed);
    BUSY.store(0, Ordering::Relaxed);
    LATE_ERROR.store(0, Ordering::Relaxed);
    OCR_BUSY.store(0, Ordering::Relaxed);
    RESPONSE_MISMATCH.store(false, Ordering::Relaxed);
    CARD_ERROR.store(0, Ordering::Relaxed);
    DATA_COMMAND.store(0, Ordering::Relaxed);
    APP.store(false, Ordering::Relaxed);
    COMMANDS.lock().unwrap().clear();
    WRITES.lock().unwrap().clear();
}

pub fn after_read(address: usize) {
    if address == pac::SDIO.sta().as_ptr() as usize {
        let error = LATE_ERROR.swap(0, Ordering::Relaxed);
        unsafe {
            pac::SDIO_WORDS[13] |= error;
        }
    }
}

pub fn dma_done(error: bool) {
    unsafe {
        if !error {
            pac::DMA2_WORDS[(8 + 3 * 20 + 4) / 4] = 0;
        }
        pac::DMA2_WORDS[0] |= (if error { 8 } else { 2 }) << 12;
        crate::dma::AnyChannel { id: 16 }.on_irq();
    }
}

pub fn data_done(flags: u32) {
    unsafe {
        pac::SDIO_WORDS[12] = 0;
        pac::SDIO_WORDS[13] = (pac::SDIO_WORDS[13] & !0x3000) | 0x500 | flags;
        if pac::SDIO_WORDS[15] != 0 {
            crate::sdio::InterruptHandler::<crate::peripherals::SDIO>::on_interrupt();
        }
    }
}

pub fn complete() {
    let index = DATA_COMMAND.swap(0, Ordering::Relaxed);
    assert_ne!(index, 0);
    let channel = pac::DMA2.ch(3);
    assert!(channel.cfgr().read().en());
    assert_eq!(channel.cfgr().read().msize().to_bits(), 2);
    assert_eq!(channel.cfgr().read().psize().to_bits(), 2);
    assert!(channel.cfgr().read().minc());
    let count = unsafe { pac::SDIO_WORDS[10] as usize };
    let address = DMA_ADDRESS.load(Ordering::Relaxed) as *mut u8;
    if index == 24 || index == 25 {
        assert!(!pac::SDIO.dctrl().read().dtdir());
        WRITES
            .lock()
            .unwrap()
            .push(unsafe { core::slice::from_raw_parts(address, count).to_vec() });
    } else {
        assert!(pac::SDIO.dctrl().read().dtdir());
        let mut bytes: Vec<u8> = (0..count)
            .map(|offset| (offset + DATA_ARGUMENT.load(Ordering::Relaxed) as usize) as u8)
            .collect();
        if index == 51 {
            bytes = vec![2, 5, 0, 0, 0, 0, 0, 0];
        }
        if index == 6 {
            bytes.fill(0);
            bytes[13] = if HIGH_SPEED.load(Ordering::Relaxed) { 2 } else { 0 };
            bytes[16] = SWITCH_SELECTED.load(Ordering::Relaxed);
            bytes[29] = if SWITCH_BUSY.load(Ordering::Relaxed) { 2 } else { 0 };
        }
        if index == 13 {
            bytes.fill(0);
            bytes[0] = if pac::SDIO.clkcr().read().widbus() == 1 {
                0x80
            } else {
                0
            };
            bytes[8] = 10;
        }
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), address, count);
        }
    }
    dma_done(false);
    data_done(0);
}

fn respond(index: u8, argument: u32) {
    let app = APP.swap(false, Ordering::Relaxed);
    let mut response = [0x900, 0, 0, 0];
    let mut status = 0x40;
    let mut data = matches!(index, 6 | 17 | 18 | 24 | 25 | 51) || (index == 13 && app);
    if index == 6 && app {
        data = false;
    }
    match index {
        0 => {
            status = 0x80;
            DATA_COMMAND.store(0, Ordering::Relaxed);
        }
        8 => {
            if LEGACY.load(Ordering::Relaxed) {
                status = 4;
            }
            response[0] = 0x1aa;
        }
        55 => {
            APP.store(true, Ordering::Relaxed);
            response[0] |= 0x20;
        }
        41 => {
            response[0] = 0x8030_0000 | if LEGACY.load(Ordering::Relaxed) { 0 } else { 1 << 30 };
            if OCR_BUSY.load(Ordering::Relaxed) != 0 {
                OCR_BUSY.fetch_sub(1, Ordering::Relaxed);
                response[0] &= !(1 << 31);
            }
            if CRC_R3.load(Ordering::Relaxed) {
                status = 1;
            }
        }
        3 => response[0] = 1 << 16,
        2 | 9 => {
            let value: u128 = if index == 2 {
                0x123456789abcdef0
            } else if LEGACY.load(Ordering::Relaxed) {
                (0x32 << 96) | (9 << 80) | (0xfff << 62) | (7 << 47)
            } else {
                (1 << 126) | (0x32 << 96) | (9 << 80) | (0x1234 << 48)
            };
            response = core::array::from_fn(|word| (value >> ((3 - word) * 32)) as u32);
        }
        13 if !app && BUSY.load(Ordering::Relaxed) != 0 => {
            BUSY.fetch_sub(1, Ordering::Relaxed);
            response[0] = 7 << 9;
        }
        _ => {}
    }
    if NO_CARD.load(Ordering::Relaxed) && index != 0 || FAIL_COMMAND.load(Ordering::Relaxed) == index {
        status = 4;
    }
    if index == 13 && !app {
        response[0] |= CARD_ERROR.load(Ordering::Relaxed);
    }
    unsafe {
        pac::SDIO_WORDS[3] &= !(1 << 10);
        pac::SDIO_WORDS[4] = u32::from(if RESPONSE_MISMATCH.load(Ordering::Relaxed) {
            index ^ 1
        } else {
            index
        });
        pac::SDIO_WORDS[5..9].copy_from_slice(&response);
        pac::SDIO_WORDS[13] = (pac::SDIO_WORDS[13] & !0x8c5) | status;
    }
    if data && status == 0x40 {
        let read = index != 24 && index != 25;
        assert_eq!(
            pac::SDIO.dctrl().read().dten(),
            read,
            "read DMA must precede CMD, write DMA follows CMD"
        );
        DATA_COMMAND.store(index, Ordering::Relaxed);
        DATA_ARGUMENT.store(argument, Ordering::Relaxed);
        if read && AUTO_COMPLETE.load(Ordering::Relaxed) {
            complete();
        }
    }
}

pub fn write(address: usize, value: u32, size: usize) -> bool {
    let dma = core::ptr::addr_of_mut!(pac::DMA2_WORDS) as usize;
    if address == dma + 8 + 3 * 20 + 12 {
        let stack = core::ptr::addr_of!(value) as usize;
        DMA_ADDRESS.store((stack & !(u32::MAX as usize)) | value as usize, Ordering::Relaxed);
    }
    if address == dma + 4 {
        unsafe {
            pac::DMA2_WORDS[0] &= !value;
        }
        return true;
    }
    let base = pac::SDIO.as_ptr() as usize;
    if !(base..base + 144).contains(&address) {
        return false;
    }
    assert!(CLOCK.load(Ordering::Relaxed));
    assert_eq!(size, 4);
    let offset = address - base;
    unsafe {
        if offset == 0x38 {
            pac::SDIO_WORDS[13] &= !value;
            return true;
        }
        (address as *mut u32).write(value);
        match offset {
            0 if value == 0 => pac::SDIO_WORDS[13] &= !0x3800,
            0x0c if value & (1 << 10) != 0 => {
                let index = (value & 63) as u8;
                let argument = pac::SDIO_WORDS[2];
                COMMANDS.lock().unwrap().push((index, argument));
                respond(index, argument);
            }
            0x2c if value & 1 == 0 => {
                pac::SDIO_WORDS[13] &= !0x3000;
            }
            0x2c => {
                pac::SDIO_WORDS[12] = pac::SDIO_WORDS[10];
                pac::SDIO_WORDS[13] |= if value & 2 != 0 { 1 << 13 } else { 1 << 12 };
                if DATA_COMMAND.load(Ordering::Relaxed) != 0 && AUTO_COMPLETE.load(Ordering::Relaxed) {
                    complete();
                }
            }
            _ => {}
        }
    }
    true
}
