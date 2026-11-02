use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Mutex;

use crate::interrupt::typelevel::Handler;
use crate::pac;

pub static CLOCK: AtomicBool = AtomicBool::new(false);
pub static CLOCK_HZ: AtomicU32 = AtomicU32::new(200_000_000);
pub static IRQ_ENABLED: AtomicBool = AtomicBool::new(false);
pub static SMI_STUCK: AtomicBool = AtomicBool::new(false);
pub static DMA_RESET_STUCK: AtomicBool = AtomicBool::new(false);
pub static PHY_RESET_STUCK: AtomicBool = AtomicBool::new(false);
pub static PHY_MISSING: AtomicBool = AtomicBool::new(false);
pub static ID_POLLS: AtomicU32 = AtomicU32::new(0);
pub static WRITES: Mutex<Vec<(usize, u32)>> = Mutex::new(Vec::new());
static mut PHY: [[u16; 32]; 8] = [[0; 32]; 8];
static mut PAGE: usize = 0;

pub fn reset() {
    assert!(!CLOCK.load(Ordering::Relaxed));
    CLOCK_HZ.store(200_000_000, Ordering::Relaxed);
    IRQ_ENABLED.store(false, Ordering::Relaxed);
    SMI_STUCK.store(false, Ordering::Relaxed);
    DMA_RESET_STUCK.store(false, Ordering::Relaxed);
    PHY_RESET_STUCK.store(false, Ordering::Relaxed);
    PHY_MISSING.store(false, Ordering::Relaxed);
    ID_POLLS.store(0, Ordering::Relaxed);
    WRITES.lock().unwrap().clear();
    unsafe {
        pac::ETH_WORDS = [0; 1056];
        pac::RCC_WORDS = [0; 64];
        pac::RCC_WORDS[0] = (1 << 17) | (1 << 23);
        PHY = [[0; 32]; 8];
        PAGE = 0;
        PHY[0][1] = 0x7849;
        PHY[0][2] = 0x7371;
        PHY[0][3] = 0x9011;
        PHY[7][0x13] = 0x30;
    }
}

pub fn link(up: bool, fast: bool, full: bool) {
    unsafe {
        PHY[0][1] = if up { 0x786d } else { 0x7849 };
        PHY[0][0x1a] = if up {
            (if fast { 8 } else { 4 }) | (if full { 2 } else { 1 })
        } else {
            0
        };
        PHY[0][0x1e] = 1 << 11;
    }
}

pub fn irq(flags: u32) {
    assert!(IRQ_ENABLED.load(Ordering::Relaxed));
    unsafe {
        pac::ETH_WORDS[0x1014 / 4] |= flags;
        crate::eth::InterruptHandler::on_interrupt();
    }
}

pub fn after_read(address: usize) {
    let base = pac::ETH.as_ptr() as usize;
    if (base..base + 1056 * 4).contains(&address) {
        assert!(CLOCK.load(Ordering::Relaxed), "register read with MAC clock disabled");
    }
}

pub fn write(address: usize, value: u32, size: usize) -> bool {
    let base = pac::ETH.as_ptr() as usize;
    if !(base..base + 1056 * 4).contains(&address) {
        return false;
    }
    assert!(CLOCK.load(Ordering::Relaxed), "register write with MAC clock disabled");
    assert_eq!(size, 4);
    let offset = address - base;
    WRITES.lock().unwrap().push((offset, value));
    unsafe {
        match offset {
            0x1018 => {
                let previous = pac::ETH_WORDS[offset / 4];
                if previous & (1 << 13) != 0 && value & (1 << 13) == 0 {
                    pac::ETH_WORDS[0x1014 / 4] |= 1 << 1;
                }
                if previous & 2 != 0 && value & 2 == 0 {
                    pac::ETH_WORDS[0x1014 / 4] |= 1 << 8;
                }
                pac::ETH_WORDS[offset / 4] = value;
            }
            0x1014 => pac::ETH_WORDS[offset / 4] &= !value,
            0x80 => pac::ETH_WORDS[offset / 4] = 0,
            0x1000 if value & 1 != 0 && !DMA_RESET_STUCK.load(Ordering::Relaxed) => pac::ETH_WORDS[offset / 4] = 0,
            0x10 if value & 1 != 0 && !SMI_STUCK.load(Ordering::Relaxed) => {
                let register = ((value >> 6) & 31) as usize;
                if value & 2 != 0 {
                    let data = pac::ETH_WORDS[0x14 / 4] as u16;
                    if register == 31 {
                        PAGE = data as usize;
                        assert!(PAGE < 8);
                    } else {
                        PHY[PAGE][register] = if PAGE == 0
                            && register == 0
                            && data & 0x8000 != 0
                            && !PHY_RESET_STUCK.load(Ordering::Relaxed)
                        {
                            0
                        } else {
                            data
                        };
                    }
                } else {
                    pac::ETH_WORDS[0x14 / 4] = if matches!(register, 2 | 3) && ID_POLLS.load(Ordering::Relaxed) != 0 {
                        if register == 3 {
                            ID_POLLS.store(ID_POLLS.load(Ordering::Relaxed) - 1, Ordering::Relaxed);
                        }
                        0
                    } else if PHY_MISSING.load(Ordering::Relaxed) {
                        0xffff
                    } else {
                        u32::from(PHY[PAGE][register])
                    };
                    if PAGE == 0 && register == 0x1e {
                        PHY[PAGE][register] = 0;
                    }
                }
                pac::ETH_WORDS[offset / 4] = value & !1;
            }
            _ => pac::ETH_WORDS[offset / 4] = value,
        }
    }
    true
}
