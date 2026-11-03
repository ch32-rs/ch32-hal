//! Minimal register model for the CH32V4x7 SPI status flags.
//!
//! The only hardware behaviour encoded here is the one that caused the
//! `hal_spi` hang measured on a CH32V467: while `SPE=0`, `STATR.TXE` reads 0.
//! A disabled peripheral therefore never satisfies a `TXE && !BSY` poll.
//! Everything else is permissive: `TXE` follows `SPE`, and `BSY` is controlled
//! by the test.

use std::sync::Mutex;

const CTLR1: usize = 0x00;
const STATR: usize = 0x08;
const DATAR: usize = 0x0c;

const SPE: u32 = 1 << 6;
const RXNE: u32 = 1 << 0;
const TXE: u32 = 1 << 1;
const BSY: u32 = 1 << 7;

/// After this many disabled-state polls, the model reports `TXE=1`. This is a
/// safety fence, not hardware behaviour: it turns the regression's infinite
/// hang into a failing assertion so the test binary cannot wedge CI.
const DISABLED_POLL_LIMIT: usize = 1_000;

pub struct Model {
    /// `STATR.RXNE`: set at reset, cleared when `DATAR` is read.
    pub rxne: bool,
    /// `STATR.BSY`, controlled by the test.
    pub bsy: bool,
    pub statr_reads: usize,
    /// `STATR` reads observed while `SPE=0`.
    pub disabled_polls: usize,
}

pub static MODEL: Mutex<Model> = Mutex::new(Model {
    rxne: true,
    bsy: false,
    statr_reads: 0,
    disabled_polls: 0,
});

fn bases() -> [usize; 3] {
    [
        core::ptr::addr_of_mut!(crate::pac::SPI1_WORDS) as usize,
        core::ptr::addr_of_mut!(crate::pac::SPI2_WORDS) as usize,
        core::ptr::addr_of_mut!(crate::pac::SPI3_WORDS) as usize,
    ]
}

fn slot(address: usize) -> Option<(usize, usize)> {
    bases()
        .into_iter()
        .find_map(|base| (base..base + 0x28).contains(&address).then_some((base, address - base)))
}

unsafe fn word(base: usize, offset: usize) -> u32 {
    ((base + offset) as *const u32).read_volatile()
}

unsafe fn set_word(base: usize, offset: usize, value: u32) {
    ((base + offset) as *mut u32).write_volatile(value);
}

/// Reset the model together with the RAM-backed register block.
pub fn reset() {
    unsafe {
        crate::pac::SPI1_WORDS = [0; 16];
        crate::pac::SPI2_WORDS = [0; 16];
        crate::pac::SPI3_WORDS = [0; 16];
    }
    *MODEL.lock().unwrap() = Model {
        rxne: true,
        bsy: false,
        statr_reads: 0,
        disabled_polls: 0,
    };
}

pub fn set_busy(bsy: bool) {
    MODEL.lock().unwrap().bsy = bsy;
}

pub fn statr_reads() -> usize {
    MODEL.lock().unwrap().statr_reads
}

/// Called before every 32-bit register read.
pub fn read(address: usize) {
    let Some((base, offset)) = slot(address) else {
        return;
    };
    let mut model = MODEL.lock().unwrap();
    match offset {
        STATR => {
            model.statr_reads += 1;
            // CH32V467 measurement: TXE is only meaningful while SPE=1.
            let spe = unsafe { word(base, CTLR1) } & SPE != 0;
            let mut txe = spe;
            if !spe {
                model.disabled_polls += 1;
                if model.disabled_polls > DISABLED_POLL_LIMIT {
                    txe = true;
                }
            }
            let value = (model.rxne as u32 * RXNE) | (txe as u32 * TXE) | (model.bsy as u32 * BSY);
            unsafe { set_word(base, STATR, value) };
        }
        DATAR => model.rxne = false,
        _ => {}
    }
}

/// Called before every 32-bit register write. Returns `true` if the model
/// consumed the write.
pub fn write(_address: usize, _value: u32) -> bool {
    false
}
