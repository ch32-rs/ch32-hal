use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::collections::VecDeque;
use std::sync::Mutex;

use crate::pac;

pub static CLOCK: AtomicBool = AtomicBool::new(false);
pub static CLOCK_HZ: AtomicU32 = AtomicU32::new(20_000_000);
pub static ERROR: AtomicU32 = AtomicU32::new(u32::MAX);
pub static STALL: AtomicBool = AtomicBool::new(false);
pub static SHORT: AtomicBool = AtomicBool::new(false);
pub static DEFER_STOP: AtomicBool = AtomicBool::new(false);
pub static FRAME_DONE: AtomicBool = AtomicBool::new(false);
pub static DMA_STALL: AtomicBool = AtomicBool::new(false);
pub static DMA_ERROR: AtomicBool = AtomicBool::new(false);
pub static CPU_DATA_READS: AtomicU32 = AtomicU32::new(0);
pub static CPU_DATA_WRITES: AtomicU32 = AtomicU32::new(0);
pub static DMA_BYTES: AtomicU32 = AtomicU32::new(0);
static DMA_BUFFERS: Mutex<Vec<(u32, usize, usize)>> = Mutex::new(Vec::new());
static DMA_PROGRESS: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
pub static COMMANDS: Mutex<Vec<u32>> = Mutex::new(Vec::new());
pub static TX: Mutex<Vec<u8>> = Mutex::new(Vec::new());
pub static DAA_TARGETS: Mutex<Vec<[u8; 8]>> = Mutex::new(Vec::new());
pub static DAA_COUNT_MISMATCH: AtomicBool = AtomicBool::new(false);
pub static DAA_ADDRESS_NACK: AtomicBool = AtomicBool::new(false);
static DAA: AtomicBool = AtomicBool::new(false);
static DAA_INDEX: AtomicU32 = AtomicU32::new(0);
static MESSAGES: Mutex<VecDeque<Message>> = Mutex::new(VecDeque::new());
static RX: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
static TX_FIFO: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
static SENT: AtomicU32 = AtomicU32::new(0);
static RECEIVED: AtomicU32 = AtomicU32::new(0);
static STARTED: AtomicBool = AtomicBool::new(false);
static LAST_QUEUED: AtomicBool = AtomicBool::new(false);

struct Message {
    length: usize,
    read: bool,
    last: bool,
}

pub fn reset() {
    DMA_STALL.store(false, Ordering::Relaxed);
    DMA_ERROR.store(false, Ordering::Relaxed);
    CPU_DATA_READS.store(0, Ordering::Relaxed);
    CPU_DATA_WRITES.store(0, Ordering::Relaxed);
    DMA_BYTES.store(0, Ordering::Relaxed);
    DMA_BUFFERS.lock().unwrap().clear();
    unsafe {
        pac::DMA1_WORDS = [0; 64];
        pac::DMA2_WORDS = [0; 64];
    }
    assert!(!CLOCK.load(Ordering::Relaxed));
    CLOCK_HZ.store(20_000_000, Ordering::Relaxed);
    ERROR.store(u32::MAX, Ordering::Relaxed);
    STALL.store(false, Ordering::Relaxed);
    SHORT.store(false, Ordering::Relaxed);
    DEFER_STOP.store(false, Ordering::Relaxed);
    FRAME_DONE.store(false, Ordering::Relaxed);
    COMMANDS.lock().unwrap().clear();
    TX.lock().unwrap().clear();
    DAA_TARGETS.lock().unwrap().clear();
    DAA_COUNT_MISMATCH.store(false, Ordering::Relaxed);
    DAA_ADDRESS_NACK.store(false, Ordering::Relaxed);
}

pub fn reset_registers() {
    CLOCK.store(true, Ordering::Relaxed);
    unsafe {
        pac::WORDS = [0; 64];
    }
    MESSAGES.lock().unwrap().clear();
    RX.lock().unwrap().clear();
    TX_FIFO.lock().unwrap().clear();
    SENT.store(0, Ordering::Relaxed);
    RECEIVED.store(0, Ordering::Relaxed);
    STARTED.store(false, Ordering::Relaxed);
    LAST_QUEUED.store(false, Ordering::Relaxed);
    DAA.store(false, Ordering::Relaxed);
    DAA_INDEX.store(0, Ordering::Relaxed);
}

fn advance_daa() {
    let mut rx = RX.lock().unwrap();
    let targets = DAA_TARGETS.lock().unwrap();
    let index = DAA_INDEX.load(Ordering::Relaxed) as usize;
    unsafe {
        if !STALL.load(Ordering::Relaxed) {
            if let Some(identity) = targets.get(index) {
                let generated = RECEIVED.load(Ordering::Relaxed) as usize;
                if generated < 8 {
                    rx.push_back(identity[generated]);
                    RECEIVED.fetch_add(1, Ordering::Relaxed);
                }
            } else {
                pac::WORDS[0x30 / 4] = index as u32 + u32::from(DAA_COUNT_MISMATCH.load(Ordering::Relaxed));
                pac::WORDS[0x50 / 4] |= 1 << 9;
            }
        }
        pac::WORDS[0x50 / 4] &= !((1 << 2) | (1 << 5));
        if let Some(byte) = rx.front() {
            pac::WORDS[0x10 / 4] = u32::from(*byte);
            pac::WORDS[0x50 / 4] |= 1 << 5;
        }
    }
}

fn advance() {
    let error = ERROR.load(Ordering::Relaxed);
    unsafe {
        if STARTED.load(Ordering::Relaxed) && error != u32::MAX {
            pac::WORDS[0x34 / 4] = error;
            pac::WORDS[0x50 / 4] |= 1 << 11;
            return;
        }
        if DAA.load(Ordering::Relaxed) {
            advance_daa();
            return;
        }
        let mut messages = MESSAGES.lock().unwrap();
        if let Some(message) = messages.front().filter(|_| !STALL.load(Ordering::Relaxed)) {
            if message.read {
                let mut rx = RX.lock().unwrap();
                let generated = RECEIVED.load(Ordering::Relaxed) as usize;
                if rx.len() < 8 && generated < message.length {
                    rx.push_back((generated % 251) as u8);
                    RECEIVED.fetch_add(1, Ordering::Relaxed);
                }
                if SHORT.load(Ordering::Relaxed) {
                    pac::WORDS[0x50 / 4] |= 1 << 10;
                }
            } else if SENT.load(Ordering::Relaxed) < message.length as u32 {
                if TX_FIFO.lock().unwrap().pop_front().is_some() {
                    SENT.fetch_add(1, Ordering::Relaxed);
                }
            }
            let complete = if message.read {
                RECEIVED.load(Ordering::Relaxed) == message.length as u32
            } else {
                SENT.load(Ordering::Relaxed) == message.length as u32
            };
            if complete {
                let message = messages.pop_front().unwrap();
                SENT.store(0, Ordering::Relaxed);
                RECEIVED.store(0, Ordering::Relaxed);
                pac::WORDS[0x30 / 4] = message.length as u32;
                if message.last {
                    FRAME_DONE.store(true, Ordering::Relaxed);
                    if !DEFER_STOP.load(Ordering::Relaxed) {
                        pac::WORDS[0x50 / 4] |= 1 << 9;
                    }
                }
            }
        }
        pac::WORDS[0x50 / 4] &= !((1 << 2) | (1 << 4) | (1 << 5));
        if STARTED.load(Ordering::Relaxed) {
            if messages.len() < 2 && !LAST_QUEUED.load(Ordering::Relaxed) {
                pac::WORDS[0x50 / 4] |= 1 << 2;
            }
            if TX_FIFO.lock().unwrap().len() < 8 {
                pac::WORDS[0x50 / 4] |= 1 << 4;
            }
        }
        if let Some(byte) = RX.lock().unwrap().front() {
            pac::WORDS[0x10 / 4] = u32::from(*byte);
            pac::WORDS[0x50 / 4] |= 1 << 5;
        }
    }
}

pub fn release_stop() {
    assert!(FRAME_DONE.load(Ordering::Relaxed));
    unsafe {
        pac::WORDS[0x50 / 4] |= 1 << 9;
    }
}

pub fn after_read(address: usize) {
    if dma_offset(address).is_some() {
        advance();
        dma_step();
        return;
    }
    if !(pac::I3C.as_ptr() as usize..pac::I3C.as_ptr() as usize + 256).contains(&address) {
        return;
    }
    let offset = address - pac::I3C.as_ptr() as usize;
    assert!(CLOCK.load(Ordering::Relaxed));
    if offset == 0x10 {
        CPU_DATA_READS.fetch_add(1, Ordering::Relaxed);
        RX.lock().unwrap().pop_front();
        unsafe {
            pac::WORDS[0x50 / 4] &= !(1 << 5);
        }
    }
    advance();
    dma_step();
}

pub fn write(address: usize, value: u32, size: usize) -> bool {
    if let Some((base, offset)) = dma_offset(address) {
        assert_eq!(size, 4);
        unsafe {
            let words = base as *mut u32;
            if offset == 4 || offset == 0xf4 || offset == 0xd4 {
                let flags = words.add(offset / 4 - 1);
                let mut mask = value;
                for channel in 0..7 {
                    if value & (1 << (channel * 4)) != 0 {
                        mask |= 15 << (channel * 4);
                    }
                }
                flags.write_volatile(flags.read_volatile() & !mask);
            } else {
                words.add(offset / 4).write_volatile(value);
                for (index, channel) in [3, 4].into_iter().enumerate() {
                    if offset == 12 + channel * 20 {
                        DMA_PROGRESS[index].store(0, Ordering::Relaxed);
                    }
                }
            }
        }
        return true;
    }
    if !(pac::I3C.as_ptr() as usize..pac::I3C.as_ptr() as usize + 256).contains(&address) {
        return false;
    }
    assert_eq!(size, 4);
    assert!(CLOCK.load(Ordering::Relaxed));
    let offset = address - pac::I3C.as_ptr() as usize;
    unsafe {
        match offset {
            0x58 => pac::WORDS[0x50 / 4] &= !value,
            0x04 => {
                pac::WORDS[1] = value & !(1 << 30);
                if value & (1 << 30) != 0 {
                    STARTED.store(true, Ordering::Relaxed);
                    LAST_QUEUED.store(false, Ordering::Relaxed);
                    FRAME_DONE.store(false, Ordering::Relaxed);
                    pac::WORDS[0x50 / 4] |= (1 << 2) | (1 << 4);
                }
            }
            0 => {
                if value >> 27 & 15 == 4 {
                    assert_ne!(value & (1 << 31), 0, "RM requires MEND=1 for legacy I2C");
                }
                assert!(MESSAGES.lock().unwrap().len() < 2);
                COMMANDS.lock().unwrap().push(value);
                LAST_QUEUED.store(value & (1 << 31) != 0, Ordering::Relaxed);
                if (value >> 27) & 15 == 6 && (value >> 16) as u8 == 7 {
                    DAA.store(true, Ordering::Relaxed);
                } else {
                    MESSAGES.lock().unwrap().push_back(Message {
                        length: (value & 0xffff) as usize,
                        read: (value >> 27) & 15 != 6 && value & (1 << 16) != 0,
                        last: value & (1 << 31) != 0,
                    });
                }
            }
            0x18 => {
                CPU_DATA_WRITES.fetch_add(1, Ordering::Relaxed);
                TX.lock().unwrap().push(value as u8);
                if DAA.load(Ordering::Relaxed) {
                    assert!(RX.lock().unwrap().is_empty());
                    assert_eq!(RECEIVED.load(Ordering::Relaxed), 8);
                    DAA_INDEX.fetch_add(1, Ordering::Relaxed);
                    RECEIVED.store(0, Ordering::Relaxed);
                    if DAA_ADDRESS_NACK.load(Ordering::Relaxed) {
                        ERROR.store(1 << 8, Ordering::Relaxed);
                    }
                } else {
                    assert!(TX_FIFO.lock().unwrap().len() < 8);
                    TX_FIFO.lock().unwrap().push_back(value as u8);
                }
            }
            _ => pac::WORDS[offset / 4] = value,
        }
    }
    advance();
    dma_step();
    true
}

pub fn track(buffer: &[u8]) {
    DMA_BUFFERS
        .lock()
        .unwrap()
        .push((buffer.as_ptr() as u32, buffer.as_ptr() as usize, buffer.len()));
}

fn dma_offset(address: usize) -> Option<(usize, usize)> {
    for base in [
        core::ptr::addr_of_mut!(pac::DMA1_WORDS) as usize,
        core::ptr::addr_of_mut!(pac::DMA2_WORDS) as usize,
    ] {
        if let Some(offset) = address.checked_sub(base).filter(|offset| *offset < 256) {
            return Some((base, offset));
        }
    }
    None
}

fn dma_step() {
    if DMA_STALL.load(Ordering::Relaxed) {
        return;
    }
    unsafe {
        let words = core::ptr::addr_of_mut!(pac::DMA1_WORDS).cast::<u32>();
        for (index, channel) in [3usize, 4].into_iter().enumerate() {
            let registers = words.add((8 + channel * 20) / 4);
            let config = registers.read_volatile();
            if config & 1 == 0 {
                continue;
            }
            if DMA_ERROR.load(Ordering::Relaxed) {
                words.write_volatile(words.read_volatile() | (8 << (channel * 4)));
                continue;
            }
            let remaining = registers.add(1).read_volatile();
            if remaining == 0 {
                continue;
            }
            let receive = channel == 4;
            let request = if receive { 1 << 8 } else { 1 << 12 };
            if pac::WORDS[1] & request == 0 {
                continue;
            }
            assert_eq!(config & ((3 << 8) | (3 << 10)), 0);
            assert_eq!(config & (1 << 4) != 0, !receive);
            assert_ne!(config & (1 << 7), 0);
            assert_eq!(
                registers.add(2).read_volatile(),
                pac::I3C.as_ptr() as u32 + if receive { 0x10 } else { 0x18 }
            );
            if receive && RX.lock().unwrap().is_empty() {
                continue;
            }
            if !receive && TX_FIFO.lock().unwrap().len() == 8 {
                continue;
            }
            let memory = registers.add(3).read_volatile();
            let progress = DMA_PROGRESS[index].load(Ordering::Relaxed) as usize;
            let pointer = DMA_BUFFERS
                .lock()
                .unwrap()
                .iter()
                .find_map(|&(low, full, length)| {
                    let offset = memory.wrapping_sub(low) as usize + progress;
                    (offset < length).then_some(full + offset)
                })
                .expect("register the DMA buffer before polling");
            if receive {
                (pointer as *mut u8).write_volatile(RX.lock().unwrap().pop_front().unwrap());
                if RX.lock().unwrap().is_empty() {
                    pac::WORDS[0x50 / 4] &= !(1 << 5);
                }
            } else {
                let byte = (pointer as *const u8).read_volatile();
                TX.lock().unwrap().push(byte);
                TX_FIFO.lock().unwrap().push_back(byte);
            }
            DMA_PROGRESS[index].fetch_add(1, Ordering::Relaxed);
            DMA_BYTES.fetch_add(1, Ordering::Relaxed);
            registers.add(1).write_volatile(remaining - 1);
            if remaining == 1 {
                words.write_volatile(words.read_volatile() | (2 << (channel * 4)));
            }
        }
    }
}
