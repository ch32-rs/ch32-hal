use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

pub static CLOCKS: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];
pub static MEMORY: [AtomicUsize; 2] = [const { AtomicUsize::new(0) }; 2];
pub static SETUP_AFTER_SNAPSHOT: AtomicBool = AtomicBool::new(false);

pub fn after_read(address: usize) {
    if address == base(0) as usize + 0x9f && SETUP_AFTER_SNAPSHOT.swap(false, Ordering::Relaxed) {
        out(0, 0, &[0, 5, 43, 0, 0, 0, 0, 0], true, true);
    }
}

fn base(instance: usize) -> *mut u8 {
    if instance == 0 {
        core::ptr::addr_of_mut!(crate::pac::USB1_WORDS).cast()
    } else {
        core::ptr::addr_of_mut!(crate::pac::USB2_WORDS).cast()
    }
}

unsafe fn transfer_flag(instance: usize) {
    let registers = base(instance);
    let pending = (0..8).any(|index| (0..2).any(|dir| registers.add(0x9e + index * 4 + dir).read() & 0x80 != 0));
    let flags = registers.add(8);
    flags.write((flags.read() & !0x10) | if pending { 0x10 } else { 0 });
}

pub unsafe fn memory(instance: usize, index: usize, incoming: bool) -> *mut u8 {
    let offset = if index == 0 {
        0x20
    } else if incoming {
        0x40 + (index - 1) * 4
    } else {
        0x24 + (index - 1) * 4
    };
    let dma = base(instance).add(offset).cast::<u32>().read();
    let memory = MEMORY[instance].load(Ordering::Relaxed);
    (memory + dma.wrapping_sub(memory as u32) as usize) as *mut u8
}

pub fn out(instance: usize, index: usize, data: &[u8], setup: bool, matches: bool) {
    unsafe {
        let registers = base(instance);
        assert!(CLOCKS[instance].load(Ordering::Relaxed));
        assert!(registers.add(0x12).cast::<u16>().read() & (1 << index) != 0);
        if !setup {
            assert_eq!(registers.add(0x9f + index * 4).read() & 3, 2);
        }
        let max = registers.add(0x5c + index * 4).cast::<u32>().read() as usize;
        assert!(data.len() <= max);
        core::ptr::copy_nonoverlapping(data.as_ptr(), memory(instance, index, false), data.len());
        registers
            .add(if index == 0 { 0x7c } else { 0x80 + (index - 1) * 4 })
            .cast::<u16>()
            .write(data.len() as u16);
        let control = registers.add(0x9f + index * 4);
        control.write((control.read() & 0x0f & !8) | 0x80 | if setup { 8 } else { 0 } | if matches { 0x10 } else { 0 });
        transfer_flag(instance);
    }
}

pub fn complete_in(instance: usize, index: usize) -> Vec<u8> {
    unsafe {
        let registers = base(instance);
        assert!(registers.add(0x10).cast::<u16>().read() & (1 << index) != 0);
        let control = registers.add(0x9e + index * 4);
        assert_eq!(control.read() & 3, 2);
        let length = registers.add(0x9c + index * 4).cast::<u16>().read() as usize;
        let data = core::slice::from_raw_parts(memory(instance, index, true), length).to_vec();
        control.write(control.read() | 0x80);
        transfer_flag(instance);
        data
    }
}

pub fn flags(instance: usize, flags: u8, status: u8) {
    unsafe {
        base(instance).add(8).write(base(instance).add(8).read() | flags);
        base(instance).add(10).write(status);
    }
}

pub fn write(address: usize, value: u32, size: usize) -> bool {
    for instance in 0..2 {
        let registers = base(instance);
        let start = registers as usize;
        if !(start..start + 0xc0).contains(&address) {
            continue;
        }
        assert!(CLOCKS[instance].load(Ordering::Relaxed));
        let offset = address - start;
        unsafe {
            if offset == 8 {
                assert_eq!(size, 1);
                assert_eq!(value & 0x10, 0, "TRANSFER is RO, not W1C");
                registers.add(8).write(registers.add(8).read() & !(value as u8));
            } else if offset >= 0x9e && offset <= 0xbb && (offset - 0x9e) % 4 <= 1 {
                assert_eq!(size, 1);
                let old = (address as *mut u8).read();
                let incoming = (offset - 0x9e) % 4 == 0;
                let readonly = if incoming {
                    0
                } else if offset == 0x9f {
                    0x38
                } else {
                    0x30
                };
                (address as *mut u8)
                    .write((old & readonly) | (old & value as u8 & 0xc0) | (value as u8 & !readonly & 0x3f));
                transfer_flag(instance);
            } else {
                match size {
                    1 => (address as *mut u8).write(value as u8),
                    2 => (address as *mut u16).write(value as u16),
                    4 => (address as *mut u32).write(value),
                    _ => panic!("unexpected register width"),
                }
                if offset == 0 && value & 2 != 0 {
                    for index in 0..8 {
                        registers.add(0x9e + index * 4).write(0);
                        registers.add(0x9f + index * 4).write(0);
                    }
                    registers.add(8).write(0);
                }
            }
        }
        return true;
    }
    false
}
