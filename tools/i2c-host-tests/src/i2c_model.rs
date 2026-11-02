use std::sync::Mutex;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    Start,
    Stop,
    Address(u8),
    Read(u8),
    Write(u8),
    Dma(usize),
    AddressCleared(bool, bool),
    Reset,
}

pub struct Model {
    pub enabled: bool,
    pub automatic: bool,
    pub stall_dma: bool,
    pub error_dma: u32,
    pub blocked_status: u32,
    pub pending_restart: bool,
    pub phase: u8,
    pub bytes: Vec<u8>,
    pub received: usize,
    pub events: Vec<Event>,
    pub buffers: Vec<(u32, usize, usize)>,
}

pub static MODEL: Mutex<Model> = Mutex::new(Model {
    enabled: false,
    automatic: true,
    stall_dma: false,
    error_dma: 0,
    blocked_status: 0,
    pending_restart: false,
    phase: 0,
    bytes: Vec::new(),
    received: 0,
    events: Vec::new(),
    buffers: Vec::new(),
});

fn base() -> usize {
    core::ptr::addr_of_mut!(crate::pac::I2C_WORDS) as usize
}
unsafe fn get(offset: usize) -> u32 {
    ((base() + offset) as *const u32).read_volatile()
}
unsafe fn put(offset: usize, value: u32) {
    ((base() + offset) as *mut u32).write_volatile(value);
}

pub fn setup(bytes: &[u8]) {
    *MODEL.lock().unwrap() = Model {
        enabled: true,
        automatic: true,
        stall_dma: false,
        error_dma: 0,
        blocked_status: 0,
        pending_restart: false,
        phase: 0,
        bytes: bytes.to_vec(),
        received: 0,
        events: Vec::new(),
        buffers: Vec::new(),
    };
    unsafe {
        crate::pac::I2C_WORDS = [0; 16];
        crate::pac::DMA1_WORDS = [0; 64];
    }
    embassy_time::MockDriver::get().reset();
}

pub fn track(buffer: &[u8]) {
    MODEL
        .lock()
        .unwrap()
        .buffers
        .push((buffer.as_ptr() as u32, buffer.as_ptr() as usize, buffer.len()));
}

fn receive(model: &mut Model) -> u8 {
    let value = model.bytes[model.received];
    model.received += 1;
    value
}

fn refresh(model: &Model) {
    if !model.automatic {
        return;
    }
    unsafe {
        let errors = get(0x14) & 0xdf00;
        let flags = match model.phase {
            1 => 1,
            2 => 2,
            3 => 0x80 | 4,
            4 => {
                let remaining = model.bytes.len() - model.received;
                if remaining >= 2 {
                    0x40 | 4
                } else if remaining == 1 {
                    0x40
                } else {
                    0
                }
            }
            _ => 0,
        };
        put(0x14, errors | (flags & !model.blocked_status));
    }
}

pub fn read(address: usize) {
    if !(base()..base() + 0x24).contains(&address) {
        return;
    }
    let mut model = MODEL.lock().unwrap();
    if !model.enabled {
        return;
    }
    match address - base() {
        0x14 => refresh(&model),
        0x18 if model.phase == 2 && model.automatic => unsafe {
            let control = get(0);
            model.events.push(Event::AddressCleared(
                control & (1 << 10) != 0,
                control & (1 << 11) != 0,
            ));
            let read = get(0x10) & 1 != 0;
            model.phase = if read { 4 } else { 3 };
            refresh(&model);
        },
        0x10 if model.phase == 4 && model.automatic => {
            let value = receive(&mut model);
            model.events.push(Event::Read(value));
            unsafe {
                put(0x10, u32::from(value));
            }
            if model.pending_restart && model.received == model.bytes.len() {
                model.phase = 1;
                model.pending_restart = false;
                unsafe {
                    put(0, get(0) & !(1 << 8));
                    put(0x18, 3);
                }
            }
            refresh(&model);
        }
        _ => {}
    }
}

pub fn write(address: usize, mut value: u32) -> bool {
    if !(base()..base() + 0x24).contains(&address) {
        return false;
    }
    let mut model = MODEL.lock().unwrap();
    if !model.enabled {
        return false;
    }
    let offset = address - base();
    unsafe {
        match offset {
            0 => {
                if value & 0x8000 != 0 {
                    crate::pac::I2C_WORDS = [0; 16];
                    model.phase = 0;
                    model.events.push(Event::Reset);
                }
                if value & (1 << 8) != 0 && get(0) & (1 << 8) == 0 {
                    model.events.push(Event::Start);
                    if model.automatic {
                        if model.phase == 4 && get(0x18) & 1 != 0 && model.received < model.bytes.len() {
                            model.pending_restart = true;
                        } else {
                            model.phase = 1;
                            value &= !(1 << 8);
                            put(0x18, 3);
                        }
                    }
                }
                if value & (1 << 9) != 0 && get(0) & (1 << 9) == 0 {
                    model.events.push(Event::Stop);
                    if model.automatic {
                        value &= !(1 << 9);
                        put(0x18, get(0x18) & !1);
                    }
                }
                put(0, value);
                refresh(&model);
            }
            0x10 => {
                put(offset, value);
                if model.phase == 1 && model.automatic {
                    model.events.push(Event::Address(value as u8));
                    model.phase = 2;
                } else {
                    model.events.push(Event::Write(value as u8));
                }
                refresh(&model);
            }
            0x14 => put(offset, get(offset) & (value | !0xdf00)),
            4 => {
                put(offset, value);
                if value & (1 << 11) != 0 && !model.stall_dma && model.automatic {
                    for channel in [5usize, 6] {
                        let registers = core::ptr::addr_of_mut!(crate::pac::DMA1_WORDS)
                            .cast::<u32>()
                            .add((8 + channel * 20) / 4);
                        if registers.read_volatile() & 1 == 0 {
                            continue;
                        }
                        if registers.add(2).read_volatile() != (base() + 0x10) as u32 {
                            continue;
                        }
                        let length = registers.add(1).read_volatile() as usize;
                        let memory = registers.add(3).read_volatile();
                        let full = model
                            .buffers
                            .iter()
                            .find_map(|&(low, full, count)| {
                                let delta = memory.wrapping_sub(low) as usize;
                                (delta + length <= count).then_some(full + delta)
                            })
                            .expect("DMA buffer must be registered with model");
                        model.events.push(Event::Dma(length));
                        if channel == 6 {
                            for index in 0..length {
                                (full as *mut u8).add(index).write_volatile(receive(&mut model));
                            }
                        } else {
                            for index in 0..length {
                                model
                                    .events
                                    .push(Event::Write((full as *const u8).add(index).read_volatile()));
                            }
                        }
                        registers.add(1).write_volatile(0);
                        let status = core::ptr::addr_of_mut!(crate::pac::DMA1_WORDS).cast::<u32>();
                        status.write_volatile(status.read_volatile() | (2 << (channel * 4)));
                    }
                    let errors = model.error_dma;
                    if errors & (1 << 9) != 0 {
                        put(0x18, get(0x18) & !1);
                    }
                    put(0x14, get(0x14) | errors);
                    refresh(&model);
                }
            }
            _ => put(offset, value),
        }
    }
    true
}
