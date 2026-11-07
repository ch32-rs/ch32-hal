//! Create and verify a new file in the first MBR exFAT partition.
//! Creates /H4TST00.TXT through /H4TST99.TXT without overwriting files.
#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}
use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use exfat_embedded::{BlockDevice, Error, FileSystem, Scratch};
use hal::sdio::{Config, DataBlock, InterruptHandler, Sdmmc};
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals};
use panic_halt as _;
ipc::mailbox!(ipc::protocols::SdioCheck);
bind_interrupts!(struct Irqs { SDIO => InterruptHandler<peripherals::SDIO>; });

struct Device<'d> {
    sd: Sdmmc<'d, peripherals::SDIO, peripherals::DMA1_CH1>,
}
impl BlockDevice for Device<'_> {
    type Error = hal::sdio::Error;
    fn sector_size(&self) -> usize {
        512
    }
    fn sector_count(&self) -> u64 {
        self.sd.card().map(|c| c.block_count).unwrap_or(0)
    }
    fn read_sector(&mut self, block: u64, out: &mut [u8]) -> Result<(), Self::Error> {
        let mut b = DataBlock([0; 512]);
        self.sd.blocking_read_block(
            u32::try_from(block).map_err(|_| Self::Error::AddressOutOfRange)?,
            &mut b,
        )?;
        out.copy_from_slice(&b.0);
        Ok(())
    }
    fn write_sector(&mut self, block: u64, data: &[u8]) -> Result<(), Self::Error> {
        let mut b = DataBlock([0; 512]);
        b.0.copy_from_slice(data);
        self.sd
            .blocking_write_block(u32::try_from(block).map_err(|_| Self::Error::AddressOutOfRange)?, &b)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
fn run(device: Device<'_>) -> Result<(), Error<hal::sdio::Error>> {
    let m = mailbox();
    let mut scratch = [0u8; 512];
    m.payload.stage.store(3, Relaxed);
    let mut fs = FileSystem::mount(device, &mut Scratch::new(&mut scratch))?;
    m.payload.stage.store(4, Relaxed);
    let mut name = *b"H4TST00.TXT";
    let mut file = None;
    for n in 0..100u8 {
        name[5] = b'0' + n / 10;
        name[6] = b'0' + n % 10;
        let path = core::str::from_utf8(&name).unwrap();
        // create() truncates existing files, so check before calling it.
        match fs.lookup(path, &mut Scratch::new(&mut scratch)) {
            Ok(_) => continue,
            Err(Error::PathNotFound) => {}
            Err(e) => return Err(e),
        }
        let f = fs.create(path, &mut Scratch::new(&mut scratch))?;
        m.payload.first_word.store(n as u32, Relaxed);
        file = Some(f);
        break;
    }
    let mut file = file.ok_or(Error::AlreadyExists)?;
    m.payload.stage.store(5, Relaxed);
    let mut contents = [0u8; 1300];
    for (i, byte) in contents.iter_mut().enumerate() {
        *byte = ((i * 37 + 11) % 251) as u8;
    }
    let written = fs.append(&mut file, &contents, &mut Scratch::new(&mut scratch))?;
    if written != contents.len() {
        return Err(Error::Corrupt);
    }
    fs.flush(&mut Scratch::new(&mut scratch))?;
    m.payload.stage.store(6, Relaxed);
    let mut fs = FileSystem::mount(fs.into_device(), &mut Scratch::new(&mut scratch))?;
    let mut file = fs.open(core::str::from_utf8(&name).unwrap(), &mut Scratch::new(&mut scratch))?;
    let mut readback = [0u8; 1300];
    let mut read = 0;
    while read < readback.len() {
        let n = fs.read(&mut file, &mut readback[read..], &mut Scratch::new(&mut scratch))?;
        if n == 0 {
            break;
        }
        read += n;
    }
    m.payload.blocks.store(read as u32, Relaxed);
    if read != contents.len() || readback != contents {
        return Err(Error::Corrupt);
    }
    Ok(())
}
#[hal::entry]
fn main() -> ! {
    let m = mailbox();
    m.clear();
    m.payload.stage.store(1, Relaxed);
    let p = hal::init(hal::Config {
        rcc: hal::rcc::Config::with_sysclk_400m_v5f_400m_v3f_100m_hse(),
        ..Default::default()
    });
    let mut sd = Sdmmc::new_4bit(
        p.SDIO,
        Irqs,
        p.DMA1_CH1,
        p.PB11,
        p.PB10,
        p.PE8,
        p.PE9,
        p.PE10,
        p.PE11,
        Config::default(),
    )
    .unwrap();
    m.payload.stage.store(2, Relaxed);
    if sd.blocking_init_card(Hertz(2_500_000)).is_err() {
        m.payload.error.store(1, Relaxed);
        loop {
            core::hint::spin_loop()
        }
    }
    m.payload.clock.store(sd.clock().0, Relaxed);
    m.payload.card.store(1, Relaxed);
    let mut mbr = DataBlock([0; 512]);
    if sd.blocking_read_block(0, &mut mbr).is_err() {
        m.payload.error.store(2, Relaxed);
        loop {
            core::hint::spin_loop()
        }
    }
    let start = u32::from_le_bytes(mbr.0[454..458].try_into().unwrap());
    let count = u32::from_le_bytes(mbr.0[458..462].try_into().unwrap());
    if mbr.0[510..] != [0x55, 0xaa]
        || mbr.0[450] != 7
        || count == 0
        || u64::from(start) + u64::from(count) > sd.card().unwrap().block_count
    {
        m.payload.error.store(3, Relaxed);
        loop {
            core::hint::spin_loop()
        }
    }
    match run(Device { sd }) {
        Ok(()) => m.payload.stage.store(7, Relaxed),
        Err(e) => {
            log!("exFAT error: {:?}", e);
            m.payload.error.store(
                match e {
                    Error::Device(_) => 4,
                    Error::InvalidBootSector => 5,
                    Error::AlreadyExists => 6,
                    Error::Corrupt => 7,
                    Error::NoSpace => 8,
                    Error::ReadOnly => 9,
                    Error::InvalidPartitionTable => 10,
                    Error::PartitionNotFound => 11,
                    Error::UnsupportedClusterSize => 12,
                    Error::PathNotFound => 13,
                    Error::EndOfFile => 14,
                    _ => 15,
                },
                Relaxed,
            );
        }
    }
    loop {
        core::hint::spin_loop()
    }
}
