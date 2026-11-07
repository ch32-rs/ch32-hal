//! Read/write a newly created test file in the first MBR FAT partition.
//! Creates H4TST00.TXT through H4TST99.TXT without overwriting an existing file.
#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use core::cell::RefCell;
use core::sync::atomic::Ordering::Relaxed;

use ch32_hal as hal;
use ch32h417_ipc as ipc;
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use hal::sdio::{Config, DataBlock, InterruptHandler, Sdmmc};
use hal::time::Hertz;
use hal::{bind_interrupts, peripherals};
use panic_halt as _;
ipc::mailbox!(ipc::protocols::SdioCheck);
bind_interrupts!(struct Irqs { SDIO => InterruptHandler<peripherals::SDIO>; });

struct Device<'d>(RefCell<Sdmmc<'d, peripherals::SDIO, peripherals::DMA1_CH1>>);
impl BlockDevice for Device<'_> {
    type Error = hal::sdio::Error;
    fn read(&self, blocks: &mut [Block], start: BlockIdx) -> Result<(), Self::Error> {
        let mut sd = self.0.borrow_mut();
        for (i, block) in blocks.iter_mut().enumerate() {
            let mut buf = DataBlock([0; 512]);
            sd.blocking_read_block(
                start.0.checked_add(i as u32).ok_or(Self::Error::AddressOutOfRange)?,
                &mut buf,
            )?;
            block.contents.copy_from_slice(&buf.0);
        }
        Ok(())
    }
    fn write(&self, blocks: &[Block], start: BlockIdx) -> Result<(), Self::Error> {
        let mut sd = self.0.borrow_mut();
        for (i, block) in blocks.iter().enumerate() {
            let mut buf = DataBlock([0; 512]);
            buf.0.copy_from_slice(&block.contents);
            sd.blocking_write_block(
                start.0.checked_add(i as u32).ok_or(Self::Error::AddressOutOfRange)?,
                &buf,
            )?;
        }
        Ok(())
    }
    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        Ok(BlockCount(
            u32::try_from(self.0.borrow().card()?.block_count).map_err(|_| Self::Error::AddressOutOfRange)?,
        ))
    }
}
struct Clock;
impl TimeSource for Clock {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
fn run(device: Device<'_>) -> Result<(), embedded_sdmmc::Error<hal::sdio::Error>> {
    let m = mailbox();
    let manager = VolumeManager::new(device, Clock);
    m.payload.stage.store(3, Relaxed);
    let volume = manager.open_volume(VolumeIdx(0))?;
    m.payload.stage.store(4, Relaxed);
    let root = volume.open_root_dir()?;
    let mut name = *b"H4TST00.TXT";
    let mut chosen = None;
    for n in 0..100u8 {
        name[5] = b'0' + n / 10;
        name[6] = b'0' + n % 10;
        let filename = core::str::from_utf8(&name).unwrap();
        match root.open_file_in_dir(filename, Mode::ReadWriteCreate) {
            Ok(f) => {
                chosen = Some((n, f));
                break;
            }
            Err(embedded_sdmmc::Error::FileAlreadyExists) => continue,
            Err(e) => return Err(e),
        }
    }
    let (n, file) = chosen.ok_or(embedded_sdmmc::Error::FileAlreadyExists)?;
    m.payload.stage.store(5, Relaxed);
    m.payload.first_word.store(n as u32, Relaxed);
    let mut contents = [0u8; 1300];
    for (i, byte) in contents.iter_mut().enumerate() {
        *byte = ((i * 37 + 11) % 251) as u8;
    }
    file.write(&contents)?;
    file.close()?;
    m.payload.stage.store(6, Relaxed);
    let filename = core::str::from_utf8(&name).unwrap();
    let file = root.open_file_in_dir(filename, Mode::ReadOnly)?;
    let mut buf = [0u8; 1300];
    let read = file.read(&mut buf)?;
    m.payload.blocks.store(read as u32, Relaxed);
    if read != contents.len() || buf != contents {
        return Err(embedded_sdmmc::Error::FormatError("file contents differ"));
    }
    file.close()?;
    root.close()?;
    volume.close()?;
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
    if let Err(e) = sd.blocking_init_card(Hertz(2_500_000)) {
        log!("SD init error: {:?}", e);
        m.payload.error.store(1, Relaxed);
        loop {
            core::hint::spin_loop()
        }
    }
    m.payload.clock.store(sd.clock().0, Relaxed);
    m.payload.card.store(1, Relaxed);
    match run(Device(RefCell::new(sd))) {
        Ok(()) => {
            m.payload.stage.store(7, Relaxed);
        }
        Err(e) => {
            log!("FAT error: {:?}", e);
            m.payload.error.store(
                match e {
                    embedded_sdmmc::Error::DeviceError(_) => 1,
                    embedded_sdmmc::Error::FormatError(_) => 2,
                    embedded_sdmmc::Error::NoSuchVolume => 3,
                    embedded_sdmmc::Error::FileAlreadyExists => 4,
                    _ => 5,
                },
                Relaxed,
            );
        }
    }
    loop {
        core::hint::spin_loop()
    }
}
