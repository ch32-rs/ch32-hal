#![no_std]
#![no_main]

use ch32_hal::flash::{Error, Flash, FLASH_SIZE, WRITE_SIZE};
use ch32_hal::{pac, rcc};
use defmt_rtt as _;
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};
use qingke_rt::entry;

report_static!(FLASH_RESULT);
#[no_mangle]
pub static mut FLASH_TEST_RANGE: [u32; 2] = [0; 2];
#[no_mangle]
pub static mut FLASH_SNAPSHOTS: [[u32; 3]; 2] = [[0; 3]; 2];
#[no_mangle]
pub static mut FLASH_TEST_CASES: u32 = 0;

fn check(condition: bool, code: u32) {
    report_check!(FLASH_RESULT, condition, code, "FLASH_RESULT check failed");
}

fn snapshot() -> [u32; 3] {
    [
        pac::FLASH.ctlr().read().0,
        pac::FLASH.obr().read().0,
        pac::FLASH.wpr().read().0,
    ]
}

#[entry]
fn main() -> ! {
    report_set!(FLASH_RESULT, 1);
    let mut config = ch32_hal::Config::default();
    if cfg!(feature = "clock-pll") {
        config.rcc.sys = rcc::Sysclk::PLL;
        config.rcc.pll = Some(rcc::Pll {
            mul: rcc::PllMul::MUL20,
        });
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV2;
    }
    let peripherals = ch32_hal::init(config);
    let mut flash = Flash::new_blocking(peripherals.FLASH);
    let before = snapshot();
    let mut header = [0u8; 16];
    check(flash.capacity() == 992 * 1024 && WRITE_SIZE == 256, 10);
    check(flash.blocking_read(0, &mut header).is_ok(), 11);
    check(flash.blocking_read(u32::MAX, &mut header) == Err(Error::Size), 12);
    check(flash.blocking_erase(0, 256) == Err(Error::Unaligned), 13);
    check(flash.blocking_erase(4096, 0) == Err(Error::Size), 14);
    check(flash.blocking_write(FLASH_SIZE as u32, &[]).is_ok(), 15);
    check(flash.blocking_erase(FLASH_SIZE as u32, FLASH_SIZE as u32).is_ok(), 16);
    check(snapshot() == before, 17);

    #[cfg(feature = "flash-write-test")]
    let flash = destructive_test(flash);

    let after = snapshot();
    check(
        after[1..] == before[1..] && after[0] & (3 << 25) == before[0] & (3 << 25),
        28,
    );
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(FLASH_SNAPSHOTS), [before, after]);
    }
    report_set!(FLASH_RESULT, 2);
    defmt::info!(
        "FLASH_RESULT=PASS capacity={} write_test={}",
        flash.capacity(),
        cfg!(feature = "flash-write-test")
    );
    let mut heartbeat = 0u32;
    loop {
        heartbeat = heartbeat.wrapping_add(1);
        ch32v467_examples::heartbeat(heartbeat);
        qingke::riscv::asm::delay(rcc::clocks().hclk.0);
    }
}

#[cfg(feature = "flash-write-test")]
fn destructive_test(mut flash: Flash<'static, ch32_hal::flash::Blocking>) -> Flash<'static, ch32_hal::flash::Blocking> {
    use ch32_hal::flash::{BLOCK_ERASE_SIZE, ERASED_HALFWORD, ERASE_SIZE, FLASH_BASE};
    use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

    extern "C" {
        static __flash_test_start: u8;
        static __flash_test_end: u8;
    }
    let from = (core::ptr::addr_of!(__flash_test_start) as usize - FLASH_BASE) as u32;
    let to = (core::ptr::addr_of!(__flash_test_end) as usize - FLASH_BASE) as u32;
    check(
        (to - from) as usize == BLOCK_ERASE_SIZE && from as usize % BLOCK_ERASE_SIZE == 0,
        18,
    );
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(FLASH_TEST_RANGE), [from, to]) };
    defmt::info!("Flash write test offsets={=u32:08x}..{=u32:08x}", from, to);
    check(flash.blocking_erase(from, from + ERASE_SIZE as u32).is_ok(), 19);
    let mut page = [0u8; 512];
    check(flash.blocking_read(from, &mut page).is_ok(), 20);
    check(
        page.chunks_exact(2)
            .all(|pair| u16::from_le_bytes([pair[0], pair[1]]) == ERASED_HALFWORD),
        20,
    );
    let source: [u8; 513] = core::array::from_fn(|index| (index as u8).wrapping_mul(37));
    check(flash.blocking_write(from, &source[1..]).is_ok(), 21);
    check(flash.blocking_read(from, &mut page).is_ok() && page == source[1..], 22);
    check(flash.blocking_write(from, &source[1..257]) == Err(Error::NotErased), 23);
    check(
        flash
            .blocking_erase(from + ERASE_SIZE as u32, from + 2 * ERASE_SIZE as u32)
            .is_ok(),
        24,
    );
    let halfwords = [0x12, 0x34, 0xff, 0xff, 0x00, 0x00];
    check(
        flash
            .blocking_write_halfwords(from + ERASE_SIZE as u32, &halfwords)
            .is_ok(),
        24,
    );
    let mut halfword_read = [0u8; 6];
    check(
        flash
            .blocking_read(from + ERASE_SIZE as u32, &mut halfword_read)
            .is_ok()
            && halfword_read == halfwords,
        24,
    );
    let mut nor = flash.into_nor_flash(from, to).unwrap();
    check(nor.erase(0, ERASE_SIZE as u32).is_ok(), 25);
    check(nor.read(1, &mut page[..7]).is_ok() && page[..7] == [0xff; 7], 25);
    check(nor.write(0, &source[1..257]).is_ok(), 26);
    check(nor.read(3, &mut page[..7]).is_ok() && page[..7] == source[4..11], 26);
    let mut flash = nor.into_inner();
    check(flash.blocking_erase_blocks(from, to).is_ok(), 27);
    check(pac::FLASH.ctlr().read().lock() && pac::FLASH.ctlr().read().flock(), 28);
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(FLASH_TEST_CASES), 6) };
    flash
}
