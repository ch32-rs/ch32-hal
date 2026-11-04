#![no_std]
#![no_main]

use ch32_hal::dma::word::Word;
use ch32_hal::dma::{AnyChannel, Transfer, TransferOptions};
use ch32_hal::Peri;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, Timer};
use panic_halt as _;
use ch32v467_examples::{report_check, report_set, report_static};

report_static!(DMA_RESULT);
#[no_mangle]
pub static mut DMA_CASES: u32 = 0;
#[no_mangle]
pub static mut DMA_CHANNELS_PASSED: u32 = 0;
#[no_mangle]
pub static mut DMA_LAST: [u32; 3] = [0; 3];

static mut SOURCE: [u8; 65_535] = [0; 65_535];
static mut DESTINATION: [u8; 65_537] = [0; 65_537];

fn check(condition: bool, code: u32) {
    report_check!(DMA_RESULT, condition, code, "DMA_RESULT check failed");
}

fn record_case(channel: u32, words: usize, word_size: usize) {
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(DMA_LAST),
            [channel, words as u32, word_size as u32],
        );
        let cases = core::ptr::read_volatile(core::ptr::addr_of!(DMA_CASES));
        core::ptr::write_volatile(core::ptr::addr_of_mut!(DMA_CASES), cases + 1);
    }
}

async fn copy<W: Word + PartialEq>(channel: Peri<'_, AnyChannel>, source: &[W], destination: &mut [W]) {
    let mut options = TransferOptions::default();
    options.memory_to_memory = true;
    options.peripheral_increment = true;
    let transfer = unsafe { Transfer::new_read(channel, (), source.as_ptr() as *mut W, destination, options) };
    check(with_timeout(Duration::from_secs(2), transfer).await.is_ok(), 10);
    check(source == destination, 11);
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) {
    report_set!(DMA_RESULT, 1);
    qingke::riscv::asm::delay(2_000_000);
    let peripherals = ch32_hal::init(Default::default());
    let channels: [Peri<'_, AnyChannel>; 24] = [
        peripherals.DMA1_CH1.into(),
        peripherals.DMA1_CH2.into(),
        peripherals.DMA1_CH3.into(),
        peripherals.DMA1_CH4.into(),
        peripherals.DMA1_CH5.into(),
        peripherals.DMA1_CH6.into(),
        peripherals.DMA1_CH7.into(),
        peripherals.DMA1_CH8.into(),
        peripherals.DMA1_CH9.into(),
        peripherals.DMA1_CH10.into(),
        peripherals.DMA1_CH11.into(),
        peripherals.DMA1_CH12.into(),
        peripherals.DMA1_CH13.into(),
        peripherals.DMA2_CH1.into(),
        peripherals.DMA2_CH2.into(),
        peripherals.DMA2_CH3.into(),
        peripherals.DMA2_CH4.into(),
        peripherals.DMA2_CH5.into(),
        peripherals.DMA2_CH6.into(),
        peripherals.DMA2_CH7.into(),
        peripherals.DMA2_CH8.into(),
        peripherals.DMA2_CH9.into(),
        peripherals.DMA2_CH10.into(),
        peripherals.DMA2_CH11.into(),
    ];
    let source = unsafe { &mut *core::ptr::addr_of_mut!(SOURCE) };
    let destination = unsafe { &mut *core::ptr::addr_of_mut!(DESTINATION) };
    for (index, byte) in source.iter_mut().enumerate() {
        *byte = (index as u8).wrapping_mul(37).wrapping_add(13);
    }
    for (index, mut channel) in channels.into_iter().enumerate() {
        for length in [1, 2, 3, 31, 256, 65_535] {
            destination[..length + 2].fill(0xa5);
            record_case(index as u32, length, 1);
            copy(channel.reborrow(), &source[..length], &mut destination[1..length + 1]).await;
            check(destination[0] == 0xa5 && destination[length + 1] == 0xa5, 12);
        }
        let source_halfwords = [0x1357u16, 0x2468, 0xffff, 0, 0x8001];
        let mut halfwords = [0u16; 5];
        record_case(index as u32, 5, 2);
        copy(channel.reborrow(), &source_halfwords, &mut halfwords).await;
        let source_words = [0x1357_2468u32, u32::MAX, 0, 0x8000_0001, 0x55aa_aa55];
        let mut words = [0u32; 5];
        record_case(index as u32, 5, 4);
        copy(channel.reborrow(), &source_words, &mut words).await;
        {
            let transfer = unsafe {
                Transfer::new_read(
                    channel.reborrow(),
                    (),
                    source.as_ptr() as *mut u8,
                    &mut destination[1..32],
                    TransferOptions::default(),
                )
            };
            Timer::after_millis(1).await;
            check(transfer.get_remaining_transfers() == 31, 13);
            drop(transfer);
        }
        record_case(index as u32, 31, 1);
        copy(channel.reborrow(), &source[..31], &mut destination[1..32]).await;
        unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(DMA_CHANNELS_PASSED), index as u32 + 1) };
        defmt::info!("DMA channel {} PASS", index);
    }
    report_set!(DMA_RESULT, 2);
    defmt::info!("DMA_RESULT=PASS channels=24 copies=216");
    loop {
        Timer::after_secs(1).await;
    }
}
