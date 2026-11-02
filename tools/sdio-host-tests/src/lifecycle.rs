use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::Ordering;
use core::task::{Context, Poll, Waker};

use crate::interrupt::typelevel as irq;
use crate::sdio::{Config, DataBlock, Error, InterruptHandler, Sdmmc, Signalling};
use crate::time::Hertz;
use crate::{dma, gpio, model, pac, peripherals, Peri};

struct Irqs;
unsafe impl irq::Binding<irq::SDIO, InterruptHandler<peripherals::SDIO>> for Irqs {}

fn poll<Output>(future: Pin<&mut impl Future<Output = Output>>) -> Poll<Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}

fn pin() -> Peri<'static, gpio::AnyPin> {
    unsafe { Peri::new_unchecked(gpio::AnyPin) }
}
fn config() -> Config {
    Config {
        poll_limit: 200,
        card_ready_attempts: 3,
        software_timeout_ms: 2,
        ..Default::default()
    }
}
fn controller(wide: bool) -> Sdmmc<'static, peripherals::SDIO, dma::AnyChannel> {
    let peri = unsafe { Peri::new_unchecked(peripherals::SDIO) };
    let dma = unsafe { Peri::new_unchecked(dma::AnyChannel { id: 16 }) };
    if wide {
        Sdmmc::new_4bit::<1>(peri, Irqs, dma, pin(), pin(), pin(), pin(), pin(), pin(), config()).unwrap()
    } else {
        Sdmmc::new_1bit::<0>(peri, Irqs, dma, pin(), pin(), pin(), config()).unwrap()
    }
}

#[test]
fn default_init_single_and_multiple_blocks_follow_dma_and_command_order() {
    model::reset();
    let mut sd = controller(true);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    assert_eq!(model::REMAP.load(Ordering::Relaxed), 1);
    assert_eq!(sd.clock(), Hertz(10_000_000));
    assert!(sd.card().unwrap().scr.bus_width_four());
    let mut blocks = [DataBlock([0; 512]), DataBlock([0; 512])];
    sd.blocking_read_blocks(7, &mut blocks).unwrap();
    assert_eq!(blocks[0].0, core::array::from_fn(|index| (index + 7) as u8));
    sd.blocking_write_blocks(9, &blocks).unwrap();
    assert_eq!(model::WRITES.lock().unwrap()[0].len(), 1024);
    let commands = model::COMMANDS.lock().unwrap();
    assert!(commands.contains(&(18, 7)) && commands.contains(&(25, 9)));
    assert_eq!(commands.iter().filter(|(index, _)| *index == 12).count(), 2);
    drop(commands);
    let status = sd.blocking_read_sd_status().unwrap();
    assert_eq!(status.bus_width(), sdio_host::BusWidth::Four);
    assert_eq!(status.speed_class(), 10);
    assert_eq!(pac::SDIO.dctrl().read().0, 0);
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
    drop(sd);
    assert!(!model::CLOCK.load(Ordering::Relaxed));
}

#[test]
fn legacy_sdsc_uses_byte_addresses_and_does_not_request_high_capacity() {
    model::reset();
    model::LEGACY.store(true, Ordering::Relaxed);
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(10_000_000)).unwrap();
    sd.blocking_read_block(7, &mut DataBlock([0; 512])).unwrap();
    let commands = model::COMMANDS.lock().unwrap();
    assert!(commands.contains(&(17, 7 * 512)));
    assert!(commands.contains(&(16, 512)));
    assert!(commands
        .iter()
        .filter(|(index, _)| *index == 41)
        .all(|(_, argument)| argument & (1 << 30) == 0));
}

#[test]
fn r3_ignores_crc_and_high_speed_switches_after_checking_support() {
    model::reset();
    model::CLOCK_HZ.store(100_000_000, Ordering::Relaxed);
    model::CRC_R3.store(true, Ordering::Relaxed);
    let mut sd = controller(true);
    sd.blocking_init_card(Hertz(50_000_000)).unwrap();
    assert_eq!(sd.signalling(), Signalling::SDR25);
    assert_eq!(sd.clock(), Hertz(50_000_000));
    let commands = model::COMMANDS.lock().unwrap();
    assert!(commands.contains(&(6, 0x00ff_fff1)) && commands.contains(&(6, 0x80ff_fff1)));
}

#[test]
fn no_card_is_bounded_and_reinitialization_recovers() {
    model::reset();
    model::NO_CARD.store(true, Ordering::Relaxed);
    let mut sd = controller(false);
    assert_eq!(sd.blocking_init_card(Hertz(25_000_000)), Err(Error::Timeout));
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    assert_eq!(pac::SDIO.power().read().0, 0);
    model::NO_CARD.store(false, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    assert!(sd.card().is_ok());
}

#[test]
fn asynchronous_completion_wakes_and_requires_both_dma_and_dataend() {
    use std::sync::atomic::AtomicUsize;
    use std::sync::Arc;
    struct Counter(AtomicUsize);
    impl std::task::Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }
    model::reset();
    let mut sd = controller(false);
    assert_eq!(
        poll(std::pin::pin!(sd.init_card(Hertz(25_000_000)))),
        Poll::Ready(Ok(()))
    );
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    let mut block = DataBlock([0; 512]);
    {
        let mut future = std::pin::pin!(sd.read_block(0, &mut block));
        let counter = Arc::new(Counter(AtomicUsize::new(0)));
        let waker = Waker::from(counter.clone());
        assert!(future.as_mut().poll(&mut Context::from_waker(&waker)).is_pending());
        model::dma_done(false);
        assert!(counter.0.load(Ordering::Relaxed) > 0);
        assert!(poll(future.as_mut()).is_pending());
        model::data_done(0);
        assert_eq!(poll(future.as_mut()), Poll::Ready(Ok(())));
    }
    assert!(sd.card().is_ok());
}

#[test]
fn cancellation_disables_dma_flushes_controller_and_requires_reinit() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    {
        let mut block = DataBlock([0; 512]);
        let mut future = std::pin::pin!(sd.read_block(0, &mut block));
        assert!(poll(future.as_mut()).is_pending());
        assert!(pac::DMA2.ch(3).cfgr().read().en());
    }
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
    assert_eq!(pac::SDIO.dctrl().read().0, 0);
    assert_eq!(pac::SDIO.mask().read().0, 0);
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    model::AUTO_COMPLETE.store(true, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    sd.blocking_read_block(0, &mut DataBlock([0; 512])).unwrap();
}

#[test]
fn dma_error_returns_an_error_and_software_timeout_cleans_up() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    {
        let mut block = DataBlock([0; 512]);
        let mut future = std::pin::pin!(sd.read_block(0, &mut block));
        assert!(poll(future.as_mut()).is_pending());
        model::dma_done(true);
        assert_eq!(poll(future.as_mut()), Poll::Ready(Err(Error::Dma)));
    }
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    model::AUTO_COMPLETE.store(true, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    {
        let mut block = DataBlock([0; 512]);
        let mut future = std::pin::pin!(sd.read_block(0, &mut block));
        assert!(poll(future.as_mut()).is_pending());
        embassy_time::MockDriver::get().advance(embassy_time::Duration::from_millis(3));
        assert_eq!(poll(future.as_mut()), Poll::Ready(Err(Error::SoftwareTimeout)));
    }
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
    assert_eq!(sd.card().err(), Some(Error::NoCard));
}

#[test]
fn bad_clock_and_config_do_not_enable_the_peripheral() {
    model::reset();
    model::CLOCK_HZ.store(200_000_000, Ordering::Relaxed);
    let result = Sdmmc::new_1bit::<0>(
        unsafe { Peri::new_unchecked(peripherals::SDIO) },
        Irqs,
        unsafe { Peri::new_unchecked(dma::AnyChannel { id: 16 }) },
        pin(),
        pin(),
        pin(),
        config(),
    );
    assert!(matches!(result, Err(Error::BadClock)));
    assert!(!model::CLOCK.load(Ordering::Relaxed));
    model::CLOCK_HZ.store(20_000_000, Ordering::Relaxed);
    let mut invalid = config();
    invalid.poll_limit = 0;
    let result = Sdmmc::new_1bit::<0>(
        unsafe { Peri::new_unchecked(peripherals::SDIO) },
        Irqs,
        unsafe { Peri::new_unchecked(dma::AnyChannel { id: 16 }) },
        pin(),
        pin(),
        pin(),
        invalid,
    );
    assert!(matches!(result, Err(Error::InvalidConfig)));
    assert!(!model::CLOCK.load(Ordering::Relaxed));
}

#[test]
fn invalid_reinitialization_and_block_ranges_preserve_an_existing_card() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    let commands = model::COMMANDS.lock().unwrap().len();
    let clock = pac::SDIO.clkcr().read().0;
    assert_eq!(sd.blocking_init_card(Hertz(0)), Err(Error::BadClock));
    assert_eq!(sd.blocking_read_blocks(0, &mut []), Err(Error::InvalidBufferLength));
    let end = sd.card().unwrap().block_count as u32;
    assert_eq!(
        sd.blocking_read_block(end, &mut DataBlock([0; 512])),
        Err(Error::AddressOutOfRange)
    );
    assert_eq!(model::COMMANDS.lock().unwrap().len(), commands);
    assert_eq!(pac::SDIO.clkcr().read().0, clock);
    assert!(sd.card().is_ok());
}

#[test]
fn all_data_faults_win_over_dma_and_data_completion_and_require_reinit() {
    for (flags, expected) in [
        (8, Error::Timeout),
        (2, Error::Crc),
        (0x10, Error::Underrun),
        (0x20, Error::Overrun),
        (0x200, Error::StBitErr),
    ] {
        model::reset();
        let mut sd = controller(false);
        sd.blocking_init_card(Hertz(25_000_000)).unwrap();
        model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
        {
            let mut block = DataBlock([0; 512]);
            let mut future = std::pin::pin!(sd.read_block(0, &mut block));
            assert!(poll(future.as_mut()).is_pending());
            model::dma_done(false);
            model::data_done(flags);
            assert_eq!(poll(future.as_mut()), Poll::Ready(Err(expected)));
        }
        assert_eq!(sd.card().err(), Some(Error::NoCard));
        assert_eq!(pac::SDIO.power().read().0, 0);
    }
}

#[test]
fn dataend_before_dma_and_late_crc_failure_do_not_return_success() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    let mut block = DataBlock([0; 512]);
    let mut future = std::pin::pin!(sd.read_block(0, &mut block));
    assert!(poll(future.as_mut()).is_pending());
    model::data_done(0);
    assert!(poll(future.as_mut()).is_pending());
    assert_eq!(pac::SDIO.mask().read().0 & 0x100, 0);
    model::dma_done(false);
    model::LATE_ERROR.store(2, Ordering::Relaxed);
    assert_eq!(poll(future.as_mut()), Poll::Ready(Err(Error::Crc)));
}

#[test]
fn incomplete_dma_and_a_dma_error_before_its_isr_are_detected() {
    for dma_error in [false, true] {
        model::reset();
        let mut sd = controller(false);
        sd.blocking_init_card(Hertz(25_000_000)).unwrap();
        model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
        let mut block = DataBlock([0; 512]);
        let mut future = std::pin::pin!(sd.read_block(0, &mut block));
        assert!(poll(future.as_mut()).is_pending());
        unsafe {
            if dma_error {
                pac::DMA2_WORDS[0] |= 8 << 12;
            } else {
                pac::DMA2_WORDS[(8 + 3 * 20) / 4] &= !1;
            }
        }
        model::data_done(0);
        assert_eq!(
            poll(future.as_mut()),
            Poll::Ready(Err(if dma_error {
                Error::Dma
            } else {
                Error::IncompleteTransfer
            }))
        );
    }
}

#[test]
fn command_failure_after_dma_arming_is_cleaned_up_without_buffer_access() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::FAIL_COMMAND.store(17, Ordering::Relaxed);
    let mut block = DataBlock([0xa5; 512]);
    assert_eq!(sd.blocking_read_block(0, &mut block), Err(Error::Timeout));
    assert_eq!(block.0, [0xa5; 512]);
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    model::FAIL_COMMAND.store(255, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    assert_eq!(sd.blocking_read_block(0, &mut block), Err(Error::SoftwareTimeout));
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
}

#[test]
fn asynchronous_powerup_is_cancellable_and_stuck_cards_time_out() {
    model::reset();
    model::OCR_BUSY.store(100, Ordering::Relaxed);
    let mut sd = controller(false);
    {
        let mut future = std::pin::pin!(sd.init_card(Hertz(25_000_000)));
        assert!(poll(future.as_mut()).is_pending());
    }
    assert_eq!(pac::SDIO.power().read().0, 0);
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    assert_eq!(sd.blocking_init_card(Hertz(25_000_000)), Err(Error::SoftwareTimeout));
    model::OCR_BUSY.store(0, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
}

#[test]
fn completed_write_waits_for_programming_and_can_be_cancelled() {
    model::reset();
    let mut sd = controller(false);
    sd.blocking_init_card(Hertz(25_000_000)).unwrap();
    model::AUTO_COMPLETE.store(false, Ordering::Relaxed);
    {
        let block = DataBlock([0x37; 512]);
        let mut future = std::pin::pin!(sd.write_block(0, &block));
        assert!(poll(future.as_mut()).is_pending());
        model::BUSY.store(2, Ordering::Relaxed);
        model::complete();
        assert!(poll(future.as_mut()).is_pending());
        assert!(!pac::DMA2.ch(3).cfgr().read().en());
    }
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    assert_eq!(model::WRITES.lock().unwrap()[0], [0x37; 512]);
}

#[test]
fn response_mismatch_card_errors_and_unsupported_high_speed_are_reported() {
    model::reset();
    let mut sd = controller(false);
    model::RESPONSE_MISMATCH.store(true, Ordering::Relaxed);
    assert_eq!(sd.blocking_init_card(Hertz(25_000_000)), Err(Error::ResponseMismatch));
    model::RESPONSE_MISMATCH.store(false, Ordering::Relaxed);
    model::HIGH_SPEED.store(false, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(50_000_000)).unwrap();
    assert_eq!(sd.signalling(), Signalling::SDR12);
    model::CARD_ERROR.store(0x8000_0000, Ordering::Relaxed);
    assert!(matches!(
        sd.blocking_read_block(0, &mut DataBlock([0; 512])),
        Err(Error::CardStatus(_))
    ));
    model::CARD_ERROR.store(0x0200_0000, Ordering::Relaxed);
    assert_eq!(
        sd.blocking_read_block(0, &mut DataBlock([0; 512])),
        Err(Error::CardLocked)
    );
}

#[test]
fn busy_high_speed_function_keeps_default_signalling_and_clock() {
    model::reset();
    model::CLOCK_HZ.store(100_000_000, Ordering::Relaxed);
    model::SWITCH_BUSY.store(true, Ordering::Relaxed);
    let mut sd = controller(true);
    sd.blocking_init_card(Hertz(50_000_000)).unwrap();
    assert_eq!(sd.signalling(), Signalling::SDR12);
    assert_eq!(sd.clock(), Hertz(25_000_000));
    let commands = model::COMMANDS.lock().unwrap();
    assert!(commands.contains(&(6, 0x00ff_fff1)));
    assert!(!commands.contains(&(6, 0x80ff_fff1)));
}

#[test]
fn rejected_high_speed_selection_powers_down_and_can_be_reinitialized() {
    model::reset();
    model::CLOCK_HZ.store(100_000_000, Ordering::Relaxed);
    model::SWITCH_SELECTED.store(0, Ordering::Relaxed);
    let mut sd = controller(true);
    assert_eq!(
        sd.blocking_init_card(Hertz(50_000_000)),
        Err(Error::SignalingSwitchFailed)
    );
    assert_eq!(sd.card().err(), Some(Error::NoCard));
    assert_eq!(pac::SDIO.power().read().0, 0);
    assert!(!pac::DMA2.ch(3).cfgr().read().en());
    model::SWITCH_SELECTED.store(1, Ordering::Relaxed);
    sd.blocking_init_card(Hertz(50_000_000)).unwrap();
    assert_eq!(sd.signalling(), Signalling::SDR25);
}

#[test]
fn unchecked_dma_errors_keep_existing_panic_behavior() {
    model::reset();
    let mut words = [0u32; 4];
    let transfer = unsafe {
        dma::Transfer::new_read(
            Peri::new_unchecked(dma::AnyChannel { id: 16 }),
            (),
            pac::SDIO.fifo().as_ptr().cast(),
            &mut words,
            Default::default(),
        )
    };
    assert!(std::panic::catch_unwind(|| model::dma_done(true)).is_err());
    drop(transfer);
}
