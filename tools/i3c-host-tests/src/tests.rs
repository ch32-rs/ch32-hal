use core::future::Future;
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Poll, Waker};
use std::sync::Arc;
use std::task::Wake;

use embedded_hal::i2c::{I2c, Operation};

use crate::i3c::{Config, Error, I3c, Timings};
use crate::interrupt::typelevel::Handler;
use crate::{gpio, model, pac, peripherals, Irqs, Peri};

pub(super) fn config() -> Config {
    let mut config = Config::new(Timings {
        hclk_hz: 20_000_000,
        pp_low: 0x89,
        i3c_high: 0x88,
        od_low: 0x88,
        i2c_high: 0x88,
        aval: 19,
        free: 0x77,
        asncr: 2,
        sda_hold_1_5: true,
    });
    config.poll_limit = 4096;
    config
}

fn blocking() -> I3c<'static, peripherals::I3C, crate::mode::Blocking> {
    model::reset();
    I3c::new_blocking(
        unsafe { Peri::new_unchecked(peripherals::I3C) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        config(),
    )
    .unwrap()
}

fn asynchronous() -> I3c<'static, peripherals::I3C, crate::mode::Async> {
    asynchronous_with_config(config())
}

fn asynchronous_with_config(config: Config) -> I3c<'static, peripherals::I3C, crate::mode::Async> {
    model::reset();
    I3c::new(
        unsafe { Peri::new_unchecked(peripherals::I3C) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        config,
    )
    .unwrap()
}

struct WakeCount(AtomicU32);
impl Wake for WakeCount {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn initialization_and_drop() {
    let driver = blocking();
    assert_eq!(pac::I3C.timingr0().read().0, 0x88888889);
    assert_eq!(pac::I3C.timingr1().read().0, 0x10770213);
    assert_eq!(pac::I3C.timingr2().read().0, 0);
    assert_eq!(pac::I3C.cfgr().read().0, (1 << 19) | (1 << 6) | 3);
    assert_eq!(pac::I3C.owndevr().read().0, 0x45);
    for index in 0..2 {
        assert_eq!(pac::I3C.devr(index).read().0, 1 << 31);
        assert_eq!(pac::I3C.devrx(index).read().0, 1 << 31);
    }
    drop(driver);
    assert!(!model::CLOCK.load(Ordering::Relaxed));
}

#[test]
fn legacy_write_larger_than_fifo() {
    let mut driver = blocking();
    driver.write(0x50, &[0x5a; 65]).unwrap();
    assert_eq!(*model::TX.lock().unwrap(), vec![0x5a; 65]);
    assert_eq!(
        *model::COMMANDS.lock().unwrap(),
        vec![(1 << 31) | (4 << 27) | (0x50 << 17) | 65]
    );
    assert_ne!(pac::I3C.cfgr().read().0 & (1 << 2), 0);
    assert!(!driver.needs_recovery());
}

#[test]
fn restart_and_receive_at_frame_complete() {
    let mut driver = blocking();
    let mut read = [0xff; 31];
    driver
        .blocking_private_transaction(0x30, &mut [Operation::Write(&[0x10, 0x20]), Operation::Read(&mut read)])
        .unwrap();
    assert_eq!(read, core::array::from_fn::<_, 31, _>(|index| index as u8));
    let commands = model::COMMANDS.lock().unwrap();
    assert_eq!(commands.len(), 2);
    assert_eq!(commands[0] >> 31, 0);
    assert_eq!(commands[1] >> 31, 1);
    assert_ne!(commands[1] & (1 << 16), 0);
}

#[test]
fn adjacent_operations_are_merged() {
    let mut driver = blocking();
    let mut first = [0xff; 3];
    let mut second = [0xff; 9];
    driver
        .blocking_private_transaction(
            0x30,
            &mut [
                Operation::Write(&[1, 2]),
                Operation::Write(&[3]),
                Operation::Read(&mut first),
                Operation::Read(&mut second),
                Operation::Write(&[4]),
            ],
        )
        .unwrap();
    let commands = model::COMMANDS.lock().unwrap();
    assert_eq!(commands.len(), 3);
    assert_eq!(commands[0] & 0xffff, 3);
    assert_eq!(commands[1] & 0xffff, 12);
    assert_eq!(commands[2] & 0xffff, 1);
    assert_eq!(first, [0, 1, 2]);
    assert_eq!(second, [3, 4, 5, 6, 7, 8, 9, 10, 11]);
    assert_eq!(*model::TX.lock().unwrap(), [1, 2, 3, 4]);
}

#[test]
fn receive_then_write_prefills_without_losing_data() {
    let mut driver = blocking();
    let mut read = [0xff; 30];
    driver
        .blocking_private_transaction(0x30, &mut [Operation::Read(&mut read), Operation::Write(&[7; 24])])
        .unwrap();
    assert_eq!(read[29], 29);
    assert_eq!(model::TX.lock().unwrap().len(), 24);
}

#[test]
fn validation_precedes_hardware_and_does_not_poison() {
    let mut driver = blocking();
    assert_eq!(driver.write(0x80, &[1]), Err(Error::InvalidAddress));
    assert_eq!(driver.write(0x50, &[]), Err(Error::ZeroLength));
    let data = vec![1; 65_536];
    assert_eq!(driver.write(0x50, &data), Err(Error::TooLong));
    let data = vec![1; 40_000];
    assert_eq!(
        driver.blocking_transaction(0x50, &mut [Operation::Write(&data), Operation::Write(&data)]),
        Err(Error::TooLong)
    );
    assert_eq!(driver.write(0x22, &[1]), Err(Error::InvalidAddress));
    assert!(model::COMMANDS.lock().unwrap().is_empty());
    assert!(!driver.needs_recovery());
    driver.write(0x50, &[1]).unwrap();
}

#[test]
fn reserved_dynamic_addresses_and_private_message_encoding() {
    let mut driver = blocking();
    for address in [0, 7, 0x3e, 0x5e, 0x6e, 0x76, 0x78, 0x7e, 0xff] {
        assert_eq!(
            driver.blocking_private_transaction(address, &mut [Operation::Write(&[1])]),
            Err(Error::InvalidAddress)
        );
    }
    driver
        .blocking_private_transaction(0x30, &mut [Operation::Write(&[1, 2])])
        .unwrap();
    assert_eq!(model::COMMANDS.lock().unwrap()[0] >> 27 & 15, 2);
    assert_eq!(pac::I3C.cfgr().read().0 & (1 << 2), 0);
}

#[test]
fn zero_payload_broadcast_and_rejected_special_commands() {
    let mut driver = blocking();
    for command in [0x07, 0x2a, 0x80, 0xff] {
        assert_eq!(driver.blocking_broadcast_ccc(command, &[]), Err(Error::UnsupportedCcc));
    }
    driver.blocking_broadcast_ccc(0x06, &[]).unwrap();
    assert_eq!(model::COMMANDS.lock().unwrap()[0], (1 << 31) | (6 << 27) | (0x06 << 16));
}

#[test]
fn broadcast_payload() {
    let mut driver = blocking();
    driver.blocking_broadcast_ccc(0x01, &[0x0b]).unwrap();
    assert_eq!(*model::TX.lock().unwrap(), [0x0b]);
}

#[test]
fn errors_take_priority_and_require_reinitialization() {
    let mut driver = blocking();
    for (status, expected) in [
        (0, Error::Controller(0)),
        (1, Error::Controller(1)),
        (1 << 8, Error::AddressNack),
        (1 << 9, Error::DataNack),
        (1 << 6, Error::Fifo),
        (1 << 7, Error::Fifo),
        (1 << 4, Error::Protocol),
        (1 << 10, Error::Data),
    ] {
        model::ERROR.store(status, Ordering::Relaxed);
        assert_eq!(driver.write(0x50, &[1]), Err(expected));
        assert!(driver.needs_recovery());
        assert_eq!(pac::I3C.intenr().read().0, 0);
        assert_eq!(pac::I3C.cfgr().read().0, 0);
        assert_eq!(driver.write(0x50, &[1]), Err(Error::NeedsRecovery));
        model::ERROR.store(u32::MAX, Ordering::Relaxed);
        driver.reinitialize().unwrap();
    }
    driver.write(0x50, &[2]).unwrap();
}

#[test]
fn blocking_timeout_is_bounded() {
    let mut driver = blocking();
    model::STALL.store(true, Ordering::Relaxed);
    assert_eq!(driver.write(0x50, &[1]), Err(Error::Timeout));
    assert!(driver.needs_recovery());
}

#[test]
fn short_private_read_terminates_instead_of_waiting_for_requested_count() {
    let mut driver = blocking();
    model::SHORT.store(true, Ordering::Relaxed);
    let mut read = [0; 8];
    assert_eq!(
        driver.blocking_private_transaction(0x30, &mut [Operation::Read(&mut read)]),
        Err(Error::ShortRead)
    );
    assert!(driver.needs_recovery());
}

#[test]
fn changed_hclk_is_rejected_before_transfer() {
    let mut driver = blocking();
    model::CLOCK_HZ.store(10_000_000, Ordering::Relaxed);
    assert_eq!(driver.write(0x50, &[1]), Err(Error::ClockChanged));
    assert!(!driver.needs_recovery());
    assert!(model::COMMANDS.lock().unwrap().is_empty());
}

#[test]
fn async_waits_for_stop_and_interrupt_wakes_registered_task() {
    let mut driver = asynchronous();
    model::DEFER_STOP.store(true, Ordering::Relaxed);
    let wakes = Arc::new(WakeCount(AtomicU32::new(0)));
    let waker = Waker::from(wakes.clone());
    let mut context = Context::from_waker(&waker);
    let mut operations = [Operation::Write(&[1, 2])];
    let mut future = pin!(driver.transaction(0x50, &mut operations));
    assert!(future.as_mut().poll(&mut context).is_pending());
    assert!(model::FRAME_DONE.load(Ordering::Relaxed));
    model::release_stop();
    unsafe {
        crate::i3c::EventInterruptHandler::<peripherals::I3C>::on_interrupt();
    }
    assert_eq!(wakes.0.load(Ordering::Relaxed), 1);
    assert_eq!(pac::I3C.intenr().read().0, 0);
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Ok(())));
}

#[test]
fn cancellation_disables_requests_and_requires_explicit_reinitialization() {
    let mut driver = asynchronous();
    model::STALL.store(true, Ordering::Relaxed);
    let waker = Waker::noop();
    let mut context = Context::from_waker(waker);
    {
        let mut read = [0; 8];
        let mut operations = [Operation::Read(&mut read)];
        let mut future = pin!(driver.transaction(0x50, &mut operations));
        assert!(future.as_mut().poll(&mut context).is_pending());
    }
    assert_eq!(pac::I3C.intenr().read().0, 0);
    assert_eq!(pac::I3C.cfgr().read().0, 0);
    assert!(driver.needs_recovery());
    model::STALL.store(false, Ordering::Relaxed);
    driver.reinitialize().unwrap();
    driver.write(0x50, &[9]).unwrap();
}

#[test]
fn successive_frames_do_not_consume_stale_completion() {
    let mut driver = blocking();
    driver.write(0x50, &[1, 2]).unwrap();
    driver.write(0x50, &[3, 4]).unwrap();
    assert_eq!(model::COMMANDS.lock().unwrap().len(), 2);
    assert_eq!(*model::TX.lock().unwrap(), [1, 2, 3, 4]);
}

#[test]
fn daa_multiple_targets_preserves_each_identity_and_address() {
    let mut driver = blocking();
    let identities = [[1, 2, 3, 4, 5, 6, 0x20, 0x30], [11, 12, 13, 14, 15, 16, 0x40, 0x50]];
    model::DAA_TARGETS.lock().unwrap().extend(identities);
    let mut targets = [crate::i3c::Target::default(); 2];
    assert_eq!(driver.blocking_assign_addresses(&[0x30, 0x31], &mut targets), Ok(2));
    assert_eq!(targets[0].pid, [1, 2, 3, 4, 5, 6]);
    assert_eq!(targets[1].pid, [11, 12, 13, 14, 15, 16]);
    assert_eq!((targets[0].bcr, targets[0].dcr), (0x20, 0x30));
    assert_eq!((targets[1].bcr, targets[1].dcr), (0x40, 0x50));
    assert_eq!((targets[0].dynamic_address, targets[1].dynamic_address), (0x30, 0x31));
    assert_eq!(*model::TX.lock().unwrap(), [0x30, 0x31]);
    assert_eq!(model::COMMANDS.lock().unwrap()[0], (1 << 31) | (6 << 27) | (7 << 16));
}

#[test]
fn daa_no_unaddressed_target_can_complete_without_buffer_access() {
    let mut driver = blocking();
    let mut targets = [crate::i3c::Target::default(); 1];
    assert_eq!(driver.blocking_assign_addresses(&[0x30], &mut targets), Ok(0));
    assert_eq!(targets[0], crate::i3c::Target::default());
}

#[test]
fn daa_capacity_exhaustion_does_not_write_past_output_or_assign_extra_address() {
    let mut driver = blocking();
    model::DAA_TARGETS.lock().unwrap().extend([[1; 8], [2; 8]]);
    let mut targets = [crate::i3c::Target::default(); 1];
    assert_eq!(
        driver.blocking_assign_addresses(&[0x30], &mut targets),
        Err(Error::Capacity)
    );
    assert_eq!(*model::TX.lock().unwrap(), [0x30]);
    assert!(driver.needs_recovery());
    assert_eq!(pac::I3C.cfgr().read().0, 0);
}

#[test]
fn daa_rejects_duplicate_reserved_and_controller_addresses_before_start() {
    let mut driver = blocking();
    let mut targets = [crate::i3c::Target::default(); 2];
    for addresses in [[0x30, 0x30], [0x30, 0x22], [0x30, 0x76]] {
        assert_eq!(
            driver.blocking_assign_addresses(&addresses, &mut targets),
            Err(Error::InvalidAddress)
        );
    }
    assert_eq!(
        driver.blocking_assign_addresses(&[], &mut targets),
        Err(Error::Capacity)
    );
    assert_eq!(driver.blocking_assign_addresses(&[0x30], &mut []), Err(Error::Capacity));
    assert!(model::COMMANDS.lock().unwrap().is_empty());
    assert!(!driver.needs_recovery());
}

#[test]
fn daa_count_mismatch_and_address_nack_are_not_reported_as_success() {
    let mut driver = blocking();
    model::DAA_TARGETS.lock().unwrap().push([1; 8]);
    let mut targets = [crate::i3c::Target::default(); 1];
    model::DAA_COUNT_MISMATCH.store(true, Ordering::Relaxed);
    assert_eq!(
        driver.blocking_assign_addresses(&[0x30], &mut targets),
        Err(Error::Protocol)
    );
    driver.reinitialize().unwrap();
    model::DAA_COUNT_MISMATCH.store(false, Ordering::Relaxed);
    model::DAA_ADDRESS_NACK.store(true, Ordering::Relaxed);
    assert_eq!(
        driver.blocking_assign_addresses(&[0x30], &mut targets),
        Err(Error::AddressNack)
    );
}

#[test]
fn async_daa_and_short_read_complete() {
    let mut driver = asynchronous();
    model::DAA_TARGETS.lock().unwrap().push([0x11; 8]);
    let mut targets = [crate::i3c::Target::default(); 1];
    let mut context = Context::from_waker(Waker::noop());
    {
        let mut future = pin!(driver.assign_addresses(&[0x30], &mut targets));
        assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Ok(1)));
    }
    driver.reinitialize().unwrap();
    model::SHORT.store(true, Ordering::Relaxed);
    let mut read = [0; 8];
    let mut operations = [Operation::Read(&mut read)];
    let mut future = pin!(driver.private_transaction(0x30, &mut operations));
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::ShortRead)));
}

#[test]
fn async_timeout_leaves_controller_quiet() {
    let mut settings = config();
    settings.timeout = embassy_time::Duration::from_millis(1);
    let mut driver = asynchronous_with_config(settings);
    model::STALL.store(true, Ordering::Relaxed);
    let mut context = Context::from_waker(Waker::noop());
    {
        let mut operations = [Operation::Write(&[1])];
        let mut future = pin!(driver.transaction(0x50, &mut operations));
        assert!(future.as_mut().poll(&mut context).is_pending());
        std::thread::sleep(std::time::Duration::from_millis(5));
        assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Timeout)));
    }
    assert!(driver.needs_recovery());
    assert_eq!(pac::I3C.intenr().read().0, 0);
    assert_eq!(pac::I3C.cfgr().read().0, 0);
}

#[test]
fn late_interrupt_after_drop_does_not_access_gated_registers() {
    let driver = asynchronous();
    drop(driver);
    unsafe {
        crate::i3c::EventInterruptHandler::<peripherals::I3C>::on_interrupt();
        crate::i3c::ErrorInterruptHandler::<peripherals::I3C>::on_interrupt();
    }
}

#[test]
fn direct_ccc_read_uses_ccc_prefix_then_target_message() {
    let mut driver = blocking();
    let mut pid = [0xff; 6];
    driver
        .blocking_direct_ccc(0x8d, 0x30, Operation::Read(&mut pid))
        .unwrap();
    assert_eq!(pid, [0, 1, 2, 3, 4, 5]);
    let commands = model::COMMANDS.lock().unwrap();
    assert_eq!(
        *commands,
        [
            (6 << 27) | (0x8d << 16),
            (1 << 31) | (3 << 27) | (0x30 << 17) | (1 << 16) | 6,
        ]
    );
}

#[test]
fn direct_ccc_write_and_unsupported_forms() {
    let mut driver = blocking();
    for command in [0x00, 0x06, 0x7f, 0x9a] {
        assert_eq!(
            driver.blocking_direct_ccc(command, 0x30, Operation::Write(&[1])),
            Err(Error::UnsupportedCcc)
        );
    }
    assert_eq!(
        driver.blocking_direct_ccc(0x80, 0x30, Operation::Write(&[])),
        Err(Error::ZeroLength)
    );
    driver
        .blocking_direct_ccc(0x81, 0x30, Operation::Write(&[0x0b]))
        .unwrap();
    assert_eq!(*model::TX.lock().unwrap(), [0x0b]);
}

#[test]
fn error_simultaneous_with_completion_wins() {
    let mut driver = asynchronous();
    model::DEFER_STOP.store(true, Ordering::Relaxed);
    let mut operations = [Operation::Write(&[1])];
    let mut context = Context::from_waker(Waker::noop());
    let mut future = pin!(driver.transaction(0x50, &mut operations));
    assert!(future.as_mut().poll(&mut context).is_pending());
    model::release_stop();
    model::ERROR.store(1 << 8, Ordering::Relaxed);
    let _ = pac::I3C.evr().read();
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::AddressNack)));
}

#[test]
fn invalid_config_never_enables_clock() {
    model::reset();
    for variant in 0..6 {
        let mut settings = config();
        match variant {
            0 => settings.timings.hclk_hz = 10_000_000,
            1 => settings.timings.free = 128,
            2 => settings.timings.asncr = 4,
            3 => settings.poll_limit = 0,
            4 => settings.controller_address = 0x76,
            _ => settings.timings.hclk_hz = 0,
        }
        assert!(I3c::new_blocking(
            unsafe { Peri::new_unchecked(peripherals::I3C) },
            unsafe { Peri::new_unchecked(gpio::AnyPin) },
            unsafe { Peri::new_unchecked(gpio::AnyPin) },
            settings,
        )
        .is_err());
        assert!(!model::CLOCK.load(Ordering::Relaxed));
    }
}

#[test]
fn legacy_direction_changes_are_rejected_before_any_bus_activity() {
    let mut driver = blocking();
    let mut read = [0xff; 8];
    assert_eq!(
        driver.write_read(0x50, &[1], &mut read),
        Err(Error::UnsupportedTransaction)
    );
    assert_eq!(
        driver.blocking_transaction(0x50, &mut [Operation::Read(&mut read), Operation::Write(&[1])]),
        Err(Error::UnsupportedTransaction)
    );
    assert!(model::COMMANDS.lock().unwrap().is_empty());
    assert!(!driver.needs_recovery());
    assert_eq!(read, [0xff; 8]);
    driver.write(0x50, &[1]).unwrap();
}

#[test]
fn same_direction_legacy_scatter_buffers_use_one_message() {
    let mut driver = blocking();
    driver
        .blocking_transaction(0x50, &mut [Operation::Write(&[1, 2]), Operation::Write(&[3, 4])])
        .unwrap();
    let mut first = [0xff; 2];
    let mut second = [0xff; 3];
    driver
        .blocking_transaction(0x50, &mut [Operation::Read(&mut first), Operation::Read(&mut second)])
        .unwrap();
    assert_eq!((first, second), ([0, 1], [2, 3, 4]));
    let commands = model::COMMANDS.lock().unwrap();
    assert_eq!(commands.len(), 2);
    assert_eq!((commands[0] & 0xffff, commands[1] & 0xffff), (4, 5));
    assert!(commands.iter().all(|command| command >> 31 == 1));
}
