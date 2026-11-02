use core::future::Future;
use core::pin::pin;
use core::sync::atomic::Ordering;
use core::task::{Context, Poll, Waker};

use embedded_hal::i2c::Operation;

use crate::i3c::{Error, I3c};
use crate::{gpio, model, pac, peripherals, Irqs, Peri};

fn driver() -> I3c<'static, peripherals::I3C, crate::mode::Async> {
    model::reset();
    I3c::new_with_dma(
        unsafe { Peri::new_unchecked(peripherals::I3C) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        unsafe { Peri::new_unchecked(peripherals::DMA1_CH4) },
        unsafe { Peri::new_unchecked(peripherals::DMA1_CH5) },
        crate::tests::config(),
    )
    .unwrap()
}

fn complete<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut context = Context::from_waker(Waker::noop());
    for _ in 0..32768 {
        if let Poll::Ready(result) = future.as_mut().poll(&mut context) {
            return result;
        }
    }
    panic!("transaction did not complete");
}

fn idle() {
    assert_eq!(pac::I3C.cfgr().read().0 & ((1 << 8) | (1 << 12) | (1 << 20)), 0);
    assert_eq!(pac::I3C.intenr().read().0, 0);
    assert!(!pac::DMA1.ch(3).cfgr().read().en());
    assert!(!pac::DMA1.ch(4).cfgr().read().en());
}

#[test]
fn contiguous_transfers_use_byte_dma_at_fifo_and_count_boundaries() {
    for length in [1usize, 3, 4, 7, 8, 9, 31, 257, 65535] {
        let mut bus = driver();
        let bytes: Vec<_> = (0..length).map(|index| (index % 251) as u8).collect();
        model::track(&bytes);
        assert_eq!(
            complete(bus.private_transaction(0x30, &mut [Operation::Write(&bytes)])),
            Ok(())
        );
        assert_eq!(*model::TX.lock().unwrap(), bytes);
        assert_eq!(model::CPU_DATA_WRITES.load(Ordering::Relaxed), 0);
        idle();
        let mut buffer = vec![0; length];
        model::track(&buffer);
        assert_eq!(
            complete(bus.private_transaction(0x30, &mut [Operation::Read(&mut buffer)])),
            Ok(())
        );
        assert_eq!(buffer, bytes);
        assert_eq!(model::CPU_DATA_READS.load(Ordering::Relaxed), 0);
        assert_eq!(model::DMA_BYTES.load(Ordering::Relaxed), length as u32 * 2);
        idle();
    }
}

#[test]
fn both_direction_orders_keep_repeated_start_and_only_final_stop() {
    for read_first in [false, true] {
        let mut bus = driver();
        let bytes = [31u8; 19];
        let mut buffer = [0; 29];
        model::track(&bytes);
        model::track(&buffer);
        let mut operations = if read_first {
            [Operation::Read(&mut buffer), Operation::Write(&bytes)]
        } else {
            [Operation::Write(&bytes), Operation::Read(&mut buffer)]
        };
        assert_eq!(complete(bus.private_transaction(0x30, &mut operations)), Ok(()));
        assert_eq!(buffer, core::array::from_fn(|index| index as u8));
        let commands = model::COMMANDS.lock().unwrap();
        assert_eq!(commands.len(), 2);
        assert_eq!(commands[0] & (1 << 31), 0);
        assert_ne!(commands[1] & (1 << 31), 0);
        assert_eq!(commands[0] & (1 << 16) != 0, read_first);
        assert_eq!(model::CPU_DATA_READS.load(Ordering::Relaxed), 0);
        assert_eq!(model::CPU_DATA_WRITES.load(Ordering::Relaxed), 0);
        idle();
    }
}

#[test]
fn legacy_one_direction_uses_dma_and_direction_changes_remain_rejected() {
    let mut bus = driver();
    let bytes = [1, 2, 3];
    model::track(&bytes);
    assert_eq!(
        complete(embedded_hal_async::i2c::I2c::write(&mut bus, 0x50, &bytes)),
        Ok(())
    );
    let mut buffer = [0; 9];
    model::track(&buffer);
    assert_eq!(
        complete(embedded_hal_async::i2c::I2c::read(&mut bus, 0x50, &mut buffer)),
        Ok(())
    );
    assert_eq!(model::DMA_BYTES.load(Ordering::Relaxed), 12);
    let previous = model::COMMANDS.lock().unwrap().len();
    assert_eq!(
        complete(embedded_hal_async::i2c::I2c::write_read(
            &mut bus,
            0x50,
            &bytes,
            &mut buffer
        )),
        Err(Error::UnsupportedTransaction)
    );
    assert_eq!(model::COMMANDS.lock().unwrap().len(), previous);
    idle();
}

#[test]
fn scattered_transactions_and_ccc_retain_cpu_fifo_servicing() {
    let mut bus = driver();
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Write(&[1, 2]), Operation::Write(&[3, 4])])),
        Ok(())
    );
    let mut first = [0; 2];
    let mut second = [0; 3];
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Read(&mut first), Operation::Read(&mut second)])),
        Ok(())
    );
    assert_eq!(first, [0, 1]);
    assert_eq!(second, [2, 3, 4]);
    assert_eq!(complete(bus.broadcast_ccc(0x00, &[5])), Ok(()));
    assert_eq!(model::DMA_BYTES.load(Ordering::Relaxed), 0);
    assert_eq!(model::CPU_DATA_WRITES.load(Ordering::Relaxed), 5);
    assert_eq!(model::CPU_DATA_READS.load(Ordering::Relaxed), 5);
    idle();
}

#[test]
fn dma_completion_waits_for_frame_stop() {
    let mut bus = driver();
    model::DEFER_STOP.store(true, Ordering::Relaxed);
    let bytes = [42; 13];
    model::track(&bytes);
    let mut operations = [Operation::Write(&bytes)];
    let mut future = pin!(bus.private_transaction(0x30, &mut operations));
    let mut context = Context::from_waker(Waker::noop());
    for _ in 0..256 {
        assert!(future.as_mut().poll(&mut context).is_pending());
        if model::FRAME_DONE.load(Ordering::Relaxed) {
            break;
        }
    }
    assert!(model::FRAME_DONE.load(Ordering::Relaxed));
    assert_eq!(pac::DMA1.ch(3).cntr().read().ndt(), 0);
    model::release_stop();
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Ok(())));
    idle();
}

#[test]
fn frame_completion_does_not_succeed_with_pending_dma_and_cancellation_quiesces() {
    let mut bus = driver();
    model::DMA_STALL.store(true, Ordering::Relaxed);
    let mut buffer = [0; 17];
    model::track(&buffer);
    let mut operations = [Operation::Read(&mut buffer)];
    let mut future = Box::pin(bus.private_transaction(0x30, &mut operations));
    let mut context = Context::from_waker(Waker::noop());
    assert!(future.as_mut().poll(&mut context).is_pending());
    unsafe {
        pac::WORDS[0x50 / 4] |= 1 << 9;
    }
    assert!(future.as_mut().poll(&mut context).is_pending());
    assert_eq!(pac::I3C.intenr().read().0 & (1 << 9), 0);
    drop(future);
    idle();
    assert!(bus.needs_recovery());
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Write(&[1])])),
        Err(Error::NeedsRecovery)
    );
    model::DMA_STALL.store(false, Ordering::Relaxed);
    bus.reinitialize().unwrap();
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Read(&mut buffer)])),
        Ok(())
    );
    idle();
}

#[test]
fn dma_fault_is_reported_and_peripheral_error_wins_simultaneous_faults() {
    for peripheral_error in [false, true] {
        let mut bus = driver();
        let bytes = [1; 11];
        model::track(&bytes);
        model::DMA_ERROR.store(true, Ordering::Relaxed);
        if peripheral_error {
            model::ERROR.store(1 << 8, Ordering::Relaxed);
        }
        assert_eq!(
            complete(bus.private_transaction(0x30, &mut [Operation::Write(&bytes)])),
            Err(if peripheral_error {
                Error::AddressNack
            } else {
                Error::Dma
            })
        );
        idle();
        assert!(bus.needs_recovery());
    }
}

#[test]
fn short_read_and_timeout_stop_both_channels() {
    let mut bus = driver();
    let mut buffer = [0; 23];
    model::track(&buffer);
    model::SHORT.store(true, Ordering::Relaxed);
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Read(&mut buffer)])),
        Err(Error::ShortRead)
    );
    idle();
    model::SHORT.store(false, Ordering::Relaxed);
    bus.reinitialize().unwrap();
    bus.blocking_private_transaction(0x30, &mut []).unwrap();
    drop(bus);

    let mut config = crate::tests::config();
    config.timeout = embassy_time::Duration::from_millis(1);
    model::reset();
    let mut bus = I3c::new_with_dma(
        unsafe { Peri::new_unchecked(peripherals::I3C) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        unsafe { Peri::new_unchecked(peripherals::DMA1_CH4) },
        unsafe { Peri::new_unchecked(peripherals::DMA1_CH5) },
        config,
    )
    .unwrap();
    model::DMA_STALL.store(true, Ordering::Relaxed);
    let mut operations = [Operation::Read(&mut buffer)];
    let mut future = Box::pin(bus.private_transaction(0x30, &mut operations));
    let mut context = Context::from_waker(Waker::noop());
    assert!(future.as_mut().poll(&mut context).is_pending());
    std::thread::sleep(std::time::Duration::from_millis(3));
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Timeout)));
    drop(future);
    assert!(bus.needs_recovery());
    idle();
}

#[test]
fn validation_precedes_dma_and_bus_activity() {
    let mut bus = driver();
    let bytes = vec![0; 65536];
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Write(&bytes)])),
        Err(Error::TooLong)
    );
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Write(&[])])),
        Err(Error::ZeroLength)
    );
    assert_eq!(
        complete(bus.private_transaction(0x22, &mut [Operation::Write(&[1])])),
        Err(Error::InvalidAddress)
    );
    model::CLOCK_HZ.store(10_000_000, Ordering::Relaxed);
    assert_eq!(
        complete(bus.private_transaction(0x30, &mut [Operation::Write(&[1])])),
        Err(Error::ClockChanged)
    );
    assert!(!bus.needs_recovery());
    assert!(model::COMMANDS.lock().unwrap().is_empty());
    idle();
}

#[test]
fn checked_dma_error_irq_wakes_without_panicking_on_either_channel() {
    use std::sync::Arc;
    use std::task::Wake;
    struct Counter(core::sync::atomic::AtomicU32);
    impl Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }
    for receive in [false, true] {
        let mut bus = driver();
        let mut bytes = [0; 17];
        model::track(&bytes);
        model::DMA_STALL.store(true, Ordering::Relaxed);
        let mut operations = [if receive {
            Operation::Read(&mut bytes)
        } else {
            Operation::Write(&bytes)
        }];
        let mut future = Box::pin(bus.private_transaction(0x30, &mut operations));
        let count = Arc::new(Counter(core::sync::atomic::AtomicU32::new(0)));
        let waker = Waker::from(count.clone());
        let mut context = Context::from_waker(&waker);
        assert!(future.as_mut().poll(&mut context).is_pending());
        unsafe {
            pac::DMA1_WORDS[0] |= 8 << (if receive { 4 } else { 3 } * 4);
            if receive {
                <peripherals::DMA1_CH5 as crate::dma::ChannelInterrupt>::on_irq();
            } else {
                <peripherals::DMA1_CH4 as crate::dma::ChannelInterrupt>::on_irq();
            }
        }
        assert!(count.0.load(Ordering::Relaxed) > 0);
        assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Dma)));
        drop(future);
        idle();
        assert!(bus.needs_recovery());
    }
}

#[test]
fn stopped_dma_with_remaining_count_is_an_error() {
    let mut bus = driver();
    let bytes = [0; 17];
    model::track(&bytes);
    model::DMA_STALL.store(true, Ordering::Relaxed);
    let mut operations = [Operation::Write(&bytes)];
    let mut future = Box::pin(bus.private_transaction(0x30, &mut operations));
    let mut context = Context::from_waker(Waker::noop());
    assert!(future.as_mut().poll(&mut context).is_pending());
    pac::DMA1.ch(3).cfgr().modify(|reg| reg.set_en(false));
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Dma)));
    drop(future);
    idle();
}

#[test]
fn error_at_frame_completion_overrides_successful_dma() {
    let mut bus = driver();
    model::DEFER_STOP.store(true, Ordering::Relaxed);
    let mut bytes = [0; 17];
    model::track(&bytes);
    let mut operations = [Operation::Read(&mut bytes)];
    let mut future = Box::pin(bus.private_transaction(0x30, &mut operations));
    let mut context = Context::from_waker(Waker::noop());
    for _ in 0..256 {
        assert!(future.as_mut().poll(&mut context).is_pending());
        if model::FRAME_DONE.load(Ordering::Relaxed) {
            break;
        }
    }
    assert!(model::FRAME_DONE.load(Ordering::Relaxed));
    model::ERROR.store(1 << 9, Ordering::Relaxed);
    model::release_stop();
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::DataNack)));
    drop(future);
    idle();
}
