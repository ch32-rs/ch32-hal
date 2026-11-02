use core::future::Future;
use core::task::{Context, Poll};

use super::*;
use crate::i2c_model::{self as model, Event, MODEL};
use crate::pac;

impl SclPin<peripherals::I2C, 0> for crate::gpio::AnyPin {}
impl SdaPin<peripherals::I2C, 0> for crate::gpio::AnyPin {}

fn channel(id: u8) -> ChannelAndRequest<'static> {
    ChannelAndRequest {
        channel: unsafe { Peri::new_unchecked(crate::dma::AnyChannel { id }) },
        request: (),
    }
}

fn driver<M: Mode>(dma: bool) -> I2c<'static, peripherals::I2C, M> {
    let mut config = Config::default();
    config.timeout = embassy_time::Duration::from_millis(10);
    I2c::new_inner::<0>(
        unsafe { Peri::new_unchecked(peripherals::I2C) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        dma.then(|| channel(5)),
        dma.then(|| channel(6)),
        Hertz(400_000),
        config,
    )
}

fn ready<F: Future>(future: F) -> F::Output {
    let mut future = core::pin::pin!(future);
    let mut context = Context::from_waker(std::task::Waker::noop());
    for _ in 0..32 {
        if let Poll::Ready(result) = future.as_mut().poll(&mut context) {
            return result;
        }
    }
    panic!("unexpected pending transfer: {:?}", MODEL.lock().unwrap().events);
}

fn events() -> Vec<Event> {
    MODEL.lock().unwrap().events.clone()
}
fn clear_events() {
    MODEL.lock().unwrap().events.clear();
}
fn stop_count() -> usize {
    events().iter().filter(|event| **event == Event::Stop).count()
}
fn start_count() -> usize {
    events().iter().filter(|event| **event == Event::Start).count()
}
fn assert_idle() {
    let control = pac::I2C.ctlr2().read();
    assert!(!control.dmaen() && !control.itevten() && !control.iterren() && !control.itbufen() && !control.last());
    assert!(!pac::DMA1.ch(5).cfgr().read().en());
    assert!(!pac::DMA1.ch(6).cfgr().read().en());
}

#[test]
fn clock_fields_short_reads_and_scattered_read_tails() {
    for dma in [false, true] {
        for length in [1usize, 2, 3, 4, 5, 31] {
            let expected: Vec<_> = (0..length).map(|value| value as u8 + 10).collect();
            model::setup(&expected);
            let mut blocking: I2c<'_, _, Blocking> = driver(dma);
            assert_eq!(pac::I2C.ctlr2().read().freq(), 60);
            assert_eq!(pac::I2C.rtr().read().trise(), 19);
            assert_eq!(pac::I2C.ckcfgr().read().ccr(), 167);
            clear_events();
            let mut bytes = vec![0; length];
            model::track(&bytes);
            if dma {
                drop(blocking);
                let mut asynchronous: I2c<'_, _, Async> = driver(true);
                clear_events();
                assert_eq!(ready(asynchronous.read(0x50, &mut bytes)), Ok(()));
                assert_idle();
            } else {
                assert_eq!(blocking.blocking_read(0x50, &mut bytes), Ok(()));
            }
            assert_eq!(bytes, expected);
            assert_eq!(stop_count(), 1);
            assert_eq!(start_count(), 1);
            let cleared = events().into_iter().find_map(|event| match event {
                Event::AddressCleared(ack, pos) => Some((ack, pos)),
                _ => None,
            });
            assert_eq!(cleared, Some((length > 2, length == 2)));
        }
        for lengths in [[1usize, 1, 1], [2, 1, 2], [8, 1, 1], [1, 8, 2]] {
            let total = lengths.iter().sum::<usize>();
            let expected: Vec<_> = (0..total).map(|value| value as u8).collect();
            model::setup(&expected);
            let mut first = vec![0; lengths[0]];
            let mut second = vec![0; lengths[1]];
            let mut third = vec![0; lengths[2]];
            model::track(&first);
            model::track(&second);
            model::track(&third);
            let mut operations = [
                Operation::Read(&mut first),
                Operation::Read(&mut second),
                Operation::Read(&mut third),
            ];
            if dma {
                let mut bus: I2c<'_, _, Async> = driver(true);
                clear_events();
                assert_eq!(
                    ready(embedded_hal_async::i2c::I2c::transaction(
                        &mut bus,
                        0x50,
                        &mut operations
                    )),
                    Ok(())
                );
                assert_idle();
            } else {
                let mut bus: I2c<'_, _, Blocking> = driver(false);
                clear_events();
                assert_eq!(
                    embedded_hal::i2c::I2c::transaction(&mut bus, 0x50, &mut operations),
                    Ok(())
                );
            }
            assert_eq!([first, second, third].concat(), expected);
            assert_eq!(stop_count(), 1);
            assert_eq!(start_count(), 1);
        }
    }
}

#[test]
fn repeated_starts_and_empty_address_probes_do_not_insert_stop() {
    model::setup(&[9, 8, 7, 6]);
    let mut bus: I2c<'_, _, Async> = driver(true);
    let write = [1, 2];
    let mut read = [0; 4];
    model::track(&write);
    model::track(&read);
    clear_events();
    assert_eq!(ready(bus.write_read(0x50, &write, &mut read)), Ok(()));
    assert_eq!(read, [9, 8, 7, 6]);
    assert_eq!(stop_count(), 1);
    assert_eq!(start_count(), 2);
    assert!(matches!(events().last(), Some(Event::Read(6))));
    assert_idle();
    clear_events();
    assert_eq!(ready(bus.write(0x20, &[])), Ok(()));
    assert_eq!(stop_count(), 1);
    assert_eq!(start_count(), 1);
    assert!(!events().iter().any(|event| matches!(event, Event::Dma(_))));
}

#[test]
fn read_then_write_and_consecutive_writes_keep_transaction_boundaries() {
    for dma in [false, true] {
        for length in [1usize, 2, 3, 5] {
            let expected: Vec<_> = (0..length).map(|byte| byte as u8 + 5).collect();
            model::setup(&expected);
            let mut read = vec![0; length];
            let first = [1, 2];
            let second = [3, 4];
            model::track(&read);
            model::track(&first);
            model::track(&second);
            let mut operations = [
                Operation::Read(&mut read),
                Operation::Write(&first),
                Operation::Write(&second),
            ];
            if dma {
                let mut bus: I2c<'_, _, Async> = driver(true);
                clear_events();
                assert_eq!(ready(bus.transaction(0x42, &mut operations)), Ok(()));
                assert_idle();
            } else {
                let mut bus: I2c<'_, _, Blocking> = driver(false);
                clear_events();
                assert_eq!(bus.blocking_transaction(0x42, &mut operations), Ok(()));
            }
            assert_eq!(read, expected);
            assert_eq!(start_count(), 2);
            assert_eq!(stop_count(), 1);
            let trace = events();
            let restart = trace.iter().rposition(|event| *event == Event::Start).unwrap();
            let last_read = trace.iter().rposition(|event| matches!(event, Event::Read(_))).unwrap();
            assert!(restart < last_read);
            let writes: Vec<_> = trace
                .into_iter()
                .filter_map(|event| match event {
                    Event::Write(byte) => Some(byte),
                    _ => None,
                })
                .collect();
            assert_eq!(writes, [1, 2, 3, 4]);
        }
    }
}

#[test]
fn invalid_inputs_do_not_start_the_bus() {
    model::setup(&[]);
    let mut bus: I2c<'_, _, Async> = driver(true);
    clear_events();
    let mut empty = [];
    let mut oversized = vec![0; 65536];
    assert_eq!(ready(bus.read(0x50, &mut empty)), Err(Error::Overrun));
    assert_eq!(ready(bus.write(0x80, &[])), Err(Error::InvalidAddress));
    assert_eq!(ready(bus.write(0x50, &oversized)), Err(Error::BufferTooLong));
    assert_eq!(
        ready(bus.write_read(0x50, &[1], &mut oversized)),
        Err(Error::BufferTooLong)
    );
    assert_eq!(
        ready(bus.transaction(0x50, &mut [Operation::Write(&[1]), Operation::Read(&mut empty)])),
        Err(Error::Overrun)
    );
    assert_eq!(start_count(), 0);
    assert_eq!(stop_count(), 0);
}

#[test]
fn pending_start_timeout_resets_the_peripheral_and_can_be_reused() {
    model::setup(&[]);
    let mut bus: I2c<'_, _, Async> = driver(true);
    MODEL.lock().unwrap().automatic = false;
    let mut future = Box::pin(bus.write(0x50, &[]));
    let mut context = Context::from_waker(std::task::Waker::noop());
    assert!(future.as_mut().poll(&mut context).is_pending());
    embassy_time::MockDriver::get().advance(embassy_time::Duration::from_millis(11));
    assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Timeout)));
    drop(future);
    assert_idle();
    assert!(!pac::I2C.ctlr1().read().start());
    assert_eq!(pac::I2C.ctlr2().read().freq(), 60);
    assert_eq!(pac::I2C.ckcfgr().read().ccr(), 167);
    MODEL.lock().unwrap().automatic = true;
    assert_eq!(ready(bus.write(0x50, &[])), Ok(()));
}

#[test]
fn cancelled_dma_stops_requests_and_reuses_the_channel() {
    model::setup(&[1, 2, 3, 4, 5, 6]);
    let mut bus: I2c<'_, _, Async> = driver(true);
    MODEL.lock().unwrap().stall_dma = true;
    let mut bytes = [0; 6];
    model::track(&bytes);
    let mut context = Context::from_waker(std::task::Waker::noop());
    let mut future = Box::pin(bus.read(0x50, &mut bytes));
    assert!(future.as_mut().poll(&mut context).is_pending());
    assert!(pac::I2C.ctlr2().read().dmaen());
    drop(future);
    assert_idle();
    MODEL.lock().unwrap().stall_dma = false;
    assert_eq!(ready(bus.read(0x50, &mut bytes)), Ok(()));
    assert_eq!(bytes, [1, 2, 3, 4, 5, 6]);
}

#[test]
fn errors_win_simultaneous_dma_completion_and_arbitration_does_not_send_stop() {
    for (flag, expected) in [
        (1 << 10, Error::Nack),
        (1 << 9, Error::Arbitration),
        (1 << 8, Error::Bus),
    ] {
        model::setup(&[]);
        let mut bus: I2c<'_, _, Async> = driver(true);
        let bytes = [1, 2, 3];
        model::track(&bytes);
        MODEL.lock().unwrap().error_dma = flag;
        clear_events();
        assert_eq!(ready(bus.write(0x50, &bytes)), Err(expected));
        assert_idle();
        if flag == 1 << 9 {
            assert_eq!(stop_count(), 0);
        }
        MODEL.lock().unwrap().error_dma = 0;
        assert_eq!(ready(bus.write(0x50, &bytes)), Ok(()));
    }
}

#[test]
fn receive_waits_for_rxne_and_btf_and_times_out_without_consuming_data() {
    for (length, blocked) in [(1usize, 1 << 6), (2, 1 << 2), (3, 1 << 2), (6, 1 << 2)] {
        model::setup(&[1, 2, 3, 4, 5, 6][..length]);
        let mut bus: I2c<'_, _, Async> = driver(true);
        let mut bytes = vec![0; length];
        model::track(&bytes);
        MODEL.lock().unwrap().blocked_status = blocked;
        clear_events();
        let mut context = Context::from_waker(std::task::Waker::noop());
        let mut future = Box::pin(bus.read(0x50, &mut bytes));
        assert!(future.as_mut().poll(&mut context).is_pending());
        assert!(!events().iter().any(|event| matches!(event, Event::Read(_))));
        assert_eq!(pac::I2C.ctlr2().read().itbufen(), length == 1);
        unsafe {
            on_interrupt::<peripherals::I2C>();
        }
        assert!(!pac::I2C.ctlr2().read().itevten());
        assert!(future.as_mut().poll(&mut context).is_pending());
        embassy_time::MockDriver::get().advance(embassy_time::Duration::from_millis(11));
        assert_eq!(future.as_mut().poll(&mut context), Poll::Ready(Err(Error::Timeout)));
        drop(future);
        assert_idle();
        assert_eq!(stop_count(), 1);
    }
}

#[test]
fn pending_stop_is_cancellable_without_issuing_another_start() {
    model::setup(&[]);
    let mut bus: I2c<'_, _, Async> = driver(true);
    MODEL.lock().unwrap().automatic = false;
    pac::I2C.ctlr1().modify(|w| w.set_stop(true));
    clear_events();
    let mut context = Context::from_waker(std::task::Waker::noop());
    let mut future = Box::pin(bus.write(0x50, &[]));
    assert!(future.as_mut().poll(&mut context).is_pending());
    assert_eq!(start_count(), 0);
    drop(future);
    assert_idle();
    assert!(!pac::I2C.ctlr1().read().stop());
}

#[test]
fn combined_error_flags_are_cleared_from_one_snapshot_with_arbitration_priority() {
    model::setup(&[]);
    let _bus: I2c<'_, _, Async> = driver(true);
    MODEL.lock().unwrap().automatic = false;
    for bits in 0..64u32 {
        let flags = (bits & 31) << 8 | (bits & 32) << 9;
        unsafe {
            pac::I2C_WORDS[0x14 / 4] = flags;
        }
        let result = I2c::<peripherals::I2C, Async>::check_and_clear_error_flags();
        let expected = if flags & (1 << 9) != 0 {
            Some(Error::Arbitration)
        } else if flags & (1 << 14) != 0 {
            Some(Error::Timeout)
        } else if flags & (1 << 12) != 0 {
            Some(Error::Crc)
        } else if flags & (1 << 11) != 0 {
            Some(Error::Overrun)
        } else if flags & (1 << 10) != 0 {
            Some(Error::Nack)
        } else if flags & (1 << 8) != 0 {
            Some(Error::Bus)
        } else {
            None
        };
        assert_eq!(result.err(), expected);
        assert_eq!(pac::I2C.star1().read().0 & 0x5f00, 0);
    }
}
