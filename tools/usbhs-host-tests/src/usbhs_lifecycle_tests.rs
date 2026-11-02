use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::Ordering;
use core::task::{Context, Poll, Waker};

use embassy_usb_driver::{
    Bus as _, ControlPipe as _, Driver as _, Endpoint as _, EndpointIn as _, EndpointOut as _, *,
};

use crate::interrupt::typelevel as irq;
use crate::usbhs::*;
use crate::{gpio, pac, usbhs_model as model, Peri, USBHS1, USBHS2};

struct Irqs;
unsafe impl irq::Binding<irq::USBHS1, InterruptHandler<USBHS1>> for Irqs {}
unsafe impl irq::Binding<irq::USBHS2, InterruptHandler<USBHS2>> for Irqs {}
fn poll<F: Future>(future: Pin<&mut F>) -> Poll<F::Output> {
    future.poll(&mut Context::from_waker(Waker::noop()))
}
fn irq1() {
    unsafe {
        <InterruptHandler<USBHS1> as irq::Handler<irq::USBHS1>>::on_interrupt();
    }
}
fn driver1(speed: Speed, size: usize) -> crate::usbhs::Driver<'static, USBHS1> {
    pac::RCC.ctlr().modify(|reg| reg.set_usbhs_pllrdy(true));
    let memory = Box::leak(vec![0u8; size].into_boxed_slice());
    model::MEMORY[0].store(memory.as_mut_ptr() as usize, Ordering::Relaxed);
    crate::usbhs::Driver::new(
        unsafe { Peri::new_unchecked(USBHS1) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        memory,
        Config { speed },
    )
    .unwrap()
}
fn start() -> (
    crate::usbhs::Bus<'static, USBHS1>,
    crate::usbhs::ControlPipe<'static, USBHS1>,
    crate::usbhs::Endpoint<'static, USBHS1, true>,
    crate::usbhs::Endpoint<'static, USBHS1, false>,
) {
    let mut driver = driver1(Speed::Full, 512);
    let tx = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    let rx = driver.alloc_endpoint_out(EndpointType::Bulk, None, 64, 0).unwrap();
    assert_eq!(tx.info().addr.index(), 1);
    assert_eq!(rx.info().addr.index(), 1);
    let (mut bus, pipe) = driver.start(64);
    assert_eq!(poll(std::pin::pin!(bus.poll())), Poll::Ready(Event::PowerDetected));
    assert_eq!(poll(std::pin::pin!(bus.enable())), Poll::Ready(()));
    bus.endpoint_set_enabled(tx.info().addr, true);
    bus.endpoint_set_enabled(rx.info().addr, true);
    (bus, pipe, tx, rx)
}
fn setup(pipe: &mut crate::usbhs::ControlPipe<'_, USBHS1>, bytes: [u8; 8]) {
    model::out(0, 0, &bytes, true, true);
    irq1();
    assert_eq!(poll(std::pin::pin!(pipe.setup())), Poll::Ready(bytes));
}

#[test]
fn setup_arriving_during_the_isr_is_not_consumed_as_control_out_data() {
    let (_bus, mut pipe, _tx, _rx) = start();
    model::SETUP_AFTER_SNAPSHOT.store(true, Ordering::Relaxed);
    irq1();
    irq1();
    assert_eq!(
        poll(std::pin::pin!(pipe.setup())),
        Poll::Ready([0, 5, 43, 0, 0, 0, 0, 0])
    );
}

#[test]
fn high_speed_all_endpoint_pairs_keep_complete_packets_and_independent_dma() {
    let mut driver = driver1(Speed::High, 64 + 14 * 512);
    let mut inputs = Vec::new();
    let mut outputs = Vec::new();
    for index in 1..8 {
        let input = driver.alloc_endpoint_in(EndpointType::Bulk, None, 512, 0).unwrap();
        let output = driver.alloc_endpoint_out(EndpointType::Bulk, None, 512, 0).unwrap();
        assert_eq!(input.info().addr.index(), index);
        assert_eq!(output.info().addr.index(), index);
        inputs.push(input);
        outputs.push(output);
    }
    assert!(driver.alloc_endpoint_in(EndpointType::Bulk, None, 512, 0).is_err());
    let (mut bus, _pipe) = driver.start(64);
    assert!(poll(std::pin::pin!(bus.enable())).is_ready());
    for index in 1..8 {
        bus.endpoint_set_enabled(inputs[index - 1].info().addr, true);
        bus.endpoint_set_enabled(outputs[index - 1].info().addr, true);
        let packet: Vec<_> = (0..512).map(|offset| (offset + index) as u8).collect();
        let mut input = std::pin::pin!(inputs[index - 1].write(&packet));
        assert!(poll(input.as_mut()).is_pending());
        model::out(0, index, &packet, false, true);
        assert_eq!(model::complete_in(0, index), packet);
        irq1();
        assert_eq!(poll(input.as_mut()), Poll::Ready(Ok(())));
        let mut received = [0; 512];
        assert_eq!(
            poll(std::pin::pin!(outputs[index - 1].read(&mut received))),
            Poll::Ready(Ok(512))
        );
        assert_eq!(received.as_slice(), packet);
    }
}

#[test]
fn pending_hardware_setup_aborts_old_transfer_before_the_isr_runs() {
    let (_bus, mut pipe, _tx, _rx) = start();
    setup(&mut pipe, [0x80, 6, 0, 1, 0, 0, 4, 0]);
    let replacement = [0, 5, 31, 0, 0, 0, 0, 0];
    {
        let mut transfer = std::pin::pin!(pipe.data_in(&[1, 2, 3, 4], true, true));
        assert!(poll(transfer.as_mut()).is_pending());
        model::out(0, 0, &replacement, true, true);
        assert_eq!(poll(transfer.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    }
    irq1();
    assert_eq!(poll(std::pin::pin!(pipe.setup())), Poll::Ready(replacement));
}

#[test]
fn malformed_setup_stalls_then_a_valid_setup_recovers_the_control_pipe() {
    let (_bus, mut pipe, _tx, _rx) = start();
    model::out(0, 0, &[0; 7], true, true);
    irq1();
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(pac::USBHS1.as_ptr()) };
    assert_eq!(device.uep0_rx_ctrl().read().0 & 3, 1);
    assert_eq!(device.uep0_tx_ctrl().read().0 & 3, 1);
    setup(&mut pipe, [0, 5, 42, 0, 0, 0, 0, 0]);
    let mut transfer = std::pin::pin!(pipe.accept_set_address(42));
    assert!(poll(transfer.as_mut()).is_pending());
    assert!(model::complete_in(0, 0).is_empty());
    irq1();
    assert!(poll(transfer.as_mut()).is_ready());
    assert_eq!(pac::USBHS1.usb_dev_ad().read().0, 42);
}

#[test]
fn interrupts_wake_the_registered_bus_and_both_endpoint_tasks() {
    use std::sync::atomic::AtomicUsize;
    use std::sync::Arc;
    struct WakeCounter(AtomicUsize);
    impl std::task::Wake for WakeCounter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.fetch_add(1, Ordering::Relaxed);
        }
    }
    let (mut bus, _pipe, mut tx, mut rx) = start();
    let counters: [_; 3] = core::array::from_fn(|_| Arc::new(WakeCounter(AtomicUsize::new(0))));
    let wakers = counters.clone().map(Waker::from);
    let mut received = [0; 64];
    let mut input = std::pin::pin!(tx.write(&[1, 2]));
    let mut output = std::pin::pin!(rx.read(&mut received));
    let mut event = std::pin::pin!(bus.poll());
    assert!(input.as_mut().poll(&mut Context::from_waker(&wakers[0])).is_pending());
    assert!(output.as_mut().poll(&mut Context::from_waker(&wakers[1])).is_pending());
    assert!(event.as_mut().poll(&mut Context::from_waker(&wakers[2])).is_pending());
    model::out(0, 1, &[3, 4], false, true);
    assert_eq!(model::complete_in(0, 1), [1, 2]);
    model::flags(0, 2, 2);
    irq1();
    assert!(counters.iter().all(|counter| counter.0.load(Ordering::Relaxed) > 0));
    assert_eq!(poll(input.as_mut()), Poll::Ready(Ok(())));
    assert_eq!(poll(output.as_mut()), Poll::Ready(Ok(2)));
    assert_eq!(poll(event.as_mut()), Poll::Ready(Event::Suspend));
}

#[test]
fn constructor_checks_clock_speed_and_memory_without_enabling_hardware() {
    pac::RCC.ctlr().modify(|reg| reg.set_usbhs_pllrdy(false));
    let result = crate::usbhs::Driver::new(
        unsafe { Peri::new_unchecked(USBHS1) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        Box::leak(Box::new([0u8; 64])),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::ClockNotReady)));
    pac::RCC.ctlr().modify(|reg| reg.set_usbhs_pllrdy(true));
    let result = crate::usbhs::Driver::new(
        unsafe { Peri::new_unchecked(USBHS1) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        Box::leak(Box::new([0u8; 63])),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::BufferTooSmall)));
    let result = crate::usbhs::Driver::new(
        unsafe { Peri::new_unchecked(USBHS1) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        Box::leak(Box::new([0u8; 64])),
        Config { speed: Speed::Low },
    );
    assert!(matches!(result, Err(Error::UnsupportedSpeed)));
    assert!(!model::CLOCKS[0].load(Ordering::Relaxed));
}

#[test]
fn endpoint_allocation_enforces_shared_maximum_without_losing_memory_on_failure() {
    let mut driver = driver1(Speed::Full, 64 + 64 + 32);
    let input = driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).unwrap();
    assert!(driver
        .alloc_endpoint_out(
            EndpointType::Bulk,
            Some(EndpointAddress::from_parts(1, Direction::Out)),
            32,
            0
        )
        .is_err());
    assert!(driver
        .alloc_endpoint_out(EndpointType::Bulk, Some(input.info().addr), 32, 0)
        .is_err());
    assert!(driver
        .alloc_endpoint_out(
            EndpointType::Bulk,
            Some(EndpointAddress::from_parts(8, Direction::Out)),
            32,
            0
        )
        .is_err());
    let output = driver.alloc_endpoint_out(EndpointType::Bulk, None, 32, 0).unwrap();
    assert_eq!(output.info().addr.index(), 2);
    assert!(driver.alloc_endpoint_in(EndpointType::Bulk, None, 64, 0).is_err());
}

#[test]
fn dma_addresses_and_simultaneous_directions_are_distinct() {
    let (bus, _pipe, _tx, _rx) = start();
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(pac::USBHS1.as_ptr()) };
    assert_ne!(device.uep1_rx_dma().read().0, device.uep1_tx_dma().read().0);
    assert_eq!(
        device.uep0_dma().read().0,
        model::MEMORY[0].load(Ordering::Relaxed) as u32
    );
    assert_eq!(device.uep1_max_len().read().0, 64);
    assert_eq!(device.uep_tx_en().read().0, 3);
    assert_eq!(device.uep_rx_en().read().0, 3);
    drop(bus);
    assert!(!model::CLOCKS[0].load(Ordering::Relaxed));
    assert!(!pac::RCC.cfgr2().read().utmi1on());
    assert!(pac::RCC.ctlr().read().usbhs_pllrdy());
}

#[test]
fn in_cancellation_keeps_dma_packet_until_completion_before_reuse() {
    let (_bus, _pipe, mut tx, _rx) = start();
    {
        let mut pending = std::pin::pin!(tx.write(&[1, 2, 3]));
        assert!(poll(pending.as_mut()).is_pending());
    }
    let mut next = std::pin::pin!(tx.write(&[4, 5]));
    assert!(poll(next.as_mut()).is_pending());
    assert_eq!(model::complete_in(0, 1), vec![1, 2, 3]);
    irq1();
    assert!(poll(next.as_mut()).is_pending());
    assert_eq!(model::complete_in(0, 1), vec![4, 5]);
    irq1();
    assert_eq!(poll(next.as_mut()), Poll::Ready(Ok(())));
}

#[test]
fn canceled_out_read_retains_packet_and_short_buffer_error_rearms_endpoint() {
    let (_bus, _pipe, _tx, mut rx) = start();
    let mut buffer = [0; 64];
    {
        let mut pending = std::pin::pin!(rx.read(&mut buffer));
        assert!(poll(pending.as_mut()).is_pending());
    }
    model::out(0, 1, &[1, 2, 3, 4], false, true);
    irq1();
    assert_eq!(
        poll(std::pin::pin!(rx.read(&mut [0; 2]))),
        Poll::Ready(Err(EndpointError::BufferOverflow))
    );
    model::out(0, 1, &[7, 8], false, true);
    irq1();
    assert_eq!(poll(std::pin::pin!(rx.read(&mut buffer))), Poll::Ready(Ok(2)));
    assert_eq!(buffer[..2], [7, 8]);
}

#[test]
fn reset_invalidates_pending_reads_and_writes_and_returns_bus_event() {
    let (mut bus, _pipe, mut tx, mut rx) = start();
    let mut buffer = [0; 64];
    let mut input = std::pin::pin!(tx.write(&[1]));
    let mut output = std::pin::pin!(rx.read(&mut buffer));
    assert!(poll(input.as_mut()).is_pending());
    assert!(poll(output.as_mut()).is_pending());
    model::flags(0, 1, 0);
    irq1();
    assert_eq!(poll(input.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    assert_eq!(poll(output.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    assert_eq!(poll(std::pin::pin!(bus.poll())), Poll::Ready(Event::Reset));
    assert_eq!(pac::USBHS1.usb_dev_ad().read().0, 0);
}

#[test]
fn control_in_waits_for_out_status_after_last_packet() {
    let (_bus, mut pipe, _tx, _rx) = start();
    setup(&mut pipe, [0x80, 6, 0, 1, 0, 0, 3, 0]);
    let mut transfer = std::pin::pin!(pipe.data_in(&[1, 2, 3], true, true));
    assert!(poll(transfer.as_mut()).is_pending());
    assert_eq!(model::complete_in(0, 0), vec![1, 2, 3]);
    irq1();
    assert!(poll(transfer.as_mut()).is_pending());
    model::out(0, 0, &[], false, true);
    irq1();
    assert_eq!(poll(transfer.as_mut()), Poll::Ready(Ok(())));
}

#[test]
fn set_address_is_committed_only_after_status_in_ack() {
    let (_bus, mut pipe, _tx, _rx) = start();
    setup(&mut pipe, [0, 5, 42, 0, 0, 0, 0, 0]);
    let mut transfer = std::pin::pin!(pipe.accept_set_address(42));
    assert!(poll(transfer.as_mut()).is_pending());
    assert_eq!(pac::USBHS1.usb_dev_ad().read().0, 0);
    assert!(model::complete_in(0, 0).is_empty());
    irq1();
    assert_eq!(poll(transfer.as_mut()), Poll::Ready(()));
    assert_eq!(pac::USBHS1.usb_dev_ad().read().0, 42);
}

#[test]
fn new_setup_aborts_control_data_and_prevents_stale_address_commit() {
    let (_bus, mut pipe, _tx, _rx) = start();
    setup(&mut pipe, [0x80, 6, 0, 1, 0, 0, 4, 0]);
    let replacement = [0, 5, 31, 0, 0, 0, 0, 0];
    {
        let mut transfer = std::pin::pin!(pipe.data_in(&[1, 2, 3, 4], true, true));
        assert!(poll(transfer.as_mut()).is_pending());
        model::out(0, 0, &replacement, true, true);
        irq1();
        assert_eq!(poll(transfer.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
    }
    assert_eq!(poll(std::pin::pin!(pipe.setup())), Poll::Ready(replacement));
    {
        let mut transfer = std::pin::pin!(pipe.accept_set_address(31));
        assert!(poll(transfer.as_mut()).is_pending());
        model::out(0, 0, &[0x80, 8, 0, 0, 0, 0, 1, 0], true, true);
        irq1();
        assert_eq!(poll(transfer.as_mut()), Poll::Ready(()));
    }
    assert_eq!(pac::USBHS1.usb_dev_ad().read().0, 0);
}

#[test]
fn control_out_data_then_accept_sends_data1_zero_length_status() {
    let (_bus, mut pipe, _tx, _rx) = start();
    setup(&mut pipe, [0x21, 0x20, 0, 0, 0, 0, 7, 0]);
    let mut buffer = [0; 7];
    {
        let mut transfer = std::pin::pin!(pipe.data_out(&mut buffer, true, true));
        assert!(poll(transfer.as_mut()).is_pending());
        model::out(0, 0, &[1, 2, 3, 4, 5, 6, 7], false, true);
        irq1();
        assert_eq!(poll(transfer.as_mut()), Poll::Ready(Ok(7)));
    }
    assert_eq!(buffer, [1, 2, 3, 4, 5, 6, 7]);
    let mut status = std::pin::pin!(pipe.accept());
    assert!(poll(status.as_mut()).is_pending());
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(pac::USBHS1.as_ptr()) };
    assert_eq!(device.uep0_tx_ctrl().read().0 & 7, 6);
    assert!(model::complete_in(0, 0).is_empty());
    irq1();
    assert_eq!(poll(status.as_mut()), Poll::Ready(()));
}

#[test]
fn stall_clear_resets_toggle_and_disable_wakes_waiters() {
    let (mut bus, _pipe, mut tx, mut rx) = start();
    let address = tx.info().addr;
    bus.endpoint_set_stalled(address, true);
    assert!(bus.endpoint_is_stalled(address));
    assert_eq!(
        poll(std::pin::pin!(tx.write(&[]))),
        Poll::Ready(Err(EndpointError::Disabled))
    );
    bus.endpoint_set_stalled(address, false);
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(pac::USBHS1.as_ptr()) };
    assert_eq!(device.uep1_tx_ctrl().read().0 & 0x0f, 0);
    let mut buffer = [0; 64];
    let mut read = std::pin::pin!(rx.read(&mut buffer));
    assert!(poll(read.as_mut()).is_pending());
    bus.endpoint_set_enabled(EndpointAddress::from_parts(1, Direction::Out), false);
    assert_eq!(poll(read.as_mut()), Poll::Ready(Err(EndpointError::Disabled)));
}

#[test]
fn duplicate_out_toggle_is_not_delivered_or_advanced() {
    let (_bus, _pipe, _tx, mut rx) = start();
    let mut buffer = [0; 64];
    model::out(0, 1, &[1], false, true);
    irq1();
    assert_eq!(poll(std::pin::pin!(rx.read(&mut buffer))), Poll::Ready(Ok(1)));
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(pac::USBHS1.as_ptr()) };
    assert_eq!(device.uep1_rx_ctrl().read().0 & 0x0c, 4);
    model::out(0, 1, &[1], false, false);
    irq1();
    assert!(poll(std::pin::pin!(rx.read(&mut buffer))).is_pending());
    assert_eq!(device.uep1_rx_ctrl().read().0 & 0x0c, 4);
    model::out(0, 1, &[2], false, true);
    irq1();
    assert_eq!(poll(std::pin::pin!(rx.read(&mut buffer))), Poll::Ready(Ok(1)));
    assert_eq!(buffer[0], 2);
}

#[test]
fn fifo_overflow_is_reported_and_transfer_flag_is_cleared_via_done() {
    let (_bus, _pipe, _tx, mut rx) = start();
    model::flags(0, 0x80, 0);
    irq1();
    assert_eq!(
        poll(std::pin::pin!(rx.read(&mut [0; 64]))),
        Poll::Ready(Err(EndpointError::BufferOverflow))
    );
    assert_eq!(pac::USBHS1.usb_int_fg().read().0 & 0x80, 0);
    model::out(0, 1, &[1, 2], false, true);
    assert_eq!(pac::USBHS1.usb_int_fg().read().0 & 0x10, 0x10);
    irq1();
    assert_eq!(pac::USBHS1.usb_int_fg().read().0 & 0x10, 0);
    assert_eq!(poll(std::pin::pin!(rx.read(&mut [0; 64]))), Poll::Ready(Ok(2)));
}

#[test]
fn suspend_resume_and_disable_reenable_keep_configuration() {
    let (mut bus, _pipe, _tx, rx) = start();
    model::flags(0, 2, 2);
    irq1();
    assert_eq!(poll(std::pin::pin!(bus.poll())), Poll::Ready(Event::Suspend));
    model::flags(0, 2, 0);
    irq1();
    assert_eq!(poll(std::pin::pin!(bus.poll())), Poll::Ready(Event::Resume));
    assert_eq!(poll(std::pin::pin!(bus.disable())), Poll::Ready(()));
    assert!(!model::CLOCKS[0].load(Ordering::Relaxed));
    assert_eq!(poll(std::pin::pin!(bus.enable())), Poll::Ready(()));
    bus.endpoint_set_enabled(rx.info().addr, true);
    model::out(0, 1, &[1], false, true);
    irq1();
}

#[test]
fn usbhs_instances_own_separate_clocks_and_state() {
    let (bus1, _pipe1, _tx1, _rx1) = start();
    let memory = Box::leak(Box::new([0u8; 512]));
    model::MEMORY[1].store(memory.as_mut_ptr() as usize, Ordering::Relaxed);
    let driver = crate::usbhs::Driver::new(
        unsafe { Peri::new_unchecked(USBHS2) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        unsafe { Peri::new_unchecked(gpio::AnyPin) },
        Irqs,
        memory,
        Config::default(),
    )
    .unwrap();
    let (mut bus2, _pipe2) = driver.start(64);
    assert_eq!(poll(std::pin::pin!(bus2.enable())), Poll::Ready(()));
    drop(bus1);
    assert!(!model::CLOCKS[0].load(Ordering::Relaxed));
    assert!(model::CLOCKS[1].load(Ordering::Relaxed));
    assert!(pac::RCC.cfgr2().read().utmi2on());
    assert!(!pac::RCC.cfgr2().read().utmi1on());
    drop(bus2);
    assert!(!model::CLOCKS[1].load(Ordering::Relaxed));
}

#[test]
fn isochronous_out_does_not_require_a_data_toggle_match() {
    let mut driver = driver1(Speed::Full, 2048);
    let mut rx = driver
        .alloc_endpoint_out(EndpointType::Isochronous, None, 1023, 1)
        .unwrap();
    let (mut bus, _pipe) = driver.start(64);
    assert_eq!(poll(std::pin::pin!(bus.enable())), Poll::Ready(()));
    bus.endpoint_set_enabled(rx.info().addr, true);
    model::out(0, 1, &[1, 2, 3, 4], false, false);
    irq1();
    let mut data = [0; 1023];
    assert_eq!(poll(std::pin::pin!(rx.read(&mut data))), Poll::Ready(Ok(4)));
    assert_eq!(data[..4], [1, 2, 3, 4]);
}
