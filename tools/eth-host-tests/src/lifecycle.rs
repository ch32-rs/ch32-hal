use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Waker};
use std::sync::Arc;
use std::task::Wake;

use embassy_net_driver::{Driver, LinkState, RxToken, TxToken};

use crate::eth::{Config, Error, Ethernet, Resources};
use crate::{model, pac, peripherals, Irqs, Peri};

struct WakeCount(AtomicU32);
impl Wake for WakeCount {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
    fn wake_by_ref(self: &Arc<Self>) {
        self.0.fetch_add(1, Ordering::Relaxed);
    }
}

struct Fixture {
    driver: Ethernet<'static, 2, 2>,
    memory: *mut u8,
}
impl Fixture {
    fn new() -> Self {
        let resources = Box::leak(Box::new(Resources::<2, 2>::new()));
        let memory = core::ptr::from_mut(resources).cast();
        let driver = Ethernet::new(
            unsafe { Peri::new_unchecked(peripherals::ETH) },
            Irqs,
            resources,
            Config {
                poll_limit: 16,
                ..Default::default()
            },
        )
        .unwrap();
        Self { driver, memory }
    }
    fn descriptor(&self, receive: bool, index: usize) -> *mut u32 {
        unsafe {
            self.memory
                .add(if receive { 32 } else { 0 })
                .cast::<u32>()
                .add(index * 4)
        }
    }
    fn buffer(&self, receive: bool, index: usize) -> *mut u8 {
        unsafe { self.memory.add(64 + if receive { 3072 } else { 0 }).add(index * 1536) }
    }
    fn received(&self, index: usize, status: u32) {
        unsafe {
            core::ptr::write_bytes(self.buffer(true, index), 0x37, 1536);
            self.descriptor(true, index).write_volatile(status);
        }
    }
    fn up(&mut self) {
        model::link(true, true, true);
        model::irq(1 << 11);
        assert!(self.driver.link_state(&mut Context::from_waker(Waker::noop())) == LinkState::Up);
    }
}

#[test]
fn initializes_phy_and_chained_rings_without_writing_reserved_dma_fields() {
    model::reset();
    let fixture = Fixture::new();
    assert_eq!(fixture.driver.phy_id(), 0x73719011);
    assert!(!fixture.driver.link().up);
    assert_eq!(pac::ETH.phy_cr().read().0, 0);
    unsafe {
        for index in 0..2 {
            let rx = fixture.descriptor(true, index);
            assert_eq!(rx.read_volatile(), 1 << 31);
            assert_eq!(rx.add(1).read_volatile(), (1 << 14) | 1536);
            assert_eq!(rx.add(2).read_volatile(), fixture.buffer(true, index) as usize as u32);
            assert_eq!(
                rx.add(3).read_volatile(),
                fixture.descriptor(true, 1 - index) as usize as u32
            );
        }
    }
    let writes = model::WRITES.lock().unwrap();
    assert!(writes
        .iter()
        .any(|&(offset, value)| offset == 0x80 && value == (1 << 31) | 0x81));
    assert!(writes
        .iter()
        .filter(|&&(offset, _)| offset == 0x1000)
        .all(|&(_, value)| value & !0x7d == 0));
    assert!(writes
        .iter()
        .filter(|&&(offset, _)| offset == 0x101c)
        .all(|&(_, value)| value & !0x1cfff == 0));
    for (offset, allowed) in [(0x1018, 0x0430_20c2), (0x10c, 0x0002_0060), (0x110, 0x0020_c000)] {
        assert!(writes
            .iter()
            .filter(|&&(address, _)| address == offset)
            .all(|&(_, value)| value & !allowed == 0));
    }
}

#[test]
fn transmit_tokens_pad_short_frames_and_release_ownership_after_callback() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    let descriptor = fixture.descriptor(false, 0);
    let buffer = fixture.buffer(false, 0);
    let token = fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .unwrap();
    token.consume(42, |bytes| {
        assert_eq!(unsafe { descriptor.read_volatile() } & (1 << 31), 0);
        bytes.fill(0xa5);
    });
    unsafe {
        assert_eq!(
            descriptor.read_volatile(),
            (1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20)
        );
        assert_eq!(descriptor.add(1).read_volatile(), 60);
        assert_eq!(core::slice::from_raw_parts(buffer, 42), [0xa5; 42]);
        assert_eq!(core::slice::from_raw_parts(buffer.add(42), 18), [0; 18]);
    }
    fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .unwrap()
        .consume(1514, |bytes| bytes.fill(1));
    assert!(fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .is_none());
}

#[test]
fn receive_strips_fcs_and_drop_returns_descriptor_even_without_consume() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    fixture.received(0, (64 << 16) | 0x300);
    let descriptor = fixture.descriptor(true, 0);
    let (rx, tx) = fixture.driver.receive(&mut Context::from_waker(Waker::noop())).unwrap();
    drop(tx);
    rx.consume(|bytes| {
        assert_eq!(bytes, [0x37; 60]);
        assert_eq!(unsafe { descriptor.read_volatile() } & (1 << 31), 0);
    });
    assert_eq!(unsafe { descriptor.read_volatile() }, 1 << 31);
    fixture.received(1, (1518 << 16) | 0x300);
    drop(fixture.driver.receive(&mut Context::from_waker(Waker::noop())).unwrap());
    assert_eq!(unsafe { fixture.descriptor(true, 1).read_volatile() }, 1 << 31);
}

#[test]
fn malformed_and_oversized_frames_are_recycled_without_exposing_buffers() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    for status in [
        (64 << 16) | 0x8300,
        (64 << 16) | 0x100,
        (17 << 16) | 0x300,
        (1519 << 16) | 0x300,
    ] {
        fixture.received(0, status);
        fixture.received(1, status);
        assert!(fixture
            .driver
            .receive(&mut Context::from_waker(Waker::noop()))
            .is_none());
        assert_eq!(unsafe { fixture.descriptor(true, 0).read_volatile() }, 1 << 31);
    }
    assert_eq!(fixture.driver.error_counts().1, 8);
}

#[test]
fn interrupts_wake_waiters_and_phy_status_uses_bit_eleven() {
    model::reset();
    let mut fixture = Fixture::new();
    let counter = Arc::new(WakeCount(AtomicU32::new(0)));
    let waker = Waker::from(counter.clone());
    assert!(fixture.driver.transmit(&mut Context::from_waker(&waker)).is_none());
    model::link(true, false, false);
    model::irq((1 << 11) | (1 << 16));
    assert!(counter.0.load(Ordering::Relaxed) > 0);
    assert!(!pac::ETH.dmaier().read().phylink_ier());
    assert!(fixture.driver.link_state(&mut Context::from_waker(&waker)) == LinkState::Up);
    assert!(!fixture.driver.link().speed_100m && !fixture.driver.link().full_duplex);
    assert!(pac::ETH.dmaier().read().phylink_ier());
    model::link(false, false, false);
    model::irq(1 << 11);
    assert!(fixture.driver.link_state(&mut Context::from_waker(&waker)) == LinkState::Down);
}

#[test]
fn stopped_dma_requires_explicit_recovery_and_drop_disables_the_clock() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    model::irq(1 << 8);
    assert!(fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .is_none());
    assert_eq!(fixture.driver.last_error(), Some(Error::DmaStopped));
    fixture.driver.recover().unwrap();
    assert!(fixture.driver.link_state(&mut Context::from_waker(Waker::noop())) == LinkState::Up);
    drop(fixture);
    assert!(!model::CLOCK.load(Ordering::Relaxed));
    assert!(!model::IRQ_ENABLED.load(Ordering::Relaxed));
    unsafe {
        <crate::eth::InterruptHandler as crate::interrupt::typelevel::Handler<crate::interrupt::typelevel::ETH>>::on_interrupt();
    }
}

#[test]
fn invalid_configuration_and_clock_do_not_touch_the_mac() {
    for config in [
        Config {
            phy_address: 32,
            ..Default::default()
        },
        Config {
            mac_address: [1; 6],
            ..Default::default()
        },
        Config {
            poll_limit: 0,
            ..Default::default()
        },
    ] {
        model::reset();
        let resources = Box::leak(Box::new(Resources::<2, 2>::new()));
        let result = Ethernet::new(
            unsafe { Peri::new_unchecked(peripherals::ETH) },
            Irqs,
            resources,
            config,
        );
        assert!(matches!(result, Err(Error::InvalidConfig)));
        assert!(!model::CLOCK.load(Ordering::Relaxed));
        assert!(model::WRITES.lock().unwrap().is_empty());
    }
    model::reset();
    model::CLOCK_HZ.store(250_000_000, Ordering::Relaxed);
    let result = Ethernet::new(
        unsafe { Peri::new_unchecked(peripherals::ETH) },
        Irqs,
        Box::leak(Box::new(Resources::<2, 2>::new())),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::BadClock)));
    assert!(model::WRITES.lock().unwrap().is_empty());
}

#[test]
fn hardware_timeouts_and_missing_phy_are_bounded_and_cleaned_up() {
    for (flag, expected) in [
        (&model::DMA_RESET_STUCK, Error::ResetTimeout),
        (&model::SMI_STUCK, Error::SmiTimeout),
        (&model::PHY_RESET_STUCK, Error::ResetTimeout),
        (&model::PHY_MISSING, Error::PhyNotFound),
    ] {
        model::reset();
        flag.store(true, Ordering::Relaxed);
        let result = Ethernet::new(
            unsafe { Peri::new_unchecked(peripherals::ETH) },
            Irqs,
            Box::leak(Box::new(Resources::<2, 2>::new())),
            Config {
                poll_limit: 8,
                ..Default::default()
            },
        );
        assert!(matches!(result, Err(error) if error == expected));
        assert!(!model::CLOCK.load(Ordering::Relaxed));
    }
}

#[test]
fn unused_empty_and_panicking_transmit_tokens_do_not_publish_descriptors() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    let descriptor = fixture.descriptor(false, 0);
    drop(
        fixture
            .driver
            .transmit(&mut Context::from_waker(Waker::noop()))
            .unwrap(),
    );
    fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .unwrap()
        .consume(0, |_| ());
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        fixture
            .driver
            .transmit(&mut Context::from_waker(Waker::noop()))
            .unwrap()
            .consume(64, |_| panic!("callback"));
    }))
    .is_err());
    assert_eq!(unsafe { descriptor.read_volatile() }, 0);
    fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .unwrap()
        .consume(64, |bytes| bytes.fill(7));
    assert_ne!(unsafe { descriptor.read_volatile() } & (1 << 31), 0);
}

#[test]
fn receive_callback_unwind_returns_ownership_to_dma() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    fixture.received(0, (64 << 16) | 0x300);
    let descriptor = fixture.descriptor(true, 0);
    assert!(std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let (rx, _) = fixture.driver.receive(&mut Context::from_waker(Waker::noop())).unwrap();
        rx.consume(|_| panic!("callback"));
    }))
    .is_err());
    assert_eq!(unsafe { descriptor.read_volatile() }, 1 << 31);
}

#[test]
fn full_tx_ring_keeps_received_frame_until_completion_wakes_the_waiter() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    for _ in 0..2 {
        fixture
            .driver
            .transmit(&mut Context::from_waker(Waker::noop()))
            .unwrap()
            .consume(64, |bytes| bytes.fill(0));
    }
    fixture.received(0, (64 << 16) | 0x300);
    let counter = Arc::new(WakeCount(AtomicU32::new(0)));
    let waker = Waker::from(counter.clone());
    assert!(fixture.driver.receive(&mut Context::from_waker(&waker)).is_none());
    assert_eq!(
        unsafe { fixture.descriptor(true, 0).read_volatile() },
        (64 << 16) | 0x300
    );
    unsafe {
        fixture.descriptor(false, 0).write_volatile(0);
    }
    model::irq((1 << 16) | 1);
    assert!(counter.0.load(Ordering::Relaxed) > 0);
    drop(fixture.driver.receive(&mut Context::from_waker(&waker)).unwrap());
    assert_eq!(unsafe { fixture.descriptor(true, 0).read_volatile() }, 1 << 31);
}

#[test]
fn link_changes_clear_only_the_intentional_dma_stop_events() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    model::link(false, false, false);
    model::irq(1 << 11);
    assert!(fixture.driver.link_state(&mut Context::from_waker(Waker::noop())) == LinkState::Down);
    assert_eq!(fixture.driver.last_error(), None);
    assert_eq!(pac::ETH.dmasr().read().0 & 0x102, 0);
    fixture.up();
    assert!(fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .is_some());
    assert_eq!(fixture.driver.last_error(), None);
}

#[test]
fn runtime_smi_failure_and_failed_recovery_leave_a_quiet_recoverable_driver() {
    model::reset();
    let mut fixture = Fixture::new();
    fixture.up();
    model::SMI_STUCK.store(true, Ordering::Relaxed);
    model::irq(1 << 11);
    assert!(fixture.driver.link_state(&mut Context::from_waker(Waker::noop())) == LinkState::Down);
    assert_eq!(fixture.driver.last_error(), Some(Error::SmiTimeout));
    assert_eq!(pac::ETH.dmaier().read().0, 0);
    assert_eq!(fixture.driver.recover(), Err(Error::SmiTimeout));
    assert!(!model::CLOCK.load(Ordering::Relaxed));
    assert!(fixture
        .driver
        .transmit(&mut Context::from_waker(Waker::noop()))
        .is_none());
    model::SMI_STUCK.store(false, Ordering::Relaxed);
    fixture.driver.recover().unwrap();
    assert!(fixture.driver.link_state(&mut Context::from_waker(Waker::noop())) == LinkState::Up);
}

#[test]
fn mdc_dividers_match_all_documented_hclk_ranges() {
    for (frequency, setting, divisor) in [
        (20_000_000, 2, 16),
        (34_999_999, 2, 16),
        (35_000_000, 3, 26),
        (59_999_999, 3, 26),
        (60_000_000, 0, 42),
        (89_999_999, 0, 42),
        (90_000_000, 1, 62),
        (119_999_999, 1, 62),
        (120_000_000, 5, 74),
        (149_999_999, 5, 74),
        (150_000_000, 4, 102),
        (200_000_000, 4, 102),
    ] {
        model::reset();
        model::CLOCK_HZ.store(frequency, Ordering::Relaxed);
        let fixture = Fixture::new();
        assert_eq!(pac::ETH.macmiiar().read().cr(), setting);
        assert!(frequency <= divisor * 2_500_000);
        drop(fixture);
    }
}

#[test]
fn zero_sized_rings_and_missing_pll_are_rejected_before_register_writes() {
    model::reset();
    let result = Ethernet::new(
        unsafe { Peri::new_unchecked(peripherals::ETH) },
        Irqs,
        Box::leak(Box::new(Resources::<0, 2>::new())),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::InvalidConfig)));
    let result = Ethernet::new(
        unsafe { Peri::new_unchecked(peripherals::ETH) },
        Irqs,
        Box::leak(Box::new(Resources::<2, 0>::new())),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::InvalidConfig)));
    unsafe {
        pac::RCC_WORDS[0] = 0;
    }
    let result = Ethernet::new(
        unsafe { Peri::new_unchecked(peripherals::ETH) },
        Irqs,
        Box::leak(Box::new(Resources::<2, 2>::new())),
        Config::default(),
    );
    assert!(matches!(result, Err(Error::BadClock)));
    assert!(model::WRITES.lock().unwrap().is_empty());
}

#[test]
fn phy_powerup_can_take_several_smi_reads_before_identification() {
    model::reset();
    model::ID_POLLS.store(3, Ordering::Relaxed);
    let fixture = Fixture::new();
    assert_eq!(fixture.driver.phy_id(), 0x73719011);
    assert_eq!(model::ID_POLLS.load(Ordering::Relaxed), 0);
}
