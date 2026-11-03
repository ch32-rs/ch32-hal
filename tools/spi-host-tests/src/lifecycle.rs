use core::future::Future;
use core::task::{Context, Poll};

use super::*;
use crate::pac;
use crate::spi_model as model;

impl SckPin<peripherals::SPI1, 0> for crate::gpio::AnyPin {}
impl MosiPin<peripherals::SPI1, 0> for crate::gpio::AnyPin {}

fn driver() -> Spi<'static, peripherals::SPI1, Blocking> {
    model::reset();
    Spi::new_blocking_txonly::<0>(
        unsafe { Peri::new_unchecked(peripherals::SPI1) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        Config::default(),
    )
}

struct IrqBinding;
unsafe impl interrupt::typelevel::Binding<
    crate::interrupt::typelevel::SPI1_GLOBAL,
    InterruptHandler<peripherals::SPI1>,
> for IrqBinding
{
}
impl TxDma<peripherals::SPI1> for crate::dma::AnyChannel {
    fn request(&self) {}
}

/// The async `flush` lives on `Spi<_, Async>`. The DMA channel is never used
/// by `flush`; it only satisfies the constructor.
fn async_driver() -> Spi<'static, peripherals::SPI1, Async> {
    model::reset();
    Spi::new_txonly::<0>(
        unsafe { Peri::new_unchecked(peripherals::SPI1) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        unsafe { Peri::new_unchecked(crate::gpio::AnyPin) },
        IrqBinding,
        unsafe { Peri::new_unchecked(crate::dma::AnyChannel { id: 0 }) },
        Config::default(),
    )
}

fn poll_once<F: Future>(future: F) -> Poll<F::Output> {
    let mut future = core::pin::pin!(future);
    let mut cx = Context::from_waker(std::task::Waker::noop());
    future.as_mut().poll(&mut cx)
}

/// Regression for the `hal_spi` hang measured on CH32V467: the constructor
/// leaves `SPE=0`, where `STATR.TXE` reads 0. `blocking_flush` must not poll
/// the status register in that state.
#[test]
fn blocking_flush_is_noop_while_disabled() {
    let mut spi = driver();
    assert!(!pac::SPI1.ctlr1().read().spe());
    let before = model::statr_reads();
    assert_eq!(spi.blocking_flush(), Ok(()));
    assert_eq!(model::statr_reads(), before);
}

/// The first `blocking_write` goes through `prepare()` -> `blocking_flush()`
/// before it enables `SPE`; this is the exact path that used to deadlock.
#[test]
fn blocking_write_starts_with_a_disabled_peripheral() {
    let mut spi = driver();
    assert!(!pac::SPI1.ctlr1().read().spe());
    assert_eq!(spi.blocking_write(&[0xa5u8]), Ok(()));
}

/// Once enabled, `blocking_flush` still waits for `TXE && !BSY`.
#[test]
fn blocking_flush_waits_when_enabled() {
    let mut spi = driver();
    pac::SPI1.ctlr1().modify(|w| w.set_spe(true));
    let before = model::statr_reads();
    assert_eq!(spi.blocking_flush(), Ok(()));
    assert!(model::statr_reads() > before);
}

/// The async `flush` has the same boundary as the blocking one.
#[test]
fn async_flush_is_ready_while_disabled() {
    let mut spi = async_driver();
    assert!(!pac::SPI1.ctlr1().read().spe());
    assert_eq!(poll_once(spi.flush()), Poll::Ready(Ok(())));
}

/// Enabled and busy: the async flush must stay pending until `BSY` clears.
#[test]
fn async_flush_polls_status_when_enabled() {
    let mut spi = async_driver();
    pac::SPI1.ctlr1().modify(|w| w.set_spe(true));
    model::set_busy(true);
    let mut future = core::pin::pin!(spi.flush());
    let mut cx = Context::from_waker(std::task::Waker::noop());
    assert!(future.as_mut().poll(&mut cx).is_pending());
    model::set_busy(false);
    assert_eq!(future.as_mut().poll(&mut cx), Poll::Ready(Ok(())));
}
