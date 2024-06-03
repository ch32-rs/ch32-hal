use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;
use qingke_rt::interrupt;

use crate::gpio::{AnyPin, Input, Level, Pin as GpioPin, Pull};
use crate::{impl_peripheral, into_ref, peripherals, Peripheral};

const EXTI_COUNT: usize = 24;
const NEW_AW: AtomicWaker = AtomicWaker::new();
static EXTI_WAKERS: [AtomicWaker; EXTI_COUNT] = [NEW_AW; EXTI_COUNT];

pub unsafe fn on_irq() {
    let exti = &crate::pac::EXTI;

    let bits = exti.intfr().read();

    // We don't handle or change any EXTI lines above 24.
    let bits = bits.0 & 0x00FFFFFF;

    // Clear pending - Clears the EXTI's line pending bits.
    exti.intfr().write(|w| w.0 = bits);

    exti.intenr().modify(|w| w.0 = w.0 & !bits);

    // Wake the tasks
    for pin in BitIter(bits) {
        EXTI_WAKERS[pin as usize].wake();
    }
}

struct BitIter(u32);

impl Iterator for BitIter {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b)
            }
        }
    }
}

/// EXTI input driver
pub struct ExtiInput<'d> {
    pin: Input<'d>,
}

impl<'d> Unpin for ExtiInput<'d> {}

impl<'d> ExtiInput<'d> {
    pub fn new<T: GpioPin>(
        pin: impl Peripheral<P = T> + 'd,
        ch: impl Peripheral<P = T::ExtiChannel> + 'd,
        pull: Pull,
    ) -> Self {
        into_ref!(pin, ch);
        // Needed if using AnyPin+AnyChannel.
        assert_eq!(pin.pin(), ch.number());

        Self {
            pin: Input::new(pin, pull),
        }
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    pub fn get_level(&self) -> Level {
        self.pin.get_level()
    }

    pub async fn wait_for_high<'a>(&'a mut self) {
        let fut = ExtiInputFuture::new(self.pin.pin.pin.pin(), self.pin.pin.pin.port(), true, false);
        if self.is_high() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_low<'a>(&'a mut self) {
        let fut = ExtiInputFuture::new(self.pin.pin.pin.pin(), self.pin.pin.pin.port(), false, true);
        if self.is_low() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_rising_edge<'a>(&'a mut self) {
        ExtiInputFuture::new(self.pin.pin.pin.pin(), self.pin.pin.pin.port(), true, false).await
    }

    pub async fn wait_for_falling_edge<'a>(&'a mut self) {
        ExtiInputFuture::new(self.pin.pin.pin.pin(), self.pin.pin.pin.port(), false, true).await
    }

    pub async fn wait_for_any_edge<'a>(&'a mut self) {
        ExtiInputFuture::new(self.pin.pin.pin.pin(), self.pin.pin.pin.port(), true, true).await
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct ExtiInputFuture<'a> {
    pin: u8,
    phantom: PhantomData<&'a mut AnyPin>,
}

// EXTI0-EXTI23 Px0-Px23（x=A/B/C）
impl<'a> ExtiInputFuture<'a> {
    fn new(pin: u8, port: u8, rising: bool, falling: bool) -> Self {
        critical_section::with(|_| {
            let exti = &crate::pac::EXTI;
            let afio = &crate::pac::AFIO;

            let port = port as u8;
            let pin = pin as usize;

            #[cfg(afio_v0)]
            {
                // AFIO_EXTICR
                // stride: 2, len: 15, 8 lines
                afio.exticr().modify(|w| w.set_exti(pin, port));
            }
            // V1, V2, V3, L1
            #[cfg(any(afio_v3, afio_l1))]
            {
                // AFIO_EXTICRx
                // stride: 4, len: 4, 16 lines
                afio.exticr(pin / 4).modify(|w| w.set_exti(pin % 4, port));
            }
            #[cfg(afio_x0)]
            {
                // stride: 2, len: 15, 24 lines
                afio.exticr(pin / 16).modify(|w| w.set_exti(pin % 16, port));
            }
            #[cfg(afio_ch641)]
            {
                // single register
                afio.exticr().modify(|w| w.set_exti(pin, port != 0));
            }

            // See-also: 7.4.3
            exti.intenr().modify(|w| w.set_mr(pin, true)); // enable interrupt

            exti.rtenr().modify(|w| w.set_tr(pin, rising));
            exti.ftenr().modify(|w| w.set_tr(pin, falling));
        });

        Self {
            pin,
            phantom: PhantomData,
        }
    }
}

impl<'a> Drop for ExtiInputFuture<'a> {
    fn drop(&mut self) {
        critical_section::with(|_| {
            let exti = &crate::pac::EXTI;
            let pin = self.pin;
            exti.intenr().modify(|w| w.0 = w.0 & !(1 << pin));
        });
    }
}

impl<'a> Future for ExtiInputFuture<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let exti = &crate::pac::EXTI;

        EXTI_WAKERS[self.pin as usize].register(cx.waker());

        if exti.intenr().read().mr(self.pin as _) == false {
            // intenr cleared by on_irq, then we can assume it is triggered
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

trait SealedChannel {}

#[allow(private_bounds)]
pub trait Channel: SealedChannel + Sized {
    fn number(&self) -> u8;
    fn degrade(self) -> AnyChannel {
        AnyChannel {
            number: self.number() as u8,
        }
    }
}

pub struct AnyChannel {
    number: u8,
}
impl_peripheral!(AnyChannel);
impl SealedChannel for AnyChannel {}
impl Channel for AnyChannel {
    fn number(&self) -> u8 {
        self.number
    }
}

macro_rules! impl_exti {
    ($type:ident, $number:expr) => {
        impl SealedChannel for peripherals::$type {}
        impl Channel for peripherals::$type {
            fn number(&self) -> u8 {
                $number
            }
        }
    };
}

mod _exti_8lines {
    use super::*;

    impl_exti!(EXTI0, 0);
    impl_exti!(EXTI1, 1);
    impl_exti!(EXTI2, 2);
    impl_exti!(EXTI3, 3);
    impl_exti!(EXTI4, 4);
    impl_exti!(EXTI5, 5);
    impl_exti!(EXTI6, 6);
    impl_exti!(EXTI7, 7);
}

#[cfg(not(any(ch32v0, ch643)))]
mod _exti_16lines {
    use super::*;

    impl_exti!(EXTI8, 8);
    impl_exti!(EXTI9, 9);
    impl_exti!(EXTI10, 10);
    impl_exti!(EXTI11, 11);
    impl_exti!(EXTI12, 12);
    impl_exti!(EXTI13, 13);
    impl_exti!(EXTI14, 14);
    impl_exti!(EXTI15, 15);
}

#[cfg(any(ch32x0, ch643))]
mod _exti_24lines {
    use super::*;

    impl_exti!(EXTI16, 16);
    impl_exti!(EXTI17, 17);
    impl_exti!(EXTI18, 18);
    impl_exti!(EXTI19, 19);
    impl_exti!(EXTI20, 20);
    impl_exti!(EXTI21, 21);
    impl_exti!(EXTI22, 22);
    impl_exti!(EXTI23, 23);
}

/*
EXTI0
EXTI1
EXTI2
EXTI3
EXTI4
EXTI9_5
EXTI15_10
EXTI7_0
EXTI15_8
EXTI25_16
*/

/// safety: must be called only once
#[cfg(gpio_x0)]
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use crate::pac::Interrupt;

    #[interrupt]
    unsafe fn EXTI7_0() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI15_8() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI25_16() {
        on_irq();
    }

    qingke::pfic::enable_interrupt(Interrupt::EXTI7_0 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI15_8 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI25_16 as u8);
}

#[cfg(all(gpio_v3, not(ch641)))]
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use qingke_rt::interrupt;

    use crate::pac::Interrupt;

    #[interrupt]
    unsafe fn EXTI0() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI1() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI2() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI3() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI4() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI9_5() {
        on_irq();
    }
    #[interrupt]
    unsafe fn EXTI15_10() {
        on_irq();
    }

    qingke::pfic::enable_interrupt(Interrupt::EXTI0 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI1 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI2 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI3 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI4 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI9_5 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI15_10 as u8);
}

#[cfg(gpio_v0)]
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use crate::pac::Interrupt;

    #[interrupt]
    unsafe fn EXTI7_0() {
        on_irq();
    }

    qingke::pfic::enable_interrupt(Interrupt::EXTI7_0 as u8);
}

#[cfg(all(gpio_v3, ch641))]
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use crate::pac::Interrupt;

    #[interrupt]
    unsafe fn EXTI7_0() {
        on_irq();
    }

    #[interrupt]
    unsafe fn EXTI15_8() {
        on_irq();
    }

    qingke::pfic::enable_interrupt(Interrupt::EXTI7_0 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI15_8 as u8);
}
