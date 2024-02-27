use core::future::Future;
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::{Context, Poll};

use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{AnyPin, Input, Level, Pin as GpioPin};
use crate::{impl_peripheral, pac, peripherals, Peripheral};

const EXTI_COUNT: usize = 24;
const NEW_AW: AtomicWaker = AtomicWaker::new();
static EXTI_WAKERS: [AtomicWaker; EXTI_COUNT] = [NEW_AW; EXTI_COUNT];

pub unsafe fn on_irq() {
    let exti = unsafe { &*pac::EXTI::PTR };

    let bits = exti.intfr().read().bits();

    // We don't handle or change any EXTI lines above 16.
    let bits = bits & 0x0000FFFF;

    exti.intenr().modify(|r, w| unsafe { w.bits(r.bits() & !bits) });

    // Wake the tasks
    for pin in BitIter(bits) {
        EXTI_WAKERS[pin as usize].wake();
    }

    // Clear pending - Clears the EXTI's line pending bits.
    exti.intfr().write(|w| w.bits(bits)); // write 1 to clear
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

#[no_mangle]
unsafe extern "C" fn EXTI0() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI1() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI2() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI3() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI4() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI9_5() {
    on_irq();
}
#[no_mangle]
unsafe extern "C" fn EXTI15_10() {
    on_irq();
}

/// EXTI input driver
pub struct ExtiInput<'d, T: GpioPin> {
    pin: Input<'d, T>,
}

impl<'d, T: GpioPin> Unpin for ExtiInput<'d, T> {}

impl<'d, T: GpioPin> ExtiInput<'d, T> {
    pub fn new(pin: Input<'d, T>, _ch: impl Peripheral<P = T::ExtiChannel> + 'd) -> Self {
        Self { pin }
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
            let exti = unsafe { &*pac::EXTI::PTR };
            let afio = unsafe { &*pac::AFIO::PTR };

            let port = port as u32;

            // AFIO_EXTICRx
            if pin < 16 {
                let shift = pin * 2;
                afio.exticr1()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << shift) | port << shift) });
            } else {
                let shift = (pin - 16) * 2;
                afio.exticr2()
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << shift) | port << shift) });
            }

            // See-also: 7.4.3
            exti.intenr().modify(|r, w| unsafe { w.bits(r.bits() | 1 << pin) });

            exti.rtenr()
                .modify(|r, w| unsafe { w.bits(r.bits() | (rising as u32) << pin) });
            exti.ftenr()
                .modify(|r, w| unsafe { w.bits(r.bits() | (falling as u32) << pin) });
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
            let pin = self.pin;
            let exti = unsafe { &*pac::EXTI::PTR };
            exti.intenr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin)) });
        });
    }
}

impl<'a> Future for ExtiInputFuture<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let exti = unsafe { &*pac::EXTI::PTR };

        EXTI_WAKERS[self.pin as usize].register(cx.waker());

        if exti.intenr().read().bits() & (1 << self.pin) == 0 {
            // intenr cleared by on_irq, then we can assume it is triggered
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

pub(crate) mod sealed {
    pub trait Channel {}
}

pub trait Channel: sealed::Channel + Sized {
    fn number(&self) -> usize;
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
impl sealed::Channel for AnyChannel {}
impl Channel for AnyChannel {
    fn number(&self) -> usize {
        self.number as usize
    }
}

macro_rules! impl_exti {
    ($type:ident, $number:expr) => {
        impl sealed::Channel for peripherals::$type {}
        impl Channel for peripherals::$type {
            fn number(&self) -> usize {
                $number as usize
            }
        }
    };
}

impl_exti!(EXTI0, 0);
impl_exti!(EXTI1, 1);
impl_exti!(EXTI2, 2);
impl_exti!(EXTI3, 3);
impl_exti!(EXTI4, 4);
impl_exti!(EXTI5, 5);
impl_exti!(EXTI6, 6);
impl_exti!(EXTI7, 7);
impl_exti!(EXTI8, 8);
impl_exti!(EXTI9, 9);
impl_exti!(EXTI10, 10);
impl_exti!(EXTI11, 11);
impl_exti!(EXTI12, 12);
impl_exti!(EXTI13, 13);
impl_exti!(EXTI14, 14);
impl_exti!(EXTI15, 15);

/// safety: must be called only once
pub(crate) unsafe fn init(_cs: critical_section::CriticalSection) {
    use crate::pac::Interrupt;
    qingke::pfic::enable_interrupt(Interrupt::EXTI0 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI1 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI2 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI3 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI4 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI9_5 as u8);
    qingke::pfic::enable_interrupt(Interrupt::EXTI15_10 as u8);
}
