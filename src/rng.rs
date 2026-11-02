//! Hardware random number generator with blocking and interrupt-driven reads.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::num::NonZeroU32;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;
use rand_core::{CryptoRng, RngCore};

use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peri};

#[cfg(test)]
#[path = "rng/tests.rs"]
mod tests;

static RNG_WAKER: AtomicWaker = AtomicWaker::new();

/// RNG hardware failure. Retry or reset explicitly after inspecting the cause.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Noise source or seed failure; the peripheral has been restarted.
    SeedError,
    /// The selected RNG clock is unavailable or has failed its clock check.
    ClockError,
}

impl From<Error> for rand_core::Error {
    fn from(error: Error) -> Self {
        let code = match error {
            Error::SeedError => 0,
            Error::ClockError => 1,
        };
        NonZeroU32::new(rand_core::Error::CUSTOM_START + code).unwrap().into()
    }
}

fn ready_or_error(status: &pac::rng::regs::Sr) -> bool {
    status.drdy() || status.secs() || status.seis() || status.cecs() || status.ceis()
}

/// RNG interrupt handler.
pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        if ready_or_error(&T::regs().sr().read()) {
            T::regs().cr().modify(|reg| reg.set_ie(false));
            RNG_WAKER.wake();
        }
    }
}

/// Random number generator. Cancellation disables its peripheral interrupt.
pub struct Rng<'d, T: Instance> {
    _inner: Peri<'d, T>,
    discard_first: bool,
}

impl<'d, T: Instance> Rng<'d, T> {
    /// Enable the generator. This constructor does not wait for entropy.
    pub fn new(
        inner: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        #[cfg(ch32v4)]
        assert!(
            crate::rcc::rng_source_frequency().is_some(),
            "RNG source clock is not ready"
        );
        T::enable_and_reset();
        let mut random = Self {
            _inner: inner,
            discard_first: true,
        };
        random.reset();
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };
        random
    }

    /// Restart the generator and discard its first ready word on the next read.
    ///
    /// This does not wait for clock or seed faults to disappear.
    pub fn reset(&mut self) {
        T::regs().cr().write(|reg| reg.set_rngen(false));
        T::regs().sr().write_value(pac::rng::regs::Sr(0));
        self.discard_first = true;
        T::regs().cr().modify(|reg| reg.set_rngen(true));
    }

    /// Restart after a seed failure. Subsequent reads report persistent errors.
    pub fn recover_seed_error(&mut self) {
        self.reset();
    }

    fn check_status(&mut self, status: &pac::rng::regs::Sr) -> Result<(), Error> {
        if status.seis() || status.secs() {
            self.reset();
            return Err(Error::SeedError);
        }
        if status.ceis() || status.cecs() {
            T::regs().sr().write_value(pac::rng::regs::Sr(1 << 6));
            return Err(Error::ClockError);
        }
        Ok(())
    }

    /// Read one available word without waiting. None means more entropy is needed.
    pub fn try_next_u32(&mut self) -> Result<Option<u32>, Error> {
        let status = T::regs().sr().read();
        self.check_status(&status)?;
        if !status.drdy() {
            return Ok(None);
        }
        let value = T::regs().dr().read();
        self.check_status(&T::regs().sr().read())?;
        if self.discard_first {
            self.discard_first = false;
            return Ok(None);
        }
        Ok(Some(value))
    }

    /// Wait for a random word, reporting hardware errors instead of retrying them.
    pub fn blocking_next_u32(&mut self) -> Result<u32, Error> {
        loop {
            if let Some(value) = self.try_next_u32()? {
                return Ok(value);
            }
        }
    }

    /// Wait for a word using the RNG interrupt. Errors take priority over data.
    pub async fn async_next_u32(&mut self) -> Result<u32, Error> {
        let _cleanup = OnDrop::new(|| T::regs().cr().modify(|reg| reg.set_ie(false)));
        poll_fn(|context| {
            RNG_WAKER.register(context.waker());
            match self.try_next_u32() {
                Ok(Some(value)) => return Poll::Ready(Ok(value)),
                Err(error) => return Poll::Ready(Err(error)),
                Ok(None) => {}
            }
            T::regs().cr().modify(|reg| reg.set_ie(true));
            match self.try_next_u32() {
                Ok(Some(value)) => Poll::Ready(Ok(value)),
                Err(error) => Poll::Ready(Err(error)),
                Ok(None) => Poll::Pending,
            }
        })
        .await
    }

    /// Fill bytes, returning a hardware error if generation fails.
    ///
    /// On failure or cancellation an already filled prefix is retained.
    pub async fn async_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Error> {
        for chunk in dest.chunks_mut(4) {
            let word = self.async_next_u32().await?;
            chunk.copy_from_slice(&word.to_le_bytes()[..chunk.len()]);
        }
        Ok(())
    }
}

impl<'d, T: Instance> Drop for Rng<'d, T> {
    fn drop(&mut self) {
        T::regs().cr().write(|reg| {
            reg.set_ie(false);
            reg.set_rngen(false);
        });
        T::Interrupt::disable();
        T::disable();
    }
}

impl<'d, T: Instance> RngCore for Rng<'d, T> {
    fn next_u32(&mut self) -> u32 {
        self.blocking_next_u32().expect("RNG hardware failure")
    }

    fn next_u64(&mut self) -> u64 {
        u64::from(self.next_u32()) | (u64::from(self.next_u32()) << 32)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.try_fill_bytes(dest).expect("RNG hardware failure");
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        for chunk in dest.chunks_mut(4) {
            let word = self.blocking_next_u32()?;
            chunk.copy_from_slice(&word.to_le_bytes()[..chunk.len()]);
        }
        Ok(())
    }
}

impl<'d, T: Instance> CryptoRng for Rng<'d, T> {}

trait SealedInstance {
    fn regs() -> pac::rng::Rng;
}

/// RNG instance trait.
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + embassy_hal_internal::PeripheralType + crate::peripheral::RccPeripheral + 'static + Send
{
    /// RNG interrupt.
    type Interrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (rng, $inst:ident) => {
        impl Instance for peripherals::$inst {
            type Interrupt = crate::_generated::peripheral_interrupts::$inst::GLOBAL;
        }

        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::rng::Rng { crate::pac::$inst }
        }
    };
);
