//! CH32V407/CH32V467 independent-mode ADC1/ADC2 with blocking, interrupt, and
//! regular scan DMA reads.
#![macro_use]

use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use crate::dma::{Transfer, TransferOptions};
use crate::internal::drop::OnDrop;
use crate::interrupt::typelevel::Interrupt;
use crate::{interrupt, pac, peripherals, Peri};

pub const ADC_MAX: u32 = 4095;
/// Nominal internal reference in mV; this is not a factory calibration value.
pub const VREF_INT: u32 = 1200;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum SampleTime {
    S1_5 = 0,
    S7_5 = 1,
    S13_5 = 2,
    S28_5 = 3,
    S41_5 = 4,
    S55_5 = 5,
    S71_5 = 6,
    #[default]
    S239_5 = 7,
    S2_5 = 12,
    S3_5 = 13,
    S4_5 = 14,
    S5_5 = 15,
}

impl SampleTime {
    pub const fn cycles_x2(self) -> u16 {
        match self {
            Self::S1_5 => 3,
            Self::S7_5 => 15,
            Self::S13_5 => 27,
            Self::S28_5 => 57,
            Self::S41_5 => 83,
            Self::S55_5 => 111,
            Self::S71_5 => 143,
            Self::S239_5 => 479,
            Self::S2_5 => 5,
            Self::S3_5 => 7,
            Self::S4_5 => 9,
            Self::S5_5 => 11,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    ClockUnavailable,
    ClockTooFast,
    CalibrationTimeout,
    ConversionTimeout,
    InvalidSequence,
    InvalidBufferLength,
    /// Conversion would exceed 2 MS/s, or temperature acquisition is below 17.1 us.
    SampleTimeTooShort,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Maximum register polls for each calibration phase or blocking conversion.
    pub poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { poll_limit: 1_000_000 }
    }
}

pub struct State {
    waker: AtomicWaker,
}

/// Shared ADC1_2 vector handler. It never consumes RDATAR or interferes with DMA.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::ADC1_2> for InterruptHandler {
    unsafe fn on_interrupt() {
        on_interrupt::<peripherals::ADC1>();
        on_interrupt::<peripherals::ADC2>();
    }
}

fn on_interrupt<T: Instance>() {
    if T::regs().ctlr1().read().eocie() && T::regs().statr().read().eoc() {
        T::regs().ctlr1().modify(|reg| reg.set_eocie(false));
        T::state().waker.wake();
    }
}

/// One scan rank, borrowing its pin or internal channel for the sequence lifetime.
pub struct Channel<'a, T: Instance> {
    number: u8,
    sample_time: SampleTime,
    _borrow: PhantomData<&'a mut T>,
}

impl<'a, T: Instance> Channel<'a, T> {
    pub fn new(channel: &'a mut impl AdcChannel<T>, sample_time: SampleTime) -> Self {
        channel.set_as_analog();
        Self {
            number: channel.channel(),
            sample_time,
            _borrow: PhantomData,
        }
    }
}

/// Clock selection is shared by ADC1/ADC2 and configured through RCC Config.
pub struct Adc<'d, T: Instance> {
    _inner: Peri<'d, T>,
    config: Config,
    clock_source: u32,
    clock_divisor: u32,
}

impl<'d, T: Instance> Adc<'d, T> {
    /// Enable and calibrate an ADC. Failures leave its conversion and clock off.
    pub fn new(inner: Peri<'d, T>, config: Config) -> Result<Self, Error> {
        let (source, divisor) = crate::rcc::adc_clock_ratio().ok_or(Error::ClockUnavailable)?;
        if source == 0 || divisor == 0 || source / divisor == 0 {
            return Err(Error::ClockUnavailable);
        }
        if u64::from(source) > 30_000_000 * u64::from(divisor) {
            return Err(Error::ClockTooFast);
        }
        T::enable_and_reset();
        let mut adc = Self {
            _inner: inner,
            config,
            clock_source: source,
            clock_divisor: divisor,
        };
        adc.calibrate()?;
        Ok(adc)
    }

    fn stabilize(&self) {
        let hclk = u64::from(crate::rcc::clocks().hclk.0);
        let two_adc_cycles = (2 * hclk * u64::from(self.clock_divisor)).div_ceil(u64::from(self.clock_source));
        let cycles = two_adc_cycles.max(hclk.div_ceil(50_000)).max(1);
        qingke::riscv::asm::delay((cycles * 4).min(u64::from(u32::MAX)) as u32);
    }

    pub fn calibrate(&mut self) -> Result<(), Error> {
        Self::stop();
        T::regs().ctlr1().write(|_| {});
        T::regs().ctlr2().write(|reg| reg.set_adon(true));
        self.stabilize();
        T::regs().ctlr2().modify(|reg| reg.set_rstcal(true));
        let reset = self.wait_for(|| !T::regs().ctlr2().read().rstcal());
        if reset {
            T::regs().ctlr2().modify(|reg| reg.set_cal(true));
        }
        let calibrated = reset && self.wait_for(|| !T::regs().ctlr2().read().cal());
        Self::stop();
        if calibrated {
            Ok(())
        } else {
            Err(Error::CalibrationTimeout)
        }
    }

    fn wait_for(&self, mut ready: impl FnMut() -> bool) -> bool {
        for _poll in 0..self.config.poll_limit {
            if ready() {
                return true;
            }
        }
        false
    }

    fn prepare(&mut self, sequence: &[Channel<'_, T>], continuous: bool, dma: bool) -> Result<(), Error> {
        validate_sequence(sequence, self.clock_source, self.clock_divisor)?;
        Self::stop();
        let regs = T::regs();
        regs.rsqr1().write(|reg| reg.set_l(sequence.len() as u8 - 1));
        regs.rsqr2().write(|_| {});
        regs.rsqr3().write(|_| {});
        for (rank, channel) in sequence.iter().enumerate() {
            let (sample_shift, bits, extended) = sample_bits(channel.number, channel.sample_time);
            if channel.number < 10 {
                regs.samptr2()
                    .modify(|reg| reg.0 = (reg.0 & !(7 << sample_shift)) | bits);
            } else {
                regs.samptr1()
                    .modify(|reg| reg.0 = (reg.0 & !(7 << sample_shift)) | bits);
            }
            regs.aux().modify(|reg| {
                let mask = 1 << channel.number;
                reg.0 = (reg.0 & !mask) | if extended { mask } else { 0 };
            });
            let shift = (rank % 6) * 5;
            let bits = u32::from(channel.number) << shift;
            match rank / 6 {
                0 => regs.rsqr3().modify(|reg| reg.0 |= bits),
                1 => regs.rsqr2().modify(|reg| reg.0 |= bits),
                _ => regs.rsqr1().modify(|reg| reg.0 |= bits),
            }
        }
        regs.ctlr1().write(|reg| reg.set_scan(sequence.len() > 1));
        regs.ctlr2().write(|reg| {
            reg.set_extsel(7);
            reg.set_exttrig(true);
            reg.set_tsvrefe(sequence.iter().any(|channel| channel.number >= 16));
            reg.set_cont(continuous);
            reg.set_dma(dma);
            reg.set_adon(true);
        });
        self.stabilize();
        regs.statr().write_value(pac::adc::regs::Statr(0));
        Ok(())
    }

    fn start() {
        T::regs().ctlr2().modify(|reg| reg.set_swstart(true));
    }

    fn stop() {
        T::regs().ctlr1().modify(|reg| reg.set_eocie(false));
        T::regs().ctlr2().write(|_| {});
        T::regs().statr().write_value(pac::adc::regs::Statr(0));
    }

    pub fn blocking_read(&mut self, channel: &mut impl AdcChannel<T>, sample_time: SampleTime) -> Result<u16, Error> {
        self.prepare(&[Channel::new(channel, sample_time)], false, false)?;
        let _cleanup = OnDrop::new(Self::stop);
        Self::start();
        if !self.wait_for(|| T::regs().statr().read().eoc()) {
            return Err(Error::ConversionTimeout);
        }
        Ok(T::regs().rdatar().read().data())
    }

    /// Read one channel using the shared IRQ. Dropping the future aborts conversion.
    pub async fn read(
        &mut self,
        channel: &mut impl AdcChannel<T>,
        sample_time: SampleTime,
        _irq: impl interrupt::typelevel::Binding<interrupt::typelevel::ADC1_2, InterruptHandler>,
    ) -> Result<u16, Error> {
        self.prepare(&[Channel::new(channel, sample_time)], false, false)?;
        let _cleanup = OnDrop::new(Self::stop);
        unsafe { interrupt::typelevel::ADC1_2::enable() };
        Self::start();
        poll_fn(|context| {
            T::state().waker.register(context.waker());
            T::regs().ctlr1().modify(|reg| reg.set_eocie(true));
            if T::regs().statr().read().eoc() {
                Poll::Ready(Ok(T::regs().rdatar().read().data()))
            } else {
                Poll::Pending
            }
        })
        .await
    }

    /// Acquire complete scan frames. The buffer length must be a multiple of the
    /// sequence length, at most 65535 halfwords. Samples are right-aligned 12-bit.
    /// Cancellation stops conversion and DMA before releasing the buffer.
    pub async fn read_dma(
        &mut self,
        dma: Peri<'_, impl RxDma<T>>,
        sequence: &[Channel<'_, T>],
        buffer: &mut [u16],
    ) -> Result<(), Error> {
        validate_sequence(sequence, self.clock_source, self.clock_divisor)?;
        if buffer.len() > 65535 || buffer.len() % sequence.len() != 0 {
            return Err(Error::InvalidBufferLength);
        }
        if buffer.is_empty() {
            return Ok(());
        }
        self.prepare(sequence, buffer.len() > sequence.len(), false)?;
        let request = dma.request();
        let mut transfer = unsafe {
            Transfer::new_read(
                dma,
                request,
                T::regs().rdatar().as_ptr().cast::<u16>(),
                buffer,
                TransferOptions::default(),
            )
        };
        let _cleanup = OnDrop::new(Self::stop);
        T::regs().ctlr2().modify(|reg| reg.set_dma(true));
        Self::start();
        (&mut transfer).await;
        Ok(())
    }
}

impl<T: Instance> Drop for Adc<'_, T> {
    fn drop(&mut self) {
        Self::stop();
        T::disable();
    }
}

fn sample_bits(channel: u8, sample_time: SampleTime) -> (u32, u32, bool) {
    let shift = u32::from(if channel < 10 { channel } else { channel - 10 }) * 3;
    (
        shift,
        u32::from(sample_time as u8 & 7) << shift,
        sample_time as u8 & 8 != 0,
    )
}

fn validate_sequence<T: Instance>(sequence: &[Channel<'_, T>], source: u32, divisor: u32) -> Result<(), Error> {
    if sequence.is_empty() || sequence.len() > 16 {
        return Err(Error::InvalidSequence);
    }
    for (rank, channel) in sequence.iter().enumerate() {
        if channel.number > 17 || (channel.number >= 16 && T::regs().as_ptr() != pac::ADC1.as_ptr()) {
            return Err(Error::InvalidSequence);
        }
        if sequence[..rank]
            .iter()
            .any(|other| other.number == channel.number && other.sample_time != channel.sample_time)
        {
            return Err(Error::InvalidSequence);
        }
        if 2 * u64::from(source) > 2_000_000 * u64::from(divisor) * (u64::from(channel.sample_time.cycles_x2()) + 25) {
            return Err(Error::SampleTimeTooShort);
        }
        if channel.number == 16
            && u64::from(channel.sample_time.cycles_x2()) * 5_000_000 * u64::from(divisor) < 171 * u64::from(source)
        {
            return Err(Error::SampleTimeTooShort);
        }
    }
    Ok(())
}

trait SealedInstance {
    fn regs() -> pac::adc::Adc;
    fn state() -> &'static State;
}

#[allow(private_bounds)]
pub trait Instance: SealedInstance + embassy_hal_internal::PeripheralType + crate::peripheral::RccPeripheral {}

pub(crate) trait SealedAdcChannel<T: Instance> {
    fn set_as_analog(&mut self) {}
    fn channel(&self) -> u8;
}

#[allow(private_bounds)]
pub trait AdcChannel<T: Instance>: SealedAdcChannel<T> + Sized {
    fn degrade_adc(mut self) -> AnyAdcChannel<T> {
        self.set_as_analog();
        AnyAdcChannel {
            channel: self.channel(),
            _phantom: PhantomData,
        }
    }
}

pub struct AnyAdcChannel<T> {
    channel: u8,
    _phantom: PhantomData<T>,
}

impl<T: Instance> AdcChannel<T> for AnyAdcChannel<T> {}
impl<T: Instance> SealedAdcChannel<T> for AnyAdcChannel<T> {
    fn channel(&self) -> u8 {
        self.channel
    }
}

pub struct Vref {
    _private: (),
}
pub struct Temperature {
    _private: (),
}

impl Adc<'_, peripherals::ADC1> {
    pub fn enable_vref(&mut self) -> Vref {
        Vref { _private: () }
    }
    pub fn enable_temperature(&mut self) -> Temperature {
        Temperature { _private: () }
    }
}

impl AdcChannel<peripherals::ADC1> for Vref {}
impl SealedAdcChannel<peripherals::ADC1> for Vref {
    fn channel(&self) -> u8 {
        17
    }
}
impl AdcChannel<peripherals::ADC1> for Temperature {}
impl SealedAdcChannel<peripherals::ADC1> for Temperature {
    fn channel(&self) -> u8 {
        16
    }
}

foreach_peripheral! {
    (adc, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> pac::adc::Adc { pac::$inst }
            fn state() -> &'static State {
                static STATE: State = State { waker: AtomicWaker::new() };
                &STATE
            }
        }
        impl Instance for peripherals::$inst {}
    };
}

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $channel:expr) => {
        impl crate::adc::AdcChannel<peripherals::$inst> for peripherals::$pin {}
        impl crate::adc::SealedAdcChannel<peripherals::$inst> for peripherals::$pin {
            fn set_as_analog(&mut self) {
                <Self as crate::gpio::SealedPin>::set_as_analog(self);
            }
            fn channel(&self) -> u8 {
                $channel
            }
        }
        impl crate::adc::AdcChannel<peripherals::$inst> for crate::Peri<'_, peripherals::$pin> {}
        impl crate::adc::SealedAdcChannel<peripherals::$inst> for crate::Peri<'_, peripherals::$pin> {
            fn set_as_analog(&mut self) {
                <peripherals::$pin as crate::gpio::SealedPin>::set_as_analog(self);
            }
            fn channel(&self) -> u8 {
                $channel
            }
        }
    };
}

dma_trait!(RxDma, Instance);

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
