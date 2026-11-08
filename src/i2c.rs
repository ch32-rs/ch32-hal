//! Inter-Integrated-Circuit (I2C)

use core::future::{poll_fn, Future};
use core::marker::PhantomData;
use core::pin::Pin;
use core::task::Poll;

use embassy_futures::select::{select, Either};
use embassy_sync::waitqueue::AtomicWaker;
use embedded_hal::i2c::Operation;

use crate::dma::ChannelAndRequest;
use crate::gpio::{AfType, AnyPin, OutputType, SealedPin, Speed};
use crate::internal::drop::OnDrop;
use crate::mode::{Async, Blocking, Mode};
// use crate::interrupt::Interrupt;
use crate::time::Hertz;
use crate::{interrupt, peripherals, Peri, Timeout};

/// I2C operating modes.
pub mod mode {
    trait SealedMode {}

    /// Trait for I2C modes that select master or slave operation.
    #[allow(private_bounds)]
    pub trait OperatingMode: SealedMode {}

    /// Mode allowing I2C master operations.
    pub struct Master;

    /// Mode allowing I2C slave operations.
    pub struct Slave;

    impl SealedMode for Master {}
    impl OperatingMode for Master {}

    impl SealedMode for Slave {}
    impl OperatingMode for Slave {}
}

use mode::{Master, OperatingMode, Slave};

/// Event interrupt handler.
pub struct EventInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::EventInterrupt> for EventInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

/// Error interrupt handler.
pub struct ErrorInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        on_interrupt::<T>()
    }
}

pub unsafe fn on_interrupt<T: Instance>() {
    let regs = T::regs();
    // i2c v2 only woke the task on transfer complete interrupts. v1 uses interrupts for a bunch of
    // other stuff, so we wake the task on every interrupt.
    T::state().waker.wake();
    critical_section::with(|_| {
        // Clear event interrupt flag.
        regs.ctlr2().modify(|w| {
            w.set_itevten(false);
            w.set_iterren(false);
            // `ITBUFEN` gates TxE/RxNE independently of `ITEVTEN`; leaving it
            // set would keep the event interrupt asserted for a slave.
            w.set_itbufen(false);
        });
    });
}

/// I2C error.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error. (BERR)
    Bus,
    /// Arbitration lost (ARLO)
    Arbitration,
    /// ACK not received (either to the address or to a data byte) (AF)
    Nack,
    /// Timeout
    Timeout,
    /// CRC error
    Crc,
    /// Overrun error (OVR)
    Overrun,
    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,
    /// DMA buffer exceeds the hardware transfer counter.
    BufferTooLong,
    /// Address is outside the 7-bit address range.
    InvalidAddress,
}

/// Address used by an I2C slave.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlaveAddress {
    /// 7-bit address (`0x00..=0x7F`).
    SevenBit(u8),
    /// 10-bit address (`0x000..=0x3FF`).
    TenBit(u16),
}

/// I2C slave configuration.
#[non_exhaustive]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveConfig {
    /// Primary address, programmed into `OADDR1`.
    pub address: SlaveAddress,
    /// Optional second 7-bit address, programmed into `OADDR2`.
    pub second_address: Option<u8>,
    /// Respond to the general call address (`0x00`).
    pub general_call: bool,
    /// Allow clock stretching (`NOSTRETCH = 0`).
    pub clock_stretching: bool,
}

impl SlaveConfig {
    /// Create a configuration with only the primary address set.
    pub fn new(address: impl Into<SlaveAddress>) -> Self {
        Self {
            address: address.into(),
            second_address: None,
            general_call: false,
            clock_stretching: true,
        }
    }

    fn validate(&self) -> Result<(), Error> {
        match self.address {
            SlaveAddress::SevenBit(addr) => {
                if addr > 0x7f || (addr == 0x00 && !self.general_call) {
                    return Err(Error::InvalidAddress);
                }
            }
            SlaveAddress::TenBit(addr) => {
                if addr > 0x3ff {
                    return Err(Error::InvalidAddress);
                }
            }
        }

        if let Some(addr2) = self.second_address {
            // Only 7-bit addresses can be programmed into OADDR2, and the
            // hardware cannot combine a 10-bit primary with a dual address.
            if addr2 > 0x7f || matches!(self.address, SlaveAddress::TenBit(_)) {
                return Err(Error::InvalidAddress);
            }
        }

        Ok(())
    }
}

impl Default for SlaveConfig {
    fn default() -> Self {
        Self::new(SlaveAddress::SevenBit(0x55))
    }
}

impl From<u8> for SlaveAddress {
    fn from(value: u8) -> Self {
        Self::SevenBit(value)
    }
}

impl From<u16> for SlaveAddress {
    fn from(value: u16) -> Self {
        Self::TenBit(value)
    }
}

/// Direction of a transaction started by the I2C master.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlaveCommandKind {
    /// General call address was matched; treat it as a write.
    GeneralCall,
    /// Master is reading from this slave.
    Read,
    /// Master is writing to this slave.
    Write,
}

/// Command received from the I2C master.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveCommand {
    /// Transaction direction.
    pub kind: SlaveCommandKind,
    /// Address that was matched.
    pub address: SlaveAddress,
}

/// Result of a slave transmit operation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SendStatus {
    /// All requested bytes were transmitted, including any read padding.
    Done,
    /// The master stopped early; this many bytes were not sent.
    LeftoverBytes(usize),
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Duty {
    Duty2_1 = 0,
    Duty16_9 = 1,
}

/// I2C config
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Timeout. Without the `embassy` feature, waits have no deadline.
    #[cfg(feature = "embassy")]
    pub timeout: embassy_time::Duration,
    pub duty: Duty,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(feature = "embassy")]
            timeout: embassy_time::Duration::from_millis(1000),
            duty: Duty::Duty2_1,
        }
    }
}

struct Timing {
    input_mhz: u8,
    #[cfg(i2c_v3)]
    rise: u8,
    divider: u16,
    fast: bool,
    duty: bool,
}

fn timing(input: Hertz, frequency: Hertz, duty: Duty) -> Timing {
    assert!(frequency.0 > 0 && frequency.0 <= 400_000);
    let fast = frequency.0 > 100_000;
    // The legacy families have always stored the truncated megahertz value in
    // the 6-bit FREQ/TRISE fields. CH32V303/V305/V307 run at 144 MHz and
    // CH32H417 at 100/120 MHz, so rejecting values above 63 MHz here would
    // panic in the constructor on those chips. Keep the historical truncation
    // for them; CH32V4x7 clamps to the range documented for its I2C block.
    let input_mhz = input.0 / 1_000_000;
    #[cfg(ch32v4)]
    assert!(input_mhz >= 4);
    #[cfg(ch32v4)]
    let input_mhz = input_mhz.min(60);
    let factor = if !fast {
        2
    } else if duty == Duty::Duty2_1 {
        3
    } else {
        25
    };
    let divider = u64::from(input.0).div_ceil(u64::from(frequency.0) * factor);
    let divider = divider.max(if fast { 1 } else { 4 });
    assert!(divider <= 0xfff);
    Timing {
        input_mhz: input_mhz as u8,
        #[cfg(i2c_v3)]
        rise: (if fast {
            input_mhz * 300 / 1000 + 1
        } else {
            input_mhz + 1
        }) as u8,
        divider: divider as u16,
        fast,
        duty: fast && duty == Duty::Duty16_9,
    }
}

/// I2C driver.
///
/// Without the `embassy` feature, operations have no timeout and may wait
/// indefinitely if the bus or peripheral does not make progress.
pub struct I2c<'d, T: Instance, M: Mode, O: OperatingMode = Master> {
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    #[cfg(feature = "embassy")]
    timeout: embassy_time::Duration,
    pins: I2cPins<'d, T>,
    _phantom: PhantomData<(M, O)>,
}

/// Owns the pins and tears the peripheral down. Kept separate from [`I2c`]
/// so `into_slave()` can move the driver fields without running `Drop`.
struct I2cPins<'d, T: Instance> {
    scl: Peri<'d, AnyPin>,
    sda: Peri<'d, AnyPin>,
    _phantom: PhantomData<&'d mut T>,
}

impl<'d, T: Instance> Drop for I2cPins<'d, T> {
    fn drop(&mut self) {
        use crate::interrupt::typelevel::Interrupt;

        T::regs().ctlr2().modify(|w| {
            w.set_dmaen(false);
            w.set_iterren(false);
            w.set_itevten(false);
            w.set_itbufen(false);
        });
        T::regs().ctlr1().modify(|w| w.set_pe(false));
        T::EventInterrupt::disable();
        T::ErrorInterrupt::disable();
        self.scl.set_as_disconnected();
        self.sda.set_as_disconnected();
        T::disable();
    }
}

impl<'d, T: Instance> I2c<'d, T, Async, Master> {
    async fn wait_status(mask: u32) -> Result<(), Error> {
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            match Self::check_and_clear_error_flags() {
                Err(error) => Poll::Ready(Err(error)),
                Ok(status) if status.0 & mask != 0 => Poll::Ready(Ok(())),
                Ok(_) => {
                    T::regs().ctlr2().modify(|w| {
                        w.set_itbufen(mask == (1 << 6));
                        w.set_iterren(true);
                        w.set_itevten(true);
                    });
                    Poll::Pending
                }
            }
        })
        .await
    }

    async fn wait_stop_async() {
        poll_fn(|cx| {
            if T::regs().ctlr1().read().stop() {
                cx.waker().wake_by_ref();
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await
    }

    async fn launch_start_async(ack: bool) -> Result<(), Error> {
        Self::wait_stop_async().await;
        if T::regs().ctlr1().read().start() || T::regs().star1().read().sb() {
            return Ok(());
        }
        T::regs().ctlr1().modify(|w| {
            w.set_start(true);
            w.set_ack(ack);
            w.set_pos(false);
        });
        Ok(())
    }

    /// Create a new I2C driver.
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, scl, sda, new_dma!(tx_dma), new_dma!(rx_dma), freq, config)
    }
}

impl<'d, T: Instance> I2c<'d, T, Blocking, Master> {
    /// Create a new blocking I2C driver.
    pub fn new_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner(peri, scl, sda, None, None, freq, config)
    }
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M, Master> {
    /// Create a new I2C driver.
    fn new_inner<#[cfg(not(afio_h4))] const REMAP: u8>(
        _peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        let timing = timing(T::frequency(), freq, config.duty);
        apply_remap!();
        use crate::interrupt::typelevel::Interrupt;

        T::enable_and_reset();

        // gpio_x0 doesn't expose an open-drain AF variant; SCL/SDA fall back
        // to push-pull and rely on external pull-ups (already required for I2C).
        #[cfg(not(gpio_x0))]
        let af = AfType::output(OutputType::OpenDrain, Speed::High);
        #[cfg(gpio_x0)]
        let af = AfType::output(OutputType::PushPull, Speed::High);

        set_as_af!(scl, af);
        set_as_af!(sda, af);

        let mut this = Self {
            tx_dma,
            rx_dma,
            #[cfg(feature = "embassy")]
            timeout: config.timeout,
            pins: I2cPins {
                scl: scl.into(),
                sda: sda.into(),
                _phantom: PhantomData,
            },
            _phantom: PhantomData,
        };

        this.init(timing);
        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        if this.tx_dma.is_some() {
            unsafe { T::EventInterrupt::enable() };
            unsafe { T::ErrorInterrupt::enable() };
        }

        this
    }
}

impl<'d, T: Instance, M: Mode, O: OperatingMode> I2c<'d, T, M, O> {
    fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "embassy")]
            deadline: embassy_time::Instant::now() + self.timeout,
        }
    }

    fn clear_error(mask: u16) {
        T::regs()
            .star1()
            .write_value(crate::pac::i2c::regs::Star1(u32::from(!mask)));
    }

    /// Clear `STOPF` by reading `STAR1` and then writing `CTLR1` (RM §18.12.6).
    fn clear_stopf() {
        let _ = T::regs().star1().read();
        T::regs().ctlr1().modify(|_| {});
    }

    /// Clear `AF`. In slave mode a NACK is the normal end of a read.
    fn clear_af() {
        T::regs().star1().modify(|w| w.set_af(false));
    }

    /// Clear hardware error flags but treat `AF` as normal slave behaviour.
    fn slave_handle_errors(status: &crate::pac::i2c::regs::Star1) -> Result<(), Error> {
        // AF must still be cleared here or it would keep the error interrupt
        // asserted; it is simply not reported as an error in slave mode.
        let observed = (status.0 & 0x5f00) as u16;
        if observed != 0 {
            Self::clear_error(observed);
        }

        if status.arlo() {
            return Err(Error::Arbitration);
        }
        #[cfg(i2c_v3)]
        if status.timeout() {
            return Err(Error::Timeout);
        }
        if status.pecerr() {
            return Err(Error::Crc);
        }
        if status.ovr() {
            return Err(Error::Overrun);
        }
        #[cfg(ch32v4)]
        if status.berr() {
            return Err(Error::Bus);
        }
        Ok(())
    }

    /// Discard stale flags left over by the previous transaction.
    fn clear_slave_stale_flags() {
        Self::clear_stopf();
        let status = T::regs().star1().read();
        let observed = (status.0 & 0x5f00) as u16;
        if observed != 0 {
            Self::clear_error(observed);
        }
    }

    /// Decode which own address matched, using `STAR2.DUALF`.
    fn decode_slave_address(star2: crate::pac::i2c::regs::Star2) -> SlaveAddress {
        if star2.gencall() {
            return SlaveAddress::SevenBit(0x00);
        }
        if star2.dualf() {
            return SlaveAddress::SevenBit(T::regs().oaddr2().read().add2());
        }

        let oaddr1 = T::regs().oaddr1().read();
        if oaddr1.addmode() {
            let add0 = if oaddr1.add0() { 1 } else { 0 };
            SlaveAddress::TenBit((u16::from(oaddr1.add9_8()) << 8) | (u16::from(oaddr1.add7_1()) << 1) | add0)
        } else {
            SlaveAddress::SevenBit(oaddr1.add7_1())
        }
    }

    fn send_status(total: usize, sent: usize) -> SendStatus {
        let data_sent = sent.min(total);
        if data_sent >= total {
            SendStatus::Done
        } else {
            SendStatus::LeftoverBytes(total - data_sent)
        }
    }

    /// Bytes actually clocked out of the DMA transfer. The DMA can preload the
    /// next byte into DR, which then never reaches the master.
    fn dma_bytes_sent(total: usize, remaining: u16, tx_e: bool) -> usize {
        let loaded = total.saturating_sub(remaining as usize);
        if !tx_e && loaded > 0 {
            loaded - 1
        } else {
            loaded
        }
    }

    /// Apply the slave address and control bits without resetting the timing.
    fn init_slave(config: SlaveConfig) {
        let regs = T::regs();

        // Disabling the peripheral clears ADDR/STOPF and the error flags.
        regs.ctlr1().modify(|w| w.set_pe(false));
        regs.ctlr2().modify(|w| w.set_itbufen(false));

        match config.address {
            SlaveAddress::SevenBit(addr) => regs.oaddr1().write(|w| {
                w.set_addmode(false);
                // Bit 14 is marked "must be 1" in the reference manual.
                w.set_must1(true);
                w.set_add7_1(addr);
            }),
            SlaveAddress::TenBit(addr) => regs.oaddr1().write(|w| {
                w.set_addmode(true);
                w.set_must1(true);
                w.set_add9_8((addr >> 8) as u8);
                w.set_add7_1((addr >> 1) as u8);
                w.set_add0(addr & 0x0001 != 0);
            }),
        }

        if let Some(addr2) = config.second_address {
            regs.oaddr2().write(|w| {
                w.set_add2(addr2);
                w.set_endual(true);
            });
        } else {
            regs.oaddr2().write(|w| w.set_endual(false));
        }

        regs.ctlr1().modify(|w| {
            w.set_engc(config.general_call);
            w.set_nostretch(!config.clock_stretching);
            w.set_ack(true);
            w.set_pe(true);
        });
    }

    fn enable_slave_interrupts(wait_data: bool) {
        T::regs().ctlr2().modify(|w| {
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_itbufen(wait_data);
        });
    }

    fn disable_slave_transfers() {
        T::regs().ctlr2().modify(|w| {
            w.set_dmaen(false);
            w.set_iterren(false);
            w.set_itevten(false);
            w.set_itbufen(false);
        });
    }

    /// Release the bus when a slave operation is cancelled.
    fn abort_slave_transfer() {
        Self::disable_slave_transfers();
        let regs = T::regs();
        let star2 = regs.star2().read();
        if star2.busy() && !star2.msl() {
            // In slave mode STOP releases SDA/SCL after the current byte.
            regs.ctlr1().modify(|w| w.set_stop(true));
        }
    }
}

impl<'d, T: Instance, M: Mode> I2c<'d, T, M, Master> {
    // init as master mode
    fn init(&mut self, timing: Timing) {
        let regs = T::regs();

        regs.ctlr1().modify(|w| w.set_pe(false)); // disale i2c

        // soft reset
        regs.ctlr1().modify(|w| w.set_swrst(true));
        regs.ctlr1().modify(|w| w.set_swrst(false));

        regs.ctlr2().modify(|w| w.set_freq(timing.input_mhz));
        #[cfg(i2c_v3)]
        regs.rtr().write(|w| w.set_trise(timing.rise));
        regs.ckcfgr().write(|w| {
            w.set_f_s(timing.fast);
            w.set_duty(timing.duty);
            w.set_ccr(timing.divider);
        });

        regs.ctlr1().modify(|w| w.set_pe(true));
    }

    fn generate_stop() {
        T::regs().ctlr1().modify(|w| w.set_stop(true));
    }

    fn finish_transfer() {
        T::regs().ctlr2().modify(|w| {
            w.set_dmaen(false);
            w.set_iterren(false);
            w.set_itevten(false);
            w.set_itbufen(false);
            w.set_last(false);
        });
    }

    fn abort_transfer() {
        Self::finish_transfer();
        let regs = T::regs();
        let control = regs.ctlr1().read();
        if control.start() || control.stop() {
            let frequency = regs.ctlr2().read().freq();
            let divider = regs.ckcfgr().read();
            #[cfg(i2c_v3)]
            let rise = regs.rtr().read();
            regs.ctlr1().write(|w| w.set_swrst(true));
            regs.ctlr1().write(|w| w.set_swrst(false));
            regs.ctlr2().write(|w| w.set_freq(frequency));
            regs.ckcfgr().write_value(divider);
            #[cfg(i2c_v3)]
            regs.rtr().write_value(rise);
            regs.ctlr1().write(|w| w.set_pe(true));
        } else if regs.star2().read().msl() {
            Self::generate_stop();
        }
    }

    fn validate_address(address: u8) -> Result<(), Error> {
        if address > 0x7f {
            Err(Error::InvalidAddress)
        } else {
            Ok(())
        }
    }

    /// Wait until a previously requested STOP has been generated and HW has
    /// cleared CTLR1.STOP. Needed before the next START RMW (#221): a plain
    /// `ctlr1().modify(|r| r.set_start(true))` can capture STOP=1 and write it
    /// back after HW cleared it, so a second STOP follows the new START.
    fn wait_stop_finished(timeout: Timeout) -> Result<(), Error> {
        while T::regs().ctlr1().read().stop() {
            timeout.check().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    /// Issue a START (optionally with ACK) only after any pending STOP finishes.
    fn launch_start(ack: bool, timeout: Timeout) -> Result<(), Error> {
        Self::wait_stop_finished(timeout)?;
        if T::regs().ctlr1().read().start() || T::regs().star1().read().sb() {
            return Ok(());
        }
        T::regs().ctlr1().modify(|reg| {
            reg.set_start(true);
            if ack {
                reg.set_ack(true);
            }
        });
        Ok(())
    }

    fn check_and_clear_error_flags() -> Result<crate::pac::i2c::regs::Star1, Error> {
        let status = T::regs().star1().read();
        let observed = (status.0 & 0x5f00) as u16;
        if observed != 0 {
            Self::clear_error(observed);
        }
        if status.arlo() {
            return Err(Error::Arbitration);
        }
        #[cfg(i2c_v3)]
        if status.timeout() {
            return Err(Error::Timeout);
        }
        if status.pecerr() {
            return Err(Error::Crc);
        }
        if status.ovr() {
            return Err(Error::Overrun);
        }
        if status.af() {
            return Err(Error::Nack);
        }
        #[cfg(ch32v4)]
        if status.berr() {
            return Err(Error::Bus);
        }
        Ok(status)
    }

    fn write_bytes(&mut self, addr: u8, bytes: &[u8], timeout: Timeout, frame: FrameOptions) -> Result<(), Error> {
        Self::validate_address(addr)?;
        let on_drop = OnDrop::new(Self::abort_transfer);
        if frame.send_start() {
            // Send a START condition
            Self::launch_start(false, timeout)?;

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar(addr << 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Clear condition by reading SR2
            let _ = T::regs().star2().read();
        }

        // Send bytes
        for c in bytes {
            self.send_byte(*c, timeout)?;
        }

        if frame.send_stop() {
            // Send a STOP condition
            T::regs().ctlr1().modify(|reg| reg.set_stop(true));
            Self::wait_stop_finished(timeout)?;
        }

        on_drop.defuse();
        Ok(())
    }

    fn send_byte(&self, byte: u8, timeout: Timeout) -> Result<(), Error> {
        // Wait until we're ready for sending
        while {
            // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
            !Self::check_and_clear_error_flags()?.tx_e()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        // Push out a byte of data
        T::regs().datar().write(|reg| reg.set_datar(byte));

        // Wait until byte is transferred
        while {
            // Check for any potential error conditions.
            !Self::check_and_clear_error_flags()?.btf()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        Ok(())
    }

    fn recv_byte(&self, timeout: Timeout) -> Result<u8, Error> {
        while {
            // Check for any potential error conditions.
            Self::check_and_clear_error_flags()?;

            !T::regs().star1().read().rx_ne()
        } {
            timeout.check().ok_or(Error::Timeout)?;
        }

        let value = T::regs().datar().read().datar();
        Ok(value)
    }

    fn blocking_read_timeout(
        &mut self,
        addr: u8,
        buffer: &mut [u8],
        timeout: Timeout,
        frame: FrameOptions,
    ) -> Result<(), Error> {
        let length = buffer.len();
        self.blocking_read_iter(addr, buffer.iter_mut(), length, timeout, frame)
    }

    fn wait_btf(timeout: Timeout) -> Result<(), Error> {
        while !Self::check_and_clear_error_flags()?.btf() {
            timeout.check().ok_or(Error::Timeout)?;
        }
        Ok(())
    }

    fn blocking_read_iter<'a>(
        &mut self,
        addr: u8,
        mut buffer: impl Iterator<Item = &'a mut u8>,
        mut remaining: usize,
        timeout: Timeout,
        frame: FrameOptions,
    ) -> Result<(), Error> {
        Self::validate_address(addr)?;
        if remaining == 0 {
            return Err(Error::Overrun);
        }
        let on_drop = OnDrop::new(Self::abort_transfer);
        let regs = T::regs();
        regs.ctlr1().modify(|w| w.set_pos(remaining == 2 && frame.send_nack()));

        if frame.send_start() {
            // Send a START condition and set ACK bit
            Self::launch_start(true, timeout)?;

            // Wait until START condition was generated
            while !Self::check_and_clear_error_flags()?.sb() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            // Check if we were the ones to generate START
            if T::regs().ctlr1().read().start() || !T::regs().star2().read().msl() {
                return Err(Error::Arbitration);
            }

            // Set up current address we're trying to talk to
            T::regs().datar().write(|reg| reg.set_datar((addr << 1) + 1));

            // Wait until address was sent
            // Wait for the address to be acknowledged
            while !Self::check_and_clear_error_flags()?.addr() {
                timeout.check().ok_or(Error::Timeout)?;
            }

            critical_section::with(|_| {
                if remaining <= 2 && frame.send_nack() {
                    regs.ctlr1().modify(|w| w.set_ack(false));
                }
                let _ = regs.star2().read();
                if remaining == 1 && frame.send_stop() {
                    Self::generate_stop();
                } else if remaining == 1 && frame.send_nack() {
                    regs.ctlr1().modify(|w| w.set_start(true));
                }
            });
        }

        if !frame.send_nack() {
            for value in buffer {
                *value = self.recv_byte(timeout)?;
            }
        } else if remaining == 1 {
            *buffer.next().unwrap() = self.recv_byte(timeout)?;
        } else {
            while remaining > 3 {
                *buffer.next().unwrap() = self.recv_byte(timeout)?;
                remaining -= 1;
            }
            Self::wait_btf(timeout)?;
            if remaining == 3 {
                regs.ctlr1().modify(|w| w.set_ack(false));
                *buffer.next().unwrap() = regs.datar().read().datar();
                Self::wait_btf(timeout)?;
            }
            critical_section::with(|_| {
                if frame.send_stop() {
                    Self::generate_stop();
                } else {
                    regs.ctlr1().modify(|w| w.set_start(true));
                }
                *buffer.next().unwrap() = regs.datar().read().datar();
                *buffer.next().unwrap() = regs.datar().read().datar();
            });
        }
        if frame.send_stop() {
            Self::wait_stop_finished(timeout)?;
            regs.ctlr1().modify(|w| w.set_pos(false));
        }
        on_drop.defuse();
        Ok(())
    }

    /// Blocking read.
    pub fn blocking_read(&mut self, addr: u8, read: &mut [u8]) -> Result<(), Error> {
        self.blocking_read_timeout(addr, read, self.timeout(), FrameOptions::FirstAndLastFrame)
    }

    /// Blocking write.
    pub fn blocking_write(&mut self, addr: u8, write: &[u8]) -> Result<(), Error> {
        self.write_bytes(addr, write, self.timeout(), FrameOptions::FirstAndLastFrame)?;

        // Fallthrough is success
        Ok(())
    }

    /// Blocking write, restart, read.
    pub fn blocking_write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }

        let timeout = self.timeout();

        self.write_bytes(addr, write, timeout, FrameOptions::FirstFrame)?;
        self.blocking_read_timeout(addr, read, timeout, FrameOptions::FirstAndLastFrame)?;

        Ok(())
    }

    /// Blocking transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
    pub fn blocking_transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        Self::validate_address(addr)?;
        validate_operations(operations, false)?;
        let timeout = self.timeout();
        let mut operations = operations;
        while !operations.is_empty() {
            let read = matches!(operations[0], Operation::Read(_));
            let count = operations
                .iter()
                .take_while(|op| matches!(op, Operation::Read(_)) == read)
                .count();
            let (group, rest) = operations.split_at_mut(count);
            if read {
                let length = group
                    .iter()
                    .map(|op| match op {
                        Operation::Read(buffer) => buffer.len(),
                        _ => 0,
                    })
                    .sum();
                let buffer = group.iter_mut().flat_map(|op| match op {
                    Operation::Read(buffer) => buffer.iter_mut(),
                    _ => unreachable!(),
                });
                let frame = if rest.is_empty() {
                    FrameOptions::FirstAndLastFrame
                } else {
                    FrameOptions::FirstFrame
                };
                self.blocking_read_iter(addr, buffer, length, timeout, frame)?;
            } else {
                for (index, operation) in group.iter().enumerate() {
                    let Operation::Write(buffer) = operation else {
                        unreachable!()
                    };
                    let first = index == 0;
                    let stop = rest.is_empty() && index + 1 == count;
                    let frame = match (first, stop) {
                        (true, true) => FrameOptions::FirstAndLastFrame,
                        (true, false) => FrameOptions::FirstFrame,
                        (false, true) => FrameOptions::LastFrame,
                        (false, false) => FrameOptions::NextFrame,
                    };
                    self.write_bytes(addr, buffer, timeout, frame)?;
                }
            }
            operations = rest;
        }

        Ok(())
    }
}

// ======== Slave

impl<'d, T: Instance, M: Mode> I2c<'d, T, M, Master> {
    /// Switch this driver to slave mode.
    ///
    /// The bus timing configured by the constructor is kept; only the own
    /// address and the slave control bits are reprogrammed.
    pub fn into_slave(mut self, config: SlaveConfig) -> Result<I2c<'d, T, M, Slave>, Error> {
        config.validate()?;
        Self::init_slave(config);

        Ok(I2c {
            tx_dma: self.tx_dma.take(),
            rx_dma: self.rx_dma.take(),
            #[cfg(feature = "embassy")]
            timeout: self.timeout,
            pins: self.pins,
            _phantom: PhantomData,
        })
    }
}

impl<'d, T: Instance> I2c<'d, T, Blocking, Slave> {
    /// Create a new blocking I2C slave.
    pub fn new_slave_blocking<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        I2c::<'d, T, Blocking, Master>::new_inner(peri, scl, sda, None, None, freq, Config::default())
            .into_slave(config)
    }

    /// Block until the master addresses this slave.
    pub fn blocking_listen(&mut self) -> Result<SlaveCommand, Error> {
        let timeout = self.timeout();

        // A stale STOPF from the previous transaction would end the next
        // response immediately, so flush it before arming ACK.
        Self::clear_slave_stale_flags();
        T::regs().ctlr2().modify(|w| w.set_itbufen(false));
        T::regs().ctlr1().modify(|w| w.set_ack(true));

        loop {
            let star1 = T::regs().star1().read();

            if star1.addr() {
                // Reading STAR1 and then STAR2 clears ADDR and releases SCL.
                let star2 = T::regs().star2().read();
                let kind = if star2.gencall() {
                    SlaveCommandKind::GeneralCall
                } else if star2.tra() {
                    SlaveCommandKind::Read
                } else {
                    SlaveCommandKind::Write
                };
                return Ok(SlaveCommand {
                    kind,
                    address: Self::decode_slave_address(star2),
                });
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }

    /// Receive bytes from a write transaction.
    ///
    /// Returns the number of bytes stored in `buf`. Bytes beyond `buf.len()`
    /// are acknowledged and discarded, so the bus is never held low waiting
    /// for space.
    pub fn blocking_respond_to_write(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let timeout = self.timeout();
        let mut stored = 0usize;

        loop {
            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let byte = T::regs().datar().read().datar();
                if stored < buf.len() {
                    buf[stored] = byte;
                    stored += 1;
                }
                continue;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Ok(stored);
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Ok(stored);
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }

    /// Transmit bytes for a read transaction.
    ///
    /// Data beyond `data` is padded with `0x00` until the master ends the
    /// transaction, matching the reference manual's slave transmitter flow.
    pub fn blocking_respond_to_read(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let timeout = self.timeout();
        let mut sent = 0usize;

        loop {
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.stopf() {
                Self::clear_stopf();
                // If the transmit register is still full, the last byte was
                // never clocked out.
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Ok(Self::send_status(data.len(), sent));
            }

            if star1.tx_e() {
                let byte = data.get(sent).copied().unwrap_or(0x00);
                T::regs().datar().write(|w| w.set_datar(byte));
                sent += 1;
                continue;
            }

            Self::slave_handle_errors(&star1).map_err(|error| {
                Self::abort_slave_transfer();
                error
            })?;
            timeout.check().ok_or_else(|| {
                Self::abort_slave_transfer();
                Error::Timeout
            })?;
        }
    }
}

impl<'d, T: Instance> I2c<'d, T, Async, Slave> {
    /// Create a new interrupt-driven I2C slave.
    pub fn new_slave<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        use crate::interrupt::typelevel::Interrupt;

        let slave = I2c::<'d, T, Async, Master>::new_inner(peri, scl, sda, None, None, freq, Config::default())
            .into_slave(config)?;

        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        Ok(slave)
    }

    /// Create a new I2C slave that uses DMA for the data phase.
    pub fn new_slave_with_dma<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        scl: Peri<'d, if_remap!(impl SclPin<T, REMAP>)>,
        sda: Peri<'d, if_remap!(impl SdaPin<T, REMAP>)>,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: Peri<'d, impl TxDma<T>>,
        rx_dma: Peri<'d, impl RxDma<T>>,
        freq: Hertz,
        config: SlaveConfig,
    ) -> Result<Self, Error> {
        use crate::interrupt::typelevel::Interrupt;

        let slave = I2c::<'d, T, Async, Master>::new_inner(
            peri,
            scl,
            sda,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            freq,
            Config::default(),
        )
        .into_slave(config)?;

        T::EventInterrupt::unpend();
        T::ErrorInterrupt::unpend();
        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        Ok(slave)
    }

    /// Wait until the master addresses this slave.
    pub async fn listen(&mut self) -> Result<SlaveCommand, Error> {
        Self::clear_slave_stale_flags();
        T::regs().ctlr2().modify(|w| w.set_itbufen(false));
        T::regs().ctlr1().modify(|w| w.set_ack(true));

        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.addr() {
                let star2 = T::regs().star2().read();
                let kind = if star2.gencall() {
                    SlaveCommandKind::GeneralCall
                } else if star2.tra() {
                    SlaveCommandKind::Read
                } else {
                    SlaveCommandKind::Write
                };
                return Poll::Ready(Ok(SlaveCommand {
                    kind,
                    address: Self::decode_slave_address(star2),
                }));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(false);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Receive bytes from a write transaction.
    pub async fn respond_to_write(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if self.rx_dma.is_some() {
            self.respond_to_write_dma(buf).await
        } else {
            self.respond_to_write_irq(buf).await
        }
    }

    async fn respond_to_write_irq(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);
        let mut stored = 0usize;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            // Spin for the first byte instead of sleeping on the RXNE
            // interrupt. The address match of a repeated START switches the
            // data register back to transmit, so a received byte that has not
            // been read yet is gone by then (measured: `STAR1` reads
            // `ADDR|TXE` with `RXNE` clear). That leaves the byte only about
            // one byte time of headroom, which the interrupt-to-task wake on
            // this core does not reliably meet: a write-then-restart-read
            // transaction lost its register pointer every time. The loop exits
            // as soon as anything happens on the bus, so it only spins while
            // the byte is already on its way.
            for _ in 0..20_000 {
                let star1 = T::regs().star1().read();
                if star1.rx_ne() || star1.stopf() || star1.addr() {
                    break;
                }
            }

            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let byte = T::regs().datar().read().datar();
                if stored < buf.len() {
                    buf[stored] = byte;
                    stored += 1;
                }
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(stored));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(stored));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Transmit bytes for a read transaction.
    pub async fn respond_to_read(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        if self.tx_dma.is_some() {
            self.respond_to_read_dma(data).await
        } else {
            self.respond_to_read_irq(data).await
        }
    }

    async fn respond_to_read_irq(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);
        let mut sent = 0usize;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.stopf() {
                Self::clear_stopf();
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                if !star1.tx_e() && sent > 0 {
                    sent -= 1;
                }
                return Poll::Ready(Ok(Self::send_status(data.len(), sent)));
            }

            if star1.tx_e() {
                let byte = data.get(sent).copied().unwrap_or(0x00);
                T::regs().datar().write(|w| w.set_datar(byte));
                sent += 1;
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    async fn respond_to_write_dma(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let total = buf.len();
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        if total == 0 {
            T::regs().ctlr2().modify(|w| w.set_dmaen(false));
            on_drop.defuse();
            self.drain_excess_bytes().await?;
            return Ok(0);
        }

        let mut transfer = unsafe {
            let src = T::regs().datar().as_ptr().cast();
            self.rx_dma.as_mut().unwrap().read(src, buf, Default::default())
        };

        T::regs().ctlr2().modify(|w| {
            w.set_itbufen(false);
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_dmaen(true);
        });

        let mut remaining = total;
        let mut buffer_full = false;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            // Polling the DMA future registers its waker and reports when the
            // buffer is full.
            let dma_ready = Pin::new(&mut transfer).poll(cx).is_ready();
            let star1 = T::regs().star1().read();

            // A byte can arrive in the same cycle as STOPF; let the DMA take
            // it before ending the transaction.
            if star1.stopf() && !star1.rx_ne() {
                Self::clear_stopf();
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Ok(()));
            }

            if let Err(error) = Self::slave_handle_errors(&star1) {
                remaining = transfer.get_remaining_transfers() as usize;
                return Poll::Ready(Err(error));
            }

            if dma_ready || !transfer.is_running() {
                remaining = transfer.get_remaining_transfers() as usize;
                buffer_full = remaining == 0;
                return Poll::Ready(Ok(()));
            }

            Self::enable_slave_interrupts(false);
            Poll::Pending
        })
        .await;

        // Stop the DMA before `buf` can be touched again.
        drop(transfer);
        Self::disable_slave_transfers();

        let received = total.saturating_sub(remaining);
        if let Err(error) = result {
            // `on_drop` is still armed and releases the bus on return.
            return Err(error);
        }
        on_drop.defuse();

        if buffer_full {
            // The master is still sending; acknowledge and discard until it
            // ends the transaction.
            self.drain_excess_bytes().await?;
            Ok(total)
        } else {
            Ok(received)
        }
    }

    async fn respond_to_read_dma(&mut self, data: &[u8]) -> Result<SendStatus, Error> {
        let total = data.len();
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        if total == 0 {
            T::regs().ctlr2().modify(|w| w.set_dmaen(false));
            on_drop.defuse();
            self.send_padding_bytes().await?;
            return Ok(SendStatus::Done);
        }

        let mut transfer = unsafe {
            let dst = T::regs().datar().as_ptr().cast();
            self.tx_dma.as_mut().unwrap().write(data, dst, Default::default())
        };

        T::regs().ctlr2().modify(|w| {
            w.set_itbufen(false);
            w.set_itevten(true);
            w.set_iterren(true);
            w.set_dmaen(true);
        });

        let mut sent = 0usize;
        let mut dma_done = false;

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let dma_ready = Pin::new(&mut transfer).poll(cx).is_ready();
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if star1.stopf() {
                Self::clear_stopf();
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Ok(()));
            }

            if let Err(error) = Self::slave_handle_errors(&star1) {
                sent = Self::dma_bytes_sent(total, transfer.get_remaining_transfers(), star1.tx_e());
                return Poll::Ready(Err(error));
            }

            if dma_ready || !transfer.is_running() {
                dma_done = true;
                return Poll::Ready(Ok(()));
            }

            Self::enable_slave_interrupts(false);
            Poll::Pending
        })
        .await;

        drop(transfer);
        Self::disable_slave_transfers();

        if let Err(error) = result {
            // `on_drop` is still armed and releases the bus on return.
            return Err(error);
        }
        on_drop.defuse();

        if dma_done {
            // The DMA only filled DR; the master may still be clocking bytes.
            self.send_padding_bytes().await?;
            Ok(SendStatus::Done)
        } else {
            Ok(Self::send_status(total, sent))
        }
    }

    /// Drain bytes after the receive buffer is full.
    async fn drain_excess_bytes(&mut self) -> Result<(), Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.rx_ne() {
                let _ = T::regs().datar().read().datar();
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(()));
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }

    /// Send `0x00` padding after the transmit buffer is exhausted.
    async fn send_padding_bytes(&mut self) -> Result<(), Error> {
        let on_drop = OnDrop::new(Self::abort_slave_transfer);

        let result = poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            let star1 = T::regs().star1().read();

            if star1.af() {
                Self::clear_af();
                return Poll::Ready(Ok(()));
            }

            if star1.stopf() {
                Self::clear_stopf();
                return Poll::Ready(Ok(()));
            }

            if star1.addr() {
                // Repeated START: leave ADDR set for the next listen().
                return Poll::Ready(Ok(()));
            }

            if star1.tx_e() {
                T::regs().datar().write(|w| w.set_datar(0x00));
                Self::enable_slave_interrupts(true);
                return Poll::Pending;
            }

            match Self::slave_handle_errors(&star1) {
                Err(error) => Poll::Ready(Err(error)),
                Ok(()) => {
                    Self::enable_slave_interrupts(true);
                    Poll::Pending
                }
            }
        })
        .await;

        // On error the guard stays armed, so returning releases the bus.
        if result.is_ok() {
            on_drop.defuse();
        }
        result
    }
}

// ======== Async

impl<'d, T: Instance> I2c<'d, T, Async, Master> {
    async fn write_frame(&mut self, address: u8, write: &[u8], frame: FrameOptions) -> Result<(), Error> {
        Self::validate_address(address)?;
        if write.len() > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        self.timeout()
            .with(async { Some(self.write_frame_inner(address, write, frame).await) })
            .await
            .unwrap_or(Err(Error::Timeout))
    }

    async fn wait_error() -> Error {
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());
            match Self::check_and_clear_error_flags() {
                Err(error) => Poll::Ready(error),
                Ok(_) => {
                    T::regs().ctlr2().modify(|w| {
                        w.set_itevten(false);
                        w.set_itbufen(false);
                        w.set_iterren(true);
                    });
                    Poll::Pending
                }
            }
        })
        .await
    }

    async fn write_frame_inner(&mut self, address: u8, write: &[u8], frame: FrameOptions) -> Result<(), Error> {
        let regs = T::regs();
        Self::finish_transfer();
        let on_drop = OnDrop::new(Self::abort_transfer);
        if frame.send_start() {
            Self::launch_start_async(false).await?;
            Self::wait_status(1).await?;
            if regs.ctlr1().read().start() || !regs.star2().read().msl() {
                return Err(Error::Arbitration);
            }
            regs.datar().write(|w| w.set_datar(address << 1));
            Self::wait_status(1 << 1).await?;
            let _ = regs.star2().read();
        }
        if !write.is_empty() {
            let transfer = unsafe {
                self.tx_dma
                    .as_mut()
                    .unwrap()
                    .write(write, regs.datar().as_ptr().cast(), Default::default())
            };
            regs.ctlr2().modify(|w| w.set_dmaen(true));
            if let Either::First(error) = select(Self::wait_error(), transfer).await {
                return Err(error);
            }
            regs.ctlr2().modify(|w| w.set_dmaen(false));
            Self::wait_status(1 << 2).await?;
        }
        if frame.send_stop() {
            Self::generate_stop();
            Self::wait_stop_async().await;
        }
        Self::finish_transfer();
        on_drop.defuse();
        Ok(())
    }

    /// Write.
    pub async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Error> {
        self.write_frame(address, write, FrameOptions::FirstAndLastFrame)
            .await?;

        Ok(())
    }

    /// Read.
    pub async fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.read_group(address, &mut [Operation::Read(buffer)], true).await
    }

    async fn read_group(&mut self, address: u8, operations: &mut [Operation<'_>], stop: bool) -> Result<(), Error> {
        Self::validate_address(address)?;
        validate_operations(operations, true)?;
        self.timeout()
            .with(async { Some(self.read_group_inner(address, operations, stop).await) })
            .await
            .unwrap_or(Err(Error::Timeout))
    }

    async fn read_group_inner(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
        stop: bool,
    ) -> Result<(), Error> {
        let total: usize = operations
            .iter()
            .map(|op| match op {
                Operation::Read(buffer) => buffer.len(),
                _ => unreachable!(),
            })
            .sum();
        let regs = T::regs();
        Self::finish_transfer();
        let on_drop = OnDrop::new(Self::abort_transfer);
        Self::launch_start_async(true).await?;
        Self::wait_status(1).await?;
        if regs.ctlr1().read().start() || !regs.star2().read().msl() {
            return Err(Error::Arbitration);
        }
        regs.datar().write(|w| w.set_datar((address << 1) | 1));
        Self::wait_status(1 << 1).await?;
        critical_section::with(|_| {
            regs.ctlr1().modify(|w| {
                w.set_pos(total == 2);
                w.set_ack(total > 2);
            });
            let _ = regs.star2().read();
            if total == 1 && stop {
                Self::generate_stop();
            } else if total == 1 {
                regs.ctlr1().modify(|w| w.set_start(true));
            }
        });

        let mut prefix = total.saturating_sub(3);
        let mut tail: [Option<&mut u8>; 3] = [None, None, None];
        let mut tail_index = 0;
        for operation in operations {
            let Operation::Read(buffer) = operation else {
                unreachable!()
            };
            let count = prefix.min(buffer.len());
            let (dma_buffer, tail_buffer) = buffer.split_at_mut(count);
            if count != 0 {
                let transfer = unsafe {
                    self.rx_dma
                        .as_mut()
                        .unwrap()
                        .read(regs.datar().as_ptr().cast(), dma_buffer, Default::default())
                };
                regs.ctlr2().modify(|w| w.set_dmaen(true));
                if let Either::First(error) = select(Self::wait_error(), transfer).await {
                    return Err(error);
                }
                regs.ctlr2().modify(|w| w.set_dmaen(false));
                prefix -= count;
            }
            for value in tail_buffer {
                tail[tail_index] = Some(value);
                tail_index += 1;
            }
        }
        if total == 1 {
            Self::wait_status(1 << 6).await?;
            *tail[0].take().unwrap() = regs.datar().read().datar();
        } else {
            Self::wait_status(1 << 2).await?;
            let offset = if total >= 3 {
                regs.ctlr1().modify(|w| w.set_ack(false));
                *tail[0].take().unwrap() = regs.datar().read().datar();
                Self::wait_status(1 << 2).await?;
                1
            } else {
                0
            };
            critical_section::with(|_| {
                if stop {
                    Self::generate_stop();
                } else {
                    regs.ctlr1().modify(|w| w.set_start(true));
                }
                *tail[offset].take().unwrap() = regs.datar().read().datar();
                *tail[offset + 1].take().unwrap() = regs.datar().read().datar();
            });
        }
        if stop {
            Self::wait_stop_async().await;
            regs.ctlr1().modify(|w| w.set_pos(false));
        }
        Self::finish_transfer();
        on_drop.defuse();
        Ok(())
    }

    /// Write, restart, read.
    pub async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Error> {
        // Check empty read buffer before starting transaction. Otherwise, we would not generate the
        // stop condition below.
        if read.is_empty() {
            return Err(Error::Overrun);
        }
        if read.len() > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }

        self.write_frame(address, write, FrameOptions::FirstFrame).await?;
        self.read(address, read).await
    }

    /// Transaction with operations.
    ///
    /// Consecutive operations of same type are merged. See [transaction contract] for details.
    ///
    /// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
    pub async fn transaction(&mut self, addr: u8, operations: &mut [Operation<'_>]) -> Result<(), Error> {
        Self::validate_address(addr)?;
        validate_operations(operations, true)?;
        let mut operations = operations;
        while !operations.is_empty() {
            let read = matches!(operations[0], Operation::Read(_));
            let count = operations
                .iter()
                .take_while(|op| matches!(op, Operation::Read(_)) == read)
                .count();
            let (group, rest) = operations.split_at_mut(count);
            if read {
                self.read_group(addr, group, rest.is_empty()).await?;
            } else {
                for (index, operation) in group.iter().enumerate() {
                    let Operation::Write(buffer) = operation else {
                        unreachable!()
                    };
                    let frame = match (index == 0, rest.is_empty() && index + 1 == count) {
                        (true, true) => FrameOptions::FirstAndLastFrame,
                        (true, false) => FrameOptions::FirstFrame,
                        (false, true) => FrameOptions::LastFrame,
                        (false, false) => FrameOptions::NextFrame,
                    };
                    self.write_frame(addr, buffer, frame).await?;
                }
            }
            operations = rest;
        }

        Ok(())
    }
}

// ======== Common

struct State {
    #[allow(unused)]
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: crate::peripheral::RccPeripheral {
    fn regs() -> crate::pac::i2c::I2c;
    fn state() -> &'static State;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance:
    SealedInstance + embassy_hal_internal::PeripheralType + crate::peripheral::RemapBound + 'static
{
    /// Event interrupt for this instance
    type EventInterrupt: interrupt::typelevel::Interrupt;
    /// Error interrupt for this instance
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

foreach_peripheral!(
    (i2c, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::i2c::I2c {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type EventInterrupt = crate::_generated::peripheral_interrupts::$inst::EV;
            type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::ER;
        }
    };
);

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
// pin_trait!(SmbaPin, Instance);
dma_trait!(RxDma, Instance);
dma_trait!(TxDma, Instance);

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => embedded_hal::i2c::ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::Timeout => embedded_hal::i2c::ErrorKind::Other,
            Self::Crc => embedded_hal::i2c::ErrorKind::Other,
            Self::Overrun => embedded_hal::i2c::ErrorKind::Overrun,
            Self::ZeroLengthTransfer => embedded_hal::i2c::ErrorKind::Other,
            Self::BufferTooLong | Self::InvalidAddress => embedded_hal::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::ErrorType for I2c<'d, T, M, Master> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode> embedded_hal::i2c::I2c for I2c<'d, T, M, Master> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address, read)
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address, write)
    }

    fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address, write, read)
    }

    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.blocking_transaction(address, operations)
    }
}

impl<'d, T: Instance> embedded_hal_async::i2c::I2c for I2c<'d, T, Async, Master> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read(address, read).await
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.write(address, write).await
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.write_read(address, write, read).await
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(address, operations).await
    }
}

/// Frame type in I2C transaction.
///
/// This tells each method what kind of framing to use, to generate a (repeated) start condition (ST
/// or SR), and/or a stop condition (SP). For read operations, this also controls whether to send an
/// ACK or NACK after the last byte received.
///
/// For write operations, the following options are identical because they differ only in the (N)ACK
/// treatment relevant for read operations:
///
/// - `FirstFrame` and `FirstAndNextFrame`
/// - `NextFrame` and `LastFrameNoStop`
///
/// Abbreviations used below:
///
/// - `ST` = start condition
/// - `SR` = repeated start condition
/// - `SP` = stop condition
/// - `ACK`/`NACK` = last byte in read operation
#[derive(Copy, Clone)]
#[allow(dead_code)]
enum FrameOptions {
    /// `[ST/SR]+[NACK]+[SP]` First frame (of this type) in transaction and also last frame overall.
    FirstAndLastFrame,
    /// `[ST/SR]+[NACK]` First frame of this type in transaction, last frame in a read operation but
    /// not the last frame overall.
    FirstFrame,
    /// `[ST/SR]+[ACK]` First frame of this type in transaction, neither last frame overall nor last
    /// frame in a read operation.
    FirstAndNextFrame,
    /// `[ACK]` Middle frame in a read operation (neither first nor last).
    NextFrame,
    /// `[NACK]+[SP]` Last frame overall in this transaction but not the first frame.
    LastFrame,
    /// `[NACK]` Last frame in a read operation but not last frame overall in this transaction.
    LastFrameNoStop,
}

fn validate_operations(operations: &[Operation<'_>], dma: bool) -> Result<(), Error> {
    let mut total = 0usize;
    for operation in operations {
        let length = match operation {
            Operation::Read(buffer) if buffer.is_empty() => return Err(Error::Overrun),
            Operation::Read(buffer) => buffer.len(),
            Operation::Write(buffer) => buffer.len(),
        };
        if dma && length > u16::MAX as usize {
            return Err(Error::BufferTooLong);
        }
        total = total.checked_add(length).ok_or(Error::BufferTooLong)?;
    }
    Ok(())
}

#[allow(dead_code)]
impl FrameOptions {
    /// Sends start or repeated start condition before transfer.
    fn send_start(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::FirstAndNextFrame => true,
            Self::NextFrame | Self::LastFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends stop condition after transfer.
    fn send_stop(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::LastFrame => true,
            Self::FirstFrame | Self::FirstAndNextFrame | Self::NextFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends NACK after last byte received, indicating end of read operation.
    fn send_nack(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::LastFrame | Self::LastFrameNoStop => true,
            Self::FirstAndNextFrame | Self::NextFrame => false,
        }
    }
}

/// Iterates over operations in transaction.
///
/// Returns necessary frame options for each operation to uphold the [transaction contract] and have
/// the right start/stop/(N)ACK conditions on the wire.
///
/// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
#[allow(dead_code)]
fn operation_frames<'a, 'b: 'a>(
    operations: &'a mut [embedded_hal::i2c::Operation<'b>],
) -> Result<impl IntoIterator<Item = (&'a mut embedded_hal::i2c::Operation<'b>, FrameOptions)>, Error> {
    use core::iter;

    use embedded_hal::i2c::Operation::{Read, Write};

    // Check empty read buffer before starting transaction. Otherwise, we would risk halting with an
    // error in the middle of the transaction.
    //
    // In principle, we could allow empty read frames within consecutive read operations, as long as
    // at least one byte remains in the final (merged) read operation, but that makes the logic more
    // complicated and error-prone.
    if operations.iter().any(|op| match op {
        Read(read) => read.is_empty(),
        Write(_) => false,
    }) {
        return Err(Error::Overrun);
    }

    let mut operations = operations.iter_mut().peekable();

    let mut next_first_frame = true;

    Ok(iter::from_fn(move || {
        let Some(op) = operations.next() else {
            return None;
        };

        // Is `op` first frame of its type?
        let first_frame = next_first_frame;
        let next_op = operations.peek();

        // Get appropriate frame options as combination of the following properties:
        //
        // - For each first operation of its type, generate a (repeated) start condition.
        // - For the last operation overall in the entire transaction, generate a stop condition.
        // - For read operations, check the next operation: if it is also a read operation, we merge
        //   these and send ACK for all bytes in the current operation; send NACK only for the final
        //   read operation's last byte (before write or end of entire transaction) to indicate last
        //   byte read and release the bus for transmission of the bus master's next byte (or stop).
        //
        // We check the third property unconditionally, i.e. even for write opeartions. This is okay
        // because the resulting frame options are identical for write operations.
        let frame = match (first_frame, next_op) {
            (true, None) => FrameOptions::FirstAndLastFrame,
            (true, Some(Read(_))) => FrameOptions::FirstAndNextFrame,
            (true, Some(Write(_))) => FrameOptions::FirstFrame,
            //
            (false, None) => FrameOptions::LastFrame,
            (false, Some(Read(_))) => FrameOptions::NextFrame,
            (false, Some(Write(_))) => FrameOptions::LastFrameNoStop,
        };

        // Pre-calculate if `next_op` is the first operation of its type. We do this here and not at
        // the beginning of the loop because we hand out `op` as iterator value and cannot access it
        // anymore in the next iteration.
        next_first_frame = match (&op, next_op) {
            (_, None) => false,
            (Read(_), Some(Write(_))) | (Write(_), Some(Read(_))) => true,
            (Read(_), Some(Read(_))) | (Write(_), Some(Write(_))) => false,
        };

        Some((op, frame))
    }))
}
