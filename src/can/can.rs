use core::future::poll_fn;
use core::marker::PhantomData;
use core::task::Poll;

use embassy_sync::waitqueue::AtomicWaker;

use super::enums::*;
use super::filter::{BitMode, FilterMode};
use super::{CanFilter, CanFrame};
use crate::can::registers::Registers;
use crate::can::util;
use crate::internal::drop::OnDrop;
use crate::mode::{Async, Blocking, Mode, NonBlocking};
use crate::{
    interrupt, into_ref, pac, peripherals, Peripheral, PeripheralRef, RccPeripheral, RemapPeripheral, Timeout,
};

/// Receive interrupt handler.
pub struct ReceiveInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ReceiveInterrupt> for ReceiveInterruptHandler<T> {
    unsafe fn on_interrupt() {
        let regs = &T::regs();
        T::state().waker.wake();
        critical_section::with(|_| {
            regs.intenr().modify(|w| {
                w.set_fmpie0(false); // Disable FIFO 0 message pending interrupt
            });
        });
    }
}

/// Can config
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// Timeout.
    #[cfg(feature = "embassy")]
    pub timeout: embassy_time::Duration,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            #[cfg(feature = "embassy")]
            timeout: embassy_time::Duration::from_millis(1000),
        }
    }
}

pub struct Can<'d, T: Instance, M: Mode> {
    _peri: PeripheralRef<'d, T>,
    fifo: CanFifo,
    last_mailbox_used: usize,
    #[cfg(feature = "embassy")]
    timeout: embassy_time::Duration,
    _phantom: PhantomData<(&'d mut T, M)>,
}

#[derive(Debug)]
pub enum CanInitError {
    InvalidTimings,
}

impl<'d, T: Instance> Can<'d, T, Async> {
    pub fn new_async<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::ReceiveInterrupt, ReceiveInterruptHandler<T>> + 'd,
        fifo: CanFifo,
        mode: CanMode,
        bitrate: u32,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, rx, tx, fifo, mode, bitrate, config)
    }

    pub async fn recv(&self) -> Result<CanFrame, CanError> {
        let on_drop = OnDrop::new(|| {
            // Disable interrupt if the future is canceled
            T::regs().intenr().modify(|w| {
                w.set_fmpie0(false); // Disable FIFO 0 message pending interrupt
            })
        });
        poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            let regs = Registers::new::<T>();

            if regs.pending_messages(self.fifo) == 0 {
                // No messages available, wait for a new message
                regs.0.intenr().modify(|w| {
                    w.set_fmpie0(true); // Enable FIFO 0 message pending interrupt
                });
                Poll::Pending
            } else {
                Poll::Ready(Ok(()))
            }
        })
        .await?;
        drop(on_drop);

        self.receive_inner()
    }
}

impl<'d, T: Instance> Can<'d, T, Blocking> {
    pub fn new_blocking<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        fifo: CanFifo,
        mode: CanMode,
        bitrate: u32,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, rx, tx, fifo, mode, bitrate, config)
    }

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will block until a mailbox becomes available or the timeout is reached.
    fn blocking_transmit(&mut self, frame: &CanFrame) -> Result<(), CanError> {
        let regs = Registers::new::<T>();
        let timeout = self.timeout();

        let mailbox_num = loop {
            if let Some(mailbox_num) = regs.find_free_mailbox() {
                break mailbox_num;
            };
            timeout.check().ok_or(CanError::Timeout)?;
        };
        regs.write_frame_mailbox(mailbox_num, frame);
        self.last_mailbox_used = mailbox_num;
        Ok(())
    }

    /// Blocks until a frame was received or an error occurred.
    fn blocking_recv(&self) -> Result<CanFrame, CanError> {
        let timeout = self.timeout();

        while Registers::new::<T>().pending_messages(self.fifo) == 0 {
            timeout.check().ok_or(CanError::Timeout)?;
        }

        let frame = self.receive_inner()?;
        Ok(frame)
    }
}

impl<'d, T: Instance> Can<'d, T, NonBlocking> {
    pub fn new_nb<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        fifo: CanFifo,
        mode: CanMode,
        bitrate: u32,
        config: Config,
    ) -> Result<Self, CanInitError> {
        Self::new_inner(peri, rx, tx, fifo, mode, bitrate, config)
    }

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will try to replace a pending
    /// lower priority frame and return the frame that was replaced.
    /// Returns `Err(WouldBlock)` if the transmit buffer is full and no frame can be
    /// replaced.
    pub fn transmit(&mut self, frame: &CanFrame) -> nb::Result<Option<CanFrame>, CanError> {
        let mailbox_num = match Registers::new::<T>().find_free_mailbox() {
            Some(n) => n,
            None => return Err(nb::Error::WouldBlock),
        };

        Registers::new::<T>().write_frame_mailbox(mailbox_num, frame);
        self.last_mailbox_used = mailbox_num;

        // Success in readying packet for transmit. No packets can be replaced in the
        // transmit buffer so return None in accordance with embedded-can.
        Ok(None)
    }

    /// Try to read the next message from the queue.
    /// If there are no messages, an error is returned.
    pub fn try_recv(&self) -> nb::Result<CanFrame, CanError> {
        let regs = Registers::new::<T>();

        //check pending messages
        if regs.pending_messages(self.fifo) == 0 {
            return Err(nb::Error::WouldBlock);
        }

        self.receive_inner().map_err(nb::Error::Other)
    }
}

impl<'d, T: Instance, M: Mode> Can<'d, T, M> {
    /// Assumes AFIO & PORTB clocks have been enabled by HAL.
    ///
    /// CAN_RX is mapped to PB8, and CAN_TX is mapped to PB9.
    fn new_inner<const REMAP: u8>(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T, REMAP>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T, REMAP>> + 'd,
        fifo: CanFifo,
        mode: CanMode,
        bitrate: u32,
        config: Config,
    ) -> Result<Self, CanInitError> {
        into_ref!(peri, rx, tx);

        let this = Self {
            _peri: peri,
            fifo,
            last_mailbox_used: usize::MAX,
            #[cfg(feature = "embassy")]
            timeout: config.timeout,
            _phantom: PhantomData,
        };
        T::enable_and_reset(); // Enable CAN peripheral

        rx.set_mode_cnf(
            pac::gpio::vals::Mode::INPUT,
            pac::gpio::vals::Cnf::PULL_IN__AF_PUSH_PULL_OUT,
        );

        tx.set_mode_cnf(
            pac::gpio::vals::Mode::OUTPUT_50MHZ,
            pac::gpio::vals::Cnf::PULL_IN__AF_PUSH_PULL_OUT,
        );
        T::set_remap(REMAP);

        // //here should remap functionality be added
        // T::remap(0b10);

        unsafe {
            use crate::interrupt::typelevel::Interrupt;
            T::ReceiveInterrupt::enable();
        };

        Registers::new::<T>().enter_init_mode(); // CAN enter initialization mode

        // Configure bit timing parameters and CAN operating mode
        let Some(bit_timings) = util::calc_can_timings(T::frequency().0, bitrate) else {
            return Err(CanInitError::InvalidTimings);
        };

        Registers::new::<T>().set_bit_timing_and_mode(bit_timings, mode);

        Registers::new::<T>().leave_init_mode(); // Exit CAN initialization mode

        Ok(this)
    }

    /// Each filter bank consists of 2 32-bit registers CAN_FxR0 and CAN_FxR1
    pub fn add_filter<BIT: BitMode, MODE: FilterMode>(&self, filter: CanFilter<BIT, MODE>) {
        let can = T::regs();
        let fifo = &self.fifo;

        can.fctlr().modify(|w| w.set_finit(true)); // Enable filter init mode

        can.fscfgr()
            .modify(|w| w.set_fsc(filter.bank, filter.bit_mode.val_bool())); // Set filter scale config (32bit or 16bit mode)

        can.fr(filter.fr_id_value_reg())
            .write_value(crate::pac::can::regs::Fr(filter.id_value)); // Set filter's id value to match/mask

        can.fr(filter.fr_id_mask_reg())
            .write_value(crate::pac::can::regs::Fr(filter.id_mask)); // Set filter's id bits to mask

        can.fmcfgr().modify(|w| w.set_fbm(filter.bank, filter.mode.val_bool())); // Set new filter's operating mode

        can.fafifor()
            .modify(|w| w.set_ffa(filter.bank, fifo.val_bool())); // Associate CAN's FIFO to new filter

        can.fwr().modify(|w| w.set_fact(filter.bank, true)); // Activate new filter

        can.fctlr().modify(|w| w.set_finit(false)); // Exit filter init mode
    }

    /// Retrieves status of the last frame transmission
    pub fn transmit_status(&self) -> TxStatus {
        if self.last_mailbox_used > 2 {
            return TxStatus::OtherError;
        }

        Registers::new::<T>().transmit_status(self.last_mailbox_used)
    }

    /// Receives a CAN frame from the hardware. Caller must make sure that a frame is available
    /// in the FIFO before calling this method.
    fn receive_inner(&self) -> Result<CanFrame, CanError> {
        let regs = Registers::new::<T>();
        let fifo = self.fifo.val();

        let dlc = regs.0.rxmdtr(fifo).read().dlc() as usize;
        if dlc > 8 {
            return Err(CanError::Form);
        }
        let rxmir = regs.0.rxmir(fifo).read();

        let id = if rxmir.ide() {
            let raw_id = ((rxmir.stid() as u32) << 18) | rxmir.exid();
            embedded_can::Id::from(unsafe { embedded_can::ExtendedId::new_unchecked(raw_id & 0x1FFFFFFF) })
        } else {
            embedded_can::Id::Standard(embedded_can::StandardId::new(rxmir.stid()).unwrap())
        };

        let frame_data_unordered: u64 =
            ((regs.0.rxmdhr(fifo).read().0 as u64) << 32) | regs.0.rxmdlr(fifo).read().0 as u64;

        let frame = CanFrame::new_from_data_registers(id, frame_data_unordered, dlc);

        regs.0.rfifo(fifo).write(|w| {
            //set the data was read
            w.set_rfom(true);
        });

        Ok(frame)
    }

    fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "embassy")]
            deadline: embassy_time::Instant::now() + self.timeout,
        }
    }
}

/// These trait methods are only usable within the embedded_can context.
/// Under normal use of the [Can] instance,
impl<'d, T> embedded_can::nb::Can for Can<'d, T, NonBlocking>
where
    T: Instance,
{
    type Frame = CanFrame;
    type Error = CanError;

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will try to replace a pending
    /// lower priority frame and return the frame that was replaced.
    /// Returns `Err(WouldBlock)` if the transmit buffer is full and no frame can be
    /// replaced.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        Can::transmit(self, frame)
    }

    /// Returns a received frame if available.
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        Can::try_recv(self)
    }
}

impl<'d, T> embedded_can::blocking::Can for Can<'d, T, Blocking>
where
    T: Instance,
{
    type Frame = CanFrame;
    type Error = CanError;

    /// Puts a frame in the transmit buffer. Blocks until space is available in
    /// the transmit buffer.
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        Can::blocking_transmit(self, frame)
    }

    /// Blocks until a frame was received or an error occured.
    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        Can::blocking_recv(self)
    }
}

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

pub trait SealedInstance: RccPeripheral + RemapPeripheral {
    fn regs() -> pac::can::Can;
    // Either `0b00`, `0b10` or `b11` on CAN1. `0` or `1` on CAN2.
    // fn remap(rm: u8) -> ();

    fn state() -> &'static State;
}

pub trait Instance: SealedInstance + 'static {
    type ReceiveInterrupt: crate::interrupt::typelevel::Interrupt;
}

pin_trait!(RxPin, Instance);
pin_trait!(TxPin, Instance);

foreach_peripheral!(
    (can, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            // FIXME: CH32L1 supports CANFD, and is compatible with the CAN peripheral.
            fn regs() -> crate::pac::can::Can {
                #[cfg(ch32l1)]
                return unsafe { crate::pac::can::Can::from_ptr(crate::pac::$inst.as_ptr()) };
                #[cfg(not(ch32l1))]
                return crate::pac::$inst;
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
           type ReceiveInterrupt = crate::_generated::peripheral_interrupts::$inst::RX0;
        }
    };
);
