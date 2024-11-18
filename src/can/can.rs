use super::enums::*;
use super::CanFrame;
use crate as hal;
use crate::can::registers::Registers;
use crate::can::util;
use crate::{into_ref, pac, peripherals, Peripheral, PeripheralRef, RccPeripheral};

pub struct Can<'d, T: Instance> {
    _peri: PeripheralRef<'d, T>,
    fifo: CanFifo,
    last_mailbox_used: usize,
}

impl<'d, T: Instance> Can<'d, T> {
    /// Assumes AFIO & PORTB clocks have been enabled by HAL.
    ///
    /// CAN_RX is mapped to PB8, and CAN_TX is mapped to PB9.
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        tx: impl Peripheral<P = impl TxPin<T>> + 'd,
        fifo: CanFifo,
        mode: CanMode,
        bitrate: u32,
    ) -> Result<Self, ()> {
        into_ref!(peri, rx, tx);

        let this = Self {
            _peri: peri,
            fifo,
            last_mailbox_used: usize::MAX,
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

        // //here should remap functionality be added
        // T::remap(0b10);

        Registers(T::regs()).enter_init_mode(); // CAN enter initialization mode

        // Configure bit timing parameters and CAN operating mode
        let Some(bit_timings) = util::calc_can_timings(T::frequency().0, bitrate) else {
            return Err(());
        };
        // .expect("Bit timing parameters weren't satisfied for CAN clock rate and desired bitrate.");
        Registers(T::regs()).set_bit_timing_and_mode(bit_timings, mode);

        Registers(T::regs()).leave_init_mode(); // Exit CAN initialization mode

        Ok(this)
    }

    pub fn add_filter(&self, filter: CanFilter) {
        Registers(T::regs()).add_filter(filter, &self.fifo);
    }

    /// Puts a frame in the transmit buffer to be sent on the bus.
    ///
    /// If the transmit buffer is full, this function will try to replace a pending
    /// lower priority frame and return the frame that was replaced.
    /// Returns `Err(WouldBlock)` if the transmit buffer is full and no frame can be
    /// replaced.
    pub fn transmit(&self, frame: &CanFrame) -> nb::Result<Option<CanFrame>, CanError> {
        let mailbox_num = match Registers(T::regs()).find_free_mailbox() {
            Some(n) => n,
            None => return Err(nb::Error::WouldBlock),
        };

        Registers(T::regs()).write_frame_mailbox(mailbox_num, frame);

        // Success in readying packet for transmit. No packets can be replaced in the
        // transmit buffer so return None in accordance with embedded-can.
        Ok(None)
    }

    /// Retrieves status of the last frame transmission
    pub fn transmit_status(&self) -> TxStatus {
        if self.last_mailbox_used > 2 {
            return TxStatus::OtherError;
        }

        Registers(T::regs()).transmit_status(self.last_mailbox_used)
    }

    /// Returns a received frame if available.
    pub fn receive(&self) -> nb::Result<CanFrame, CanError> {
        if !Registers(T::regs()).fifo_has_messages_pending(&self.fifo) {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        let frame = Registers(T::regs()).read_frame_fifo(&self.fifo);

        Ok(frame)
    }
}

/// These trait methods are only usable within the embedded_can context.
/// Under normal use of the [Can] instance,
impl<'d, T> embedded_can::nb::Can for Can<'d, T>
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
        Can::receive(self)
    }
}

pub trait SealedInstance: RccPeripheral {
    fn regs() -> pac::can::Can;
    /// Either `0b00`, `0b10` or `b11` on CAN1. `0` or `1` on CAN2.
    fn remap(rm: u8) -> ();
}

pub trait Instance: SealedInstance + 'static {}
pub trait RxPin<T: Instance>: hal::gpio::Pin {}
pub trait TxPin<T: Instance>: hal::gpio::Pin {}

impl SealedInstance for hal::peripherals::CAN1 {
    fn regs() -> pac::can::Can {
        pac::CAN1
    }
    fn remap(rm: u8) {
        pac::AFIO.pcfr1().modify(|w| w.set_can1_rm(rm));
    }
}

impl Instance for hal::peripherals::CAN1 {}

impl RxPin<hal::peripherals::CAN1> for hal::peripherals::PB8 {}
impl TxPin<hal::peripherals::CAN1> for hal::peripherals::PB9 {}

impl RxPin<hal::peripherals::CAN1> for hal::peripherals::PA11 {}
impl TxPin<hal::peripherals::CAN1> for hal::peripherals::PA12 {}
