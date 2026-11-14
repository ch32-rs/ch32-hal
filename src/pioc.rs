//! Programmable I/O controller (PIOC).
//!
//! A small 8-bit core that runs firmware out of a 4 KiB program SRAM and
//! talks to the host through 32 shared bytes (`d0..d31`) plus two host
//! registers: `CTRL_WR` starts a command, `CTRL_RD` returns the firmware
//! status byte and acknowledges the interrupt. This driver is the host side
//! only; the firmware image, its command set and the payload layout are the
//! application's business and live in a protocol crate.
//!
//! # Host sequence
//!
//! 1. `SYS_CFG` with `MST_RESET` set halts the core, which is the only state
//!    in which the program SRAM is writable.
//! 2. `SYS_CFG` with `MST_CLK_GATE` set releases reset and starts the clock.
//! 3. Shared registers or program SRAM carry the payload, then writing
//!    `CTRL_WR` starts one operation.
//! 4. The firmware raises `SYS_CFG.INT_REQ` when it is done; reading
//!    `CTRL_RD` returns the status and clears the request.
//!
//! `SYS_CFG` mixes control and status bits (`INT_REQ`, `DATA_MW_SR`,
//! `DATA_SW_MR`), so it is always written as a whole byte. A read-modify-write
//! would write back stale status bits.
//!
//! # Interrupt
//!
//! With the `embassy` feature the module owns the `PIOC` interrupt and wakes
//! the waiting task; the handler reads `CTRL_RD` because that read is the
//! acknowledge. Without Embassy only the blocking [`Pioc::wait_blocking`]
//! exists.
//!
//! # Family differences
//!
//! The register block is identical on CH32H4 and CH32X0/CH643; what differs is
//! the program-SRAM window and how the peripheral clock is released. H4 goes
//! through `RccPeripheral`; X0 has no RCC enable bit of its own and is gated by
//! `SYS_CFG.MST_CLK_GATE` alone, so a later revision has to relax the
//! [`Instance`] bound. CH32V205 uses a different payload ABI and is not wired
//! up yet.

use crate::gpio::{AfType, AnyPin, OutputType, Speed};
use crate::peripheral::{RccPeripheral, SealedRccPeripheral};
use crate::{pac, Peri, PeripheralType};

/// Instruction words in the 4 KiB program SRAM.
const PROGRAM_WORDS: usize = 2048;

/// Whole program-SRAM window in bytes.
const PROGRAM_RAM_BYTES: usize = PROGRAM_WORDS * 2;

/// Shared register window `d0..d31`, in bytes.
pub const SFR_BYTES: usize = 32;

/// Payload bytes the RGB1W firmware ABI leaves after its `0x400` offset.
///
/// This is the value the protocol layer sees as
/// `CommandPort::program_ram_capacity`; the rest of the window is the image.
pub const PAYLOAD_CAPACITY: usize = PROGRAM_RAM_BYTES - 0x400;

/// Program-SRAM window for the current family.
///
/// The window is a fixed memory range rather than a register block, so it is
/// not part of the register model. CH32V205 puts it at the bottom of SRAM
/// instead of the top 4 KiB and is deliberately absent until its firmware ABI
/// is supported.
#[cfg(ch32h4)]
const PROGRAM_SRAM_BASE: usize = 0x5004_0000;
#[cfg(any(ch32x0, ch643))]
const PROGRAM_SRAM_BASE: usize = 0x2000_4000;

/// Driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The transfer asked for a pin the driver was not built with.
    PinNotConfigured { requested: u8, configured: u8 },
    /// A command was started before an image was loaded.
    ProgramNotLoaded,
    /// The core must be halted for this operation.
    Running,
    /// The image does not fit the program SRAM (lengths in bytes).
    ImageTooLarge { len: usize, capacity: usize },
    /// The transfer does not fit `d0..d31`.
    DataRange { offset: u8, len: usize },
    /// The write does not fit the 4 KiB program SRAM.
    ProgramRamRange { offset: u16, len: usize },
}

pin_trait!(Io0Pin, Instance);
pin_trait!(Io1Pin, Instance);

/// PIOC driver.
pub struct Pioc<'d, T: Instance> {
    _peri: Peri<'d, T>,
    io0: Option<Peri<'d, AnyPin>>,
    io1: Option<Peri<'d, AnyPin>>,
    halted: bool,
    host_config: bool,
    loaded: bool,
}

impl<'d, T: Instance> Pioc<'d, T> {
    /// Take the peripheral, configure the pins and leave the core halted.
    ///
    /// `io0` and `io1` are optional: a firmware image may use either, both or
    /// neither. Passing `None` leaves the pin unclaimed, and a later transfer
    /// that asks for it fails with [`Error::PinNotConfigured`] instead of
    /// driving a pin nobody configured.
    pub fn new<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        io0: Option<Peri<'d, if_remap!(impl Io0Pin<T, REMAP>)>>,
        io1: Option<Peri<'d, impl Io1Pin<T>>>,
    ) -> Self {
        apply_remap!();
        // Both directions are configured as alternate-function push-pull; the
        // firmware switches the pin direction itself through `PORT_DIR`.
        let io0 = match io0 {
            Some(pin) => new_pin!(pin, AfType::output(OutputType::PushPull, Speed::High)),
            None => None,
        };
        let io1 = match io1 {
            Some(pin) => new_pin!(pin, AfType::output(OutputType::PushPull, Speed::High)),
            None => None,
        };
        Self::new_inner(peri, io0, io1)
    }

    /// Take the peripheral with only the IO0 pin claimed.
    ///
    /// A firmware image that drives one wire does not need the second pin, and
    /// the generic [`Pioc::new`] forces the caller to name a type for the `None`
    /// it passes for IO1.
    pub fn new_io0<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        io0: Peri<'d, if_remap!(impl Io0Pin<T, REMAP>)>,
    ) -> Self {
        apply_remap!();
        let io0 = new_pin!(io0, AfType::output(OutputType::PushPull, Speed::High));
        Self::new_inner(peri, io0, None)
    }

    fn new_inner(peri: Peri<'d, T>, io0: Option<Peri<'d, AnyPin>>, io1: Option<Peri<'d, AnyPin>>) -> Self {
        <T as SealedRccPeripheral>::enable_and_reset();

        let mut driver = Self {
            _peri: peri,
            io0,
            io1,
            halted: true,
            host_config: false,
            loaded: false,
        };

        driver.write_sys_cfg();
        // A previous owner may have left a request pending; the read is the
        // only way to clear it.
        let _ = T::regs().ctrl_rd().read();

        #[cfg(feature = "embassy")]
        irq::init();

        driver
    }

    /// Copy an instruction image into program SRAM, little-endian.
    ///
    /// The whole 4 KiB window is available, not just the part below the
    /// payload offset: the image is loaded at offset 0 and may extend past it.
    /// Writing a payload later overwrites that tail, which is why the protocol
    /// layer treats the RAM path as sticky.
    pub fn load_image(&mut self, image: &[u16]) -> Result<(), Error> {
        if !self.halted {
            return Err(Error::Running);
        }
        let bytes = image.len() * 2;
        if bytes > PROGRAM_RAM_BYTES {
            return Err(Error::ImageTooLarge {
                len: bytes,
                capacity: PROGRAM_RAM_BYTES,
            });
        }
        for (index, word) in image.iter().enumerate() {
            self.write_ram((index * 2) as u16, &word.to_le_bytes())?;
        }
        self.loaded = true;
        Ok(())
    }

    /// Wait for the firmware to finish, without Embassy.
    ///
    /// Spins until `INT_REQ` is set and returns the status byte. There is no
    /// timeout: a firmware image that never completes blocks here forever, so
    /// a caller that wants a deadline needs the async path.
    pub fn wait_blocking(&mut self) -> Result<u8, Error> {
        while !T::regs().sys_cfg().read().int_req() {
            core::hint::spin_loop();
        }
        Ok(T::regs().ctrl_rd().read())
    }

    /// Halt the core while keeping the loaded image and the pin claims.
    pub fn halt(&mut self) {
        self.halted = true;
        self.write_sys_cfg();
    }

    /// Write `SYS_CFG` from the driver state.
    ///
    /// Always a whole-byte write: the register also carries `INT_REQ`,
    /// `DATA_MW_SR` and `DATA_SW_MR`, which a `modify` would write back.
    fn write_sys_cfg(&self) {
        T::regs().sys_cfg().write(|w| {
            w.set_mst_reset(self.halted);
            w.set_mst_clk_gate(!self.halted);
            w.set_mst_io_en0(self.io0.is_some());
            w.set_mst_io_en1(self.io1.is_some());
            w.set_mst_cfg_b4(self.host_config);
        });
    }

    /// Reject a pin claim the driver was not built for.
    ///
    /// `SYS_CFG` can only enable the pins this driver owns, so asking for a
    /// third pin would run the firmware on a wire nobody configured.
    #[cfg(feature = "embassy")]
    fn check_pins(&self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        let configured = u8::from(self.io0.is_some()) | (u8::from(self.io1.is_some()) << 1);
        let requested = pins.bits();
        if requested & !configured != 0 {
            return Err(Error::PinNotConfigured { requested, configured });
        }
        Ok(())
    }

    /// Copy bytes into the program SRAM window; the core must be halted.
    fn write_ram(&mut self, offset: u16, data: &[u8]) -> Result<(), Error> {
        if !self.halted {
            return Err(Error::Running);
        }
        let end = usize::from(offset) + data.len();
        if end > PROGRAM_RAM_BYTES {
            return Err(Error::ProgramRamRange {
                offset,
                len: data.len(),
            });
        }
        for (index, byte) in data.iter().copied().enumerate() {
            let address = PROGRAM_SRAM_BASE + usize::from(offset) + index;
            // The window is ordinary SRAM on the peripheral bus; nothing else
            // in the program can alias it, so a volatile byte write is enough.
            unsafe { core::ptr::write_volatile(address as *mut u8, byte) };
        }
        Ok(())
    }

    #[cfg(feature = "embassy")]
    fn check_data_range(offset: u8, len: usize) -> Result<(), Error> {
        let end = usize::from(offset) + len;
        if end > SFR_BYTES {
            return Err(Error::DataRange { offset, len });
        }
        Ok(())
    }
}

#[cfg(feature = "embassy")]
impl<'d, T: Instance> pioc_programs::CommandPort for Pioc<'d, T> {
    type Error = Error;

    fn reset_halted(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        self.check_pins(pins)?;
        self.halted = true;
        self.write_sys_cfg();
        Ok(())
    }

    fn run(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        self.check_pins(pins)?;
        if !self.loaded {
            return Err(Error::ProgramNotLoaded);
        }
        self.halted = false;
        self.write_sys_cfg();
        Ok(())
    }

    fn set_host_config(&mut self, enabled: bool) -> Result<(), Error> {
        self.host_config = enabled;
        self.write_sys_cfg();
        Ok(())
    }

    fn write_data(&mut self, offset: u8, data: &[u8]) -> Result<(), Error> {
        Self::check_data_range(offset, data.len())?;
        for (index, value) in data.iter().copied().enumerate() {
            T::regs().data_reg(usize::from(offset) + index).write(|w| *w = value);
        }
        Ok(())
    }

    fn read_data(&mut self, offset: u8, data: &mut [u8]) -> Result<(), Error> {
        Self::check_data_range(offset, data.len())?;
        for (index, value) in data.iter_mut().enumerate() {
            *value = T::regs().data_reg(usize::from(offset) + index).read();
        }
        Ok(())
    }

    fn write_program_ram(&mut self, offset: u16, data: &[u8]) -> Result<(), Error> {
        self.write_ram(offset, data)
    }

    fn program_ram_capacity(&self) -> usize {
        PAYLOAD_CAPACITY
    }

    fn start_command(&mut self, command: u8) -> Result<(), Error> {
        // A wait that was cancelled — `embassy_time::with_timeout`, for one —
        // leaves the completion of the previous transfer in the slot. It must
        // not be handed to this one, or the driver would proceed before the
        // firmware has even started.
        irq::STATE.lock(|state| state.borrow_mut().status = None);
        T::regs().ctrl_wr().write(|w| *w = command);
        Ok(())
    }

    async fn wait_for_interrupt(&mut self) -> Result<u8, Error> {
        use core::future::poll_fn;
        use core::task::Poll;

        poll_fn(|cx| {
            // Status slot, `INT_REQ` check and waker registration all happen
            // under the lock the handler takes. Splitting them lets the handler
            // run in between: it would acknowledge the request and store a
            // status this poll never sees, or the poll would acknowledge one
            // the handler then reads again as a spurious zero.
            irq::STATE.lock(|state| {
                let mut state = state.borrow_mut();
                if let Some(status) = state.status.take() {
                    return Poll::Ready(Ok(status));
                }
                if T::regs().sys_cfg().read().int_req() {
                    // Reading `CTRL_RD` is the acknowledge.
                    return Poll::Ready(Ok(T::regs().ctrl_rd().read()));
                }
                state.waker = Some(cx.waker().clone());
                Poll::Pending
            })
        })
        .await
    }
}

#[cfg(feature = "embassy")]
mod irq {
    use core::cell::RefCell;
    use core::task::Waker;

    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::blocking_mutex::Mutex;
    use qingke::interrupt::Priority;
    use qingke_rt::interrupt;

    use crate::pac::{self, Interrupt};

    pub(super) struct State {
        /// Status byte of the last completed operation; one slot is enough for
        /// the command/response firmware the driver supports today.
        pub(super) status: Option<u8>,
        pub(super) waker: Option<Waker>,
    }

    pub(super) static STATE: Mutex<CriticalSectionRawMutex, RefCell<State>> = Mutex::const_new(
        CriticalSectionRawMutex::new(),
        RefCell::new(State {
            status: None,
            waker: None,
        }),
    );

    #[interrupt]
    unsafe fn PIOC() {
        // A request can be acknowledged by `wait_for_interrupt` before the
        // pending interrupt is delivered, which leaves this handler running
        // with nothing to read. Reading `CTRL_RD` then returns whatever the
        // register happens to hold and would be taken for the completion of the
        // *next* transfer, so the driver would start it again far too early.
        if !pac::PIOC.sys_cfg().read().int_req() {
            return;
        }
        // Reading `CTRL_RD` returns the status and clears `INT_REQ`: the read
        // is the acknowledge, so it has to happen here and not in the future.
        let status = pac::PIOC.ctrl_rd().read();
        let waker = STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = Some(status);
            state.waker.take()
        });
        // Wake outside the critical section: `wake` may run executor code.
        if let Some(waker) = waker {
            waker.wake();
        }
    }

    pub(super) fn init() {
        // Match the priority of the active Embassy time driver so the two
        // handlers cannot nest; QingKe can leave `MIE` clear after a nested
        // interrupt returns. See `awu.rs` for the same rule.
        #[cfg(time_driver_timer)]
        const PRIORITY: Priority = Priority::P0;
        #[cfg(not(time_driver_timer))]
        const PRIORITY: Priority = Priority::P15;

        unsafe {
            qingke::pfic::set_priority(Interrupt::PIOC as u8, PRIORITY as u8);
            qingke::pfic::enable_interrupt(Interrupt::PIOC as u8);
        }
    }
}

trait SealedInstance {
    fn regs() -> pac::pioc::Pioc;
}

/// PIOC instance trait.
///
/// `RccPeripheral` is part of the bound because CH32H4 releases the peripheral
/// clock through RCC. CH32X0/CH643 have no such bit and will need either a cfg
/// here or a reset-only RCC entry in the register data.
#[allow(private_bounds)]
pub trait Instance: SealedInstance + RccPeripheral + PeripheralType {}

foreach_peripheral!(
    (pioc, $inst:ident) => {
        impl SealedInstance for crate::peripherals::$inst {
            fn regs() -> pac::pioc::Pioc {
                pac::$inst
            }
        }

        impl Instance for crate::peripherals::$inst {}
    };
);
