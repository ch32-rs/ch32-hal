//! Programmable I/O controller (PIOC).
//!
//! A small 8-bit core that runs firmware out of a 4 KiB program SRAM and
//! talks to the host through 32 shared bytes (`d0..d31`) plus two host
//! registers: `CTRL_WR` starts a command, `CTRL_RD` returns the firmware
//! status byte. This driver is the host side
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
//!    `CTRL_RD` returns the status, and a write to `CTRL_RD` (data ignored)
//!    clears the request. The read alone does **not** clear it.
//!
//! `SYS_CFG` mixes control and status bits (`INT_REQ`, `DATA_MW_SR`,
//! `DATA_SW_MR`), so it is always written as a whole byte. A read-modify-write
//! would write back stale status bits.
//!
//! # Interrupt
//!
//! With the `embassy` feature the module owns the `PIOC` interrupt and wakes
//! the waiting task; the handler reads the status from `CTRL_RD` and then
//! writes it to clear the request. The transfer methods are also available
//! without Embassy:
//! [`Pioc::write_data`], [`Pioc::start_command`] and [`Pioc::wait_blocking`]
//! form the synchronous command path.
//!
//! The interrupt has one global completion slot, so use one `Pioc` per
//! program. [`Drop`] halts the core and releases the interrupt.
//!
//! # Program SRAM and the linker
//!
//! The image window is 4 KiB. CH32H4 places it in a separate block at
//! `0x5004_0000`. CH32X0 and CH643 place it at the top of the main SRAM
//! (`0x2000_4000..0x2000_5000`), so the CPU stack and heap have to stay below
//! it. A project that enables PIOC on those parts must keep the stack and heap
//! below the window in its linker script, as `examples/ch32x035/pioc.x` does by
//! moving `_stack_start`; otherwise [`Pioc::load_image`] overwrites live
//! memory. The HAL cannot enforce this at compile time.
//!
//! # Pin configuration
//!
//! The constructors put IO0/IO1 into alternate-function push-pull mode at high
//! speed. The firmware owns the direction through `pins.dir`; a 1-Wire protocol
//! has to switch to input to release the line before reading it. Only the
//! WS2812 path is board-tested.
//!
//! # Family differences
//!
//! The register block is identical on CH32H4 and CH32X0/CH643; what differs is
//! the program-SRAM window and how the peripheral is released. H4 has an RCC
//! clock-enable bit; X0 and CH643 have a reset bit only and gate the core with
//! `SYS_CFG.MST_CLK_GATE`. CH32V205 uses a different payload ABI and is not
//! wired up yet.

pub use pioc_programs::PinMask;

use crate::gpio::{AfType, AnyPin, OutputType, Speed};
#[cfg(not(any(ch32x0, ch643)))]
use crate::peripheral::{RccPeripheral, SealedRccPeripheral};
use crate::{pac, Peri, PeripheralType};

/// Instruction words in the 4 KiB program SRAM.
const PROGRAM_WORDS: usize = 2048;

/// Whole program-SRAM window in bytes.
const PROGRAM_RAM_BYTES: usize = PROGRAM_WORDS * 2;

/// Shared register window `d0..d31`, in bytes.
pub const SFR_BYTES: usize = 32;

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
    /// `SYS_CFG.MST_IO_EN0/1` for the current claim.
    claimed: u8,
}

impl<'d, T: Instance> Pioc<'d, T> {
    /// Take the peripheral with both pins claimed, and leave the core halted.
    ///
    /// A firmware image may use either pin; an unclaimed pin makes a later
    /// transfer that asks for it fail with [`Error::PinNotConfigured`] instead
    /// of driving a pin nobody configured. This constructor is only usable
    /// when both arguments are `Some`: a `None` cannot infer the opaque pin
    /// type, so use [`Pioc::new_io0`] or [`Pioc::new_io1`] for one wire.
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
    /// the generic [`Pioc::new`] cannot infer a type for the `None` it would
    /// pass for IO1.
    pub fn new_io0<#[cfg(not(afio_h4))] const REMAP: u8>(
        peri: Peri<'d, T>,
        io0: Peri<'d, if_remap!(impl Io0Pin<T, REMAP>)>,
    ) -> Self {
        apply_remap!();
        let io0 = new_pin!(io0, AfType::output(OutputType::PushPull, Speed::High));
        Self::new_inner(peri, io0, None)
    }

    /// Take the peripheral with only the IO1 pin claimed.
    ///
    /// IO1 has no pin remap on any supported family, so there is no `REMAP`
    /// parameter. This exists for the same reason as [`Pioc::new_io0`].
    pub fn new_io1(peri: Peri<'d, T>, io1: Peri<'d, impl Io1Pin<T>>) -> Self {
        let io1 = new_pin!(io1, AfType::output(OutputType::PushPull, Speed::High));
        Self::new_inner(peri, None, io1)
    }

    fn new_inner(peri: Peri<'d, T>, io0: Option<Peri<'d, AnyPin>>, io1: Option<Peri<'d, AnyPin>>) -> Self {
        // CH32H4 releases the peripheral clock through RCC. X0 and CH643 have a
        // reset bit only: the core clock is `SYS_CFG.MST_CLK_GATE`, which
        // `write_sys_cfg` drives.
        #[cfg(not(any(ch32x0, ch643)))]
        <T as SealedRccPeripheral>::enable_and_reset();
        #[cfg(any(ch32x0, ch643))]
        {
            let rcc = crate::pac::RCC;
            rcc.ahbrstr().modify(|w| w.set_piocrst(true));
            rcc.ahbrstr().modify(|w| w.set_piocrst(false));
        }

        let claimed = u8::from(io0.is_some()) | (u8::from(io1.is_some()) << 1);
        let driver = Self {
            _peri: peri,
            io0,
            io1,
            halted: true,
            host_config: false,
            loaded: false,
            claimed,
        };

        driver.write_sys_cfg();
        // A previous owner may have left a request pending; a write to
        // `CTRL_RD` is the only way to clear it.
        T::regs().ctrl_rd().write(|w| *w = 0);

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

    /// Wait for the firmware to finish and return its status byte.
    ///
    /// Spins until `INT_REQ` is set. There is no timeout: a firmware image that
    /// never completes blocks here forever, so a caller that wants a deadline
    /// needs the async path. With the `embassy` feature the interrupt handler
    /// may consume the request first; this method reads the same completion
    /// slot under the same lock.
    pub fn wait_blocking(&mut self) -> u8 {
        loop {
            if let Some(status) = self.poll_status() {
                return status;
            }
            core::hint::spin_loop();
        }
    }

    /// Take a completed status byte without blocking.
    fn poll_status(&self) -> Option<u8> {
        #[cfg(feature = "embassy")]
        {
            irq::STATE.lock(|state| {
                let mut state = state.borrow_mut();
                if let Some(status) = state.status.take() {
                    return Some(status);
                }
                if T::regs().sys_cfg().read().int_req() {
                    return Some(self.read_status());
                }
                None
            })
        }
        #[cfg(not(feature = "embassy"))]
        {
            if T::regs().sys_cfg().read().int_req() {
                Some(self.read_status())
            } else {
                None
            }
        }
    }

    /// Halt the core while keeping the loaded image and the pin claims.
    pub fn halt(&mut self) {
        self.halted = true;
        self.write_sys_cfg();
    }

    /// Assert reset, stop the clock, and claim the selected pins.
    pub fn reset_halted(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        self.check_pins(pins)?;
        self.halted = true;
        self.claimed = pins.bits();
        self.write_sys_cfg();
        Ok(())
    }

    /// Release reset, run the clock, and claim the selected pins.
    pub fn run(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        self.check_pins(pins)?;
        if !self.loaded {
            return Err(Error::ProgramNotLoaded);
        }
        self.halted = false;
        self.claimed = pins.bits();
        self.write_sys_cfg();
        Ok(())
    }

    /// Set `sys.ctrl.host_config`, used by RGB1W as the parasite-power flag.
    pub fn set_host_config(&mut self, enabled: bool) -> Result<(), Error> {
        self.host_config = enabled;
        self.write_sys_cfg();
        Ok(())
    }

    /// Access shared registers `d0..d31`.
    pub fn write_data(&mut self, offset: u8, data: &[u8]) -> Result<(), Error> {
        Self::check_data_range(offset, data.len())?;
        for (index, value) in data.iter().copied().enumerate() {
            T::regs().data_reg(usize::from(offset) + index).write(|w| *w = value);
        }
        Ok(())
    }

    /// Read shared registers `d0..d31`.
    pub fn read_data(&mut self, offset: u8, data: &mut [u8]) -> Result<(), Error> {
        Self::check_data_range(offset, data.len())?;
        for (index, value) in data.iter_mut().enumerate() {
            *value = T::regs().data_reg(usize::from(offset) + index).read();
        }
        Ok(())
    }

    /// Write firmware-owned shared SRAM while PIOC is halted.
    pub fn write_program_ram(&mut self, offset: u16, data: &[u8]) -> Result<(), Error> {
        self.write_ram(offset, data)
    }

    /// Whole program-SRAM window available through [`Self::write_program_ram`].
    ///
    /// The payload offset and the bytes left for payloads are properties of a
    /// firmware ABI, so a protocol driver applies its own limit.
    pub fn program_ram_capacity(&self) -> usize {
        PROGRAM_RAM_BYTES
    }

    /// Write `cmd.in`, starting a firmware operation.
    ///
    /// A previous completion that was never consumed is discarded first. That
    /// covers a cancelled wait (`embassy_time::with_timeout`, for one): its
    /// `INT_REQ` would otherwise be taken as the completion of this command.
    /// The acknowledge and the command write stay in one critical section.
    pub fn start_command(&mut self, command: u8) -> Result<(), Error> {
        #[cfg(feature = "embassy")]
        irq::STATE.lock(|state| {
            state.borrow_mut().status = None;
            self.acknowledge_pending();
            T::regs().ctrl_wr().write(|w| *w = command);
        });
        #[cfg(not(feature = "embassy"))]
        {
            self.acknowledge_pending();
            T::regs().ctrl_wr().write(|w| *w = command);
        }
        Ok(())
    }

    /// Write `SYS_CFG` from the driver state.
    ///
    /// Always a whole-byte write: the register also carries `INT_REQ`,
    /// `DATA_MW_SR` and `DATA_SW_MR`, which a `modify` would write back.
    fn write_sys_cfg(&self) {
        T::regs().sys_cfg().write(|w| {
            w.set_mst_reset(self.halted);
            w.set_mst_clk_gate(!self.halted);
            w.set_mst_io_en0(self.claimed & 1 != 0);
            w.set_mst_io_en1(self.claimed & 2 != 0);
            w.set_mst_cfg_b4(self.host_config);
        });
    }

    /// Pins this driver was built with, as a [`pioc_programs::PinMask`] value.
    fn configured_pins(&self) -> u8 {
        u8::from(self.io0.is_some()) | (u8::from(self.io1.is_some()) << 1)
    }

    /// Reject a pin claim the driver was not built for.
    ///
    /// `SYS_CFG` can only enable the pins this driver owns, so asking for a
    /// third pin would run the firmware on a wire nobody configured.
    fn check_pins(&self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        let configured = self.configured_pins();
        let requested = pins.bits();
        if requested & !configured != 0 {
            return Err(Error::PinNotConfigured { requested, configured });
        }
        Ok(())
    }

    /// Read the status byte and acknowledge the request.
    ///
    /// Reading `CTRL_RD` returns the status and clears `DATA_SW_MR`, but it
    /// does **not** clear `INT_REQ` on silicon. Only a write to `CTRL_RD`
    /// (data ignored) clears the request, so the write has to follow every
    /// read; skipping it makes the interrupt fire again immediately.
    fn read_status(&self) -> u8 {
        let status = T::regs().ctrl_rd().read();
        T::regs().ctrl_rd().write(|w| *w = 0);
        status
    }

    /// Acknowledge a completion left behind by a cancelled or unclaimed wait.
    fn acknowledge_pending(&self) {
        if T::regs().sys_cfg().read().int_req() {
            self.read_status();
        }
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

    fn check_data_range(offset: u8, len: usize) -> Result<(), Error> {
        let end = usize::from(offset) + len;
        if end > SFR_BYTES {
            return Err(Error::DataRange { offset, len });
        }
        Ok(())
    }
}

impl<T: Instance> Drop for Pioc<'_, T> {
    fn drop(&mut self) {
        // Stop the core and release both pins before the `Peri` handles go
        // away; a dropped driver must not keep driving firmware on its own.
        self.halted = true;
        self.claimed = 0;
        self.write_sys_cfg();
        #[cfg(feature = "embassy")]
        irq::deinit();
    }
}

#[cfg(feature = "embassy")]
impl<'d, T: Instance> pioc_programs::CommandPort for Pioc<'d, T> {
    type Error = Error;

    fn reset_halted(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        Pioc::reset_halted(self, pins)
    }

    fn run(&mut self, pins: pioc_programs::PinMask) -> Result<(), Error> {
        Pioc::run(self, pins)
    }

    fn set_host_config(&mut self, enabled: bool) -> Result<(), Error> {
        Pioc::set_host_config(self, enabled)
    }

    fn write_data(&mut self, offset: u8, data: &[u8]) -> Result<(), Error> {
        Pioc::write_data(self, offset, data)
    }

    fn read_data(&mut self, offset: u8, data: &mut [u8]) -> Result<(), Error> {
        Pioc::read_data(self, offset, data)
    }

    fn write_program_ram(&mut self, offset: u16, data: &[u8]) -> Result<(), Error> {
        Pioc::write_program_ram(self, offset, data)
    }

    fn program_ram_capacity(&self) -> usize {
        Pioc::program_ram_capacity(self)
    }

    fn start_command(&mut self, command: u8) -> Result<(), Error> {
        Pioc::start_command(self, command)
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
                    // The write in `read_status` is the acknowledge; it runs
                    // under the lock so the handler cannot acknowledge the
                    // same request a second time in between.
                    return Poll::Ready(Ok(self.read_status()));
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
        // Reading `CTRL_RD` returns the status but does not clear `INT_REQ`;
        // the write after the read is the acknowledge, so both have to happen
        // here and not in the future.
        let status = pac::PIOC.ctrl_rd().read();
        pac::PIOC.ctrl_rd().write(|w| *w = 0);
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

    pub(super) fn deinit() {
        unsafe {
            qingke::pfic::disable_interrupt(Interrupt::PIOC as u8);
            qingke::pfic::unpend_interrupt(Interrupt::PIOC as u8);
        }
        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = None;
            state.waker = None;
        });
    }
}

trait SealedInstance {
    fn regs() -> pac::pioc::Pioc;
}

/// PIOC instance trait.
///
/// `RccPeripheral` is part of the bound only where the peripheral clock has an
/// enable bit (CH32H4). X0 and CH643 reset the block through `AHBRSTR` and gate
/// the core itself with `SYS_CFG.MST_CLK_GATE`, so there is nothing for the
/// trait to enable.
#[allow(private_bounds)]
#[cfg(not(any(ch32x0, ch643)))]
pub trait Instance: SealedInstance + RccPeripheral + PeripheralType {}

/// `RemapBound` carries the `PCFR1.PIOC_RM` write on X0/CH643 and is empty on
/// CH32H4, which selects the function per pin instead.
#[allow(private_bounds)]
#[cfg(any(ch32x0, ch643))]
pub trait Instance: SealedInstance + crate::peripheral::RemapBound + PeripheralType {}

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
