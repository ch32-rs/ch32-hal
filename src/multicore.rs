//! Spawn code on hart 1 of a CH32H4, in the style of
//! `embassy_rp::multicore::spawn_core1`.
//!
//! One image runs on both harts: hart 0 boots as usual (`_start` copies
//! `.highcode` into the zero-wait shared block, clears `.bss`, calls `main`),
//! and [`spawn_core1`] wakes hart 1 into a trampoline inside the *same*
//! image. Hart 1 sets its own `gp`/`sp`, then runs the closure you pass; none
//! of `.data`/`.bss`/the heap is initialised a second time, so both harts see
//! one instance of every `static`.
//!
//! ```ignore
//! static mut CORE1_STACK: multicore::Stack<8192> = multicore::Stack::new();
//!
//! #[ch32_hal::entry]
//! fn main() -> ! {
//!     let p = ch32_hal::init(Default::default());
//!     let pf0 = p.PF0;
//!
//!     ch32_hal::multicore::spawn_core1(unsafe { &mut CORE1_STACK }, move || {
//!         let mut led = ch32_hal::gpio::Output::new(pf0, Default::default(), Default::default());
//!         loop { led.toggle(); }
//!     });
//!
//!     loop { /* hart 0 */ }
//! }
//! ```
//!
//! # What this does not give you
//!
//! - **Peripheral ownership is not checked.** There is one
//!   [`Peripherals`](crate::Peripherals) singleton per image, and hart 0 has
//!   already taken it. Move the peripherals the second hart needs into the
//!   closure (`let pf0 = p.PF0;`) or call `Peripherals::steal()` inside it and
//!   promise to touch disjoint hardware. Never let both harts own the same
//!   peripheral.
//! - **`critical-section` still masks only the calling hart's interrupts.**
//!   Per-hart state is fine; anything both harts touch (a mailbox, a logger,
//!   `embassy-sync`'s `CriticalSectionRawMutex`) needs a cross-core spinlock,
//!   either [`SpinLock`] below or a real `critical-section` implementation.
//! - **No time driver on hart 1.** Embassy's CH32H4 SysTick driver is hart 0
//!   only; hart 1's counter interrupts through `SysTick1` (vector 13), which
//!   `qingke-rt` does not expose yet. An `embassy_executor::Executor` runs on
//!   hart 1, but `embassy_time` must not be initialised there.
//! - **Interrupts are enabled with the boot core's setup.** Before the closure
//!   runs, hart 1 calls `qingke-rt`'s `_setup_interrupts`: `mtvec` points at
//!   the shared vector table, the hardware push/pop stack is on, `mstatus.MIE`
//!   is set, and `corecfgr`/`intsyscr`/`inestcr` get qingke-rt's *V3F* values.
//!   Those last writes are the open question: the vendor's V5F startup also
//!   sets V5F-only `corecfgr` bits (`NLP_EN`, `GHR_EN`, `LSU_DUAL`), which
//!   qingke-rt does not know about. It works on the bench, but a V5F-specific
//!   setup belongs in `qingke-rt` before this is called done.
//! - **IRQs above 31 are claimed by whoever enables them**, and PFIC enable
//!   bits are per-hart: hart 1 starts with no external interrupt enabled, so
//!   hart 0's interrupts stay on hart 0.
//!
//! # Mechanism
//!
//! `PFIC_WAKEIP[1]` only stores bits [31:10], so the wake entry must be 1 KiB
//! aligned. [`spawn_core1`] points it at a trampoline in
//! `.text.ch32_multicore_entry`, which the assembler aligns with
//! `.balign 1024`; the linker honours that alignment when it places the input
//! section, so no linker script change is required. The trampoline loads the
//! stack top from a static (the wake register carries no arguments), does a
//! `fence`, and calls back into Rust. Hart 0 waits for an ack before returning,
//! because the closure itself lives on hart 0's stack until hart 1 moves it
//! out, exactly like `embassy-rp`.

use core::mem::ManuallyDrop;
use core::ptr;
use core::sync::atomic::{fence, AtomicBool, AtomicPtr, AtomicUsize, Ordering};

/// Which hart is executing.
pub use qingke::pfic::HartId;

/// The hart that [`spawn_core1`] starts.
pub const SECOND_HART: HartId = HartId::C1;

/// A stack for hart 1, aligned for the RISC-V psABI.
///
/// Declare it as a `static` (it lands in `.bss`): the second hart executes in
/// the same address space, so its stack may live anywhere `.bss` does. Put it
/// in the zero-wait shared block or, if the board layout allows, in hart 1's
/// DTCM through a dedicated section.
#[repr(C, align(16))]
pub struct Stack<const N: usize> {
    mem: [u8; N],
}

impl<const N: usize> Stack<N> {
    /// A zeroed stack, for use in a `static`.
    pub const fn new() -> Self {
        Self { mem: [0; N] }
    }
}

impl<const N: usize> Default for Stack<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Hart id of the calling core.
#[inline]
pub fn hart_id() -> HartId {
    HartId::current()
}

unsafe extern "C" {
    fn _ch32_hart1_trampoline();
}

// Read by the trampoline before it touches the stack. `no_mangle` because the
// assembly loads it by symbol name.
#[used]
#[no_mangle]
static CH32_MULTICORE_STACK_TOP: AtomicUsize = AtomicUsize::new(0);

static ENTRY: AtomicPtr<()> = AtomicPtr::new(ptr::null_mut());
static ENTRY_CALL: AtomicUsize = AtomicUsize::new(0);
static ACK: AtomicBool = AtomicBool::new(false);
static SPAWNED: AtomicBool = AtomicBool::new(false);

/// Start `entry` on hart 1 with `stack`.
///
/// Returns once hart 1 has moved the closure out of hart 0's frame. The
/// closure owns those peripherals and statics it captures; `!Send` types
/// cannot be captured.
///
/// # Panics
///
/// Panics if called twice, or if hart 1 does not answer (it is not powered, or
/// the wake register was already claimed by another image).
pub fn spawn_core1<F, const N: usize>(stack: &'static mut Stack<N>, entry: F)
where
    F: FnOnce() + Send + 'static,
{
    assert!(
        !SPAWNED.swap(true, Ordering::AcqRel),
        "hart 1 has already been spawned"
    );

    let entry_addr = _ch32_hart1_trampoline as unsafe extern "C" fn() as usize as u32;
    assert_eq!(entry_addr & 0x3FF, 0, "hart 1 wake entry must be 1 KiB aligned");

    let mut entry = ManuallyDrop::new(entry);
    let stack_top = (stack.mem.as_mut_ptr() as usize + N) & !0xF;

    CH32_MULTICORE_STACK_TOP.store(stack_top, Ordering::Relaxed);
    ENTRY.store(&mut entry as *mut ManuallyDrop<F> as *mut (), Ordering::Relaxed);
    let call: extern "C" fn(*mut ()) -> ! = run_entry::<F>;
    ENTRY_CALL.store(call as usize, Ordering::Relaxed);

    // Publish the three words above before hart 1 is released. The wake
    // register itself is not an ordering barrier.
    fence(Ordering::Release);

    unsafe { qingke::pfic::wake_other_core(entry_addr) };

    // Hart 1 takes the closure out of this frame, so it must not return until
    // it has. Same handshake as embassy-rp's FIFO ack.
    let mut spins = 0u32;
    while !ACK.load(Ordering::Acquire) {
        spins += 1;
        if spins > 20_000_000 {
            panic!("hart 1 did not start");
        }
        core::hint::spin_loop();
    }

    // `entry` is now owned by hart 1; `ManuallyDrop` keeps this frame from
    // dropping it a second time.
}

/// Runs the type-erased closure and parks when it returns.
extern "C" fn run_entry<F: FnOnce()>(raw: *mut ()) -> ! {
    let entry = unsafe { ManuallyDrop::take(&mut *(raw as *mut ManuallyDrop<F>)) };
    entry();
    loop {
        core::hint::spin_loop();
    }
}

/// qingke-rt's startup only ran on hart 0; this is the per-hart part of it.
unsafe extern "C" {
    fn _setup_interrupts();
}

/// Called by the trampoline once `gp` and `sp` are set and the release fence
/// has run.
#[no_mangle]
extern "C" fn _ch32_hart1_start() -> ! {
    // Installs `mtvec` (the shared vector table), enables the hardware push/pop
    // stack and `mstatus.MIE`. Safe to call here: `sp` and `gp` are set and
    // nothing else on this hart has run.
    unsafe { _setup_interrupts() };

    let raw = ENTRY.swap(ptr::null_mut(), Ordering::Acquire);
    let call: extern "C" fn(*mut ()) -> ! =
        unsafe { core::mem::transmute(ENTRY_CALL.load(Ordering::Relaxed)) };

    // Hart 1 owns the closure now; hart 0 may return from `spawn_core1`.
    ACK.store(true, Ordering::Release);

    call(raw)
}

core::arch::global_asm!(
    r#"
    .section .text.ch32_multicore_entry, "ax", @progbits
    /* `PFIC_WAKEIP[1]` keeps only bits [31:10]: the entry must be 1 KiB
       aligned. Declaring the alignment here makes the linker align the input
       section wherever it places it, with no linker-script change. */
    .balign 1024
    .global _ch32_hart1_trampoline
_ch32_hart1_trampoline:
    /* Same `gp` as hart 0: one image, one set of globals. */
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    /* Pairs with the release fence in `spawn_core1` before SETEVENT. */
    fence rw, rw
    la t0, CH32_MULTICORE_STACK_TOP
    lw sp, 0(t0)
    andi sp, sp, -16

    /* FS = Initial, Dirty cleared: hart 1 never runs qingke-rt's startup. */
    li t0, 0x4000
    csrrc x0, mstatus, t0
    li t0, 0x2000
    csrrs x0, mstatus, t0

    call _ch32_hart1_start
1:
    j 1b
"#
);

/// A spinlock for state that both harts touch.
///
/// `critical_section::with` only masks the calling hart's interrupts on
/// QingKe, so it cannot protect memory shared between harts. This lock is the
/// smallest thing that can: an `amoswap` on a `u32`, plus a local interrupt
/// mask so an interrupt handler on the same hart cannot deadlock against its
/// own thread.
///
/// Hold it for short, non-blocking critical sections only.
pub struct SpinLock<T: ?Sized> {
    locked: AtomicBool,
    value: core::cell::UnsafeCell<T>,
}

unsafe impl<T: ?Sized + Send> Sync for SpinLock<T> {}
unsafe impl<T: ?Sized + Send> Send for SpinLock<T> {}

impl<T> SpinLock<T> {
    /// Create an unlocked lock.
    pub const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            value: core::cell::UnsafeCell::new(value),
        }
    }

    /// Run `f` with the lock held.
    pub fn with<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        // Local interrupts first: a handler that takes the same lock on this
        // hart would otherwise spin forever waiting for the thread it
        // interrupted.
        critical_section::with(|_| {
            while self
                .locked
                .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                core::hint::spin_loop();
            }
            let value = unsafe { &mut *self.value.get() };
            let result = f(value);
            self.locked.store(false, Ordering::Release);
            result
        })
    }
}

impl<T: Default> Default for SpinLock<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}
