//! One image, two Embassy executors — hart 0 and hart 1.
//!
//! `spawn_core1` starts hart 1 into the same image; hart 1 installs its own
//! `mtvec`/interrupt state (`multicore` calls qingke-rt's `_setup_interrupts`
//! before the closure) and then runs a second `embassy_executor::Executor`.
//!
//! Three tasks:
//!
//! - hart 0: `Timer::after(500 ms)`. The CH32H4 driver runs on hart 0's
//!   SysTick (counter 0), so this is the known-good time path.
//! - hart 1: a cooperative task that yields through a local `YieldOnce`
//!   future, exercising the executor's waker path on hart 1.
//! - hart 1: `Timer::after(500 ms)` through the **same** driver that hart 0
//!   initialised. The alarm interrupt runs on hart 0 and the waker wakes the
//!   hart-1 task; the spinning executor on hart 1 picks it up.
//!
//! Cross-core time works, with one measured caveat: in two runs where hart 1
//! was the only user of `embassy_time` (hart 0 just yielded), `h1_timer_ticks`
//! stayed 0 — the alarm never fired. With hart 0 also scheduling, both counters
//! advance in lockstep. So the driver's re-arm path needs investigating before
//! "hart 1 timers work" can be stated unconditionally; see the notes.
//!
//! It is still true that hart 1 must not *initialise* a driver: the CH32H4
//! driver owns counter 0 and its `SysTick` vector, and hart 1's `SysTick1`
//! (vector 13) is not exposed by `qingke-rt`.
//!
//! ```text
//! WLINK_PROBE=1a86:8010:D7708F065C7F cargo xtask flash --example multicore_embassy
//! sleep 3 && wlink dump 0x20178000 16 -o /var/tmp/obs.bin   # read once
//! ```
//!
//! Expected: `h1_hart = 1` and both counters advancing. The two rates are not
//! equal (different executors, different number of tasks); what matters is
//! that neither is stuck.

#![no_std]
#![no_main]

use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::{Context, Poll};

use ch32_hal as hal;
use embassy_executor::{Executor, Spawner};
use hal::multicore::{self, Stack};
use panic_halt as _;

/// Fixed at the start of `SRAM_SHARED` (`0x2017_8000`) by `ipc/shared.x`.
#[repr(C)]
struct Obs {
    h0_seq: AtomicU32,
    h1_seq: AtomicU32,
    h1_hart: AtomicU32,
    h1_timer_ticks: AtomicU32,
    h1_irqs: AtomicU32,
}

#[link_section = ".shared"]
static OBS: Obs = Obs {
    h0_seq: AtomicU32::new(0),
    h1_seq: AtomicU32::new(0),
    h1_hart: AtomicU32::new(0),
    h1_timer_ticks: AtomicU32::new(0),
    h1_irqs: AtomicU32::new(0),
};

static mut CORE1_STACK: Stack<8192> = Stack::new();
static mut EXEC0: MaybeUninit<Executor> = MaybeUninit::uninit();
static mut EXEC1: MaybeUninit<Executor> = MaybeUninit::uninit();

/// Yield once through the executor's waker, without `embassy-time`.
struct YieldOnce(bool);

impl Future for YieldOnce {
    type Output = ();

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
        if self.0 {
            Poll::Ready(())
        } else {
            self.0 = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

#[embassy_executor::task]
async fn hart0_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
        OBS.h0_seq.fetch_add(1, Ordering::Relaxed);
    }
}

/// Hart 1's own interrupt, pended by software. `IPC_CH0` is unused on this
/// board, so the only thing that can fire it is the `pend_interrupt` below.
///
/// The vector table is shared with hart 0; PFIC enable/pending/active state is
/// per-hart, so this lands on hart 1 only.
#[ch32_hal::interrupt]
fn IPC_CH0() {
    unsafe { qingke::pfic::unpend_interrupt(ch32_hal::pac::Interrupt::IPC_CH0 as u8) };
    OBS.h1_irqs.fetch_add(1, Ordering::Relaxed);
}

#[embassy_executor::task]
async fn hart1_task() {
    loop {
        OBS.h1_seq.fetch_add(1, Ordering::Relaxed);
        YieldOnce(false).await;
    }
}

/// Uses `embassy_time`, whose driver hart 0 initialised (`hal::init` with the
/// `embassy` feature): the SysTick alarm runs on hart 0, the waker wakes this
/// hart-1 task, and the spinning executor on hart 1 picks it up. This is the
/// first cross-core time test.
#[embassy_executor::task]
async fn hart1_timer_task() {
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(500)).await;
        OBS.h1_timer_ticks.fetch_add(1, Ordering::Relaxed);
    }
}

#[ch32_hal::entry]
fn main() -> ! {
    let _p = hal::init(hal::Config::default());

    for f in [
        &OBS.h0_seq,
        &OBS.h1_seq,
        &OBS.h1_hart,
        &OBS.h1_timer_ticks,
        &OBS.h1_irqs,
    ] {
        f.store(0, Ordering::Relaxed);
    }

    let stack = unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) };
    multicore::spawn_core1(stack, || {
        OBS.h1_hart
            .store(multicore::hart_id() as u32, Ordering::Relaxed);

        // Take one peripheral interrupt on this hart: enable it here (the
        // enable registers are per-hart), pend it by software, and let the
        // shared handler run. `enable_interrupt` writes the whole IENR register
        // — fine on hart 1, which has nothing else enabled, and exactly the
        // kind of shared-register hazard hart 0 must not run into.
        let irq = ch32_hal::pac::Interrupt::IPC_CH0 as u8;
        unsafe {
            qingke::pfic::set_priority(irq, 0);
            qingke::pfic::enable_interrupt(irq);
            qingke::pfic::pend_interrupt(irq);
        }

        // The executor lives in this image's `.bss` and is only ever touched
        // by hart 1, so a `static mut` is sound here (no `static_cell` needed).
        let executor = unsafe { (&mut *core::ptr::addr_of_mut!(EXEC1)).write(Executor::new()) };
        executor.run(|spawner: Spawner| {
            spawner.spawn(hart1_task().unwrap());
            spawner.spawn(hart1_timer_task().unwrap());
        });
    });

    // Hart 0's own executor. Same shape, no `#[embassy_executor::main]` because
    // `main` has already done the HAL bring-up and the spawn.
    let executor = unsafe { (&mut *core::ptr::addr_of_mut!(EXEC0)).write(Executor::new()) };
    executor.run(|spawner: Spawner| {
        spawner.spawn(hart0_task().unwrap());
    });
}
