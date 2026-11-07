//! V5F CPU-ID report via its own non-blocking RTT buffer and shared mailbox.
//!
//! The boot core configures shared clocks and wakes this core.

#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use ch32h417_ipc::protocols::{SdiCpuid, CPUID_CSR_NAMES, CPUID_MISA_INDEX};
use ch32h417_v5f::{cache, cpuid, mailbox};
use panic_halt as _;
use qingke::pfic::HartId;

/// Strong override of qingke-rt's weak `ExceptionHandler`: record the fault and
/// resume past the offending instruction, so probing a CSR the core does not
/// implement is recoverable (see `ch32h417_v5f::cpuid`).
#[no_mangle]
pub extern "C" fn ExceptionHandler() {
    let epc: u32;
    unsafe { core::arch::asm!("csrr {}, mepc", out(reg) epc) };
    let halfword = unsafe { core::ptr::read_volatile(epc as *const u16) } as u32;
    let len = if halfword & 0b11 == 0b11 { 4 } else { 2 };
    cpuid::note_trap();
    unsafe { core::arch::asm!("csrw mepc, {}", in(reg) epc + len) };
}

#[qingke_rt::entry]
fn main() -> ! {
    use core::sync::atomic::Ordering;

    // Progress marker: `report_self` below resets it to 0 and counts up as it
    // probes, so a *report* that shows 0x101 means hart 1 entered main and then
    // died before probing — the usual way "hart 1 never ran" is actually
    // "hart 1 ran and hung", which this distinguishes.
    let mailbox = mailbox::<SdiCpuid>();
    mailbox.payload.cpuid.progress.store(0x101, Ordering::Relaxed);
    // Hart 1 must enable its own instruction cache: it resets to disabled.
    cache::enable_icache();

    let me = HartId::current();
    log!("[V5F] hart={:?} (mhartid 1) is executing this code", me);

    // Probe our own CSRs and print the table from this core over RTT.
    // `cpuid` publishes the same block for `xtask report`: this is the variant to
    // use when you want to watch hart 1 rather than read shared memory later.
    //
    // Non-blocking RTT so a missing console cannot wedge this
    // core: with the console open every line goes out, without it the lines are
    // dropped and the loop below keeps ticking.
    cpuid::report_self(&mailbox.payload.cpuid);
    let values = mailbox.payload.cpuid.snapshot();
    let present = mailbox.payload.cpuid.present.load(Ordering::Relaxed);
    log!(
        "[V5F] image build features = {:#04x}",
        mailbox.payload.cpuid.buildcfg.load(Ordering::Relaxed)
    );
    for (i, name) in CPUID_CSR_NAMES.iter().enumerate() {
        if present & (1 << i) != 0 {
            log!("  {:<18} = {:#010x}", name, values[i]);
        } else {
            log!("  {:<18}   absent (read faulted)", name);
        }
    }
    if present & (1 << CPUID_MISA_INDEX) != 0 {
        let misa = values[CPUID_MISA_INDEX];
        let mut letters = [0u8; 26];
        let mut len = 0;
        for (bit, letter) in (b'A'..=b'Z').enumerate() {
            if misa & (1 << bit) != 0 {
                letters[len] = letter;
                len += 1;
            }
        }
        log!(
            "  misa decode        MXL={} extensions={}",
            (misa >> 30) & 3,
            core::str::from_utf8(&letters[..len]).unwrap_or("")
        );
    }

    // Report liveness through shared SRAM. A single 32-bit aligned writer and
    // a single reader need no locking.
    loop {
        let ticks = mailbox.payload.ticks.load(Ordering::Relaxed);
        mailbox.payload.ticks.store(ticks.wrapping_add(1), Ordering::Relaxed);

        // The HAL's delay is not usable here (its calibration is filled in by
        // the boot core's `hal::init()` and it drives hart 0's systick), so a
        // plain nop loop is the heartbeat.
        for _ in 0..200_000 {
            unsafe { core::arch::asm!("nop") };
        }
    }
}
