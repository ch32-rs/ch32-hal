//! Dual-core CPU-ID report via separate non-blocking RTT buffers.
//!
//! The V3F also reports the V5F results from the shared mailbox.

#![no_std]
#![no_main]

use defmt_rtt as _;
macro_rules! log {
    () => { defmt::info!("") };
    ($($args:tt)*) => { defmt::info!("{}", defmt::Display2Format(&format_args!($($args)*))) };
}

use ch32h417_ipc as ipc;
use ch32h417_ipc::protocols::{SdiCpuid, CPUID_CSR_NAMES, CPUID_DONE, CPUID_MISA_INDEX};
use qingke::pfic::{self, HartId};

ipc::mailbox!(SdiCpuid);
use ch32_hal as hal;
use panic_halt as _;

/// Must match `v5f/memory.x` FLASH ORIGIN (1KB-aligned).
const V5F_ENTRY: u32 = 0x0001_0000;

#[ch32_hal::entry]
fn main() -> ! {
    // Only the boot core runs the full bring-up: RCC, AFIO/GPIO and EXTI are
    // shared between both harts (see the V5F demo for why hart 1 must not
    // repeat this).
    let _p = hal::init(hal::Config::default());

    // Clear the whole mailbox (shared RAM survives a reset), so a report always
    // describes this run — including hart 1's progress markers.
    mailbox().clear();

    let me = HartId::current();
    // Bounded prints: this example must also work with no console attached
    // (`flash` + `report`), where a blocking print would strand the handover.
    log!("[V3F] hart={:?} (mhartid 0) is executing this code", me);
    log!("[V3F] waking the second core at {:#010x}", V5F_ENTRY);

    // Each image owns a separate RTT buffer, so no console handoff is needed.
    unsafe { pfic::wake_other_core(V5F_ENTRY) };

    let mut n = 0u32;
    loop {
        // Re-issue the wake until hart 1 reports in: its startup races with the
        // debug link attaching, and re-waking is idempotent.
        let v5f_ticks = mailbox().payload.ticks.load(core::sync::atomic::Ordering::Relaxed);
        if v5f_ticks == 0 {
            unsafe { pfic::wake_other_core(V5F_ENTRY) };
        }
        // Also publish hart 1's results through the V3F RTT buffer.
        let cpuid = &mailbox().payload.cpuid;
        if cpuid.done.load(core::sync::atomic::Ordering::Relaxed) == CPUID_DONE && n % 5 == 0 {
            let values = cpuid.snapshot();
            let present = cpuid.present.load(core::sync::atomic::Ordering::Relaxed);
            log!("=== hart 1, published through shared memory ===");
            log!(
                "  image build features = {:#04x}",
                cpuid.buildcfg.load(core::sync::atomic::Ordering::Relaxed)
            );
            for (i, name) in CPUID_CSR_NAMES.iter().enumerate() {
                if present & (1 << i) != 0 {
                    log!("  {:<18} = {:#010x}", name, values[i]);
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
        }
        log!("[V3F] tick {} | [V5F] ticks {}", n, v5f_ticks);
        n = n.wrapping_add(1);

        hal::delay::Delay.delay_ms(1000);
    }
}
