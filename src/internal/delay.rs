//! Peripheral settling delays without borrowing SysTick or another hardware timer.

/// Busy-wait for peripheral initialization, not for precise signal timing.
///
/// The clock cache must match the running core and the clock must not increase
/// during the wait. Without Embassy, count one dependent decrement per requested
/// core cycle: unlike a calibrated loop, this does not assume a fixed branch cost.
/// Flash wait states, fetch stalls, RAM execution and interrupts only change the
/// upper bound: each decrement depends on the previous one, so it cannot execute
/// more than once per core cycle.
#[allow(dead_code)]
pub(crate) fn block_for_us(micros: u32) {
    if micros == 0 {
        return;
    }

    #[cfg(feature = "embassy")]
    embassy_time::block_for(embassy_time::Duration::from_micros(micros as u64));

    #[cfg(not(feature = "embassy"))]
    {
        let clocks = crate::rcc::clocks();
        #[cfg(rcc_h4)]
        let hz = match qingke::pfic::HartId::current() {
            qingke::pfic::HartId::C0 => clocks.v3f.0,
            qingke::pfic::HartId::C1 => clocks.v5f.0,
        };
        #[cfg(not(rcc_h4))]
        let hz = clocks.hclk.0;
        assert!(hz != 0);

        let mut cycles = cycles_for_us(hz, micros);
        while cycles != 0 {
            let chunk = cycles.min(u32::MAX as u64) as u32;
            unsafe {
                core::arch::asm!(
                    "2:",
                    "addi {counter}, {counter}, -1",
                    "bnez {counter}, 2b",
                    counter = inout(reg) chunk => _,
                    // Keep peripheral accesses ordered around the settling delay.
                    options(nostack),
                );
            }
            cycles -= chunk as u64;
        }
    }
}

#[cfg(not(feature = "embassy"))]
fn cycles_for_us(hz: u32, micros: u32) -> u64 {
    // Avoid a 64-bit division when the core runs at a whole number of MHz.
    let mut cycles = micros as u64 * (hz / 1_000_000) as u64;
    let rem = hz % 1_000_000;
    if rem != 0 {
        cycles += (micros as u64 * rem as u64).div_ceil(1_000_000);
    }
    cycles
}
