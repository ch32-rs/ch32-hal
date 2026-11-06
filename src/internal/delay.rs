//! Peripheral settling delays without borrowing SysTick or another hardware timer.

/// Busy-wait for peripheral initialization, not for precise signal timing.
///
/// The clock cache must match the running core and the clock must not increase
/// during the wait. Without Embassy, count one dependent decrement per requested
/// core cycle: unlike a calibrated loop, this does not assume a fixed branch cost.
/// Flash stalls and interrupts can extend the wait. This assumes a dependent
/// integer decrement cannot execute more than once per core cycle.
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

/// Yield through the time driver when available; otherwise busy-wait briefly.
#[allow(dead_code)]
pub(crate) async fn wait_for_us(micros: u32) {
    #[cfg(feature = "embassy")]
    embassy_time::Timer::after_micros(micros as u64).await;
    #[cfg(not(feature = "embassy"))]
    block_for_us(micros);
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

#[cfg(all(test, not(feature = "embassy")))]
mod tests {
    use super::cycles_for_us;

    #[test]
    fn rounds_up() {
        assert_eq!(cycles_for_us(8_000_000, 10), 80);
        assert_eq!(cycles_for_us(144_000_000, 100_000), 14_400_000);
        assert_eq!(cycles_for_us(32_768, 1), 1);
        assert_eq!(cycles_for_us(1_500_000, 1), 2);
        assert_eq!(cycles_for_us(144_000_000, 0), 0);
    }

    #[test]
    fn long_waits_do_not_overflow() {
        assert_eq!(cycles_for_us(480_000_000, 10_000_000), 4_800_000_000);
        assert_eq!(cycles_for_us(u32::MAX, u32::MAX), 18_446_744_065_120);
    }
}
