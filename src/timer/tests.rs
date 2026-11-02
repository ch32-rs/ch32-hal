use super::period_registers;

#[test]
fn exact_periods_and_register_boundaries() {
    assert_eq!(period_registers(200_000_000, 1000, 16, false, false), (3, 49_999));
    assert_eq!(period_registers(65_536, 1, 16, false, false), (0, 65_535));
    assert_eq!(period_registers(65_536, 1, 16, false, true), (1, 32_767));
    assert_eq!(period_registers(200_000_000, 1, 32, false, false), (0, 199_999_999));
    assert_eq!(period_registers(u32::MAX, 1, 32, false, false), (0, u32::MAX - 1));
    assert_eq!(period_registers(200_000_000, 200_000_000, 16, false, true), (0, 0));
}

#[test]
fn center_aligned_periods_use_arr_without_the_edge_aligned_extra_tick() {
    assert_eq!(period_registers(200_000_000, 1000, 16, true, true), (1, 50_000));
    assert_eq!(period_registers(20_000_000, 1000, 16, true, true), (0, 10_000));
    assert_eq!(period_registers(131_070, 1, 16, true, true), (0, 65_535));
}

#[test]
fn pwm_duty_endpoints_fit_capture_compare_registers() {
    for bits in [16, 32] {
        let max_register = (1u64 << bits) - 1;
        for clock in [39_062, 20_000_000, 100_000_000, 200_000_000, u32::MAX] {
            for center in [false, true] {
                for frequency in [1, 50, 1000, 32768, clock / 2] {
                    if u64::from(frequency) * if center { 2 } else { 1 } > u64::from(clock) {
                        continue;
                    }
                    let ticks = u64::from(clock) / (u64::from(frequency) * if center { 2 } else { 1 });
                    if ticks > max_register * 65_536 {
                        continue;
                    }
                    let (prescaler, reload) = period_registers(clock, frequency, bits, center, true);
                    let duty = u64::from(reload) + u64::from(!center);
                    assert!(duty > 0 && duty <= max_register);
                    let cycles = duty * (u64::from(prescaler) + 1) * if center { 2 } else { 1 };
                    assert!(cycles <= u64::from(clock) / u64::from(frequency));
                    let error = u64::from(clock) / u64::from(frequency) - cycles;
                    assert!(error < (u64::from(prescaler) + 1) * if center { 2 } else { 1 });
                }
            }
        }
    }
}

#[test]
#[should_panic]
fn zero_frequency_is_rejected() {
    period_registers(200_000_000, 0, 16, false, false);
}

#[test]
#[should_panic]
fn frequency_above_the_clock_is_rejected() {
    period_registers(20_000_000, 20_000_001, 16, false, false);
}

#[test]
#[should_panic]
fn center_aligned_frequency_above_half_the_clock_is_rejected() {
    period_registers(20_000_000, 10_000_001, 16, true, true);
}

#[test]
#[should_panic]
fn unrepresentable_period_is_rejected() {
    period_registers(u32::MAX, 1, 16, false, true);
}
