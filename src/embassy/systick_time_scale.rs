pub(super) struct TimeScale {
    counts: u64,
    ticks: u64,
}

impl TimeScale {
    pub(super) const fn new(clock_hz: u32, counter_divisor: u32) -> Self {
        assert!(clock_hz != 0 && counter_divisor != 0);
        let tick_units = embassy_time_driver::TICK_HZ * counter_divisor as u64;
        let mut numerator = clock_hz as u64;
        let mut denominator = tick_units;
        while denominator != 0 {
            let remainder = numerator % denominator;
            numerator = denominator;
            denominator = remainder;
        }
        Self {
            counts: clock_hz as u64 / numerator,
            ticks: tick_units / numerator,
        }
    }

    pub(super) fn ticks_from_counter(&self, count: u64) -> u64 {
        multiply_ratio(count, self.ticks, self.counts, false)
    }

    pub(super) fn counter_from_ticks(&self, ticks: u64) -> u64 {
        if ticks == u64::MAX {
            return u64::MAX;
        }
        multiply_ratio(ticks, self.counts, self.ticks, true)
    }
}

fn multiply_ratio(value: u64, multiplier: u64, divisor: u64, round_up: bool) -> u64 {
    let quotient = value / divisor;
    let remainder = value % divisor;
    let fraction = if let Some(product) = remainder.checked_mul(multiplier) {
        if round_up {
            product.div_ceil(divisor)
        } else {
            product / divisor
        }
    } else {
        let product = u128::from(remainder) * u128::from(multiplier);
        if round_up {
            product.div_ceil(u128::from(divisor)) as u64
        } else {
            (product / u128::from(divisor)) as u64
        }
    };
    quotient.saturating_mul(multiplier).saturating_add(fraction)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rational_frequency_and_rounding_match_wide_reference() {
        for clock_hz in [5_000_000, 20_000_000, 25_000_000, 32_000_000, 333_333_333, 500_000_000] {
            for ahb_divisor in [1, 2, 4, 8, 16, 64, 128, 256, 512] {
                let divisor = ahb_divisor * 8;
                let scale = TimeScale::new(clock_hz, divisor);
                let tick_units = u128::from(embassy_time_driver::TICK_HZ) * u128::from(divisor);
                let mut random = 0x1234_5678_abcd_ffffu64;
                for _sample in 0..2_000 {
                    random = random.wrapping_mul(6364136223846793005).wrapping_add(1);
                    let ticks =
                        (u128::from(random) * tick_units / u128::from(clock_hz)).min(u128::from(u64::MAX)) as u64;
                    let count = (u128::from(random) * u128::from(clock_hz))
                        .div_ceil(tick_units)
                        .min(u128::from(u64::MAX)) as u64;
                    assert_eq!(scale.ticks_from_counter(random), ticks);
                    assert_eq!(scale.counter_from_ticks(random), count);
                }
                assert_eq!(scale.counter_from_ticks(u64::MAX), u64::MAX);
                for ticks in 1..128 {
                    let count = scale.counter_from_ticks(ticks);
                    assert!(scale.ticks_from_counter(count) >= ticks);
                    assert!(scale.ticks_from_counter(count - 1) < ticks);
                }
            }
        }
    }

    #[test]
    fn fractional_hsi_dividers_do_not_accumulate_truncation_error() {
        let scale = TimeScale::new(20_000_000, 256 * 8);
        assert_eq!(
            scale.ticks_from_counter(10_000),
            embassy_time_driver::TICK_HZ * 1024 / 1000
        );
        let scale = TimeScale::new(20_000_000, 512 * 8);
        assert_eq!(scale.ticks_from_counter(156_250), embassy_time_driver::TICK_HZ * 32);
    }
}
