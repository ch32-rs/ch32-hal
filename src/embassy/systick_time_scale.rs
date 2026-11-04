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
