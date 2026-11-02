use super::*;

#[test]
#[cfg(ch32v4)]
fn v4_clock_fields_follow_evt_limits_and_dividers_never_overspeed() {
    for clock in [4_000_000, 20_000_000, 60_000_000, 100_000_000, 200_000_000] {
        for frequency in [25_000, 100_000, 100_001, 400_000] {
            for duty in [Duty::Duty2_1, Duty::Duty16_9] {
                let actual = timing(Hertz(clock), Hertz(frequency), duty);
                let range = (clock / 1_000_000).min(60);
                assert_eq!(u32::from(actual.input_mhz), range);
                assert_eq!(
                    u32::from(actual.rise),
                    if frequency > 100_000 {
                        range * 3 / 10 + 1
                    } else {
                        range + 1
                    }
                );
                let factor = if !actual.fast {
                    2
                } else if actual.duty {
                    25
                } else {
                    3
                };
                let cycles = u64::from(actual.divider) * factor;
                assert!(u64::from(clock) <= cycles * u64::from(frequency));
                assert!(actual.divider >= if actual.fast { 1 } else { 4 });
            }
        }
    }
    assert_eq!(timing(Hertz(200_000_000), Hertz(400_000), Duty::Duty2_1).divider, 167);
    assert_eq!(timing(Hertz(200_000_000), Hertz(100_000), Duty::Duty2_1).divider, 1000);
}

#[test]
#[should_panic]
fn zero_frequency_is_rejected_before_register_access() {
    timing(Hertz(20_000_000), Hertz(0), Duty::Duty2_1);
}

#[test]
#[should_panic]
fn unrepresentable_clock_divider_is_rejected() {
    timing(Hertz(20_000_000), Hertz(1), Duty::Duty2_1);
}

#[test]
#[should_panic]
fn unsupported_bus_speed_is_rejected() {
    timing(Hertz(20_000_000), Hertz(400_001), Duty::Duty2_1);
}

#[test]
fn operations_are_validated_before_a_transaction_starts() {
    let mut empty = [];
    let mut read = [0; 2];
    assert_eq!(
        validate_operations(&[Operation::Write(&[]), Operation::Read(&mut empty)], true),
        Err(Error::Overrun)
    );
    assert_eq!(
        validate_operations(&[Operation::Write(&[]), Operation::Read(&mut read)], true),
        Ok(())
    );
    #[cfg(ch32v4)]
    assert_eq!(
        I2c::<crate::peripherals::I2C, Blocking>::validate_address(0x80),
        Err(Error::InvalidAddress)
    );
}
