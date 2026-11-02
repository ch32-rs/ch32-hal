use super::*;

#[test]
fn all_sample_encodings_and_cycle_counts() {
    for (sample, encoded, cycles) in [
        (SampleTime::S1_5, 0, 3),
        (SampleTime::S7_5, 1, 15),
        (SampleTime::S13_5, 2, 27),
        (SampleTime::S28_5, 3, 57),
        (SampleTime::S41_5, 4, 83),
        (SampleTime::S55_5, 5, 111),
        (SampleTime::S71_5, 6, 143),
        (SampleTime::S239_5, 7, 479),
        (SampleTime::S2_5, 12, 5),
        (SampleTime::S3_5, 13, 7),
        (SampleTime::S4_5, 14, 9),
        (SampleTime::S5_5, 15, 11),
    ] {
        assert_eq!(sample as u8, encoded);
        assert_eq!(sample.cycles_x2(), cycles);
        for channel in 0..18 {
            let (shift, bits, extended) = sample_bits(channel, sample);
            assert_eq!(shift, u32::from(if channel < 10 { channel } else { channel - 10 }) * 3);
            assert_eq!(bits >> shift, u32::from(encoded & 7));
            assert_eq!(extended, encoded & 8 != 0);
        }
    }
}

#[test]
fn register_offsets_and_corrected_data_fields() {
    let regs = unsafe { pac::adc::Adc::from_ptr(0x4001_2400 as *mut ()) };
    assert_eq!(regs.samptr2().as_ptr() as usize, 0x4001_2410);
    assert_eq!(regs.rdatar().as_ptr() as usize, 0x4001_244c);
    let mut regular = pac::adc::regs::Rdatar(0);
    regular.set_data(0x1234);
    regular.set_adc2data(0xabcd);
    assert_eq!(regular.0, 0xabcd_1234);
    let mut injected = pac::adc::regs::Idatar1(0);
    injected.set_jdata(0xabcd);
    assert_eq!(injected.0, 0xabcd);
}

fn channel<T: Instance>(number: u8, sample_time: SampleTime) -> Channel<'static, T> {
    Channel {
        number,
        sample_time,
        _borrow: PhantomData,
    }
}

#[test]
fn invalid_and_conflicting_sequences_are_rejected() {
    type T = peripherals::ADC1;
    assert_eq!(validate_sequence::<T>(&[], 20_000_000, 8), Err(Error::InvalidSequence));
    let ranks: [_; 17] = core::array::from_fn(|_| channel::<T>(0, SampleTime::S1_5));
    assert_eq!(validate_sequence(&ranks, 20_000_000, 8), Err(Error::InvalidSequence));
    let ranks = [channel::<T>(1, SampleTime::S1_5), channel::<T>(1, SampleTime::S7_5)];
    assert_eq!(validate_sequence(&ranks, 20_000_000, 8), Err(Error::InvalidSequence));
    assert_eq!(
        validate_sequence(&[channel::<T>(18, SampleTime::S1_5)], 20_000_000, 8),
        Err(Error::InvalidSequence)
    );
    assert_eq!(
        validate_sequence(&[channel::<T>(15, SampleTime::S2_5)], 20_000_000, 8),
        Ok(())
    );
}

#[test]
fn temperature_acquisition_respects_real_clock_ratio() {
    let ranks = [channel::<peripherals::ADC1>(16, SampleTime::S239_5)];
    assert_eq!(validate_sequence(&ranks, 20_000_000, 8), Ok(()));
    assert_eq!(validate_sequence(&ranks, 112_000_000, 8), Ok(()));
    assert_eq!(
        validate_sequence(&ranks, 200_000_000, 8),
        Err(Error::SampleTimeTooShort)
    );
    assert_eq!(
        validate_sequence(&[channel::<peripherals::ADC1>(16, SampleTime::S1_5)], 20_000_000, 8),
        Err(Error::SampleTimeTooShort)
    );
    for number in [16, 17] {
        assert_eq!(
            validate_sequence(
                &[channel::<peripherals::ADC2>(number, SampleTime::S239_5)],
                20_000_000,
                8
            ),
            Err(Error::InvalidSequence)
        );
    }
}

#[test]
fn shortest_acquisition_respects_two_megasamples_per_second() {
    let shortest = [channel::<peripherals::ADC1>(0, SampleTime::S1_5)];
    assert_eq!(validate_sequence(&shortest, 28_000_000, 1), Ok(()));
    assert_eq!(
        validate_sequence(&shortest, 28_000_001, 1),
        Err(Error::SampleTimeTooShort)
    );
    assert_eq!(
        validate_sequence(&shortest, 30_000_000, 1),
        Err(Error::SampleTimeTooShort)
    );
    let extended = [channel::<peripherals::ADC1>(0, SampleTime::S2_5)];
    assert_eq!(validate_sequence(&extended, 30_000_000, 1), Ok(()));
}
