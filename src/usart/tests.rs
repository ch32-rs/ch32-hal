use super::*;

#[test]
fn baudrate_dividers_match_wide_rounding_at_all_clock_rates() {
    for clock in [
        8_000_000u32,
        20_000_000,
        25_000_000,
        100_000_000,
        166_666_666,
        200_000_000,
    ] {
        for baudrate in [
            300u32, 1200, 9600, 115200, 230400, 1_000_000, 3_125_000, 6_250_000, 12_500_000,
        ] {
            let expected = (u64::from(clock) + u64::from(baudrate) / 2) / u64::from(baudrate);
            let actual = baudrate_divisor(clock, baudrate);
            if expected < 0x10 {
                assert_eq!(actual, Err(ConfigError::BaudrateTooHigh));
            } else if expected > 0xffff {
                assert_eq!(actual, Err(ConfigError::BaudrateTooLow));
            } else {
                assert_eq!(actual, Ok(expected as u32));
            }
        }
    }
    assert_eq!(baudrate_divisor(200_000_000, 115200), Ok(1736));
    assert_eq!(baudrate_divisor(200_000_000, 12_500_000), Ok(16));
    assert_eq!(baudrate_divisor(20_000_000, 20_000_000 / 65535 + 1), Ok(65359));
    assert_eq!(baudrate_divisor(16_000_000, 1_000_001), Ok(16));
    assert_eq!(
        baudrate_divisor(16_000_000, 16_000_001),
        Err(ConfigError::BaudrateTooHigh)
    );
    assert_eq!(baudrate_divisor(20_000_000, 0), Err(ConfigError::BaudrateTooLow));
}

#[test]
fn invalid_baudrates_leave_registers_unchanged() {
    for baudrate in [0, 1, 305, 13_000_000, u32::MAX] {
        let mut words = [0x5a5a_5a5au32; 8];
        let registers = unsafe { pac::usart::Usart::from_ptr(words.as_mut_ptr().cast()) };
        let mut config = Config::default();
        config.baudrate = baudrate;
        assert!(configure(&registers, &config, Hertz(200_000_000), true, true).is_err());
        assert_eq!(words, [0x5a5a_5a5a; 8]);
    }
}

#[test]
fn framing_and_half_duplex_can_be_reconfigured_without_sticky_bits() {
    let mut words = [0u32; 8];
    let registers = unsafe { pac::usart::Usart::from_ptr(words.as_mut_ptr().cast()) };
    registers.ctlr3().modify(|register| register.set_ctse(true));
    for data_bits in [DataBits::DataBits9, DataBits::DataBits8] {
        for parity in [Parity::ParityOdd, Parity::ParityEven, Parity::ParityNone] {
            for stop_bits in [StopBits::STOP2, StopBits::STOP1P5, StopBits::STOP0P5, StopBits::STOP1] {
                for half_duplex in [true, false] {
                    let mut config = Config::default();
                    config.data_bits = data_bits;
                    config.parity = parity;
                    config.stop_bits = stop_bits;
                    config.half_duplex = half_duplex;
                    configure(&registers, &config, Hertz(200_000_000), true, true).unwrap();
                    let control = registers.ctlr1().read();
                    assert!(control.ue() && control.te() && control.re());
                    assert_eq!(control.m(), data_bits == DataBits::DataBits9);
                    assert_eq!(control.pce(), parity != Parity::ParityNone);
                    assert_eq!(control.ps(), parity == Parity::ParityOdd);
                    assert_eq!(registers.ctlr2().read().stop(), stop_bits as u8);
                    assert_eq!(registers.ctlr3().read().hdsel(), half_duplex);
                    assert!(registers.ctlr3().read().ctse());
                }
            }
        }
    }
}

#[test]
fn receive_errors_have_the_same_priority_as_the_interrupt_handler() {
    for flags in 0..16u32 {
        let expected = if flags & 1 != 0 {
            Some(Error::Parity)
        } else if flags & 2 != 0 {
            Some(Error::Framing)
        } else if flags & 4 != 0 {
            Some(Error::Noise)
        } else if flags & 8 != 0 {
            Some(Error::Overrun)
        } else {
            None
        };
        assert_eq!(receive_error(pac::usart::regs::Statr(flags | 0x3f0)), expected);
    }
}
