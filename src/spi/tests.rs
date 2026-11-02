use super::*;

#[test]
fn baud_rate_uses_fastest_divisor_without_overspeed() {
    for clock in [1, 20_000_000, 100_000_000, 200_000_000, u32::MAX] {
        for frequency in [0, 1, 78_125, 390_625, 1_000_000, 33_333_333, 100_000_000, u32::MAX] {
            let actual = calculate_baud_rate(clock, frequency);
            let expected = (0..8).find(|setting| u64::from(clock) <= u64::from(frequency) * (2u64 << setting));
            assert_eq!(actual.map(|setting| setting as u8), expected);
        }
    }
    assert!(calculate_baud_rate(0, 1).is_none());
}

#[test]
fn baud_rate_rounds_at_every_divider_boundary() {
    for setting in 0..8 {
        let frequency = 200_000_000 / (2 << setting);
        assert_eq!(calculate_baud_rate(200_000_000, frequency).unwrap() as u8, setting);
        let below = calculate_baud_rate(200_000_000, frequency - 1);
        assert_eq!(below.map(|value| value as u8), (setting < 7).then_some(setting + 1));
    }
    assert!(calculate_baud_rate(100_000_000, 33_333_333) == Some(BaudRate::DIV_4));
}

#[cfg(spi_v4x7)]
#[test]
fn high_speed_receive_follows_clock_divider_and_direction() {
    let mut storage = [0u32; 10];
    let regs = unsafe { Regs::from_ptr(storage.as_mut_ptr().cast()) };
    for (clock, divisor, receive, first, second) in [
        (20_000_000, BaudRate::DIV_2, true, true, false),
        (119_999_999, BaudRate::DIV_2, true, true, false),
        (120_000_000, BaudRate::DIV_2, true, true, true),
        (200_000_000, BaudRate::DIV_4, true, false, false),
        (200_000_000, BaudRate::DIV_2, false, false, false),
    ] {
        configure_high_speed(regs, clock, divisor, receive);
        assert_eq!(regs.hscr().read().hsrxen(), first);
        assert_eq!(regs.hscr().read().hsrxen2(), second);
    }
}

#[test]
fn configuration_round_trips_modes_and_orders() {
    for mode in [
        embedded_hal::spi::MODE_0,
        embedded_hal::spi::MODE_1,
        embedded_hal::spi::MODE_2,
        embedded_hal::spi::MODE_3,
    ] {
        let config = Config {
            mode,
            ..Config::default()
        };
        let mut register = pac::spi::regs::Ctlr1(0);
        register.set_cpol(config.raw_polarity());
        register.set_cpha(config.raw_phase());
        register.set_lsbfirst(config.lsb_first());
        register.set_br(BaudRate::DIV_8);
        let recovered = Config::from_cfgr(&register, Hertz(200_000_000));
        assert_eq!(recovered.mode, mode);
        assert_eq!(recovered.frequency.0, 25_000_000);
        assert!(!recovered.lsb_first());
        #[cfg(not(spi_v0))]
        {
            register.set_lsbfirst(true);
            assert!(Config::from_cfgr(&register, Hertz(200_000_000)).lsb_first());
        }
    }
}

#[test]
fn error_flags_report_receive_and_mode_faults() {
    for flags in 0..8 {
        let mut status = pac::spi::regs::Statr(0);
        status.set_ovr(flags & 1 != 0);
        status.set_modf(flags & 2 != 0);
        status.set_crcerr(flags & 4 != 0);
        let expected = if flags & 1 != 0 {
            Err(Error::Overrun)
        } else if flags & 2 != 0 {
            Err(Error::ModeFault)
        } else if flags & 4 != 0 {
            Err(Error::Crc)
        } else {
            Ok(())
        };
        assert_eq!(check_error_flags(&status), expected);
    }
}
