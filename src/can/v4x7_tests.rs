use super::*;

#[test]
fn standard_extended_and_remote_frames_round_trip() {
    for id in [
        Id::Standard(StandardId::new(0).unwrap()),
        Id::Standard(StandardId::new(0x7ff).unwrap()),
        Id::Extended(ExtendedId::new(0x1234_567).unwrap()),
        Id::Extended(ExtendedId::new(0x1fff_ffff).unwrap()),
    ] {
        for length in 0..=8 {
            let data = [0, 1, 0x80, 0xff, 4, 5, 6, 7];
            let frame = CanFrame::new(id, &data[..length]).unwrap();
            let decoded = CanFrame::decode(frame.identifier(), length as u8, 0xff80_0100, 0x0706_0504);
            assert_eq!(decoded.id(), id);
            assert_eq!(decoded.data(), frame.data());
            assert_eq!(decoded, frame);
            assert!(!decoded.is_remote_frame());
            let remote = CanFrame::new_remote(id, length).unwrap();
            let decoded = CanFrame::decode(remote.identifier(), length as u8, 0, 0);
            assert_eq!(decoded, remote);
            assert!(embedded_can::Frame::is_remote_frame(&decoded));
            assert!(decoded.data().is_empty());
            assert_eq!(decoded.dlc(), length);
        }
    }
}

#[test]
fn frames_reject_oversized_payloads_and_normalize_classical_dlc() {
    let id = StandardId::new(42).unwrap();
    assert!(CanFrame::new(id, &[0; 9]).is_none());
    assert!(CanFrame::new_remote(id, 9).is_none());
    for dlc in 8..=15 {
        let frame = CanFrame::decode(u32::from(id.as_raw()) << 21, dlc, 0, 0);
        assert_eq!(frame.dlc(), 8);
        assert_eq!(frame.data().len(), 8);
    }
}

#[test]
fn identifier_positions_match_mailboxes_and_filters() {
    let standard = CanFrame::new(StandardId::new(0x123).unwrap(), &[]).unwrap();
    assert_eq!(standard.identifier(), 0x2460_0000);
    let extended = CanFrame::new_remote(ExtendedId::new(0x1234_567).unwrap(), 8).unwrap();
    assert_eq!(extended.identifier(), (0x1234_567 << 3) | 6);
}

#[test]
fn timing_search_never_accepts_a_rounded_bitrate() {
    for clock in [0, 39_062, 20_000_000, 25_000_000, 50_000_000, 100_000_000, 200_000_000] {
        for bitrate in [
            0,
            999,
            1_000,
            10_000,
            83_333,
            125_000,
            250_000,
            500_000,
            1_000_000,
            1_000_001,
            u32::MAX,
        ] {
            let expected = (1_000..=1_000_000).contains(&bitrate)
                && (1..=16u64).any(|seg1| {
                    (1..=8u64).any(|seg2| {
                        let divisor = u64::from(bitrate) * (1 + seg1 + seg2);
                        divisor != 0
                            && u64::from(clock) % divisor == 0
                            && (1..=1024).contains(&(u64::from(clock) / divisor))
                    })
                });
            let timing = BitTiming::from_bitrate(clock, bitrate);
            assert_eq!(timing.is_some(), expected, "clock={clock}, bitrate={bitrate}");
            if let Some(timing) = timing {
                assert_eq!(
                    u64::from(clock),
                    u64::from(bitrate)
                        * u64::from(timing.prescaler)
                        * (1 + u64::from(timing.seg1) + u64::from(timing.seg2))
                );
                assert!(timing.register().is_some());
            }
        }
    }
}

#[test]
fn timing_prescaler_uses_all_ten_bits_without_wrapping() {
    let timing = BitTiming {
        prescaler: 1024,
        seg1: 16,
        seg2: 8,
        sync_jump_width: 4,
    };
    assert_eq!(timing.register(), Some(1023 | (15 << 16) | (7 << 20) | (3 << 24)));
    assert_eq!(BitTiming { prescaler: 0, ..timing }.register(), None);
    assert_eq!(
        BitTiming {
            prescaler: 1025,
            ..timing
        }
        .register(),
        None
    );
    assert_eq!(BitTiming { seg1: 17, ..timing }.register(), None);
    assert_eq!(BitTiming { seg2: 9, ..timing }.register(), None);
    assert_eq!(BitTiming { seg2: 2, ..timing }.register(), None);
    assert_eq!(
        BitTiming {
            sync_jump_width: 5,
            ..timing
        }
        .register(),
        None
    );
}

#[test]
fn exact_filters_match_identifier_format_and_optional_remote_bit() {
    for id in [
        Id::Standard(StandardId::new(0x123).unwrap()),
        Id::Extended(ExtendedId::new(0x1234_567).unwrap()),
    ] {
        for remote in [None, Some(false), Some(true)] {
            let filter = Filter::exact(13, CanFifo::Fifo1, id, remote);
            for received_remote in [false, true] {
                let received = types::encode_id(id) | ((received_remote as u32) << 1);
                let matches = (received & filter.registers[1]) == (filter.registers[0] & filter.registers[1]);
                assert_eq!(matches, remote.is_none() || remote == Some(received_remote));
                assert_ne!(
                    (received ^ 4) & filter.registers[1],
                    filter.registers[0] & filter.registers[1]
                );
            }
        }
    }
}

#[test]
fn register_geometry_matches_v4_two_fifos_and_fourteen_banks() {
    let regs = unsafe { pac::can::Can::from_ptr(0x4000_6400 as *mut ()) };
    let base = regs.as_ptr() as usize;
    assert_eq!(regs.rfifo(0).as_ptr() as usize - base, 0x0c);
    assert_eq!(regs.rfifo(1).as_ptr() as usize - base, 0x10);
    assert_eq!(regs.txmir(2).as_ptr() as usize - base, 0x1a0);
    assert_eq!(regs.rxmir(1).as_ptr() as usize - base, 0x1c0);
    assert_eq!(regs.fr(27).as_ptr() as usize - base, 0x2ac);
    assert_eq!(CanFifo::Fifo0.interrupts(), 0x0a);
    assert_eq!(CanFifo::Fifo1.interrupts(), 0x50);
}

#[test]
fn receive_releases_only_the_selected_fifo() {
    let mut words = [0u32; 192];
    let regs = unsafe { pac::can::Can::from_ptr(words.as_mut_ptr().cast()) };
    words[0x10 / 4] = 1;
    words[0x1c0 / 4] = (0x123 << 21) | 2;
    words[0x1c4 / 4] = 8;
    let frame = receive(regs, CanFifo::Fifo1).unwrap();
    assert!(frame.is_remote_frame());
    assert_eq!(frame.dlc(), 8);
    assert_eq!(words[0x10 / 4], 1 << 5);
    assert_eq!(words[0x0c / 4], 0);
}

#[test]
fn overrun_and_protocol_errors_use_exact_clear_writes() {
    let mut words = [0u32; 192];
    let regs = unsafe { pac::can::Can::from_ptr(words.as_mut_ptr().cast()) };
    words[3] = 0x13;
    assert_eq!(receive(regs, CanFifo::Fifo0), Err(nb::Error::Other(CanError::Overrun)));
    assert_eq!(words[3], 0x10);
    for (code, error) in [
        (1, CanError::Stuff),
        (2, CanError::Form),
        (3, CanError::Acknowledge),
        (4, CanError::Bit),
        (5, CanError::Bit),
        (6, CanError::Crc),
    ] {
        words[1] = 1 << 2;
        words[6] = code << 4;
        assert_eq!(take_error(regs), Some(error));
        assert_eq!(words[6], 7 << 4);
        assert_eq!(words[1], 1 << 2);
    }
    words[6] = 4 | (3 << 4);
    assert_eq!(take_error(regs), Some(CanError::BusOff));
}

#[test]
fn initialization_wait_is_bounded() {
    let mut words = [0u32; 192];
    let regs = unsafe { pac::can::Can::from_ptr(words.as_mut_ptr().cast()) };
    assert_eq!(wait_init(regs, true, 3), Err(CanInitError::Timeout));
    assert_eq!(wait_init(regs, false, 3), Ok(()));
    assert_eq!(wait_init(regs, false, 0), Err(CanInitError::Timeout));
}
