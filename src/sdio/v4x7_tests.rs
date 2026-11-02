use super::*;

fn card(capacity: CardCapacity, blocks: u64) -> Card {
    Card {
        card_type: capacity,
        ocr: 0.into(),
        rca: 1,
        cid: 0u128.into(),
        csd: 0u128.into(),
        scr: SCR(0),
        status: SDStatus::default(),
        block_count: blocks,
    }
}

#[test]
fn clock_dividers_round_up_without_wrapping_or_overspeeding() {
    for source in [1u32, 39_062, 20_000_000, 25_000_000, 100_000_000, 200_000_000, u32::MAX] {
        for requested in [1u32, 400_000, 1_000_000, 10_000_000, 25_000_000, 50_000_000, 80_000_000] {
            let divisor = u64::from(source).div_ceil(u64::from(requested)).max(2);
            match clock_setting(Hertz(source), requested, false) {
                Ok(clock) => {
                    assert!(divisor <= 257);
                    assert_eq!(u64::from(clock.divisor) + 2, divisor);
                    assert!(!clock.bypass);
                    assert!(clock.frequency.0 <= requested);
                }
                Err(Error::BadClock) => assert!(divisor > 257),
                other => panic!("unexpected clock result: {other:?}"),
            }
        }
    }
    assert_eq!(clock_setting(Hertz(102_800_000), 400_000, false).unwrap().divisor, 255);
    assert_eq!(clock_setting(Hertz(103_200_000), 400_000, false), Err(Error::BadClock));
    assert_eq!(clock_setting(Hertz(20_000_000), 0, false), Err(Error::BadClock));
    assert_eq!(clock_setting(Hertz(0), 400_000, false), Err(Error::BadClock));
    assert_eq!(
        clock_setting(Hertz(100_000_000), 80_000_001, false),
        Err(Error::BadClock)
    );
}

#[test]
fn bypass_is_only_selected_when_explicitly_permitted() {
    let divided = clock_setting(Hertz(20_000_000), 25_000_000, false).unwrap();
    assert!(!divided.bypass);
    assert_eq!(divided.frequency, Hertz(10_000_000));
    let bypass = clock_setting(Hertz(20_000_000), 25_000_000, true).unwrap();
    assert!(bypass.bypass);
    assert_eq!(bypass.frequency, Hertz(20_000_000));
    assert!(!clock_setting(Hertz(100_000_000), 25_000_000, true).unwrap().bypass);
}

#[test]
fn csd_capacity_uses_512_byte_blocks_and_preserves_the_sdxc_boundary() {
    for length in 9u128..=11 {
        let csd = (length << 80) | (0xfff << 62) | (7 << 47);
        assert_eq!(block_count(csd), Ok(4096 * 512 * (1 << (length - 9))));
    }
    assert_eq!(block_count((1 << 126) | (0x3f_ffff << 48)), Ok(1u64 << 32));
    assert_eq!(block_count(1 << 126), Ok(1024));
    assert_eq!(block_count(2 << 126), Err(Error::UnsupportedCardType));
    assert_eq!(block_count(8 << 80), Err(Error::UnsupportedCardType));
}

#[test]
fn block_ranges_reject_dma_overflow_and_byte_address_overflow() {
    let sdxc = card(CardCapacity::SDHC, 1u64 << 32);
    assert_eq!(block_address(&sdxc, u32::MAX, 1), Ok(u32::MAX));
    assert_eq!(block_address(&sdxc, u32::MAX, 2), Err(Error::AddressOutOfRange));
    assert_eq!(block_address(&sdxc, 0, 511), Ok(0));
    assert_eq!(block_address(&sdxc, 0, 512), Err(Error::InvalidBufferLength));
    assert_eq!(block_address(&sdxc, 0, 0), Err(Error::InvalidBufferLength));
    let sdsc = card(CardCapacity::SDSC, 1u64 << 32);
    assert_eq!(block_address(&sdsc, 7, 1), Ok(7 * 512));
    assert_eq!(block_address(&sdsc, u32::MAX, 1), Err(Error::AddressOutOfRange));
    assert_eq!(core::mem::size_of::<DataBlock>(), 512);
    assert_eq!(core::mem::align_of::<DataBlock>(), 4);
}

#[test]
fn data_faults_have_priority_over_completion() {
    assert_eq!(data_error(0x100), Ok(()));
    for (flag, error) in [
        (8, Error::Timeout),
        (2, Error::Crc),
        (0x20, Error::Overrun),
        (0x10, Error::Underrun),
        (0x200, Error::StBitErr),
    ] {
        assert_eq!(data_error(flag | 0x500), Err(error));
    }
    assert_eq!(data_error(DATA_FLAGS), Err(Error::Timeout));
}

#[test]
fn register_and_fifo_byte_order_match_card_protocol_order() {
    assert_eq!(
        response_value([0x01234567, 0x89abcdef, 0xfedcba98, 0x76543210]),
        0x0123456789abcdeffedcba9876543210
    );
    let bytes: [u8; 64] = core::array::from_fn(|index| index as u8);
    let words = core::array::from_fn(|index| u32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().unwrap()));
    assert_eq!(status_bytes(&words), bytes);
    let mut words = [0; 16];
    words[0] = u32::from_le_bytes([0x80, 0, 0, 0]);
    words[2] = u32::from_le_bytes([10, 0, 0, 0]);
    let status = sd_status(&words);
    assert_eq!(status.bus_width(), sdio_host::BusWidth::Four);
    assert_eq!(status.speed_class(), 10);
}

#[test]
fn csd_transfer_rates_reject_reserved_encodings() {
    assert_eq!(transfer_rate(0x32), Ok(25_000_000));
    assert_eq!(transfer_rate(0x5a), Ok(50_000_000));
    assert_eq!(transfer_rate(0), Err(Error::UnsupportedCardType));
    assert_eq!(transfer_rate(0xb2), Err(Error::UnsupportedCardType));
    assert_eq!(transfer_rate(0x3f), Err(Error::UnsupportedCardType));
}
