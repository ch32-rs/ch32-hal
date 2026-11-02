use super::*;

#[test]
fn register_offsets_match_both_channels_and_dual_data_formats() {
    let mut words = [0u32; 13];
    let base = words.as_mut_ptr() as usize;
    let regs = unsafe { crate::pac::dac::Dac::from_ptr(words.as_mut_ptr().cast()) };
    for channel in 0..2 {
        assert_eq!(regs.dhr12r(channel).as_ptr() as usize - base, 8 + channel * 12);
        assert_eq!(regs.dhr12l(channel).as_ptr() as usize - base, 12 + channel * 12);
        assert_eq!(regs.dhr8r(channel).as_ptr() as usize - base, 16 + channel * 12);
        assert_eq!(regs.dor(channel).as_ptr() as usize - base, 44 + channel * 4);
    }
    assert_eq!(regs.dhr12rd().as_ptr() as usize - base, 32);
    assert_eq!(regs.dhr12ld().as_ptr() as usize - base, 36);
    assert_eq!(regs.dhr8rd().as_ptr() as usize - base, 40);
}

#[test]
fn sample_formats_mask_and_pack_without_touching_the_other_channel() {
    let mut words = [0u32; 13];
    let regs = unsafe { crate::pac::dac::Dac::from_ptr(words.as_mut_ptr().cast()) };
    regs.dhr12rd().write(|reg| {
        reg.set_dhr(0, 0x1234);
        reg.set_dhr(1, 0xabcd);
    });
    assert_eq!(words[8], 0x0bcd_0234);
    regs.dhr12ld().write(|reg| {
        reg.set_dhr(0, 0x1234);
        reg.set_dhr(1, 0xabcd);
    });
    assert_eq!(words[9], 0xbcd0_2340);
    regs.dhr8rd().write(|reg| {
        reg.set_dhr(0, 0x12);
        reg.set_dhr(1, 0xab);
    });
    assert_eq!(words[10], 0xab12);
    regs.dhr12l(1).write(|reg| reg.set_dhr(0xabcd));
    assert_eq!(words[6], 0xbcd0);
    assert_eq!(words[3], 0);
}

#[test]
fn dma_rejects_missing_or_software_triggers_and_unrepresentable_lengths() {
    for channel in 0..2 {
        let mut control = crate::pac::dac::regs::Cr(0);
        assert_eq!(validate_dma(&control, channel, 0), Ok(()));
        assert_eq!(validate_dma(&control, channel, 1), Err(Error::TriggerNotConfigured));
        control.set_ten(channel, true);
        for source in [
            TriggerSel::TIM6_TRGO,
            TriggerSel::TIM8_TRGO,
            TriggerSel::TIM7_TRGO,
            TriggerSel::TIM5_TRGO,
            TriggerSel::TIM2_TRGO,
            TriggerSel::TIM4_TRGO,
            TriggerSel::EXTI9,
        ] {
            control.set_tsel(channel, source);
            assert_eq!(validate_dma(&control, channel, 1), Ok(()));
            assert_eq!(validate_dma(&control, channel, 65535), Ok(()));
            assert_eq!(validate_dma(&control, channel, 65536), Err(Error::BufferTooLong));
        }
        control.set_tsel(channel, TriggerSel::SOFTWARE);
        assert_eq!(validate_dma(&control, channel, 1), Err(Error::TriggerNotConfigured));
    }
}

#[test]
fn trigger_encodings_and_channel_control_bits_match_the_manual() {
    for (source, encoding) in [
        (TriggerSel::TIM6_TRGO, 0),
        (TriggerSel::TIM8_TRGO, 1),
        (TriggerSel::TIM7_TRGO, 2),
        (TriggerSel::TIM5_TRGO, 3),
        (TriggerSel::TIM2_TRGO, 4),
        (TriggerSel::TIM4_TRGO, 5),
        (TriggerSel::EXTI9, 6),
        (TriggerSel::SOFTWARE, 7),
    ] {
        assert_eq!(source as u8, encoding);
    }
    for channel in 0..2 {
        let mut control = crate::pac::dac::regs::Cr(0);
        control.set_en(channel, true);
        control.set_dmaen(channel, true);
        control.set_ten(channel, true);
        control.set_boff(channel, true);
        assert_eq!(control.0, 0x1007 << (channel * 16));
    }
}
