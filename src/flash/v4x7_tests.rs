use super::*;

#[test]
fn geometry_matches_v4_standard_and_fast_modes() {
    let region = pac::MEMORY_LAYOUT
        .find_by_role(pac::mem_layout::MemoryRole::Application)
        .unwrap();
    assert_eq!(region.erase_size(), Some(ERASE_SIZE as u32));
    assert_eq!(region.write_size(), Some(HALFWORD_WRITE_SIZE as u32));
    assert_eq!(region.page_program_size(), Some(WRITE_SIZE as u32));
    assert_eq!(region.fast_load_size(), Some(4));
    assert_eq!(WRITE_SIZE, 256);
    assert_eq!(ERASE_SIZE, 4096);
    assert_eq!(FLASH_SIZE, 992 * 1024);
}

#[test]
fn bounds_checks_reject_wraparound_and_allow_empty_eof() {
    for (offset, length) in [(u32::MAX, 1), (0, usize::MAX), (u32::MAX, usize::MAX), (4095, 2)] {
        assert_eq!(validate_slice(4096, offset, length, 1), Err(Error::Size));
    }
    assert_eq!(validate_slice(4096, 4096, 0, 256), Ok(()));
    assert_eq!(validate_slice(4096, 4097, 0, 1), Err(Error::Size));
    assert_eq!(validate_slice(4096, 1, 256, 256), Err(Error::Unaligned));
    assert_eq!(validate_slice(4096, 0, 255, 256), Err(Error::Unaligned));
    assert_eq!(validate_slice(4096, 2, 2, 2), Ok(()));
    assert_eq!(validate_erase(8192, 4096, 0, 4096), Err(Error::Size));
    assert_eq!(validate_erase(8192, 0, 4096, 4096), Ok(()));
    assert_eq!(validate_erase(8192, 0, 256, 4096), Err(Error::Unaligned));
}

#[test]
fn protection_last_bit_covers_all_sectors_after_thirty_one() {
    assert_eq!(validate_protection(0, FLASH_SIZE, u32::MAX, false), Ok(()));
    assert_eq!(validate_protection(0, 2, u32::MAX, true), Err(Error::Protected));
    assert_eq!(validate_protection(4096, 2, u32::MAX, true), Ok(()));
    for sector in [0, 1, 30, 31, 32, 247] {
        let mask = 1 << sector.min(31);
        assert_eq!(
            validate_protection(sector * 4096, 2, !mask, false),
            Err(Error::Protected)
        );
    }
    assert_eq!(
        validate_protection(30 * 4096, 4098, !(1 << 31), false),
        Err(Error::Protected)
    );
    assert_eq!(validate_protection(FLASH_SIZE as u32, 0, 0, true), Ok(()));
}

#[test]
fn nor_encoding_normalizes_erased_pattern_and_handles_odd_reads() {
    let mut erased = [0x39, 0xe3, 0x39, 0xe3];
    xor_bytes(0, &mut erased);
    assert_eq!(erased, [0xff; 4]);
    let mut odd = [0xe3, 0x39, 0xe3];
    xor_bytes(1, &mut odd);
    assert_eq!(odd, [0xff; 3]);
    let mut data: [u8; 256] = core::array::from_fn(|index| index as u8);
    let original = data;
    xor_bytes(0, &mut data);
    assert_ne!(data, original);
    xor_bytes(0, &mut data);
    assert_eq!(data, original);
}

#[test]
fn flash_code_must_be_in_zero_wait_region_or_its_boot_alias() {
    let size = 512 * 1024;
    assert_eq!(validate_execution_address(4, size), Ok(()));
    assert_eq!(validate_execution_address(size - 2, size), Ok(()));
    assert_eq!(
        validate_execution_address(size, size),
        Err(Error::InvalidExecutionRegion)
    );
    assert_eq!(validate_execution_address(FLASH_BASE + size - 2, size), Ok(()));
    assert_eq!(
        validate_execution_address(FLASH_BASE + size, size),
        Err(Error::InvalidExecutionRegion)
    );
    assert_eq!(
        validate_execution_address(0x2000_0000, size),
        Err(Error::InvalidExecutionRegion)
    );
    assert_eq!(validate_execution_address(FLASH_BASE + size, 576 * 1024), Ok(()));
}

#[test]
fn flash_register_layout_and_operation_masks_do_not_touch_options_or_clock() {
    let regs = unsafe { pac::flash::Flash::from_ptr(0x4002_2000 as *mut ()) };
    assert_eq!(regs.statr().as_ptr() as usize, 0x4002_200c);
    assert_eq!(regs.ctlr().as_ptr() as usize, 0x4002_2010);
    assert_eq!(regs.modekeyr().as_ptr() as usize, 0x4002_2024);
    assert_eq!(
        OPERATION_MASK & ((3 << 25) | (1 << 9) | (1 << 4) | (1 << 5) | (1 << 2)),
        0
    );
    assert_eq!(CLEAR_FLAGS, 0x30);
}
