use super::*;

#[test]
fn dma_register_fields_keep_the_complete_sram_address() {
    let mut ep0 = pac::usbhs::regs::Uep0Dma(0);
    ep0.set_uep0_dma(0x2003_0000);
    assert_eq!(ep0.uep0_dma(), 0x2003_0000);
    let mut rx = pac::usbhs::regs::Uep1RxDma(0);
    rx.set_uep1_rx_dma(0x2001_2344);
    assert_eq!(rx.uep1_rx_dma(), 0x2001_2344);
    let mut tx = pac::usbhs::regs::Uep7TxDma(0);
    tx.set_uep7_tx_dma(0x2002_3454);
    assert_eq!(tx.uep7_tx_dma(), 0x2002_3454);
}

#[test]
fn indexed_registers_match_pac_device_offsets_and_widths() {
    let usb = unsafe { pac::usbhs::Usb::from_ptr(0x4002_4000 as *mut ()) };
    let device = unsafe { pac::usbhs::UsbDevice::from_ptr(usb.as_ptr()) };
    let regs = Registers(usb);
    assert_eq!(
        regs.dma(0, false).as_ptr() as usize,
        device.uep0_dma().as_ptr() as usize
    );
    assert_eq!(
        regs.dma(7, false).as_ptr() as usize,
        device.uep7_rx_dma().as_ptr() as usize
    );
    assert_eq!(
        regs.dma(7, true).as_ptr() as usize,
        device.uep7_tx_dma().as_ptr() as usize
    );
    assert_eq!(
        regs.control(0, true).as_ptr() as usize,
        device.uep0_tx_ctrl().as_ptr() as usize
    );
    assert_eq!(
        regs.control(7, false).as_ptr() as usize,
        device.uep7_rx_ctrl().as_ptr() as usize
    );
    assert_eq!(
        regs.rx_length(0).as_ptr() as usize,
        device.uep0_rx_len().as_ptr() as usize
    );
    assert_eq!(
        regs.rx_length(7).as_ptr() as usize,
        device.uep7_rx_len().as_ptr() as usize
    );
    assert_eq!(
        regs.tx_length(7).as_ptr() as usize,
        device.uep7_t_len().as_ptr() as usize
    );
    assert_eq!(
        regs.max_length(7).as_ptr() as usize,
        device.uep7_max_len().as_ptr() as usize
    );
    assert_eq!(regs.iso(true).as_ptr() as usize, device.uep_t_iso().as_ptr() as usize);
    let _: pac::common::Reg<u8, pac::common::RW> = regs.control(0, true);
    let _: pac::common::Reg<u16, pac::common::RW> = regs.tx_length(0);
    let _: pac::common::Reg<u32, pac::common::RW> = regs.dma(0, true);
}

#[test]
fn endpoint_slots_are_independent_for_both_directions() {
    let mut visited = [false; SLOTS];
    for index in 0..ENDPOINTS {
        for incoming in [false, true] {
            let slot = slot(index, incoming);
            assert!(!visited[slot]);
            visited[slot] = true;
        }
    }
    assert!(visited.into_iter().all(|value| value));
}

#[test]
fn packet_sizes_follow_full_and_high_speed_transfer_limits() {
    for size in 0..=2048 {
        assert_eq!(
            packet_size_valid(Speed::Full, EndpointType::Bulk, size, 0),
            matches!(size, 8 | 16 | 32 | 64)
        );
        assert_eq!(packet_size_valid(Speed::High, EndpointType::Bulk, size, 0), size == 512);
        assert_eq!(
            packet_size_valid(Speed::Full, EndpointType::Interrupt, size, 1),
            (1..=64).contains(&size)
        );
        assert_eq!(
            packet_size_valid(Speed::High, EndpointType::Interrupt, size, 1),
            (1..=1024).contains(&size)
        );
        assert_eq!(
            packet_size_valid(Speed::Full, EndpointType::Isochronous, size, 1),
            (1..=1023).contains(&size)
        );
        assert_eq!(
            packet_size_valid(Speed::High, EndpointType::Isochronous, size, 1),
            (1..=1024).contains(&size)
        );
    }
    assert!(!packet_size_valid(Speed::Low, EndpointType::Interrupt, 8, 1));
    assert!(!packet_size_valid(Speed::Full, EndpointType::Control, 64, 1));
    assert!(!packet_size_valid(Speed::High, EndpointType::Interrupt, 64, 0));
    assert!(!packet_size_valid(Speed::High, EndpointType::Interrupt, 64, 17));
    assert!(!packet_size_valid(Speed::Full, EndpointType::Isochronous, 64, 0));
}

#[test]
fn dma_memory_bounds_reject_external_ram_and_wrapping_ranges() {
    for size in [136 * 1024, 200 * 1024] {
        assert!(memory_valid(0x2000_0000, size, size));
        assert!(memory_valid(0x2000_0001, size - 1, size));
        assert!(!memory_valid(0x2000_0000, size + 1, size));
        assert!(!memory_valid(0x2000_0000 - 1, size, size));
        assert!(!memory_valid(0x6000_0000, 64, size));
        assert!(!memory_valid(usize::MAX - 31, 64, size));
    }
}

#[test]
fn invalidation_keeps_geometry_and_changes_transfer_generation() {
    let config = EndpointConfig {
        info: EndpointInfo {
            addr: EndpointAddress::from_parts(1, Direction::In),
            ep_type: EndpointType::Bulk,
            max_packet_size: 64,
            interval_ms: 0,
        },
        buffer: Buffer {
            address: 0x2000_0000,
            size: 64,
        },
    };
    let mut endpoint = EndpointState::new();
    endpoint.config = Some(config);
    endpoint.enabled = true;
    endpoint.tx_busy = true;
    endpoint.rx_ready = true;
    endpoint.stalled = true;
    endpoint.overflow = true;
    endpoint.generation = u32::MAX;
    endpoint.invalidate();
    assert_eq!(endpoint.config.unwrap().info, config.info);
    assert_eq!(endpoint.generation, 0);
    assert!(!endpoint.enabled && !endpoint.tx_busy && !endpoint.rx_ready && !endpoint.stalled && !endpoint.overflow);
}
