//! SuperSpeed descriptor set for the USBSS device.
//!
//! The layout follows the EVT `USBSS/DEVICE/CH372Device` example. A SuperSpeed
//! device must report `bcdUSB = 0x0300` and expose a BOS descriptor; every
//! bulk endpoint also needs an Endpoint Companion descriptor carrying its
//! burst size, which the USB 2.0 descriptors do not have.
//!
//! The device is vendor-specific (interface class `0xff`) with one bulk IN
//! and one bulk OUT endpoint, both 1024-byte packets with a 16-packet burst.
//! The endpoints are advertised from the start; the data path is wired up
//! separately.

/// Device descriptor. `bMaxPacketSize0 = 9` means a 512-byte EP0.
pub const DEVICE: &[u8] = &[
    0x12, // bLength
    0x01, // bDescriptorType: DEVICE
    0x00, 0x03, // bcdUSB 3.00
    0x00, // bDeviceClass (defined per interface)
    0x00, // bDeviceSubClass
    0x00, // bDeviceProtocol
    0x09, // bMaxPacketSize0: 2^9 = 512
    0xde, 0xc0, // idVendor 0xc0de
    0xfe, 0xca, // idProduct 0xcafe
    0x00, 0x01, // bcdDevice 1.00
    0x01, // iManufacturer
    0x02, // iProduct
    0x03, // iSerialNumber
    0x01, // bNumConfigurations
];

/// Configuration, interface and the two bulk endpoints with their companions.
pub const CONFIG: &[u8] = &[
    // Configuration descriptor.
    0x09, // bLength
    0x02, // bDescriptorType: CONFIGURATION
    0x2c, 0x00, // wTotalLength: 44
    0x01, // bNumInterfaces
    0x01, // bConfigurationValue
    0x00, // iConfiguration
    0x80, // bmAttributes: bus powered
    0x0d, // bMaxPower: 104 mA (8 mA units on SuperSpeed)
    // Interface descriptor.
    0x09, // bLength
    0x04, // bDescriptorType: INTERFACE
    0x00, // bInterfaceNumber
    0x00, // bAlternateSetting
    0x02, // bNumEndpoints
    0xff, // bInterfaceClass: vendor specific
    0x00, // bInterfaceSubClass
    0x00, // bInterfaceProtocol
    0x00, // iInterface
    // Bulk IN endpoint.
    0x07, // bLength
    0x05, // bDescriptorType: ENDPOINT
    0x81, // bEndpointAddress: IN, endpoint 1
    0x02, // bmAttributes: bulk
    0x00, 0x04, // wMaxPacketSize: 1024
    0x00, // bInterval
    // Endpoint companion: 16 packets per burst.
    0x06, // bLength
    0x30, // bDescriptorType: ENDPOINT_COMPANION
    0x0f, // bMaxBurst: 16 packets
    0x00, // bmAttributes
    0x00, 0x00, // wBytesPerInterval
    // Bulk OUT endpoint.
    0x07, // bLength
    0x05, // bDescriptorType: ENDPOINT
    0x02, // bEndpointAddress: OUT, endpoint 2
    0x02, // bmAttributes: bulk
    0x00, 0x04, // wMaxPacketSize: 1024
    0x00, // bInterval
    // Endpoint companion: 16 packets per burst.
    0x06, // bLength
    0x30, // bDescriptorType: ENDPOINT_COMPANION
    0x0f, // bMaxBurst: 16 packets
    0x00, // bmAttributes
    0x00, 0x00, // wBytesPerInterval
];

/// BOS: a USB 2.0 extension plus the SuperSpeed device capability.
///
/// The EVT example also advertises a SuperSpeedPlus capability. This device
/// only trains Gen 1, so that block is deliberately absent: advertising Gen 2
/// would invite the host to request a speed change this driver cannot do.
pub const BOS: &[u8] = &[
    // BOS descriptor.
    0x05, // bLength
    0x0f, // bDescriptorType: BOS
    0x16, 0x00, // wTotalLength: 22
    0x01, // bNumDeviceCaps
    // USB 2.0 extension: link power management.
    0x07, // bLength
    0x10, // bDescriptorType: DEVICE_CAPABILITY
    0x02, // bDevCapabilityType: USB 2.0 EXTENSION
    0x02, 0x00, 0x00, 0x00, // bmAttributes: LPM
    // SuperSpeed device capability.
    0x0a, // bLength
    0x10, // bDescriptorType: DEVICE_CAPABILITY
    0x03, // bDevCapabilityType: SUPERSPEED
    0x00, // bmAttributes
    0x0e, 0x00, // wSpeedsSupported: full, high, SuperSpeed
    0x01, // bFunctionalitySupport: full-speed
    0x0a, // bU1DevExitLat: 10 us
    0x20, 0x00, // wU2DevExitLat: 32 us
];

/// USB 2.0 device qualifier, for hosts that ask even on a SuperSpeed link.
pub const DEVICE_QUALIFIER: &[u8] = &[
    0x0a, // bLength
    0x06, // bDescriptorType: DEVICE_QUALIFIER
    0x00, 0x02, // bcdUSB 2.00
    0xff, // bDeviceClass
    0xff, // bDeviceSubClass
    0xff, // bDeviceProtocol
    0x40, // bMaxPacketSize0: 64
    0x01, // bNumConfigurations
    0x00, // bReserved
];

/// String descriptor zero: US English.
pub const LANG: &[u8] = &[0x04, 0x03, 0x09, 0x04];

/// Manufacturer string ("ch32-hal").
pub const MANUFACTURER: &[u8] = &string::<18>("ch32-hal");

/// Product string ("CH32H4 USBSS").
pub const PRODUCT: &[u8] = &string::<26>("CH32H4 USBSS");

/// Serial-number string.
pub const SERIAL: &[u8] = &string::<28>("H4-USBSS-0001");

/// Builds a UTF-16LE string descriptor. `N` is `2 + 2 * s.len()`; longer
/// strings are truncated rather than producing a bad `bLength`.
const fn string<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    out[0] = N as u8;
    out[1] = 0x03;

    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() && 3 + i * 2 < N {
        out[2 + i * 2] = bytes[i];
        i += 1;
    }
    out
}

/// Looks up a descriptor by type and index.
///
/// `GET_DESCRIPTOR` puts the type in the high byte of `wValue` and the index
/// in the low byte.
pub(crate) fn get(desc_type: u8, index: u8) -> Option<&'static [u8]> {
    match desc_type {
        0x01 => Some(DEVICE),
        0x02 => Some(CONFIG),
        0x03 => match index {
            0 => Some(LANG),
            1 => Some(MANUFACTURER),
            2 => Some(PRODUCT),
            3 => Some(SERIAL),
            _ => None,
        },
        0x06 => Some(DEVICE_QUALIFIER),
        0x0f => Some(BOS),
        _ => None,
    }
}
